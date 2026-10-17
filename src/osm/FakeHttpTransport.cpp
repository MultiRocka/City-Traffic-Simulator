#include "osm/FakeHttpTransport.h"

namespace osm_net {

void FakeHttpTransport::enqueue(FakeHttpScript script) {
    std::lock_guard<std::mutex> lock(mutex_);
    scripts_.push_back(std::move(script));
}

void FakeHttpTransport::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    scripts_.clear();
    recorded_.clear();
    calls_ = 0;
}

int FakeHttpTransport::callCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return calls_;
}

const std::vector<HttpRequest>& FakeHttpTransport::recordedRequests() const {
    return recorded_;
}

HttpResponse FakeHttpTransport::send(const HttpRequest& request, std::atomic<bool>* cancelFlag) {
    FakeHttpScript script;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        recorded_.push_back(request);
        ++calls_;

        if (scripts_.empty()) {
            HttpResponse missing;
            missing.error = "FakeHttpTransport: no scripted response left";
            return missing;
        }

        // Prefer first script whose urlContains matches (or empty contains).
        std::size_t chosen = 0;
        bool found = false;
        for (std::size_t i = 0; i < scripts_.size(); ++i) {
            if (scripts_[i].urlContains.empty()
                || request.url.find(scripts_[i].urlContains) != std::string::npos) {
                chosen = i;
                found = true;
                break;
            }
        }
        if (!found) {
            HttpResponse missing;
            missing.error = "FakeHttpTransport: no matching script for URL: " + request.url;
            return missing;
        }
        script = std::move(scripts_[chosen]);
        scripts_.erase(scripts_.begin() + static_cast<std::ptrdiff_t>(chosen));
    }

    if (cancelFlag != nullptr && cancelFlag->load()) {
        HttpResponse cancelled;
        cancelled.cancelled = true;
        cancelled.error = "Cancelled";
        return cancelled;
    }

    if (script.beforeReturn) {
        script.beforeReturn(cancelFlag);
    }

    if (cancelFlag != nullptr && cancelFlag->load()) {
        HttpResponse cancelled;
        cancelled.cancelled = true;
        cancelled.error = "Cancelled";
        return cancelled;
    }

    // Enforce size limit the same way production should (reject, do not truncate).
    if (script.response.ok
        && script.response.body.size() > request.maxResponseBytes) {
        HttpResponse oversized;
        oversized.sizeExceeded = true;
        oversized.statusCode = script.response.statusCode;
        oversized.error = "Response exceeds configured size limit";
        return oversized;
    }

    return script.response;
}

} // namespace osm_net
