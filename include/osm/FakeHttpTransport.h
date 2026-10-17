#pragma once

#include "osm/HttpTransport.h"

#include <functional>
#include <mutex>
#include <string>
#include <vector>

namespace osm_net {

struct FakeHttpScript {
    // Matched if request.url contains this substring (empty = match any).
    std::string urlContains;
    HttpResponse response;
    // Optional: delay / cancel-check hook before returning.
    std::function<void(std::atomic<bool>*)> beforeReturn;
};

// Offline scripted transport for automated tests — never opens sockets.
class FakeHttpTransport : public IHttpTransport {
public:
    void enqueue(FakeHttpScript script);
    void clear();
    int callCount() const;
    const std::vector<HttpRequest>& recordedRequests() const;

    HttpResponse send(const HttpRequest& request, std::atomic<bool>* cancelFlag = nullptr) override;

private:
    mutable std::mutex mutex_;
    std::vector<FakeHttpScript> scripts_;
    std::vector<HttpRequest> recorded_;
    int calls_ = 0;
};

} // namespace osm_net
