#pragma once

#include "osm/HttpTransport.h"

namespace osm_net {

// Native WinHTTP client (HTTPS). Linked only on Windows GUI / core builds.
class WinHttpTransport : public IHttpTransport {
public:
    HttpResponse send(const HttpRequest& request, std::atomic<bool>* cancelFlag = nullptr) override;
};

} // namespace osm_net
