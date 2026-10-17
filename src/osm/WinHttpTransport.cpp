#include "osm/WinHttpTransport.h"

#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include <windows.h>
#include <winhttp.h>

#include <vector>

#pragma comment(lib, "winhttp.lib")

namespace osm_net {
namespace {

std::wstring utf8ToWide(const std::string& utf8) {
    if (utf8.empty()) {
        return {};
    }
    const int size = MultiByteToWideChar(CP_UTF8, 0, utf8.c_str(), -1, nullptr, 0);
    if (size <= 1) {
        return {};
    }
    std::wstring out(static_cast<size_t>(size - 1), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, utf8.c_str(), -1, out.data(), size);
    return out;
}

bool cancelled(std::atomic<bool>* flag) {
    return flag != nullptr && flag->load();
}

struct UrlParts {
    std::wstring host;
    std::wstring path;
    INTERNET_PORT port = INTERNET_DEFAULT_HTTPS_PORT;
    bool https = true;
};

bool crackUrl(const std::wstring& url, UrlParts& out, std::string& error) {
    URL_COMPONENTSW parts {};
    parts.dwStructSize = sizeof(parts);
    wchar_t host[256] {};
    wchar_t path[2048] {};
    parts.lpszHostName = host;
    parts.dwHostNameLength = 256;
    parts.lpszUrlPath = path;
    parts.dwUrlPathLength = 2048;

    if (!WinHttpCrackUrl(url.c_str(), 0, 0, &parts)) {
        error = "Invalid URL";
        return false;
    }

    out.host.assign(parts.lpszHostName, parts.dwHostNameLength);
    out.path.assign(parts.lpszUrlPath, parts.dwUrlPathLength);
    out.port = parts.nPort;
    out.https = (parts.nScheme == INTERNET_SCHEME_HTTPS);
    if (!out.https && parts.nScheme != INTERNET_SCHEME_HTTP) {
        error = "Only HTTP/HTTPS URLs are supported";
        return false;
    }
    // Prefer HTTPS for public APIs; still allow http for local test endpoints.
    return true;
}

} // namespace

HttpResponse WinHttpTransport::send(const HttpRequest& request, std::atomic<bool>* cancelFlag) {
    HttpResponse response;

    if (request.url.empty()) {
        response.error = "Empty URL";
        return response;
    }

    // Callers should use https:// for public Nominatim / Overpass endpoints.
    const std::wstring wUrl = utf8ToWide(request.url);
    UrlParts parts;
    if (!crackUrl(wUrl, parts, response.error)) {
        return response;
    }

    if (cancelled(cancelFlag)) {
        response.cancelled = true;
        response.error = "Cancelled";
        return response;
    }

    HINTERNET session = WinHttpOpen(
        utf8ToWide(request.userAgent.empty() ? kEasyRiderUserAgent : request.userAgent).c_str(),
        WINHTTP_ACCESS_TYPE_DEFAULT_PROXY,
        WINHTTP_NO_PROXY_NAME,
        WINHTTP_NO_PROXY_BYPASS,
        0
    );
    if (session == nullptr) {
        response.error = "WinHttpOpen failed";
        return response;
    }

    const int timeout = request.timeoutMs > 0 ? request.timeoutMs : kDefaultHttpTimeoutMs;
    WinHttpSetTimeouts(session, timeout, timeout, timeout, timeout);

    HINTERNET connect = WinHttpConnect(session, parts.host.c_str(), parts.port, 0);
    if (connect == nullptr) {
        response.error = "WinHttpConnect failed";
        WinHttpCloseHandle(session);
        return response;
    }

    const DWORD flags = parts.https ? WINHTTP_FLAG_SECURE : 0;
    const std::wstring method = utf8ToWide(request.method.empty() ? "GET" : request.method);
    HINTERNET requestHandle = WinHttpOpenRequest(
        connect,
        method.c_str(),
        parts.path.c_str(),
        nullptr,
        WINHTTP_NO_REFERER,
        WINHTTP_DEFAULT_ACCEPT_TYPES,
        flags
    );
    if (requestHandle == nullptr) {
        response.error = "WinHttpOpenRequest failed";
        WinHttpCloseHandle(connect);
        WinHttpCloseHandle(session);
        return response;
    }

    std::wstring extraHeaders = L"Accept: application/json\r\n";
    if (!request.contentType.empty()) {
        extraHeaders += L"Content-Type: " + utf8ToWide(request.contentType) + L"\r\n";
    }

    if (cancelled(cancelFlag)) {
        response.cancelled = true;
        response.error = "Cancelled";
        WinHttpCloseHandle(requestHandle);
        WinHttpCloseHandle(connect);
        WinHttpCloseHandle(session);
        return response;
    }

    const void* bodyPtr = request.body.empty() ? WINHTTP_NO_REQUEST_DATA : request.body.data();
    const DWORD bodyLen = static_cast<DWORD>(request.body.size());

    const BOOL sent = WinHttpSendRequest(
        requestHandle,
        extraHeaders.c_str(),
        static_cast<DWORD>(-1),
        const_cast<void*>(bodyPtr),
        bodyLen,
        bodyLen,
        0
    );
    if (!sent) {
        const DWORD err = GetLastError();
        if (err == ERROR_WINHTTP_TIMEOUT) {
            response.timedOut = true;
            response.error = "HTTP request timed out";
        } else {
            response.error = "WinHttpSendRequest failed";
        }
        WinHttpCloseHandle(requestHandle);
        WinHttpCloseHandle(connect);
        WinHttpCloseHandle(session);
        return response;
    }

    if (!WinHttpReceiveResponse(requestHandle, nullptr)) {
        const DWORD err = GetLastError();
        if (err == ERROR_WINHTTP_TIMEOUT) {
            response.timedOut = true;
            response.error = "HTTP response timed out";
        } else {
            response.error = "WinHttpReceiveResponse failed";
        }
        WinHttpCloseHandle(requestHandle);
        WinHttpCloseHandle(connect);
        WinHttpCloseHandle(session);
        return response;
    }

    DWORD statusCode = 0;
    DWORD statusSize = sizeof(statusCode);
    WinHttpQueryHeaders(
        requestHandle,
        WINHTTP_QUERY_STATUS_CODE | WINHTTP_QUERY_FLAG_NUMBER,
        WINHTTP_HEADER_NAME_BY_INDEX,
        &statusCode,
        &statusSize,
        WINHTTP_NO_HEADER_INDEX
    );
    response.statusCode = static_cast<int>(statusCode);

    std::string body;
    body.reserve(4096);
    bool sizeExceeded = false;

    while (true) {
        if (cancelled(cancelFlag)) {
            response.cancelled = true;
            response.error = "Cancelled";
            break;
        }

        DWORD available = 0;
        if (!WinHttpQueryDataAvailable(requestHandle, &available)) {
            response.error = "WinHttpQueryDataAvailable failed";
            break;
        }
        if (available == 0) {
            break;
        }

        if (body.size() + static_cast<size_t>(available) > request.maxResponseBytes) {
            sizeExceeded = true;
            response.sizeExceeded = true;
            response.error = "Response exceeds configured size limit — choose a smaller area";
            // Drain is skipped; we intentionally do not keep a truncated body for parsing.
            body.clear();
            break;
        }

        std::vector<char> buffer(available);
        DWORD read = 0;
        if (!WinHttpReadData(requestHandle, buffer.data(), available, &read)) {
            const DWORD err = GetLastError();
            if (err == ERROR_WINHTTP_TIMEOUT) {
                response.timedOut = true;
                response.error = "HTTP read timed out";
            } else {
                response.error = "WinHttpReadData failed";
            }
            break;
        }
        if (read == 0) {
            break;
        }
        body.append(buffer.data(), buffer.data() + read);
    }

    WinHttpCloseHandle(requestHandle);
    WinHttpCloseHandle(connect);
    WinHttpCloseHandle(session);

    if (response.cancelled || response.timedOut || sizeExceeded) {
        return response;
    }
    if (!response.error.empty() && body.empty()) {
        return response;
    }

    response.body = std::move(body);
    if (response.statusCode >= 200 && response.statusCode < 300) {
        response.ok = true;
        response.error.clear();
    } else {
        response.ok = false;
        if (response.error.empty()) {
            response.error = "HTTP status " + std::to_string(response.statusCode);
        }
    }
    return response;
}

} // namespace osm_net
