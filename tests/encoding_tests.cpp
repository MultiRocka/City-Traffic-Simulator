#ifndef UNICODE
#define UNICODE
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <windows.h>

#include <iostream>
#include <string>

using namespace std;

namespace {

int g_failures = 0;
int g_passed = 0;

void expect(bool condition, const string& name) {
    if (condition) {
        ++g_passed;
        cout << "PASS  " << name << "\n";
    } else {
        ++g_failures;
        cout << "FAIL  " << name << "\n";
    }
}

std::string wideToUtf8(const std::wstring& wide) {
    if (wide.empty()) {
        return {};
    }
    const int size = WideCharToMultiByte(CP_UTF8, 0, wide.c_str(), -1, nullptr, 0, nullptr, nullptr);
    if (size <= 1) {
        return {};
    }
    std::string out(static_cast<size_t>(size - 1), '\0');
    WideCharToMultiByte(CP_UTF8, 0, wide.c_str(), -1, out.data(), size, nullptr, nullptr);
    return out;
}

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

bool containsCodePoint(const std::wstring& text, wchar_t cp) {
    return text.find(cp) != std::wstring::npos;
}

void testWideLiteralCodePoints() {
    // Proves MSVC /utf-8 decoded source UTF-8 wide literals correctly.
    const std::wstring sample = L"żółć";
    expect(sample.size() == 4, "enc_zolc_length");
    expect(sample[0] == 0x017C, "enc_zolc_z_dot");   // ż
    expect(sample[1] == 0x00F3, "enc_zolc_o_acute"); // ó
    expect(sample[2] == 0x0142, "enc_zolc_l_stroke"); // ł
    expect(sample[3] == 0x0107, "enc_zolc_c_acute"); // ć
}

void testPolishPhraseLiterals() {
    const std::wstring phrase =
        L"Łódź, żółć, przejście, prędkość, skrzyżowanie";
    expect(containsCodePoint(phrase, 0x0141), "enc_phrase_L_stroke"); // Ł
    expect(containsCodePoint(phrase, 0x00F3), "enc_phrase_o_acute");  // ó
    expect(containsCodePoint(phrase, 0x017C), "enc_phrase_z_dot");    // ż
    expect(containsCodePoint(phrase, 0x015B), "enc_phrase_s_acute");  // ś
    expect(containsCodePoint(phrase, 0x017A), "enc_phrase_z_acute");  // ź
    expect(containsCodePoint(phrase, 0x0107), "enc_phrase_c_acute");  // ć
    expect(containsCodePoint(phrase, 0x0119), "enc_phrase_e_ogonek"); // ę
}

void testUtf8RoundTrip() {
    const std::wstring wide = L"Łódź, żółć, przejście, prędkość, skrzyżowanie";
    const std::string utf8 = wideToUtf8(wide);
    const std::wstring back = utf8ToWide(utf8);
    expect(!utf8.empty(), "enc_roundtrip_utf8_nonempty");
    expect(back == wide, "enc_roundtrip_wide_eq");
    expect(containsCodePoint(back, 0x017C), "enc_roundtrip_z_dot");
    expect(containsCodePoint(back, 0x0142), "enc_roundtrip_l_stroke");
}

void testWin32StaticGetWindowText() {
    // Headless-friendly: create a STATIC, set Polish text, read back code points.
    const std::wstring expected =
        L"Łódź, żółć, przejście, prędkość, skrzyżowanie";

    HWND hwnd = CreateWindowExW(
        0,
        L"STATIC",
        expected.c_str(),
        WS_POPUP,
        0, 0, 320, 24,
        nullptr,
        nullptr,
        GetModuleHandleW(nullptr),
        nullptr
    );
    expect(hwnd != nullptr, "enc_win32_static_created");
    if (hwnd == nullptr) {
        return;
    }

    wchar_t buffer[256] {};
    const int got = GetWindowTextW(hwnd, buffer, static_cast<int>(sizeof(buffer) / sizeof(buffer[0])));
    expect(got > 0, "enc_win32_gettext_len");
    const std::wstring actual(buffer);
    expect(actual == expected, "enc_win32_gettext_eq");
    expect(containsCodePoint(actual, 0x0141), "enc_win32_L_stroke");
    expect(containsCodePoint(actual, 0x017C), "enc_win32_z_dot");
    expect(containsCodePoint(actual, 0x0107), "enc_win32_c_acute");
    expect(containsCodePoint(actual, 0x0119), "enc_win32_e_ogonek");

    DestroyWindow(hwnd);
}

} // namespace

int runEncodingTests() {
    g_failures = 0;
    g_passed = 0;
    cout << "\n--- utf-8 / polish encoding ---\n";
    testWideLiteralCodePoints();
    testPolishPhraseLiterals();
    testUtf8RoundTrip();
    testWin32StaticGetWindowText();
    return g_failures;
}

int encodingPassedCount() {
    return g_passed;
}
