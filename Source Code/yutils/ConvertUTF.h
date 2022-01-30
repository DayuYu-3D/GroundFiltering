#ifndef YUTILS_CONVERTUTF
#define YUTILS_CONVERTUTF 1

#include <string>

#if defined(__CYGWIN__) || defined(__ANDROID__)
namespace std
{
typedef basic_string<wchar_t> wstring;
}
#endif

namespace yUtils
{

std::string convertUTF16toUTF8(const wchar_t* source, unsigned sourceLength);
std::wstring convertUTF8toUTF16(const char* source, unsigned sourceLength);

std::string convertUTF16toUTF8(const std::wstring& s);
std::string convertUTF16toUTF8(const wchar_t* s);

std::wstring convertUTF8toUTF16(const std::string& s);
std::wstring convertUTF8toUTF16(const char* s);

std::string convertStringFromCurrentCodePageToUTF8(const char* source, unsigned sourceLength);
std::string convertStringFromUTF8toCurrentCodePage(const char* source, unsigned sourceLength);

std::string convertStringFromCurrentCodePageToUTF8(const std::string& s);
std::string convertStringFromCurrentCodePageToUTF8(const char* s);

std::string convertStringFromUTF8toCurrentCodePage(const std::string& s);
std::string convertStringFromUTF8toCurrentCodePage(const char* s);

}

#endif
