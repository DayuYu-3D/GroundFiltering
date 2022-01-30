#include <stdlib.h>
#include <string.h>
#include <limits.h>

#include "ConvertUTF.h"
#include "FileNameUtils.h"

#ifdef _WIN32
    #include <windows.h>
#endif

#if defined(__sgi)
    #include <ctype.h>
#elif defined(__GNUC__) || !defined(_WIN32) || defined(__MWERKS__)
    #include <cctype>
    using std::tolower;
#endif

#if defined(__GNU__) && !defined(PATH_MAX)
#define PATH_MAX 4096
#endif

using namespace std;

static const char * const PATH_SEPARATORS = "/\\";


std::string yUtils::getFilePath(const std::string& fileName)
{
    std::string::size_type slash = fileName.find_last_of(PATH_SEPARATORS);
    if (slash==std::string::npos) return std::string();
    else return std::string(fileName, 0, slash);
}


std::string yUtils::getSimpleFileName(const std::string& fileName)
{
    std::string::size_type slash = fileName.find_last_of(PATH_SEPARATORS);
    if (slash==std::string::npos) return fileName;
    else return std::string(fileName.begin()+slash+1,fileName.end());
}


std::string yUtils::getFileExtension(const std::string& fileName)
{
    std::string::size_type dot = fileName.find_last_of('.');
    std::string::size_type slash = fileName.find_last_of(PATH_SEPARATORS);
    if (dot==std::string::npos || (slash!=std::string::npos && dot<slash)) return std::string("");
    return std::string(fileName.begin()+dot+1,fileName.end());
}

std::string yUtils::getFileExtensionIncludingDot(const std::string& fileName)
{
    std::string::size_type dot = fileName.find_last_of('.');
    std::string::size_type slash = fileName.find_last_of(PATH_SEPARATORS);
    if (dot==std::string::npos || (slash!=std::string::npos && dot<slash)) return std::string("");
    return std::string(fileName.begin()+dot,fileName.end());
}

std::string yUtils::convertFileNameToWindowsStyle(const std::string& fileName)
{
    std::string new_fileName(fileName);

    std::string::size_type slash = 0;
    while( (slash=new_fileName.find_first_of(UNIX_PATH_SEPARATOR,slash)) != std::string::npos)
    {
        new_fileName[slash]=WINDOWS_PATH_SEPARATOR;
    }
    return new_fileName;
}

std::string yUtils::convertFileNameToUnixStyle(const std::string& fileName)
{
    std::string new_fileName(fileName);

    std::string::size_type slash = 0;
    while( (slash=new_fileName.find_first_of(WINDOWS_PATH_SEPARATOR,slash)) != std::string::npos)
    {
        new_fileName[slash]=UNIX_PATH_SEPARATOR;
    }

    return new_fileName;
}

char yUtils::getNativePathSeparator()
{
#if defined(_WIN32) && !defined(__CYGWIN__)
    return WINDOWS_PATH_SEPARATOR;
#else
    return UNIX_PATH_SEPARATOR;
#endif
}

bool yUtils::isFileNameNativeStyle(const std::string& fileName)
{
#if defined(_WIN32) && !defined(__CYGWIN__)
    return fileName.find(UNIX_PATH_SEPARATOR) == std::string::npos; // return true if no unix style slash exist
#else
    return fileName.find(WINDOWS_PATH_SEPARATOR) == std::string::npos; // return true if no windows style backslash exist
#endif
}

std::string yUtils::convertFileNameToNativeStyle(const std::string& fileName)
{
#if defined(_WIN32) && !defined(__CYGWIN__)
    return convertFileNameToWindowsStyle(fileName);
#else
    return convertFileNameToUnixStyle(fileName);
#endif
}



std::string yUtils::getLowerCaseFileExtension(const std::string& filename)
{
    return convertToLowerCase(yUtils::getFileExtension(filename));
}

std::string yUtils::convertToLowerCase(const std::string& str)
{
    std::string lowcase_str(str);
    for(std::string::iterator itr=lowcase_str.begin();
        itr!=lowcase_str.end();
        ++itr)
    {
        *itr = tolower(*itr);
    }
    return lowcase_str;
}

// strip one level of extension from the filename.
std::string yUtils::getNameLessExtension(const std::string& fileName)
{
    std::string::size_type dot = fileName.find_last_of('.');
    std::string::size_type slash = fileName.find_last_of(PATH_SEPARATORS);        // Finds forward slash *or* back slash
    if (dot==std::string::npos || (slash!=std::string::npos && dot<slash)) return fileName;
    return std::string(fileName.begin(),fileName.begin()+dot);
}


// strip all extensions from the filename.
std::string yUtils::getNameLessAllExtensions(const std::string& fileName)
{
    // Finds start search position: from last slash, or the beginning of the string if none found
    std::string::size_type startPos = fileName.find_last_of(PATH_SEPARATORS);            // Finds forward slash *or* back slash
    if (startPos == std::string::npos) startPos = 0;
    std::string::size_type dot = fileName.find_first_of('.', startPos);        // Finds *FIRST* dot from start pos
    if (dot==std::string::npos) return fileName;
    return std::string(fileName.begin(),fileName.begin()+dot);
}

std::string yUtils::getStrippedName(const std::string& fileName)
{
    std::string simpleName = getSimpleFileName(fileName);
    return getNameLessExtension( simpleName );
}


bool yUtils::equalCaseInsensitive(const std::string& lhs,const std::string& rhs)
{
    if (lhs.size()!=rhs.size()) return false;
    std::string::const_iterator litr = lhs.begin();
    std::string::const_iterator ritr = rhs.begin();
    while (litr!=lhs.end())
    {
        if (tolower(*litr)!=tolower(*ritr)) return false;
        ++litr;
        ++ritr;
    }
    return true;
}

bool yUtils::equalCaseInsensitive(const std::string& lhs,const char* rhs)
{
    if (rhs==NULL || lhs.size()!=strlen(rhs)) return false;
    std::string::const_iterator litr = lhs.begin();
    const char* cptr = rhs;
    while (litr!=lhs.end())
    {
        if (tolower(*litr)!=tolower(*cptr)) return false;
        ++litr;
        ++cptr;
    }
    return true;
}

std::string yUtils::concatPaths(const std::string& left, const std::string& right)
{
#if defined(_WIN32) && !defined(__CYGWIN__)
    const char delimiterNative  = WINDOWS_PATH_SEPARATOR;
    const char delimiterForeign = UNIX_PATH_SEPARATOR;
#else
    const char delimiterNative  = UNIX_PATH_SEPARATOR;
    const char delimiterForeign = WINDOWS_PATH_SEPARATOR;
#endif

    if(left.empty())
    {
        return(right);
    }
    char lastChar = left[left.size() - 1];

    if(lastChar == delimiterNative)
    {
        return left + right;
    }
    else if(lastChar == delimiterForeign)
    {
        return left.substr(0, left.size() - 1) + delimiterNative + right;
    }
    else // lastChar != a delimiter
    {
        return left + delimiterNative + right;
    }
}

