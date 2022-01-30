#ifndef  YUTILS_FILENAMEUTILS
#define  YUTILS_FILENAMEUTILS 1
#include <vector>
#include <string>

namespace yUtils {

/** Gets the parent path from full name (Ex: /a/b/c.Ext => /a/b). */
std::string getFilePath(const std::string& filename);
/** Gets the extension without dot (Ex: /a/b/c.Ext => Ext). */
std::string getFileExtension(const std::string& filename);
/** Gets the extension including dot (Ex: /a/b/c.Ext => .Ext). */
std::string getFileExtensionIncludingDot(const std::string& filename);
/** Gets the lowercase extension without dot (Ex: /a/b/c.Ext => ext). */
std::string getLowerCaseFileExtension(const std::string& filename);
/** Gets file name with extension (Ex: /a/b/c.Ext => c.Ext). */
std::string getSimpleFileName(const std::string& fileName);
/** Gets file path without last extension (Ex: /a/b/c.Ext => /a/b/c ; file.ext1.ext2 => file.ext1). */
std::string getNameLessExtension(const std::string& fileName);
/** Gets file path without \b all extensions (Ex: /a/b/c.Ext => /a/b/c ; file.ext1.ext2 => file). */
std::string getNameLessAllExtensions(const std::string& fileName);
/** Gets file name without last extension (Ex: /a/b/c.Ext => c ; file.ext1.ext2 => file.ext1). */
std::string getStrippedName(const std::string& fileName);
/** If 'to' is in a subdirectory of 'from' then this function returns the subpath, otherwise it just returns the file name.
  * The function does \b not automagically resolve paths as the system does, so be careful to give canonical paths.
  * However, the function interprets slashes ('/') and backslashes ('\') as they were equal.
  */
std::string getPathRelative(const std::string& from, const std::string& to);
/** Gets root part of a path ("/" or "C:"), or an empty string if none found. */
std::string getPathRoot(const std::string& path);
/** Tests if path is absolute, as !getPathRoot(path).empty(). */
bool isAbsolutePath(const std::string& path);


/** Converts forward slashes (/) to back slashes (\). */
std::string convertFileNameToWindowsStyle(const std::string& fileName);
/** Converts back slashes (\) to forward slashes (/). */
std::string convertFileNameToUnixStyle(const std::string& fileName);
std::string convertToLowerCase(const std::string& fileName);

const char UNIX_PATH_SEPARATOR = '/';
const char WINDOWS_PATH_SEPARATOR = '\\';

/** Get the path separator for the current platform. */
char getNativePathSeparator();
/** Check if the path contains only the current platform's path separators. */
bool isFileNameNativeStyle(const std::string& fileName);
/** Convert the path to contain only the current platform's path separators. */
std::string convertFileNameToNativeStyle(const std::string& fileName);

bool equalCaseInsensitive(const std::string& lhs,const std::string& rhs);
bool equalCaseInsensitive(const std::string& lhs,const char* rhs);

/** Concatenates two paths */
std::string concatPaths(const std::string& left, const std::string& right);

/** Splits a path into elements between separators (including Windows' root, if any). */
void getPathElements(const std::string& path, std::vector<std::string> & out_elements);

void stringcopy(char* dest, const char* src, size_t length);

#define stringcopyfixedsize(DEST, SRC) stringcopy(DEST, SRC, sizeof(DEST));

}

#endif
