#include "FileUtil.h"

#include <fstream>
#include <sstream>
#include <iterator>
#include <algorithm>
#include <filesystem>

namespace yUtils {
/**
 * @brief 用于字符串自然排序的对比函数
 */
bool compareNat(const std::string &a, const std::string &b)
{
    if (a.empty()) {
        return true;
    }
    if (b.empty()) {
        return false;
    }
    if (std::isdigit(a[0]) && !std::isdigit(b[0])) {
        return true;
    }
    if (!std::isdigit(a[0]) && std::isdigit(b[0])) {
        return false;
    }
    if (!std::isdigit(a[0]) && !std::isdigit(b[0]))
    {
        if (std::toupper(a[0]) == std::toupper(b[0])) {
            return compareNat(a.substr(1), b.substr(1));
        }
        return (std::toupper(a[0]) < std::toupper(b[0]));
    }

    // Both strings begin with digit --> parse both numbers
    std::istringstream issa(a);
    std::istringstream issb(b);
    int ia, ib;
    issa >> ia;
    issb >> ib;
    if (ia != ib) {
        return ia < ib;
    }

    // Numbers are the same --> remove numbers and recurse
    std::string anew, bnew;
    std::getline(issa, anew);
    std::getline(issb, bnew);
    return (compareNat(anew, bnew));
}
}

size_t yUtils::getFileLinesNum(std::string strP)
{
    struct Predicate
    {
        bool operator()(char c)
        {
            last    = c;
            return last == '\n';
        }
        char    last;
    };

    std::fstream file(strP);
    Predicate pred;
    std::size_t   count   = std::count_if(std::istreambuf_iterator<char>(file),
                                          std::istreambuf_iterator<char>(),
                                          pred);
//    if (pred.last != '\n')  // If the last character checked is not '\n'
//    {
//        ++count;
//    }
    return count + 1;
}

size_t yUtils::getNumofFilesInDirectory(std::string strP)
{
    using std::filesystem::directory_iterator;
    using fp = bool (*)( const std::filesystem::path &);
    return std::count_if(directory_iterator(strP), directory_iterator{}, (fp)std::filesystem::is_regular_file);
}

std::vector<std::string> yUtils::getFileListInDirectory(std::string strP)
{
    std::vector<std::string> fileL;
    for (const auto &file : std::filesystem::directory_iterator(strP))
    {
        fileL.push_back(file.path().string());
    }
    return fileL;
}

std::vector<std::string> yUtils::getFileListInDirectoryRecursive(std::string strP)
{
    std::vector<std::string> fileL;
    for (const auto &file : std::filesystem::recursive_directory_iterator(strP))
    {
        fileL.push_back(file.path().string());
    }
    return fileL;
}

void yUtils::sortFileList(std::vector<std::string> &vPath)
{
    std::sort(vPath.begin(), vPath.end(), compareNat);
}
