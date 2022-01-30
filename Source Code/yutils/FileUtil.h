#ifndef FILEUTIL_H
#define FILEUTIL_H

#include <string>
#include <vector>

namespace yUtils
{
    /**
     * @brief 快速计算大文件行数
     * @param strP: 文件路径
     * @return 行数
     */
    size_t getFileLinesNum(std::string strP);

    /**
     * @brief 获取某目录下子文件数量,不包含文件夹和下级文件夹的子文件
     * @param strP: 文件路径
     * @return 文件数量
     */
    std::size_t getNumofFilesInDirectory(std::string strP);

    /**
     * @brief 获取某目录下子文件路径,注意包括子文件夹(但是不递归子文件夹)
     * @param strP: 文件路径
     * @return 子文件和子文件夹列表
     */
    std::vector<std::string> getFileListInDirectory(std::string strP);

    /**
     * @brief 获取某目录下子文件路径,注意包括子文件夹(递归子文件夹)
     * @param strP: 文件路径
     * @return 子文件、子文件夹、子文件夹下的子文件和子文件夹
     */
     std::vector<std::string> getFileListInDirectoryRecursive(std::string strP);

     /**
      * @brief 用于路径或者字符串的自然排序
      * @param vPath: 需要修改的字符串列表
      */
     void sortFileList(std::vector<std::string>& vPath);
}

#endif // FILEUTIL_H
