#include "trianglereader.h"

#include <filesystem>
#include <iostream>
#include <fstream>

#include <QFile>
#include <QTextStream>

#include "yutils/FileNameUtils.h"
#include "yutils/FileUtil.h"

namespace  {
    bool isNumber(const std::string& str)
    {
        for (char const &c : str) {
            if (std::isdigit(c) == 0) return false;
        }
        return true;
    }
}

TriangleReader::TriangleReader(std::string triFP)
    : _triFP(triFP),
      _b1PointFile(false)
{
}

bool TriangleReader::readTriangleData(std::vector<Triangle> &vTri,
                                     std::map<size_t, std::vector<Vertex>>& mvVertex)
{
    if(!this->fileValidate())
    {
        std::cout << "Check file integrity failed" << std::endl;
        return false;
    }

    //计算共有多少个点和triangle,便于开辟内存
    std::cout << "calculate the numbers of triangles and vertex..." << std::endl;
    time_t tStart = std::time(0);
    this->allocateMemory(vTri);
    std::cout << "Use time:" << std::time(0) - tStart << "s" << std::endl;

    std::cout << "Read triangls..." << std::endl;
    tStart = time(0);
    int numP = 0;
    //读取三角面片
    QFile trianglesF(_triFP.c_str());
    if (trianglesF.open(QIODevice::ReadOnly | QIODevice::Text )) {
        uchar *fpr = trianglesF.map(0, trianglesF.size()); //加快读写速度,速度提升一倍,目前windows最好方案
        char *s = _strdup((char *)fpr);
        char *substr;
        char *token = NULL;
        char *next = NULL;
        while ((substr = strtok_s(s, "\n", &next)))
        {
            Triangle tri;
            s = next;
            for (int i = 0; i < 7; i++)
            {
                char *lineSubStr = strtok_s(substr, " ", &token);
                substr = token;
                if(i == 1) {
                    tri._vertexIndexs[0] = std::atoi(lineSubStr);
                }
                if(i == 2) {
                    tri._vertexIndexs[1] = std::atoi(lineSubStr);
                }
                if(i == 3) {
                    tri._vertexIndexs[2] = std::atoi(lineSubStr);
                }
                if(i == 4) {
                    tri._normal[0] = std::atof(lineSubStr);
                }
                if(i == 5) {
                    tri._normal[1] = std::atof(lineSubStr);
                }
                if(i == 6) {
                    tri._normal[2] = std::atof(lineSubStr);
                }
                if(i == 0) {
                    tri._geomIndex = std::atoi(lineSubStr);
                }
            }
            vTri.push_back(tri);
        }
        trianglesF.close();
    } else {
        std::cout << "Read triangle file error" << std::endl;
        return false;
    }

    //读取顶点
    std::cout << "Read vertexs... " << this->_b1PointFile<< std::endl;
    int nLine = this->_b1PointFile ? 8 : 6;
    std::string fp_dir = yUtils::getFilePath(_triFP);
    auto geomfileL = yUtils::getFileListInDirectory(fp_dir + "/points");    
    yUtils::sortFileList(geomfileL);
    for(uint i = 0; i < geomfileL.size(); ++i)
    {
        if(std::filesystem::is_directory(geomfileL[i])) {
            continue;
        }

        int geomIx = i;
        Vertex vex;
        QFile pointF(geomfileL[i].c_str());
        if (pointF.open(QIODevice::ReadOnly | QIODevice::Text)) {
            uchar *fpr = pointF.map(0, pointF.size()); //加快读写速度,速度提升一倍
            char *s = _strdup((char *)fpr);
            char *substr;
            char *token = NULL;
            char *next = NULL;
            while ((substr = strtok_s(s, "\n", &next)))
            {
                int pointIx = 0;
                s = next;
                for (int j = 0; j < nLine; j++)
                {
                    char *lineSubStr = strtok_s(substr, " ", &token);
                    substr = token;

                    if(this->_b1PointFile)
                    {
                        if(j == 0){
                            geomIx = std::atoi(lineSubStr);
                        }
                        else if(j == 1)
                            vex._index = std::atoi(lineSubStr);
                        else if(j == 2) {
                            vex._coor[0] = std::atof(lineSubStr);
                        }
                        else if(j == 3) {
                            vex._coor[1] = std::atof(lineSubStr);
                        }
                        else if(j == 4) {
                            vex._coor[2] = - std::atof(lineSubStr);//注意:这是负的,即将z轴翻转
                        }
                        else if(j == 5) {
                            vex._texCoor[0] = std::atof(lineSubStr);
                        }
                        else if(j == 6) {
                            vex._texCoor[1] = std::atof(lineSubStr);
                        }
                        else if(j == 7) {
                            pointIx = std::atoi(lineSubStr); //顶点的索引，相同位置不同索引
                        }
                    }
                    else
                    {
                        if(j == 0)
                            vex._index = std::atoi(lineSubStr);
                        else if(j == 1) {
                            vex._coor[0] = std::atof(lineSubStr);
                        }
                        else if(j == 2) {
                            vex._coor[1] = std::atof(lineSubStr);
                        }
                        else if(j == 3) {
                            vex._coor[2] = - std::atof(lineSubStr);//注意:这是负的,即将z轴翻转
                        }
                        else if(j == 4) {
                            vex._texCoor[0] = std::atof(lineSubStr);
                        }
                        else if(j == 5) {
                            vex._texCoor[1] = std::atof(lineSubStr);
                        }
                    }

                }

                if(!this->_b1PointFile)
                {
                    pointIx = vex._index;
                }
                //动态将顶点插入到指定位置的vector中
                if(pointIx >= mvVertex[geomIx].size())
                {
                    mvVertex[geomIx].resize(pointIx+1);
                }
                mvVertex[geomIx][pointIx] = vex;

//                mvVertex[geomIx].emplace_back(vex); //添加顶点
                numP += 1;//纯粹计数
            }
            pointF.close();
        }
    }
    std::cout << "Use time:" << time(0) - tStart << "s" << std::endl;
    std::cout << "Read number of trianlges:" << vTri.size() << std::endl;
    std::cout << "Read number of vertexs:" << numP << std::endl;
    std::cout << "Read number of geoms:" << mvVertex.size() << std::endl;

    return true;
}

void TriangleReader::writeTrianglesData(const std::vector<Triangle> &vTri,
                                        const std::vector<unsigned int> &vTriIndex,
                                        std::string triP) const
{
    QFile triF(triP.c_str());
    triF.open(QIODevice::WriteOnly | QIODevice::Text); //不存在会自动创建
    QTextStream smTri(&triF);

    for(auto triIx : vTriIndex)
    {
        auto triangle = vTri[triIx];
        int index_geom = triangle._geomIndex;
        //int index = triangle._index; //在三角网中的索引
        int index_vertex1 = triangle._vertexIndexs[0];
        int index_vertex2 = triangle._vertexIndexs[1];
        int index_vertex3 = triangle._vertexIndexs[2];
        double normalx = triangle._normal[0];
        double normaly = triangle._normal[1];
        double normalz = triangle._normal[2];
        smTri << index_geom << " " << index_vertex1 << " "
              << index_vertex2 << " " << index_vertex3 << " "
              << normalx << " " << normaly << " " << normalz << "\n";
    }
    triF.close();
}

void TriangleReader::setPointFileFlag(bool b1PointFiel)
{
    this->_b1PointFile = b1PointFiel;
}

bool TriangleReader::fileValidate()
{
    std::cout << "Check file integrity " << _triFP << std::endl;
    bool isVal = false;

    std::ifstream fin(_triFP);
    if(!fin) {
        std::cout << "triangles file not exist" << std::endl;
        return isVal;
    }
    std::string dir = yUtils::getFilePath(_triFP);
    std::ifstream fin2(dir + "/textures.txt");
    if(!fin2)
    {
        std::cout << "textures file not exist" << std::endl;
        return isVal;
    }
    auto dirPs = dir + "/points";
    if(!std::filesystem::exists(dirPs))
    {
        std::cout << "points dir not exist" << std::endl;
        return isVal;
    }

    return !isVal;
}

void TriangleReader::allocateMemory(std::vector<Triangle> &vTri)
{
    std::string fp_dir = yUtils::getFilePath(_triFP);

    size_t numTriangle = 0, numPoint = 0, numPointFile = 0;
    numTriangle = yUtils::getFileLinesNum(_triFP);
    numPointFile = yUtils::getNumofFilesInDirectory(fp_dir + "/points");
    auto geomfileL = yUtils::getFileListInDirectory(fp_dir + "/points");
    yUtils::sortFileList(geomfileL);
    //开辟内存
    ///所有三角面片
    vTri.reserve(numTriangle + 1);

    //判断顶点文件是否只有一个
//    if(numPointFile == 1 && isNumber(yUtils::getStrippedName(geomfileL[0])))
//    {
//        ///所有顶点,按geom1,2,3...组织
//        int nGeom = std::stoi(yUtils::getStrippedName(geomfileL.at(0)));
//        this->_b1PointFile = true;
//        vvVertex.resize(nGeom);

//    }else
//    {
//        this->_b1PointFile = false;
//        vvVertex.reserve(numPointFile + 1);
//         //开辟每个geom中顶点的内存
//        for(const std::string &geomF : geomfileL)
//        {
//            if(std::filesystem::is_directory(geomF)) {
//                continue;
//            }
//            size_t numPointofGeom = yUtils::getFileLinesNum(geomF);
//            std::vector<Vertex> pointsOfGeom;
//            pointsOfGeom.reserve(numPointofGeom + 1);
//            vvVertex.push_back(pointsOfGeom);
//            numPoint += numPointofGeom;
//        }
//        std::cout << "Allocate memory of vertexs:" << numPoint << std::endl;
//    }

    std::cout << "Allocate memory of triangles:" << numTriangle << std::endl;
}
