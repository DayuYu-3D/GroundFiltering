#include "triangle.h"

Triangle::Triangle()
{
    this->_vertexIndexs[0] = this->_vertexIndexs[1] = this->_vertexIndexs[2] = -1;
    this->_normal[0] = 0.0f;
    this->_normal[1] = 0.0f;
    this->_normal[2] = 0.0f;
}

Triangle::~Triangle()
{

}
