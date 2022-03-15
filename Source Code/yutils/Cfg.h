
#ifndef CFG_H
#define CFG_H

#include <iostream>
#include <string>
#include <fstream>

class Cfg
{
public:
    bool readConfigFile(const char * cfgfilepath, const std::string & key, std::string & value)
    {
        std::fstream cfgFile;
        cfgFile.open(cfgfilepath);
        if (!cfgFile.is_open())
        {
            std::cout << "can not open cfg file!" << std::endl;
            return false;
        }
        char tmp[1000];
        while (!cfgFile.eof())
        {
            cfgFile.getline(tmp, 1000);
            std::string line(tmp);
            std::size_t pos = line.find('=');
            if (pos == std::string::npos) return false;
            std::string tmpKey = line.substr(0, pos);
            if (key == tmpKey)
            {
                value = line.substr(pos + 1);
                return true;
            }
        }
        return false;
    }

    bool readBoolByString(std::string str)
    {
        if (str == "true" || str == "True") {
            return true;
        }
        else if (str == "false" || str == "False") {
            return false;
        }
        else if (atoi(str.c_str()) == 0) {
            return false;
        }
        else {
            return true;
        }
    }
};
#endif // CFG_H
