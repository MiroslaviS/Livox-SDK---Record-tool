
#include "config_reader.h"
#include <iostream>
#include <fstream>
#include <regex>
#include "lds_lidar.h"

using namespace std;

std::string ltrim(const std::string& s) {
    return std::regex_replace(s, std::regex("^\\s+"), std::string(""));
}
 
std::string rtrim(const std::string& s) {
    return std::regex_replace(s, std::regex("\\s+$"), std::string(""));
}

 
std::string trim(const std::string& s) {
    return ltrim(rtrim(s));
}

ConfigReader::ConfigReader(string config_path){
    this->configPath = config_path;
    this->readConfigs();
}

bool ConfigReader::configExists(){
    return this->fileFound;
}

void ConfigReader::readConfigs(){
    string line;
    string delimiter = ":";
    string config;
    string value;
    int delimiterPosition = 0;
    bool splitting = true;

    ifstream configFile(this->configPath);
    
    if ( !configFile.good() ){
        this->fileFound = false;
        return;
    }

    while ( getline(configFile, line) ){
        if (line.length() == 0 ){
            // Empty Line
            continue;
        }

        delimiterPosition = line.find(delimiter);
        config = line.substr(0, delimiterPosition);
        value = line.substr(delimiterPosition+1, line.length() - 1);

        if ( config == "IMUrefreshRate" ) {
            int rate = this->IMUconf[stoi(trim(value))];
            this->configs.insert( std::pair<string, int> (config, rate));
        }

        if ( config == "enableFan" ){
            int enabled = stoi(trim(value));

            this->configs.insert( std::pair<string, int>(config, enabled) );
        }

        if ( config == "coordinatesType" ){
            int type = this->coordConf[trim(value)];

            this->configs.insert( std::pair<string, int>(config, type));
        }

        if ( config == "returnType" ){
            int type = this->returnConf[value];

            this->configs.insert( std::pair<string, int>(config, type));
        }

        if ( config == "dataDirectory" ){
            this->writerConfig.first = trim(value);
        }

        if ( config == "splitData" ){
            splitting = stoi(trim(value));

            if ( !splitting ){
                this->writerConfig.second = 0;
            }
        } 

        if ( config == "splitValue" ){
            if ( splitting ){
                this->writerConfig.second = stoi(trim(value));
            }
        }
    }


    for (auto const& x: this->configs){
        cout << x.first << ":" << x.second << "\n";
    }
}

std::pair < std::string, int > ConfigReader::getWriterConfig(){
    return this->writerConfig;
}

int ConfigReader::getLidarConfig(string configKey){
    return this->configs[configKey];
}