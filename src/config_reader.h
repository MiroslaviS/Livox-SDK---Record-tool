
#ifndef CONFIG_READER_H
#define CONFIG_READER_H

#include <stdlib.h>
#include <string>
#include <map>
#include "lds_lidar.h"


class ConfigReader{ 
    public:
        ConfigReader(std::string config_path);
        int getLidarConfig(std::string configName);
        std::pair < std::string, int > getWriterConfig();
        bool configExists();
        std::map <std::string, int> configs;
    private:
        void readConfigs();
        std::string configPath; 

        std::map < std::string, CoordinateType> coordConf = {
            {"Cartezian", CoordinateType::kCoordinateCartesian},
            {"Spherical", CoordinateType::kCoordinateSpherical}
        };

        std::map < int, ImuFreq> IMUconf = {
            {0, ImuFreq::kImuFreq0Hz},
            {200, ImuFreq::kImuFreq200Hz}
        };

        std::map < std::string, PointCloudReturnMode > returnConf  = {
            {"strongest", PointCloudReturnMode::kStrongestReturn},
            {"first", PointCloudReturnMode::kFirstReturn},
            {"dual", PointCloudReturnMode::kDualReturn},
            {"triple", PointCloudReturnMode::kTripleReturn}
        };
        
        std::pair < std::string, int > writerConfig;

        bool fileFound = true;
};

#endif