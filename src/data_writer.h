
#ifndef DATA_WRITER_H
#define DATA_WRITER_H

#include <vector>
#include <mutex>
#include <thread>
#include <chrono>
#include <ctime>
#include "lds_lidar.h"


/** IMU data format. */
typedef struct {
  float gyro_x;        /**< Gyroscope X axis, Unit:rad/s */
  float gyro_y;        /**< Gyroscope Y axis, Unit:rad/s */
  float gyro_z;        /**< Gyroscope Z axis, Unit:rad/s */
  float acc_x;         /**< Accelerometer X axis, Unit:g */
  float acc_y;         /**< Accelerometer Y axis, Unit:g */
  float acc_z;         /**< Accelerometer Z axis, Unit:g */
  uint64_t timestamp;  /** Current timestamp of point from LiDAR */
} CustomLivoxImuPoint;

/** Custom extend cartesian coordinate format with timestamp **/
typedef struct {
  int32_t x;            /**< X axis, Unit:mm */
  int32_t y;            /**< Y axis, Unit:mm */
  int32_t z;            /**< Z axis, Unit:mm */
  uint8_t reflectivity; /**< Reflectivity */
  uint8_t tag;          /**< Tag */
  uint64_t timestamp;   /** Current timestamp of point from LiDAR */
} CustomLivoxExtendRawPoint;

typedef std::chrono::system_clock Clock;

#define WRITE_BUFFER_SIZE 5000
#define WRITE_IMU_BUFFER_SIZE 1000
#define POINTS_PACKET_NUMBER 96

class DataWriter {
    public:
        DataWriter(std::mutex *fileMutex, std::mutex *bufferMutex, std::string root);
        ~DataWriter();
        static void addPoint(LivoxExtendRawPoint *point, DataWriter *writer, char *broadcasId, uint64_t cur_timestamp);
        static void addIMUPoint(LivoxImuPoint *point, DataWriter *writer, char *broadcasId, uint64_t cur_timestamp);

        static std::string getFilename();
        char* lidarBroadcastId;
        static int createFolder(char *broadcastId, std::string root);
        static int createIMUFolder(char *broadcastId, std::string root);
        unsigned int queue_counter = 0;
        unsigned int recieved_points = 0;
        unsigned int stored_points = 0;

    private:
        std::vector<CustomLivoxExtendRawPoint *> pointBuffer;
        std::vector<CustomLivoxImuPoint *> imuBuffer;
        static void storeBuffer(DataWriter *this_writer, std::vector<CustomLivoxExtendRawPoint *> buffer, char *lidarBroadcastId, std::mutex *fileMutex, std::string root);
        static void storeIMUBuffer(DataWriter *this_writer, std::vector<CustomLivoxImuPoint *> buffer, char *lidarBroadcastId, std::mutex *fileMutex, std::string root);
        std::mutex *fileMutex;
        std::mutex *bufferMutex;
        std::string root;
        std::vector<std::thread*> threadVectors;
        std::string filename;
};

#endif