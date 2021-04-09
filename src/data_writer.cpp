
#include "data_writer.h"

#include <stdio.h>
#include <thread>
#include <iostream>
#include <fstream>
#include <string>
#include <mutex>
#include <string>
#include <chrono>
#include <ctime>
#include <sys/stat.h>

#include <cerrno>
#include <cstring>
#include <clocale>

DataWriter::DataWriter(std::mutex *fileMutex, std::mutex *bufferMutex, std::string root){
    this->fileMutex = fileMutex;
    this->bufferMutex = bufferMutex;
    this->root = root;
}

int DataWriter::createFolder(char *broadcastId, std::string root){
    std::string id_string(broadcastId);


    int status = mkdir(root.c_str(),  S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    if ( status != 0 && errno != 17){
        std::cout << "Couldn't create data folder: " << std::strerror(errno) << "\n";
        return status;
    }

    std::string broadcastFolder = root + id_string;
    status =  mkdir(broadcastFolder.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    if (status == 0 ){
        std::cout << "[" << id_string << "] Created new broadcastID folder\n";
    } else if ( errno == 17 ){ 
        status = 0;
    } else { 
        std::cout << "[" << broadcastId << "Couldn't create broadcast folder: " << std::strerror(errno) << "\n";
    }

    return status;
}


int DataWriter::createIMUFolder(char *broadcastId, std::string root){
    std::string id_string(broadcastId);

    int status = mkdir(root.c_str(),  S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    if ( status != 0 && errno != 17){
        std::cout << "Couldn't create data folder: " << std::strerror(errno) << "\n";
        return status;
    }

    std::string broadcastFolder = root + id_string;
    status =  mkdir(broadcastFolder.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    if (status == 0 ){
        std::cout << "[" << id_string << "] Created new broadcastID folder\n";
    } else if ( errno == 17 ){ 
        status = 0;
    } else { 
        std::cout << "[" << broadcastId << "Couldn't create broadcast folder: " << std::strerror(errno) << "\n";
    }

    std::string imuFolder = broadcastFolder + "/IMU";
    status =  mkdir(imuFolder.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    if (status == 0 ){
        std::cout << "[" << id_string << "] Created new IMU broadcastID folder\n";
    } else if ( errno == 17 ){ 
        status = 0;
    } else { 
        std::cout << "[" << broadcastId << "Couldn't create IMU broadcast folder: " << std::strerror(errno) << "\n";
    }

    return status;
}

std::string DataWriter::getFilename(){
    auto now = Clock::now();
    std::time_t now_c = Clock::to_time_t(now);
    struct tm *parts = std::localtime(&now_c);


    int year = 1900 + parts->tm_year;
    int month = 1 + parts->tm_mon;
    int mday = parts->tm_mday;

    std::string result = std::to_string(mday) + std::to_string(month) + std::to_string(year);
    
    return result;
}

void DataWriter::storeBuffer(DataWriter *this_writer,  std::vector<CustomLivoxExtendRawPoint *> buffer, char *lidarBroadcastId, std::mutex *fileMutex, std::string root){
    fileMutex->lock();

    std::string filename = DataWriter::getFilename();
    std::string broadcast_string(lidarBroadcastId);

    if ( DataWriter::createFolder(lidarBroadcastId, root) != 0 ){
        std::cout << "[" << lidarBroadcastId << "] Failed to create folder!\n";
        return;
    }

    filename = root + broadcast_string + "/" + filename; 

    std::ofstream myfile;
    std::string str_broadcastId(lidarBroadcastId);
    myfile.open(filename, std::ios::app);

    this_writer->stored_points += buffer.size();
    if ( myfile.is_open() ){
        for ( auto elem: buffer){
            myfile << std::to_string(elem->x) << "," << std::to_string(elem->y) << "," << std::to_string(elem->z) << "," << std::to_string(elem->reflectivity) << "," << std::to_string(elem->tag) << "," << std::to_string(elem->timestamp) << "\n";
            delete(elem);
        }
    } else { 
        fileMutex->unlock();
        printf("File not found\n");
        return;
    }


    printf("[%s] %d/%d/%d (stored/queued/recieved)\n", lidarBroadcastId, this_writer->stored_points,this_writer->queue_counter, this_writer->recieved_points);
    myfile.close();
    fileMutex->unlock();
}

void DataWriter::addPoint(LivoxExtendRawPoint *points_data, DataWriter *writer, char *broadcastId, uint64_t cur_timestamp){
    // Lock mutex for buffer before adding new values/deleting content on write //
    writer->bufferMutex->lock();

    for ( int i = 0; i < POINTS_PACKET_NUMBER; i++){
        CustomLivoxExtendRawPoint *point = new CustomLivoxExtendRawPoint();
        point->x = points_data[i].x;
        point->y = points_data[i].y;
        point->z = points_data[i].z;
        point->reflectivity = points_data[i].reflectivity;
        point->tag = points_data[i].tag;
        point->timestamp = cur_timestamp;


        writer->pointBuffer.push_back(point);
        writer->queue_counter += 1;
    }

    
    if ( writer->pointBuffer.size() >= WRITE_BUFFER_SIZE ){
        // Create new thread and write to file //
        std::thread *newThread = new std::thread(storeBuffer,writer,  writer->pointBuffer, broadcastId, writer->fileMutex, writer->root);
        writer->threadVectors.push_back(newThread);

        // Delete buffer for new data //
        writer->pointBuffer.clear();
    }

    // Unlock buffer for next data //
    writer->bufferMutex->unlock();
}

void DataWriter::addIMUPoint(LivoxImuPoint *x, DataWriter *writer, char *broadcastId, uint64_t cur_timestamp){
    // Lock mutex for buffer before adding new values/deleting content on write //
    CustomLivoxImuPoint *point = new CustomLivoxImuPoint();

    point->gyro_x = x->gyro_x;
    point->gyro_y = x->gyro_y;
    point->gyro_z = x->gyro_z;
    point->acc_x = x->acc_x;
    point->acc_y = x->acc_y;
    point->acc_z = x->acc_z;
    point->timestamp = cur_timestamp;

    writer->imuBuffer.push_back(point);

    if ( writer->imuBuffer.size() >= WRITE_IMU_BUFFER_SIZE ){
        // Create new thread and write to file //
        std::thread *newThread = new std::thread(storeIMUBuffer,writer,  writer->imuBuffer, broadcastId, writer->fileMutex, writer->root);
        writer->threadVectors.push_back(newThread);

        // Delete buffer for new data //
        writer->imuBuffer.clear();
    }
}

void DataWriter::storeIMUBuffer(DataWriter *this_writer, std::vector<CustomLivoxImuPoint *> buffer, char *lidarBroadcastId, std::mutex *fileMutex, std::string root){
    std::string filename = DataWriter::getFilename();
    std::string broadcast_string(lidarBroadcastId);

    if ( DataWriter::createIMUFolder(lidarBroadcastId, root) != 0 ){
        std::cout << "[" << lidarBroadcastId << "] Failed to create IMU folder!\n";
        return;
    }

    filename = root + broadcast_string + "/IMU/" + filename; 

    std::ofstream myfile;
    std::string str_broadcastId(lidarBroadcastId);
    myfile.open(filename, std::ios::app);

    if ( myfile.is_open() ){
        for ( auto elem: buffer){
            myfile << std::to_string(elem->gyro_x) << "," << std::to_string(elem->gyro_y) << "," << std::to_string(elem->gyro_z)  << "," << std::to_string(elem->acc_x) << std::to_string(elem->acc_y) << "," << std::to_string(elem->acc_z) << "," << std::to_string(elem->timestamp) << "\n";
            delete(elem);
        }
    } else { 
        fileMutex->unlock();
        printf("File not found\n");
        return;
    }


    myfile.close();
}


DataWriter::~DataWriter(){
    for ( std::thread *elem: this->threadVectors ){
        std::cout << "Joing thread element\n";
        elem->join();
    }
}