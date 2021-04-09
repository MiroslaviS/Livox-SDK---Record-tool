cmake_minimum_required(VERSION 3.0)
project(LidarWriter)

set(NAME lidar_recorder)
set(CMAKE_CXX_FLAGS "-pthread")
set(SDK $ENV{LIVOX_SDK})

set(SOURCES_H main.cpp config_reader.h data_writer.h lds_lidar.h)
set(SOURCES_CPP config_reader.cpp data_writer.cpp lds_lidar.cpp)


add_subdirectory(${SDK} build)
include_directories(${SDK}/include/)
add_executable(${NAME} main.cpp ${SOURCES_CPP} ${SOURCES_H})

target_link_libraries(${NAME} ${PROJECT_NAME}_static)
