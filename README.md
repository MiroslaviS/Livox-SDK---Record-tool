# Livox point cloud reader
Program for storing and recieving point cloud data from Livox LiDAR sensors using Livox unix SDK.

## Prerequisites
* Ubuntu 14.04/Ubuntu 16.04/Ubuntu 18.04
* C++11 compiler
* Livox SDK downloaded


# Compile and run 
Tool needs installed Livox unix [SDK](https://github.com/Livox-SDK/Livox-SDK) in your system. CMake needs path to your Livox SDK library provided by environment variable `LIVOX_SDK`. This can be done by following command:

```properties
export LIVOX_SDK=/path/to/Livox_SDK/sdk_core/
```


```
mkdir build && cd build
cmake ../src/ && make
```

Running this commands in project directory will create build directory, generates Makefile and then compile all source codes into `lidar_recorder` binary.

# Running the recording
`Lidar_recorder` connect with all available LiDARs connected to device. Then set specified configuration to LiDAR and starts recording data.

Tool will create data folder with folders corresponding to LiDAR broadcast IDs and then will store recieved data to files named with current date timestamp (%d%M%Y format). 

Tool will store IMU points in separated subfolders `IMU`.

## Stored data format
Point cloud data are stored as text filed. Each line represents one point cloud data.
```
x, y, z, reflectivity, tag, timestamp
```

## Stored IMU points format
IMU points are stored same as point cloud data. Text file with each line representing one IMU point record.
```
gyro x, gyro y, gyro_z, acc_x, acc_y, acc_z, timestamp
```


For this moment, ending recording is done by `ctr+c/SIGTERM` signal. 

# Configuration LiDAR file
LiDAR can be configured with `config.yaml` file which provides configuration for LiDAR before starting recording.

```
IMUrefreshRate: [200/0]
enableFan: [1/0]
coordinatesType: [Cartezian, Spherical]
dataDirectory: path/to/data/folder/
```

