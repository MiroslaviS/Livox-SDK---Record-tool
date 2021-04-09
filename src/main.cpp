//
// The MIT License (MIT)
//
// Copyright (c) 2019 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <chrono>
#include "cmdline.h"
#include "lds_lidar.h"
#include "data_writer.h"
#include "config_reader.h"
#include <signal.h>
#include <unistd.h>


/** Cmdline input broadcast code */
static std::vector<std::string> cmdline_broadcast_code;

/** Set the program options.
* You can input the registered device broadcast code and decide whether to save the log file.
*/
void SetProgramOption(int argc, const char *argv[]) {
  cmdline::parser cmd;
  cmd.add<std::string>("code", 'c', "Register device broadcast code", false);
  cmd.add("log", 'l', "Save the log file");
  cmd.add("help", 'h', "Show help");
  cmd.parse_check(argc, const_cast<char **>(argv));
  if (cmd.exist("code")) {
    std::string sn_list = cmd.get<std::string>("code");
    printf("Register broadcast code: %s\n", sn_list.c_str());
    size_t pos = 0;
    cmdline_broadcast_code.clear();
    while ((pos = sn_list.find("&")) != std::string::npos) {
      cmdline_broadcast_code.push_back(sn_list.substr(0, pos));
      sn_list.erase(0, pos + 1);
    }
    cmdline_broadcast_code.push_back(sn_list);
  }
  if (cmd.exist("log")) {
    printf("Save the log file.\n");
    SaveLoggerFile();
  }
  return;
}

std::mutex start_mutex;

int main(int argc, const char *argv[]) {
/** Set the program options. */
  SetProgramOption(argc, argv); 

  LdsLidar& read_lidar = LdsLidar::GetInstance("./config.yaml");

  int ret = read_lidar.InitLdsLidar(cmdline_broadcast_code);
  if (!ret) {
    start_mutex.lock();
    printf("Init lds lidar success!\n");
  } else {
    printf("Init lds lidar fail!\n");
  }

  printf("Start discovering device.\n");

  start_mutex.lock();

  read_lidar.DeInitLdsLidar();
}
