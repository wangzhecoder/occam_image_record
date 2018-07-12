/*
  Copyright 2011 - 2017 Occam Robotics Inc - All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.
  * Neither the name of Occam Vision Group, Occam Robotics Inc, nor the
  names of its contributors may be used to endorse or promote products
  derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL OCCAM ROBOTICS INC BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

// Opens the first device available and writes each frame's OCCAM_RAW_IMAGE* to bmp files
// in the directory, along with a file device_info that has various device parameters.
// After running this program to capture a sequence of unprocessed frames, run process_raw
// generate color-processed and stitched images.

#include "indigo.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <chrono>
#include "mUtility.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

timer t;
static int fps = 0;
bool downsample = true;

static void reportError(int error_code) {
  char str[1024] = {0};
  occamGetErrorString((OccamError)error_code, str, sizeof(str));
  std::cerr<<"Occam API Error: "<<str<<" ("<<error_code<<")"<<std::endl;
  abort();
}

#define OCCAM_CHECK(call) {int r = (call);if (r != OCCAM_API_SUCCESS) reportError(r);}

// Write device parameters to a file named "device_info" in the current directory
// This file is read by process_raw to perform color processing and stitching.
bool saveDeviceInfo(OccamDevice* device) {
  std::ofstream fout("device_info");
  if (!fout.is_open()) {
    std::cerr<<"failed to open device_info file"<<std::endl;
    return false;
  }

  char* model = 0;
  char* serial = 0;
  int is_color = 0;
  int sensor_count = 0;
  int sensor_width = 0;
  int sensor_height = 0;
  int image_processing_enabled = 0;
  int brightness = 0;
  int gamma = 0;
  int black_level = 0;
  int white_balance_red = 0;
  int white_balance_green = 0;
  int white_balance_blue = 0;
  int stitching_radius = 0;
  int stitching_rotation = 0;
  int stitching_scalewidth = 0;
  int stitching_crop = 0;
  OCCAM_CHECK(occamGetDeviceValues(device, OCCAM_MODEL, &model));
  OCCAM_CHECK(occamGetDeviceValues(device, OCCAM_SERIAL, &serial));
  OCCAM_CHECK(occamGetDeviceValuei(device, OCCAM_COLOR, &is_color));
  OCCAM_CHECK(occamGetDeviceValuei(device, OCCAM_SENSOR_COUNT, &sensor_count));
  OCCAM_CHECK(occamGetDeviceValuei(device, OCCAM_SENSOR_WIDTH, &sensor_width));
  OCCAM_CHECK(occamGetDeviceValuei(device, OCCAM_SENSOR_HEIGHT, &sensor_height));
  OCCAM_CHECK(occamGetDeviceValuei(device, OCCAM_IMAGE_PROCESSING_ENABLED, &image_processing_enabled));
  OCCAM_CHECK(occamGetDeviceValuei(device, OCCAM_BRIGHTNESS, &brightness));
  OCCAM_CHECK(occamGetDeviceValuei(device, OCCAM_GAMMA, &gamma));
  OCCAM_CHECK(occamGetDeviceValuei(device, OCCAM_BLACK_LEVEL, &black_level));
  if (is_color) {
    OCCAM_CHECK(occamGetDeviceValuei(device, OCCAM_WHITE_BALANCE_RED, &white_balance_red));
    OCCAM_CHECK(occamGetDeviceValuei(device, OCCAM_WHITE_BALANCE_GREEN, &white_balance_green));
    OCCAM_CHECK(occamGetDeviceValuei(device, OCCAM_WHITE_BALANCE_BLUE, &white_balance_blue));
  }
  OCCAM_CHECK(occamGetDeviceValuei(device, OCCAM_STITCHING_RADIUS, &stitching_radius));
  OCCAM_CHECK(occamGetDeviceValuei(device, OCCAM_STITCHING_ROTATION, &stitching_rotation));
  OCCAM_CHECK(occamGetDeviceValuei(device, OCCAM_STITCHING_SCALEWIDTH, &stitching_scalewidth));
  OCCAM_CHECK(occamGetDeviceValuei(device, OCCAM_STITCHING_CROP, &stitching_crop));

  fout<<model<<" "<<serial<<std::endl;
  fout<<is_color<<" ";
  fout<<sensor_count<<" ";
  fout<<sensor_width<<" ";
  fout<<sensor_height<<" ";
  fout<<image_processing_enabled<<" ";
  fout<<brightness<<" ";
  fout<<gamma<<" ";
  fout<<black_level<<" ";
  fout<<white_balance_red<<" ";
  fout<<white_balance_green<<" ";
  fout<<white_balance_blue<<" ";
  fout<<stitching_radius<<" ";
  fout<<stitching_rotation<<" ";
  fout<<stitching_scalewidth<<" ";
  fout<<stitching_crop;
  fout<<std::endl;
  fout<<std::endl;

  occamFree(model);
  occamFree(serial);

  for (int j=0;j<sensor_count;++j) {
    double D[5], K[9], R[9], T[3];

    OCCAM_CHECK(occamGetDeviceValuerv(device, OccamParam(OCCAM_SENSOR_DISTORTION_COEFS0+j), D, 5));
    OCCAM_CHECK(occamGetDeviceValuerv(device, OccamParam(OCCAM_SENSOR_INTRINSICS0+j), K, 9));
    OCCAM_CHECK(occamGetDeviceValuerv(device, OccamParam(OCCAM_SENSOR_ROTATION0+j), R, 9));
    OCCAM_CHECK(occamGetDeviceValuerv(device, OccamParam(OCCAM_SENSOR_TRANSLATION0+j), T, 3));

    for (int k=0;k<5;++k)
      fout<<(k?" ":"")<<D[k];
    fout<<std::endl;
    for (int k=0;k<9;++k)
      fout<<(k?" ":"")<<K[k];
    fout<<std::endl;
    for (int k=0;k<9;++k)
      fout<<(k?" ":"")<<R[k];
    fout<<std::endl;
    for (int k=0;k<3;++k)
      fout<<(k?" ":"")<<T[k];
    fout<<std::endl;
    fout<<std::endl;
  }

  return true;
}

time_t getCurrentTimeStamps()
{
    std::chrono::time_point<std::chrono::system_clock,std::chrono::milliseconds> tp = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());  
    auto tmp=std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch());  
    std::time_t timestamp = tmp.count();   
    return timestamp;  
}

void outputCameraInfo(){
    if (t.toc() > 1000)
    {
        std::cout << "current fps: " << fps << std::endl;
        t.tic();
        fps = 0;
    }
    ++fps;
}

void showOminiImage(OccamImage* image)
{
    cv::Mat img;
    if (image && image->format == OCCAM_GRAY8) {
      img = cv::Mat_<uchar>(image->height,image->width,(uchar*)image->data[0],image->step[0]);
    } else if (image && image->format == OCCAM_RGB24) {
      img = cv::Mat_<cv::Vec3b>(image->height,image->width,(cv::Vec3b*)image->data[0],image->step[0]);
      cv::Mat img1;
      cv::cvtColor(img, img1, cv::COLOR_BGR2RGB);
      img = img1;
    } else if (image && image->format == OCCAM_SHORT1) {
      img = cv::Mat_<short>(image->height,image->width,(short*)image->data[0],image->step[0]);
    } else {
      //      printf("image format not supported by this demo\n");
    }

    
    int small_width = 1900;
    if (image && downsample && img.cols>small_width) {
      int small_height = image->height * small_width / image->width;
      cv::Mat small_img;
      cv::resize(img, small_img, cv::Size(small_width,small_height));
      img = small_img;
    }

    if (!img.empty())
      imshow("indigosdk", img);

    cv::waitKey(16);
}

int main(int argc, const char** argv) {
  int r;
  int i;
  int dev_index = argc>=2?atoi(argv[1]):0;
  OccamDeviceList* device_list;
  OccamDevice* device;

  OCCAM_CHECK(occamInitialize());

  OCCAM_CHECK(occamEnumerateDeviceList(2000, &device_list));
  std::cerr<<device_list->entry_count<<" devices found"<<std::endl;
  for (i=0;i<device_list->entry_count;++i) {
    std::cerr<<"device["<<i<<"]: cid = "<<device_list->entries[i].cid<<std::endl;
  }
  if (dev_index<0 || dev_index >= device_list->entry_count) {
    std::cerr<<"device index "<<dev_index<<" out of range"<<std::endl;
    return 1;
  }

  OCCAM_CHECK(occamOpenDevice(device_list->entries[dev_index].cid, &device));

  if (!saveDeviceInfo(device))
    return 1;

  int sensor_count = 0;
  OCCAM_CHECK(occamGetDeviceValuei(device, OCCAM_SENSOR_COUNT, &sensor_count));

  std::vector<OccamDataName> req(sensor_count);
  for (int j=0;j<sensor_count;++j)
    req[j] = OccamDataName(OCCAM_RAW_IMAGE0+j);
  std::vector<OccamImage*> images(sensor_count);

  int frame_count = 0;
  for (i=0;;++i) {
    if ((r = occamDeviceReadData(device,sensor_count,&req[0],0,(void**)&images[0], 1)) != OCCAM_API_SUCCESS) {
      if (r != OCCAM_API_UNSUPPORTED_DATA)
	reportError(r);
      continue;
    }
    outputCameraInfo();

    cv::Mat outImg(images[0]->height,images[0]->width*5,CV_8UC1);
    // cv::Mat dstroi;
    
    // showOminiImage(images[0]);

    ++frame_count;
    std::cerr<<"Captured frame: "<<frame_count<<std::endl;

    std::time_t timeStamp = getCurrentTimeStamps();

    for (int image_index=0;image_index<images.size();++image_index) {
      OccamImage* image = images[image_index];

      cv::Mat img;
      img = cv::Mat_<uchar>(image->height,image->width,(uchar*)image->data[0],image->step[0]);

      cv::Rect r(0, 0, image->width, image->height);  // 指定src 的 ROI子图像区域
      cv::Mat dstroi = outImg(cv::Rect(image->width*image_index,0,image->width,image->height)); // 拿到 dst指定区域子图像的引用
      img(r).convertTo(dstroi,dstroi.type(), 1,0);
      // cvCopy(img,outImg.rowRange(0,image->height).colRange(image->width*image_index,image->width*image_index+image->width));

      std::stringstream sout;
      // sout<<"Frame-"<<std::setfill('0')<<std::setw(6)<<frame_count<<"-RAW_IMAGE"<<image_index<<".bmp";
      sout<<argv[1]<<image_index<<"/"<<timeStamp<<".jpg";
      // std::cerr<<"Writing "<<sout.str()<<std::endl;
      cv::imwrite(sout.str(), img);

      occamFreeImage(image);
    }
    cv::Mat small_img;
    int small_height = outImg.rows * 1900 / outImg.cols;
    cv::resize(outImg, small_img, cv::Size(1900,small_height));
    outImg = small_img;
    cv::imshow("kkkk",outImg);
    cv::waitKey(1);

  }

  occamCloseDevice(device);
  occamFreeDeviceList(device_list);
  occamShutdown();

  return 0;
}
