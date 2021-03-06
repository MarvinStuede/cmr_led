/* *****************************************************************
Copyright (c) 2019, Institute of Mechatronic Systems, Leibniz University Hannover
All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree.
**/
/**
 * @file   img_loader.h
 * @author Marvin Stuede (Marvin.Stuede@imes.uni-hannover.de)
 * @date   07/2019
*
* @brief  Classes to load and store bmps and gifs
*/
#pragma once
#include "ros/ros.h"
#include <memory>
#include <stdint.h>
#include "cmr_led/gifdec.c"

//########## MAIN ######################################################################################################
using namespace std;
struct RGB{
  RGB(){}
  RGB(uint8_t r, uint8_t g, uint8_t b){
    this->r = r;
    this->g = g;
    this->b = b;
  }
  uint8_t r = 0;
  uint8_t g = 0;
  uint8_t b = 0;
};

class RGBArray2D {

public:
  size_t m_width, m_height;
  std::vector<RGB> m_data;

  RGBArray2D(size_t x, size_t y, RGB init = RGB()):
    m_width(x), m_height(y), m_data(x*y, init)
  {}
  RGB& operator()(size_t x, size_t y) {
    return m_data.at(x + y * m_width);
  }
  const RGB& operator()(size_t x, size_t y) const {
    return m_data.at(x + y * m_width);
  }
};
using BMP = RGBArray2D;

class RGBArray3D {

public:
  size_t m_width, m_height;
  std::vector<RGBArray2D> m_data;

  RGBArray3D(size_t x, size_t y, size_t z):
    m_width(x), m_height(y), m_data(z, RGBArray2D(x,y))
  {}
  RGB& operator()(size_t x, size_t y, size_t z) {
    return m_data.at(z)(x,y);
  }
  const RGB& operator()(size_t x, size_t y, size_t z) const {
    return m_data.at(z)(x,y);
  }
  RGBArray2D& operator()(size_t z) {
    return m_data.at(z);
  }
  const RGBArray2D& operator()(size_t z) const{
    return m_data.at(z);
  }
};

class GIF {

public:
  GIF(size_t width, size_t height, size_t frames):array(width,height,frames){}
  RGBArray3D array;
  double delay = 0.05;
  size_t getNumOfFrames(){return array.m_data.size();}
  double getLength(){return delay * getNumOfFrames();}
};



class IMGLoader{

public:
  IMGLoader(){};

  inline bool fileExists(const std::string& path) {
   // cout<<"check existence "<<path.c_str()<<endl;
    if (FILE *file = fopen(path.c_str(), "r")) {
        fclose(file);
   // cout<<"checked existence"<<endl;
        return true;
    } else {
        return false;
    }   
  }
  shared_ptr<BMP> loadBMP(const std::string& path)
  {

    const char * c = path.c_str();
    FILE* f = fopen(c, "rb");

    if(f == NULL)
        throw "Argument Exception: BMP Path not valid";


    unsigned char info[54];

    fread(info, sizeof(unsigned char), 54, f); // read the 54-byte header


    // extract image height and width from header
    int width = *(int*)&info[18];
    int height = *(int*)&info[22];

    shared_ptr<BMP> bmp_ptr(new BMP(width, height));

    int row_padded = (width*3 + 3) & (~3);
 
    unsigned char* data = new unsigned char[row_padded];
    unsigned char* data_all = new unsigned char[row_padded*32];

    for(int i = 0; i < height; i++)
    {
        fread(data, sizeof(unsigned char), row_padded, f);
        for(int j = 0; j < width*3; j += 3)
        {
	    (*bmp_ptr)(j/3,height - i - 1) = RGB(data[j+2], data[j+1], data[j]);
        }

    }

    //ROS_INFO("First R Value: %c",data[0]);
    fclose(f);
    delete data, data_all;
    return bmp_ptr;
  }
  shared_ptr<GIF> loadGIF(const std::string& path)
  {


    //Open the GIF

    const char * c = path.c_str();
    gd_GIF *gd_gif = gd_open_gif(c);

    //Allocate memory
    uint8_t *color, *frame;
    frame = (uint8_t*)malloc(gd_gif->width * gd_gif->height * 3);
    color = &gd_gif->gct.colors[gd_gif->bgindex * 3];


    //Calculate the number of frames
    size_t frame_num = 0;
    int ret;
    while(true){
      ret = gd_get_frame(gd_gif);
      if(ret == 0) break;
      if(ret == -1) break;
      frame_num++;
    }
    gd_rewind(gd_gif);

    //Create the GIF
    shared_ptr<GIF> gif_ptr(new GIF(gd_gif->width,gd_gif->height,frame_num));
    gif_ptr->delay = (double)gd_gif->gce.delay/100.;
    //Iterate through the frames
    for (size_t k = 0; k < frame_num; k++) {
      int ret = gd_get_frame(gd_gif);

      //Store the colors
      gd_render_frame(gd_gif, frame);
      color = frame;

      //Iterate over the image and save to variable
      for (size_t i = 0; i < gd_gif->height; i++) {
        for (size_t j = 0; j < gd_gif->width; j++) {
          gif_ptr->array(j,i,k) = RGB(color[0], color[1], color[2]);
          color += 3;
        }

      }
      if (ret == 0)
        gd_rewind(gd_gif);
    }

    free(frame);
    gd_close_gif(gd_gif);
    return gif_ptr;
  }
};

