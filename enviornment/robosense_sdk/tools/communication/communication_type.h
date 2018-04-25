/**
 *****************************************************************************
 * COPYRIGHT STATEMENT
 * Copyright (c) 2018, Robosense Co.,Ltd. - www.robosense.ai
 * All Rights Reserved.
 *
 * You can not use, copy or spread without official authorization.
 *****************************************************************************
 *
 * Author: Robosense Perception Group
 * Version: 1.6.5
 * Date: 2018.3
 *
 * DESCRIPTION
 *
 * Robosense communication module, for output perception result to user computer.
 *
 */

#ifndef ROBOSENSE_COMMUNICATION_TYPE_H
#define ROBOSENSE_COMMUNICATION_TYPE_H

#include <stdint.h>

namespace Robosense{

struct PoseMsg
{
  float x;
  float y;
  float z;
  float roll;
  float pitch;
  float yaw;
};


struct Header{
  unsigned short int head;
  unsigned short int object_num;
  unsigned int length;
  unsigned long int frame_id;
  unsigned long int timestamp;
};


struct CommunicationMsg{
  Header header;
  PoseMsg pose;
  char perceptions[20000];
};

struct Point3fMsg{
  float x, y, z;
};

struct Direction3fMsg{
  float x,y,z;
};

struct Point2fMsg{
  float x, y;
};

struct boxMsg{
  float length;
  float width;
  float height;
  Point3fMsg location;
  Direction3fMsg direction;
  float yaw;
  Point3fMsg nearest_point;
};

struct PerceptResultMsg {
  boxMsg box;
  unsigned int track_id;
  float track_probability;
  Point2fMsg velocity;
  Point2fMsg acceleration;
  Point2fMsg velocity_abs;
  Point2fMsg acceleration_abs;
  float angle_velocity;
  float life;
  float visible_life;
  float robustness;

  signed char label;
  float label_confidence;

  bool is_background;
};
}


#endif //ROBOSENSE_COMMUNICATION_TYPE_H
