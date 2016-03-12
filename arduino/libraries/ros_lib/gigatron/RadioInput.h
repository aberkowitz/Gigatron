#ifndef _ROS_gigatron_RadioInput_h
#define _ROS_gigatron_RadioInput_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace gigatron
{

  class RadioInput : public ros::Msg
  {
    public:
      std_msgs::Header header;
      int8_t speed_left;
      int8_t speed_right;
      int8_t dir_left;
      int8_t dir_right;
      int8_t angle;
      int8_t kill;

    RadioInput():
      header(),
      speed_left(0),
      speed_right(0),
      dir_left(0),
      dir_right(0),
      angle(0),
      kill(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int8_t real;
        uint8_t base;
      } u_speed_left;
      u_speed_left.real = this->speed_left;
      *(outbuffer + offset + 0) = (u_speed_left.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->speed_left);
      union {
        int8_t real;
        uint8_t base;
      } u_speed_right;
      u_speed_right.real = this->speed_right;
      *(outbuffer + offset + 0) = (u_speed_right.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->speed_right);
      union {
        int8_t real;
        uint8_t base;
      } u_dir_left;
      u_dir_left.real = this->dir_left;
      *(outbuffer + offset + 0) = (u_dir_left.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->dir_left);
      union {
        int8_t real;
        uint8_t base;
      } u_dir_right;
      u_dir_right.real = this->dir_right;
      *(outbuffer + offset + 0) = (u_dir_right.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->dir_right);
      union {
        int8_t real;
        uint8_t base;
      } u_angle;
      u_angle.real = this->angle;
      *(outbuffer + offset + 0) = (u_angle.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->angle);
      union {
        int8_t real;
        uint8_t base;
      } u_kill;
      u_kill.real = this->kill;
      *(outbuffer + offset + 0) = (u_kill.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->kill);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int8_t real;
        uint8_t base;
      } u_speed_left;
      u_speed_left.base = 0;
      u_speed_left.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->speed_left = u_speed_left.real;
      offset += sizeof(this->speed_left);
      union {
        int8_t real;
        uint8_t base;
      } u_speed_right;
      u_speed_right.base = 0;
      u_speed_right.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->speed_right = u_speed_right.real;
      offset += sizeof(this->speed_right);
      union {
        int8_t real;
        uint8_t base;
      } u_dir_left;
      u_dir_left.base = 0;
      u_dir_left.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->dir_left = u_dir_left.real;
      offset += sizeof(this->dir_left);
      union {
        int8_t real;
        uint8_t base;
      } u_dir_right;
      u_dir_right.base = 0;
      u_dir_right.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->dir_right = u_dir_right.real;
      offset += sizeof(this->dir_right);
      union {
        int8_t real;
        uint8_t base;
      } u_angle;
      u_angle.base = 0;
      u_angle.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->angle = u_angle.real;
      offset += sizeof(this->angle);
      union {
        int8_t real;
        uint8_t base;
      } u_kill;
      u_kill.base = 0;
      u_kill.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->kill = u_kill.real;
      offset += sizeof(this->kill);
     return offset;
    }

    const char * getType(){ return "gigatron/RadioInput"; };
    const char * getMD5(){ return "ecc5379e33c68b4ae21e262b27270f73"; };

  };

}
#endif