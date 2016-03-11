#ifndef _ROS_gigatron_Steering_h
#define _ROS_gigatron_Steering_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace gigatron
{

  class Steering : public ros::Msg
  {
    public:
      std_msgs::Header header;
      int8_t angle_command;
      int8_t angle;

    Steering():
      header(),
      angle_command(0),
      angle(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int8_t real;
        uint8_t base;
      } u_angle_command;
      u_angle_command.real = this->angle_command;
      *(outbuffer + offset + 0) = (u_angle_command.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->angle_command);
      union {
        int8_t real;
        uint8_t base;
      } u_angle;
      u_angle.real = this->angle;
      *(outbuffer + offset + 0) = (u_angle.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->angle);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int8_t real;
        uint8_t base;
      } u_angle_command;
      u_angle_command.base = 0;
      u_angle_command.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->angle_command = u_angle_command.real;
      offset += sizeof(this->angle_command);
      union {
        int8_t real;
        uint8_t base;
      } u_angle;
      u_angle.base = 0;
      u_angle.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->angle = u_angle.real;
      offset += sizeof(this->angle);
     return offset;
    }

    const char * getType(){ return "gigatron/Steering"; };
    const char * getMD5(){ return "4661b988cad4d0e85a91927fc797b01f"; };

  };

}
#endif