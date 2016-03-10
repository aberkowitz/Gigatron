#ifndef _ROS_gigatron_Angle_h
#define _ROS_gigatron_Angle_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace gigatron
{

  class Angle : public ros::Msg
  {
    public:
      int16_t angle;

    Angle():
      angle(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_angle;
      u_angle.real = this->angle;
      *(outbuffer + offset + 0) = (u_angle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angle.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->angle);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_angle;
      u_angle.base = 0;
      u_angle.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angle.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->angle = u_angle.real;
      offset += sizeof(this->angle);
     return offset;
    }

    const char * getType(){ return "gigatron/Angle"; };
    const char * getMD5(){ return "ecd1f62d4afad57b40d43ebefe588fc8"; };

  };

}
#endif