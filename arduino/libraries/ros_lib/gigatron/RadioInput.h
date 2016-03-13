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
      uint8_t speed_left;
      uint8_t speed_right;
      uint8_t dir_left;
      uint8_t dir_right;
      uint8_t angle;
      uint8_t kill;

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
      *(outbuffer + offset + 0) = (this->speed_left >> (8 * 0)) & 0xFF;
      offset += sizeof(this->speed_left);
      *(outbuffer + offset + 0) = (this->speed_right >> (8 * 0)) & 0xFF;
      offset += sizeof(this->speed_right);
      *(outbuffer + offset + 0) = (this->dir_left >> (8 * 0)) & 0xFF;
      offset += sizeof(this->dir_left);
      *(outbuffer + offset + 0) = (this->dir_right >> (8 * 0)) & 0xFF;
      offset += sizeof(this->dir_right);
      *(outbuffer + offset + 0) = (this->angle >> (8 * 0)) & 0xFF;
      offset += sizeof(this->angle);
      *(outbuffer + offset + 0) = (this->kill >> (8 * 0)) & 0xFF;
      offset += sizeof(this->kill);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->speed_left =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->speed_left);
      this->speed_right =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->speed_right);
      this->dir_left =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->dir_left);
      this->dir_right =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->dir_right);
      this->angle =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->angle);
      this->kill =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->kill);
     return offset;
    }

    const char * getType(){ return "gigatron/RadioInput"; };
    const char * getMD5(){ return "2d36c2d773f9dd8d205ecda858118d0d"; };

  };

}
#endif