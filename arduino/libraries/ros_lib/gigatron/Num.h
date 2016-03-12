#ifndef _ROS_gigatron_Num_h
#define _ROS_gigatron_Num_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace gigatron
{

  class Num : public ros::Msg
  {
    public:
      uint16_t num;

    Num():
      num(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->num >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->num >> (8 * 1)) & 0xFF;
      offset += sizeof(this->num);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->num =  ((uint16_t) (*(inbuffer + offset)));
      this->num |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->num);
     return offset;
    }

    const char * getType(){ return "gigatron/Num"; };
    const char * getMD5(){ return "0285aea93c08847f409c58db5e8d9137"; };

  };

}
#endif