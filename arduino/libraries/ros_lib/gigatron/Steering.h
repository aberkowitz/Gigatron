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
      uint8_t angle_command;
      uint8_t angle;

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
      *(outbuffer + offset + 0) = (this->angle_command >> (8 * 0)) & 0xFF;
      offset += sizeof(this->angle_command);
      *(outbuffer + offset + 0) = (this->angle >> (8 * 0)) & 0xFF;
      offset += sizeof(this->angle);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->angle_command =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->angle_command);
      this->angle =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->angle);
     return offset;
    }

    const char * getType(){ return "gigatron/Steering"; };
    const char * getMD5(){ return "d462d94c1b7e973a58a35ad5ef9c54b0"; };

  };

}
#endif