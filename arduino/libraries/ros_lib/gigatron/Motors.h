#ifndef _ROS_gigatron_Motors_h
#define _ROS_gigatron_Motors_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace gigatron
{

  class Motors : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint16_t usec_left;
      uint16_t usec_right;
      int16_t rpm_left;
      int16_t rpm_right;

    Motors():
      header(),
      usec_left(0),
      usec_right(0),
      rpm_left(0),
      rpm_right(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->usec_left >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->usec_left >> (8 * 1)) & 0xFF;
      offset += sizeof(this->usec_left);
      *(outbuffer + offset + 0) = (this->usec_right >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->usec_right >> (8 * 1)) & 0xFF;
      offset += sizeof(this->usec_right);
      union {
        int16_t real;
        uint16_t base;
      } u_rpm_left;
      u_rpm_left.real = this->rpm_left;
      *(outbuffer + offset + 0) = (u_rpm_left.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rpm_left.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->rpm_left);
      union {
        int16_t real;
        uint16_t base;
      } u_rpm_right;
      u_rpm_right.real = this->rpm_right;
      *(outbuffer + offset + 0) = (u_rpm_right.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rpm_right.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->rpm_right);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->usec_left =  ((uint16_t) (*(inbuffer + offset)));
      this->usec_left |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->usec_left);
      this->usec_right =  ((uint16_t) (*(inbuffer + offset)));
      this->usec_right |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->usec_right);
      union {
        int16_t real;
        uint16_t base;
      } u_rpm_left;
      u_rpm_left.base = 0;
      u_rpm_left.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rpm_left.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->rpm_left = u_rpm_left.real;
      offset += sizeof(this->rpm_left);
      union {
        int16_t real;
        uint16_t base;
      } u_rpm_right;
      u_rpm_right.base = 0;
      u_rpm_right.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rpm_right.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->rpm_right = u_rpm_right.real;
      offset += sizeof(this->rpm_right);
     return offset;
    }

    const char * getType(){ return "gigatron/Motors"; };
    const char * getMD5(){ return "ce547d435308ba3ff0ed76d9f237030f"; };

  };

}
#endif