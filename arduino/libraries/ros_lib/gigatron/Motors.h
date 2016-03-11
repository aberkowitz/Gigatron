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
      int32_t usec_left;
      int32_t usec_right;
      float rpm_left;
      float rpm_right;

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
      union {
        int32_t real;
        uint32_t base;
      } u_usec_left;
      u_usec_left.real = this->usec_left;
      *(outbuffer + offset + 0) = (u_usec_left.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_usec_left.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_usec_left.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_usec_left.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->usec_left);
      union {
        int32_t real;
        uint32_t base;
      } u_usec_right;
      u_usec_right.real = this->usec_right;
      *(outbuffer + offset + 0) = (u_usec_right.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_usec_right.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_usec_right.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_usec_right.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->usec_right);
      union {
        float real;
        uint32_t base;
      } u_rpm_left;
      u_rpm_left.real = this->rpm_left;
      *(outbuffer + offset + 0) = (u_rpm_left.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rpm_left.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rpm_left.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rpm_left.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rpm_left);
      union {
        float real;
        uint32_t base;
      } u_rpm_right;
      u_rpm_right.real = this->rpm_right;
      *(outbuffer + offset + 0) = (u_rpm_right.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rpm_right.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rpm_right.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rpm_right.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rpm_right);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_usec_left;
      u_usec_left.base = 0;
      u_usec_left.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_usec_left.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_usec_left.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_usec_left.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->usec_left = u_usec_left.real;
      offset += sizeof(this->usec_left);
      union {
        int32_t real;
        uint32_t base;
      } u_usec_right;
      u_usec_right.base = 0;
      u_usec_right.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_usec_right.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_usec_right.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_usec_right.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->usec_right = u_usec_right.real;
      offset += sizeof(this->usec_right);
      union {
        float real;
        uint32_t base;
      } u_rpm_left;
      u_rpm_left.base = 0;
      u_rpm_left.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rpm_left.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rpm_left.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rpm_left.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rpm_left = u_rpm_left.real;
      offset += sizeof(this->rpm_left);
      union {
        float real;
        uint32_t base;
      } u_rpm_right;
      u_rpm_right.base = 0;
      u_rpm_right.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rpm_right.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rpm_right.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rpm_right.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rpm_right = u_rpm_right.real;
      offset += sizeof(this->rpm_right);
     return offset;
    }

    const char * getType(){ return "gigatron/Motors"; };
    const char * getMD5(){ return "98e68248919694cde82d52fecdc9acb8"; };

  };

}
#endif