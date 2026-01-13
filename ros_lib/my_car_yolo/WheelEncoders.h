#ifndef _ROS_my_car_yolo_WheelEncoders_h
#define _ROS_my_car_yolo_WheelEncoders_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace my_car_yolo
{

  class WheelEncoders : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int32_t _left_pulses_type;
      _left_pulses_type left_pulses;
      typedef int32_t _right_pulses_type;
      _right_pulses_type right_pulses;
      typedef float _left_velocity_type;
      _left_velocity_type left_velocity;
      typedef float _right_velocity_type;
      _right_velocity_type right_velocity;

    WheelEncoders():
      header(),
      left_pulses(0),
      right_pulses(0),
      left_velocity(0),
      right_velocity(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_left_pulses;
      u_left_pulses.real = this->left_pulses;
      *(outbuffer + offset + 0) = (u_left_pulses.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_pulses.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_pulses.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_pulses.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_pulses);
      union {
        int32_t real;
        uint32_t base;
      } u_right_pulses;
      u_right_pulses.real = this->right_pulses;
      *(outbuffer + offset + 0) = (u_right_pulses.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_pulses.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_pulses.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_pulses.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_pulses);
      offset += serializeAvrFloat64(outbuffer + offset, this->left_velocity);
      offset += serializeAvrFloat64(outbuffer + offset, this->right_velocity);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_left_pulses;
      u_left_pulses.base = 0;
      u_left_pulses.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_pulses.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_pulses.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_pulses.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_pulses = u_left_pulses.real;
      offset += sizeof(this->left_pulses);
      union {
        int32_t real;
        uint32_t base;
      } u_right_pulses;
      u_right_pulses.base = 0;
      u_right_pulses.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_pulses.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_pulses.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_pulses.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_pulses = u_right_pulses.real;
      offset += sizeof(this->right_pulses);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->left_velocity));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->right_velocity));
     return offset;
    }

    virtual const char * getType() override { return "my_car_yolo/WheelEncoders"; };
    virtual const char * getMD5() override { return "04b36400bf0a0e74c73173ddcaa70f42"; };

  };

}
#endif
