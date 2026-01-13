#ifndef _ROS_my_car_yolo_ImuProcessed_h
#define _ROS_my_car_yolo_ImuProcessed_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace my_car_yolo
{

  class ImuProcessed : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _yaw_type;
      _yaw_type yaw;
      typedef float _yaw_filtered_type;
      _yaw_filtered_type yaw_filtered;
      typedef float _yaw_rate_type;
      _yaw_rate_type yaw_rate;
      typedef float _gyro_bias_x_type;
      _gyro_bias_x_type gyro_bias_x;
      typedef float _gyro_bias_y_type;
      _gyro_bias_y_type gyro_bias_y;
      typedef float _gyro_bias_z_type;
      _gyro_bias_z_type gyro_bias_z;
      typedef float _accel_x_type;
      _accel_x_type accel_x;
      typedef float _accel_y_type;
      _accel_y_type accel_y;
      typedef float _accel_z_type;
      _accel_z_type accel_z;

    ImuProcessed():
      header(),
      yaw(0),
      yaw_filtered(0),
      yaw_rate(0),
      gyro_bias_x(0),
      gyro_bias_y(0),
      gyro_bias_z(0),
      accel_x(0),
      accel_y(0),
      accel_z(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->yaw);
      offset += serializeAvrFloat64(outbuffer + offset, this->yaw_filtered);
      offset += serializeAvrFloat64(outbuffer + offset, this->yaw_rate);
      offset += serializeAvrFloat64(outbuffer + offset, this->gyro_bias_x);
      offset += serializeAvrFloat64(outbuffer + offset, this->gyro_bias_y);
      offset += serializeAvrFloat64(outbuffer + offset, this->gyro_bias_z);
      offset += serializeAvrFloat64(outbuffer + offset, this->accel_x);
      offset += serializeAvrFloat64(outbuffer + offset, this->accel_y);
      offset += serializeAvrFloat64(outbuffer + offset, this->accel_z);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->yaw));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->yaw_filtered));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->yaw_rate));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->gyro_bias_x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->gyro_bias_y));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->gyro_bias_z));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->accel_x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->accel_y));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->accel_z));
     return offset;
    }

    virtual const char * getType() override { return "my_car_yolo/ImuProcessed"; };
    virtual const char * getMD5() override { return "f8eb09331f7dc4e2c7e0240f0df9da14"; };

  };

}
#endif
