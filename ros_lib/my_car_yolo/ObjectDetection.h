#ifndef _ROS_my_car_yolo_ObjectDetection_h
#define _ROS_my_car_yolo_ObjectDetection_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace my_car_yolo
{

  class ObjectDetection : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint32_t _object_id_type;
      _object_id_type object_id;
      typedef float _center_x_type;
      _center_x_type center_x;
      typedef float _center_y_type;
      _center_y_type center_y;
      typedef float _confidence_type;
      _confidence_type confidence;
      typedef float _bbox_x_type;
      _bbox_x_type bbox_x;
      typedef float _bbox_y_type;
      _bbox_y_type bbox_y;
      typedef float _bbox_width_type;
      _bbox_width_type bbox_width;
      typedef float _bbox_height_type;
      _bbox_height_type bbox_height;
      typedef uint32_t _image_width_type;
      _image_width_type image_width;
      typedef uint32_t _image_height_type;
      _image_height_type image_height;

    ObjectDetection():
      header(),
      object_id(0),
      center_x(0),
      center_y(0),
      confidence(0),
      bbox_x(0),
      bbox_y(0),
      bbox_width(0),
      bbox_height(0),
      image_width(0),
      image_height(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->object_id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->object_id >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->object_id >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->object_id >> (8 * 3)) & 0xFF;
      offset += sizeof(this->object_id);
      union {
        float real;
        uint32_t base;
      } u_center_x;
      u_center_x.real = this->center_x;
      *(outbuffer + offset + 0) = (u_center_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_center_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_center_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_center_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->center_x);
      union {
        float real;
        uint32_t base;
      } u_center_y;
      u_center_y.real = this->center_y;
      *(outbuffer + offset + 0) = (u_center_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_center_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_center_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_center_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->center_y);
      union {
        float real;
        uint32_t base;
      } u_confidence;
      u_confidence.real = this->confidence;
      *(outbuffer + offset + 0) = (u_confidence.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_confidence.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_confidence.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_confidence.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->confidence);
      union {
        float real;
        uint32_t base;
      } u_bbox_x;
      u_bbox_x.real = this->bbox_x;
      *(outbuffer + offset + 0) = (u_bbox_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_bbox_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_bbox_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_bbox_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->bbox_x);
      union {
        float real;
        uint32_t base;
      } u_bbox_y;
      u_bbox_y.real = this->bbox_y;
      *(outbuffer + offset + 0) = (u_bbox_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_bbox_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_bbox_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_bbox_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->bbox_y);
      union {
        float real;
        uint32_t base;
      } u_bbox_width;
      u_bbox_width.real = this->bbox_width;
      *(outbuffer + offset + 0) = (u_bbox_width.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_bbox_width.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_bbox_width.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_bbox_width.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->bbox_width);
      union {
        float real;
        uint32_t base;
      } u_bbox_height;
      u_bbox_height.real = this->bbox_height;
      *(outbuffer + offset + 0) = (u_bbox_height.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_bbox_height.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_bbox_height.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_bbox_height.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->bbox_height);
      *(outbuffer + offset + 0) = (this->image_width >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->image_width >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->image_width >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->image_width >> (8 * 3)) & 0xFF;
      offset += sizeof(this->image_width);
      *(outbuffer + offset + 0) = (this->image_height >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->image_height >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->image_height >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->image_height >> (8 * 3)) & 0xFF;
      offset += sizeof(this->image_height);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->object_id =  ((uint32_t) (*(inbuffer + offset)));
      this->object_id |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->object_id |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->object_id |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->object_id);
      union {
        float real;
        uint32_t base;
      } u_center_x;
      u_center_x.base = 0;
      u_center_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_center_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_center_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_center_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->center_x = u_center_x.real;
      offset += sizeof(this->center_x);
      union {
        float real;
        uint32_t base;
      } u_center_y;
      u_center_y.base = 0;
      u_center_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_center_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_center_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_center_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->center_y = u_center_y.real;
      offset += sizeof(this->center_y);
      union {
        float real;
        uint32_t base;
      } u_confidence;
      u_confidence.base = 0;
      u_confidence.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_confidence.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_confidence.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_confidence.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->confidence = u_confidence.real;
      offset += sizeof(this->confidence);
      union {
        float real;
        uint32_t base;
      } u_bbox_x;
      u_bbox_x.base = 0;
      u_bbox_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_bbox_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_bbox_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_bbox_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->bbox_x = u_bbox_x.real;
      offset += sizeof(this->bbox_x);
      union {
        float real;
        uint32_t base;
      } u_bbox_y;
      u_bbox_y.base = 0;
      u_bbox_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_bbox_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_bbox_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_bbox_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->bbox_y = u_bbox_y.real;
      offset += sizeof(this->bbox_y);
      union {
        float real;
        uint32_t base;
      } u_bbox_width;
      u_bbox_width.base = 0;
      u_bbox_width.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_bbox_width.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_bbox_width.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_bbox_width.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->bbox_width = u_bbox_width.real;
      offset += sizeof(this->bbox_width);
      union {
        float real;
        uint32_t base;
      } u_bbox_height;
      u_bbox_height.base = 0;
      u_bbox_height.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_bbox_height.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_bbox_height.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_bbox_height.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->bbox_height = u_bbox_height.real;
      offset += sizeof(this->bbox_height);
      this->image_width =  ((uint32_t) (*(inbuffer + offset)));
      this->image_width |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->image_width |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->image_width |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->image_width);
      this->image_height =  ((uint32_t) (*(inbuffer + offset)));
      this->image_height |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->image_height |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->image_height |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->image_height);
     return offset;
    }

    virtual const char * getType() override { return "my_car_yolo/ObjectDetection"; };
    virtual const char * getMD5() override { return "74b64be95b23c1ba8f1d1679b6ef3c59"; };

  };

}
#endif
