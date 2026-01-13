#ifndef _ROS_arm_follow_PersonKeypoints_h
#define _ROS_arm_follow_PersonKeypoints_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace arm_follow
{

  class PersonKeypoints : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint32_t _person_id_type;
      _person_id_type person_id;
      typedef float _bbox_x_type;
      _bbox_x_type bbox_x;
      typedef float _bbox_y_type;
      _bbox_y_type bbox_y;
      typedef float _bbox_width_type;
      _bbox_width_type bbox_width;
      typedef float _bbox_height_type;
      _bbox_height_type bbox_height;
      typedef float _bbox_area_type;
      _bbox_area_type bbox_area;
      typedef float _bbox_confidence_type;
      _bbox_confidence_type bbox_confidence;
      typedef float _right_wrist_x_type;
      _right_wrist_x_type right_wrist_x;
      typedef float _right_wrist_y_type;
      _right_wrist_y_type right_wrist_y;
      typedef float _right_wrist_conf_type;
      _right_wrist_conf_type right_wrist_conf;
      typedef float _right_elbow_x_type;
      _right_elbow_x_type right_elbow_x;
      typedef float _right_elbow_y_type;
      _right_elbow_y_type right_elbow_y;
      typedef float _right_elbow_conf_type;
      _right_elbow_conf_type right_elbow_conf;
      typedef float _right_shoulder_x_type;
      _right_shoulder_x_type right_shoulder_x;
      typedef float _right_shoulder_y_type;
      _right_shoulder_y_type right_shoulder_y;
      typedef float _right_shoulder_conf_type;
      _right_shoulder_conf_type right_shoulder_conf;
      typedef uint32_t _image_width_type;
      _image_width_type image_width;
      typedef uint32_t _image_height_type;
      _image_height_type image_height;

    PersonKeypoints():
      header(),
      person_id(0),
      bbox_x(0),
      bbox_y(0),
      bbox_width(0),
      bbox_height(0),
      bbox_area(0),
      bbox_confidence(0),
      right_wrist_x(0),
      right_wrist_y(0),
      right_wrist_conf(0),
      right_elbow_x(0),
      right_elbow_y(0),
      right_elbow_conf(0),
      right_shoulder_x(0),
      right_shoulder_y(0),
      right_shoulder_conf(0),
      image_width(0),
      image_height(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->person_id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->person_id >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->person_id >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->person_id >> (8 * 3)) & 0xFF;
      offset += sizeof(this->person_id);
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
      union {
        float real;
        uint32_t base;
      } u_bbox_area;
      u_bbox_area.real = this->bbox_area;
      *(outbuffer + offset + 0) = (u_bbox_area.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_bbox_area.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_bbox_area.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_bbox_area.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->bbox_area);
      union {
        float real;
        uint32_t base;
      } u_bbox_confidence;
      u_bbox_confidence.real = this->bbox_confidence;
      *(outbuffer + offset + 0) = (u_bbox_confidence.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_bbox_confidence.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_bbox_confidence.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_bbox_confidence.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->bbox_confidence);
      union {
        float real;
        uint32_t base;
      } u_right_wrist_x;
      u_right_wrist_x.real = this->right_wrist_x;
      *(outbuffer + offset + 0) = (u_right_wrist_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_wrist_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_wrist_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_wrist_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_wrist_x);
      union {
        float real;
        uint32_t base;
      } u_right_wrist_y;
      u_right_wrist_y.real = this->right_wrist_y;
      *(outbuffer + offset + 0) = (u_right_wrist_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_wrist_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_wrist_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_wrist_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_wrist_y);
      union {
        float real;
        uint32_t base;
      } u_right_wrist_conf;
      u_right_wrist_conf.real = this->right_wrist_conf;
      *(outbuffer + offset + 0) = (u_right_wrist_conf.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_wrist_conf.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_wrist_conf.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_wrist_conf.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_wrist_conf);
      union {
        float real;
        uint32_t base;
      } u_right_elbow_x;
      u_right_elbow_x.real = this->right_elbow_x;
      *(outbuffer + offset + 0) = (u_right_elbow_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_elbow_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_elbow_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_elbow_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_elbow_x);
      union {
        float real;
        uint32_t base;
      } u_right_elbow_y;
      u_right_elbow_y.real = this->right_elbow_y;
      *(outbuffer + offset + 0) = (u_right_elbow_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_elbow_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_elbow_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_elbow_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_elbow_y);
      union {
        float real;
        uint32_t base;
      } u_right_elbow_conf;
      u_right_elbow_conf.real = this->right_elbow_conf;
      *(outbuffer + offset + 0) = (u_right_elbow_conf.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_elbow_conf.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_elbow_conf.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_elbow_conf.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_elbow_conf);
      union {
        float real;
        uint32_t base;
      } u_right_shoulder_x;
      u_right_shoulder_x.real = this->right_shoulder_x;
      *(outbuffer + offset + 0) = (u_right_shoulder_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_shoulder_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_shoulder_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_shoulder_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_shoulder_x);
      union {
        float real;
        uint32_t base;
      } u_right_shoulder_y;
      u_right_shoulder_y.real = this->right_shoulder_y;
      *(outbuffer + offset + 0) = (u_right_shoulder_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_shoulder_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_shoulder_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_shoulder_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_shoulder_y);
      union {
        float real;
        uint32_t base;
      } u_right_shoulder_conf;
      u_right_shoulder_conf.real = this->right_shoulder_conf;
      *(outbuffer + offset + 0) = (u_right_shoulder_conf.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_shoulder_conf.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_shoulder_conf.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_shoulder_conf.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_shoulder_conf);
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
      this->person_id =  ((uint32_t) (*(inbuffer + offset)));
      this->person_id |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->person_id |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->person_id |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->person_id);
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
      union {
        float real;
        uint32_t base;
      } u_bbox_area;
      u_bbox_area.base = 0;
      u_bbox_area.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_bbox_area.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_bbox_area.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_bbox_area.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->bbox_area = u_bbox_area.real;
      offset += sizeof(this->bbox_area);
      union {
        float real;
        uint32_t base;
      } u_bbox_confidence;
      u_bbox_confidence.base = 0;
      u_bbox_confidence.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_bbox_confidence.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_bbox_confidence.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_bbox_confidence.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->bbox_confidence = u_bbox_confidence.real;
      offset += sizeof(this->bbox_confidence);
      union {
        float real;
        uint32_t base;
      } u_right_wrist_x;
      u_right_wrist_x.base = 0;
      u_right_wrist_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_wrist_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_wrist_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_wrist_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_wrist_x = u_right_wrist_x.real;
      offset += sizeof(this->right_wrist_x);
      union {
        float real;
        uint32_t base;
      } u_right_wrist_y;
      u_right_wrist_y.base = 0;
      u_right_wrist_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_wrist_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_wrist_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_wrist_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_wrist_y = u_right_wrist_y.real;
      offset += sizeof(this->right_wrist_y);
      union {
        float real;
        uint32_t base;
      } u_right_wrist_conf;
      u_right_wrist_conf.base = 0;
      u_right_wrist_conf.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_wrist_conf.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_wrist_conf.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_wrist_conf.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_wrist_conf = u_right_wrist_conf.real;
      offset += sizeof(this->right_wrist_conf);
      union {
        float real;
        uint32_t base;
      } u_right_elbow_x;
      u_right_elbow_x.base = 0;
      u_right_elbow_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_elbow_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_elbow_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_elbow_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_elbow_x = u_right_elbow_x.real;
      offset += sizeof(this->right_elbow_x);
      union {
        float real;
        uint32_t base;
      } u_right_elbow_y;
      u_right_elbow_y.base = 0;
      u_right_elbow_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_elbow_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_elbow_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_elbow_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_elbow_y = u_right_elbow_y.real;
      offset += sizeof(this->right_elbow_y);
      union {
        float real;
        uint32_t base;
      } u_right_elbow_conf;
      u_right_elbow_conf.base = 0;
      u_right_elbow_conf.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_elbow_conf.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_elbow_conf.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_elbow_conf.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_elbow_conf = u_right_elbow_conf.real;
      offset += sizeof(this->right_elbow_conf);
      union {
        float real;
        uint32_t base;
      } u_right_shoulder_x;
      u_right_shoulder_x.base = 0;
      u_right_shoulder_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_shoulder_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_shoulder_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_shoulder_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_shoulder_x = u_right_shoulder_x.real;
      offset += sizeof(this->right_shoulder_x);
      union {
        float real;
        uint32_t base;
      } u_right_shoulder_y;
      u_right_shoulder_y.base = 0;
      u_right_shoulder_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_shoulder_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_shoulder_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_shoulder_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_shoulder_y = u_right_shoulder_y.real;
      offset += sizeof(this->right_shoulder_y);
      union {
        float real;
        uint32_t base;
      } u_right_shoulder_conf;
      u_right_shoulder_conf.base = 0;
      u_right_shoulder_conf.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_shoulder_conf.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_shoulder_conf.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_shoulder_conf.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_shoulder_conf = u_right_shoulder_conf.real;
      offset += sizeof(this->right_shoulder_conf);
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

    virtual const char * getType() override { return "arm_follow/PersonKeypoints"; };
    virtual const char * getMD5() override { return "b8aef0b6f60577e67360824121ee7a78"; };

  };

}
#endif
