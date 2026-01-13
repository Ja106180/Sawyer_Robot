#ifndef _ROS_arm_follow_PersonsKeypoints_h
#define _ROS_arm_follow_PersonsKeypoints_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "arm_follow/PersonKeypoints.h"

namespace arm_follow
{

  class PersonsKeypoints : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t persons_length;
      typedef arm_follow::PersonKeypoints _persons_type;
      _persons_type st_persons;
      _persons_type * persons;

    PersonsKeypoints():
      header(),
      persons_length(0), st_persons(), persons(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->persons_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->persons_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->persons_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->persons_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->persons_length);
      for( uint32_t i = 0; i < persons_length; i++){
      offset += this->persons[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t persons_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      persons_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      persons_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      persons_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->persons_length);
      if(persons_lengthT > persons_length)
        this->persons = (arm_follow::PersonKeypoints*)realloc(this->persons, persons_lengthT * sizeof(arm_follow::PersonKeypoints));
      persons_length = persons_lengthT;
      for( uint32_t i = 0; i < persons_length; i++){
      offset += this->st_persons.deserialize(inbuffer + offset);
        memcpy( &(this->persons[i]), &(this->st_persons), sizeof(arm_follow::PersonKeypoints));
      }
     return offset;
    }

    virtual const char * getType() override { return "arm_follow/PersonsKeypoints"; };
    virtual const char * getMD5() override { return "2cc6b7f8f80dc708d5b607449f5f648f"; };

  };

}
#endif
