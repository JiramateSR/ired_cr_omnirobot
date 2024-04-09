#ifndef _ROS_ired_msgs_PID_h
#define _ROS_ired_msgs_PID_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ired_msgs
{

  class PID : public ros::Msg
  {
    public:
      typedef const char* _motor_type;
      _motor_type motor;
      typedef double _kp_type;
      _kp_type kp;
      typedef double _ki_type;
      _ki_type ki;
      typedef double _kd_type;
      _kd_type kd;

    PID():
      motor(""),
      kp(0),
      ki(0),
      kd(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_motor = strlen(this->motor);
      varToArr(outbuffer + offset, length_motor);
      offset += 4;
      memcpy(outbuffer + offset, this->motor, length_motor);
      offset += length_motor;
      union {
        double real;
        uint64_t base;
      } u_kp;
      u_kp.real = this->kp;
      *(outbuffer + offset + 0) = (u_kp.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kp.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kp.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kp.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_kp.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_kp.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_kp.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_kp.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->kp);
      union {
        double real;
        uint64_t base;
      } u_ki;
      u_ki.real = this->ki;
      *(outbuffer + offset + 0) = (u_ki.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ki.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ki.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ki.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_ki.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_ki.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_ki.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_ki.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->ki);
      union {
        double real;
        uint64_t base;
      } u_kd;
      u_kd.real = this->kd;
      *(outbuffer + offset + 0) = (u_kd.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kd.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kd.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kd.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_kd.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_kd.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_kd.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_kd.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->kd);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_motor;
      arrToVar(length_motor, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_motor; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_motor-1]=0;
      this->motor = (char *)(inbuffer + offset-1);
      offset += length_motor;
      union {
        double real;
        uint64_t base;
      } u_kp;
      u_kp.base = 0;
      u_kp.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kp.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kp.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kp.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_kp.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_kp.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_kp.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_kp.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->kp = u_kp.real;
      offset += sizeof(this->kp);
      union {
        double real;
        uint64_t base;
      } u_ki;
      u_ki.base = 0;
      u_ki.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ki.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ki.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ki.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_ki.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_ki.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_ki.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_ki.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->ki = u_ki.real;
      offset += sizeof(this->ki);
      union {
        double real;
        uint64_t base;
      } u_kd;
      u_kd.base = 0;
      u_kd.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kd.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kd.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kd.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_kd.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_kd.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_kd.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_kd.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->kd = u_kd.real;
      offset += sizeof(this->kd);
     return offset;
    }

    virtual const char * getType() override { return "ired_msgs/PID"; };
    virtual const char * getMD5() override { return "2169d9d8246848be6270c046ce9df384"; };

  };

}
#endif
