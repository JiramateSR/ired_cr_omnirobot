#ifndef _ROS_kbot_msgs_pid_h
#define _ROS_kbot_msgs_pid_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace kbot_msgs
{

  class pid : public ros::Msg
  {
    public:
      typedef const char* _motor_type;
      _motor_type motor;
      typedef int16_t _kp_type;
      _kp_type kp;
      typedef int16_t _ki_type;
      _ki_type ki;
      typedef int16_t _kd_type;
      _kd_type kd;

    pid():
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
        int16_t real;
        uint16_t base;
      } u_kp;
      u_kp.real = this->kp;
      *(outbuffer + offset + 0) = (u_kp.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kp.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->kp);
      union {
        int16_t real;
        uint16_t base;
      } u_ki;
      u_ki.real = this->ki;
      *(outbuffer + offset + 0) = (u_ki.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ki.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->ki);
      union {
        int16_t real;
        uint16_t base;
      } u_kd;
      u_kd.real = this->kd;
      *(outbuffer + offset + 0) = (u_kd.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kd.base >> (8 * 1)) & 0xFF;
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
        int16_t real;
        uint16_t base;
      } u_kp;
      u_kp.base = 0;
      u_kp.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kp.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->kp = u_kp.real;
      offset += sizeof(this->kp);
      union {
        int16_t real;
        uint16_t base;
      } u_ki;
      u_ki.base = 0;
      u_ki.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ki.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->ki = u_ki.real;
      offset += sizeof(this->ki);
      union {
        int16_t real;
        uint16_t base;
      } u_kd;
      u_kd.base = 0;
      u_kd.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kd.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->kd = u_kd.real;
      offset += sizeof(this->kd);
     return offset;
    }

    virtual const char * getType() override { return "kbot_msgs/pid"; };
    virtual const char * getMD5() override { return "f7e15da00dbdaa579329c6ca726f63c0"; };

  };

}
#endif
