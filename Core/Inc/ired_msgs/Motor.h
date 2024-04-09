#ifndef _ROS_ired_msgs_Motor_h
#define _ROS_ired_msgs_Motor_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ired_msgs
{

  class Motor : public ros::Msg
  {
    public:
      double speed_sp[4];
      double speed_fb[4];
      double pid_motor_front_left[3];
      double pid_motor_front_right[3];
      double pid_motor_rear_left[3];
      double pid_motor_rear_right[3];

    Motor():
      speed_sp(),
      speed_fb(),
      pid_motor_front_left(),
      pid_motor_front_right(),
      pid_motor_rear_left(),
      pid_motor_rear_right()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 4; i++){
      union {
        double real;
        uint64_t base;
      } u_speed_spi;
      u_speed_spi.real = this->speed_sp[i];
      *(outbuffer + offset + 0) = (u_speed_spi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed_spi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed_spi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed_spi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_speed_spi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_speed_spi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_speed_spi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_speed_spi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->speed_sp[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        double real;
        uint64_t base;
      } u_speed_fbi;
      u_speed_fbi.real = this->speed_fb[i];
      *(outbuffer + offset + 0) = (u_speed_fbi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed_fbi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed_fbi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed_fbi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_speed_fbi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_speed_fbi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_speed_fbi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_speed_fbi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->speed_fb[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        double real;
        uint64_t base;
      } u_pid_motor_front_lefti;
      u_pid_motor_front_lefti.real = this->pid_motor_front_left[i];
      *(outbuffer + offset + 0) = (u_pid_motor_front_lefti.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pid_motor_front_lefti.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pid_motor_front_lefti.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pid_motor_front_lefti.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_pid_motor_front_lefti.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_pid_motor_front_lefti.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_pid_motor_front_lefti.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_pid_motor_front_lefti.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->pid_motor_front_left[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        double real;
        uint64_t base;
      } u_pid_motor_front_righti;
      u_pid_motor_front_righti.real = this->pid_motor_front_right[i];
      *(outbuffer + offset + 0) = (u_pid_motor_front_righti.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pid_motor_front_righti.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pid_motor_front_righti.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pid_motor_front_righti.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_pid_motor_front_righti.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_pid_motor_front_righti.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_pid_motor_front_righti.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_pid_motor_front_righti.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->pid_motor_front_right[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        double real;
        uint64_t base;
      } u_pid_motor_rear_lefti;
      u_pid_motor_rear_lefti.real = this->pid_motor_rear_left[i];
      *(outbuffer + offset + 0) = (u_pid_motor_rear_lefti.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pid_motor_rear_lefti.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pid_motor_rear_lefti.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pid_motor_rear_lefti.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_pid_motor_rear_lefti.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_pid_motor_rear_lefti.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_pid_motor_rear_lefti.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_pid_motor_rear_lefti.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->pid_motor_rear_left[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        double real;
        uint64_t base;
      } u_pid_motor_rear_righti;
      u_pid_motor_rear_righti.real = this->pid_motor_rear_right[i];
      *(outbuffer + offset + 0) = (u_pid_motor_rear_righti.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pid_motor_rear_righti.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pid_motor_rear_righti.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pid_motor_rear_righti.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_pid_motor_rear_righti.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_pid_motor_rear_righti.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_pid_motor_rear_righti.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_pid_motor_rear_righti.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->pid_motor_rear_right[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 4; i++){
      union {
        double real;
        uint64_t base;
      } u_speed_spi;
      u_speed_spi.base = 0;
      u_speed_spi.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed_spi.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed_spi.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed_spi.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_speed_spi.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_speed_spi.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_speed_spi.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_speed_spi.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->speed_sp[i] = u_speed_spi.real;
      offset += sizeof(this->speed_sp[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        double real;
        uint64_t base;
      } u_speed_fbi;
      u_speed_fbi.base = 0;
      u_speed_fbi.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed_fbi.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed_fbi.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed_fbi.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_speed_fbi.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_speed_fbi.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_speed_fbi.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_speed_fbi.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->speed_fb[i] = u_speed_fbi.real;
      offset += sizeof(this->speed_fb[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        double real;
        uint64_t base;
      } u_pid_motor_front_lefti;
      u_pid_motor_front_lefti.base = 0;
      u_pid_motor_front_lefti.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pid_motor_front_lefti.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pid_motor_front_lefti.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pid_motor_front_lefti.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_pid_motor_front_lefti.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_pid_motor_front_lefti.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_pid_motor_front_lefti.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_pid_motor_front_lefti.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->pid_motor_front_left[i] = u_pid_motor_front_lefti.real;
      offset += sizeof(this->pid_motor_front_left[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        double real;
        uint64_t base;
      } u_pid_motor_front_righti;
      u_pid_motor_front_righti.base = 0;
      u_pid_motor_front_righti.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pid_motor_front_righti.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pid_motor_front_righti.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pid_motor_front_righti.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_pid_motor_front_righti.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_pid_motor_front_righti.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_pid_motor_front_righti.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_pid_motor_front_righti.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->pid_motor_front_right[i] = u_pid_motor_front_righti.real;
      offset += sizeof(this->pid_motor_front_right[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        double real;
        uint64_t base;
      } u_pid_motor_rear_lefti;
      u_pid_motor_rear_lefti.base = 0;
      u_pid_motor_rear_lefti.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pid_motor_rear_lefti.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pid_motor_rear_lefti.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pid_motor_rear_lefti.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_pid_motor_rear_lefti.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_pid_motor_rear_lefti.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_pid_motor_rear_lefti.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_pid_motor_rear_lefti.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->pid_motor_rear_left[i] = u_pid_motor_rear_lefti.real;
      offset += sizeof(this->pid_motor_rear_left[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        double real;
        uint64_t base;
      } u_pid_motor_rear_righti;
      u_pid_motor_rear_righti.base = 0;
      u_pid_motor_rear_righti.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pid_motor_rear_righti.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pid_motor_rear_righti.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pid_motor_rear_righti.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_pid_motor_rear_righti.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_pid_motor_rear_righti.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_pid_motor_rear_righti.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_pid_motor_rear_righti.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->pid_motor_rear_right[i] = u_pid_motor_rear_righti.real;
      offset += sizeof(this->pid_motor_rear_right[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return "ired_msgs/Motor"; };
    virtual const char * getMD5() override { return "ec2526e9a146e07c0941072e0ef16c7b"; };

  };

}
#endif
