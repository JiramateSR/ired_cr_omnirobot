#ifndef _ROS_kbot_msgs_lowlevel_h
#define _ROS_kbot_msgs_lowlevel_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace kbot_msgs
{

  class lowlevel : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t speed_sp_length;
      typedef double _speed_sp_type;
      _speed_sp_type st_speed_sp;
      _speed_sp_type * speed_sp;
      uint32_t speed_fb_length;
      typedef double _speed_fb_type;
      _speed_fb_type st_speed_fb;
      _speed_fb_type * speed_fb;
      uint32_t pid_motor_left_length;
      typedef double _pid_motor_left_type;
      _pid_motor_left_type st_pid_motor_left;
      _pid_motor_left_type * pid_motor_left;
      uint32_t pid_motor_right_length;
      typedef double _pid_motor_right_type;
      _pid_motor_right_type st_pid_motor_right;
      _pid_motor_right_type * pid_motor_right;
      typedef double _theta_type;
      _theta_type theta;

    lowlevel():
      header(),
      speed_sp_length(0), st_speed_sp(), speed_sp(nullptr),
      speed_fb_length(0), st_speed_fb(), speed_fb(nullptr),
      pid_motor_left_length(0), st_pid_motor_left(), pid_motor_left(nullptr),
      pid_motor_right_length(0), st_pid_motor_right(), pid_motor_right(nullptr),
      theta(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->speed_sp_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->speed_sp_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->speed_sp_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->speed_sp_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speed_sp_length);
      for( uint32_t i = 0; i < speed_sp_length; i++){
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
      *(outbuffer + offset + 0) = (this->speed_fb_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->speed_fb_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->speed_fb_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->speed_fb_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speed_fb_length);
      for( uint32_t i = 0; i < speed_fb_length; i++){
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
      *(outbuffer + offset + 0) = (this->pid_motor_left_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pid_motor_left_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->pid_motor_left_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->pid_motor_left_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pid_motor_left_length);
      for( uint32_t i = 0; i < pid_motor_left_length; i++){
      union {
        double real;
        uint64_t base;
      } u_pid_motor_lefti;
      u_pid_motor_lefti.real = this->pid_motor_left[i];
      *(outbuffer + offset + 0) = (u_pid_motor_lefti.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pid_motor_lefti.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pid_motor_lefti.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pid_motor_lefti.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_pid_motor_lefti.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_pid_motor_lefti.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_pid_motor_lefti.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_pid_motor_lefti.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->pid_motor_left[i]);
      }
      *(outbuffer + offset + 0) = (this->pid_motor_right_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pid_motor_right_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->pid_motor_right_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->pid_motor_right_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pid_motor_right_length);
      for( uint32_t i = 0; i < pid_motor_right_length; i++){
      union {
        double real;
        uint64_t base;
      } u_pid_motor_righti;
      u_pid_motor_righti.real = this->pid_motor_right[i];
      *(outbuffer + offset + 0) = (u_pid_motor_righti.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pid_motor_righti.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pid_motor_righti.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pid_motor_righti.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_pid_motor_righti.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_pid_motor_righti.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_pid_motor_righti.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_pid_motor_righti.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->pid_motor_right[i]);
      }
      union {
        double real;
        uint64_t base;
      } u_theta;
      u_theta.real = this->theta;
      *(outbuffer + offset + 0) = (u_theta.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_theta.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_theta.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_theta.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_theta.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_theta.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_theta.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_theta.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->theta);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t speed_sp_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      speed_sp_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      speed_sp_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      speed_sp_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->speed_sp_length);
      if(speed_sp_lengthT > speed_sp_length)
        this->speed_sp = (double*)realloc(this->speed_sp, speed_sp_lengthT * sizeof(double));
      speed_sp_length = speed_sp_lengthT;
      for( uint32_t i = 0; i < speed_sp_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_speed_sp;
      u_st_speed_sp.base = 0;
      u_st_speed_sp.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_speed_sp.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_speed_sp.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_speed_sp.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_speed_sp.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_speed_sp.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_speed_sp.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_speed_sp.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_speed_sp = u_st_speed_sp.real;
      offset += sizeof(this->st_speed_sp);
        memcpy( &(this->speed_sp[i]), &(this->st_speed_sp), sizeof(double));
      }
      uint32_t speed_fb_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      speed_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      speed_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      speed_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->speed_fb_length);
      if(speed_fb_lengthT > speed_fb_length)
        this->speed_fb = (double*)realloc(this->speed_fb, speed_fb_lengthT * sizeof(double));
      speed_fb_length = speed_fb_lengthT;
      for( uint32_t i = 0; i < speed_fb_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_speed_fb;
      u_st_speed_fb.base = 0;
      u_st_speed_fb.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_speed_fb.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_speed_fb.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_speed_fb.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_speed_fb.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_speed_fb.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_speed_fb.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_speed_fb.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_speed_fb = u_st_speed_fb.real;
      offset += sizeof(this->st_speed_fb);
        memcpy( &(this->speed_fb[i]), &(this->st_speed_fb), sizeof(double));
      }
      uint32_t pid_motor_left_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      pid_motor_left_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      pid_motor_left_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      pid_motor_left_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->pid_motor_left_length);
      if(pid_motor_left_lengthT > pid_motor_left_length)
        this->pid_motor_left = (double*)realloc(this->pid_motor_left, pid_motor_left_lengthT * sizeof(double));
      pid_motor_left_length = pid_motor_left_lengthT;
      for( uint32_t i = 0; i < pid_motor_left_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_pid_motor_left;
      u_st_pid_motor_left.base = 0;
      u_st_pid_motor_left.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_pid_motor_left.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_pid_motor_left.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_pid_motor_left.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_pid_motor_left.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_pid_motor_left.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_pid_motor_left.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_pid_motor_left.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_pid_motor_left = u_st_pid_motor_left.real;
      offset += sizeof(this->st_pid_motor_left);
        memcpy( &(this->pid_motor_left[i]), &(this->st_pid_motor_left), sizeof(double));
      }
      uint32_t pid_motor_right_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      pid_motor_right_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      pid_motor_right_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      pid_motor_right_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->pid_motor_right_length);
      if(pid_motor_right_lengthT > pid_motor_right_length)
        this->pid_motor_right = (double*)realloc(this->pid_motor_right, pid_motor_right_lengthT * sizeof(double));
      pid_motor_right_length = pid_motor_right_lengthT;
      for( uint32_t i = 0; i < pid_motor_right_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_pid_motor_right;
      u_st_pid_motor_right.base = 0;
      u_st_pid_motor_right.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_pid_motor_right.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_pid_motor_right.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_pid_motor_right.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_pid_motor_right.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_pid_motor_right.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_pid_motor_right.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_pid_motor_right.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_pid_motor_right = u_st_pid_motor_right.real;
      offset += sizeof(this->st_pid_motor_right);
        memcpy( &(this->pid_motor_right[i]), &(this->st_pid_motor_right), sizeof(double));
      }
      union {
        double real;
        uint64_t base;
      } u_theta;
      u_theta.base = 0;
      u_theta.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_theta.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_theta.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_theta.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_theta.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_theta.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_theta.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_theta.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->theta = u_theta.real;
      offset += sizeof(this->theta);
     return offset;
    }

    virtual const char * getType() override { return "kbot_msgs/lowlevel"; };
    virtual const char * getMD5() override { return "de74122342f27ae6012a65d5b39ac449"; };

  };

}
#endif
