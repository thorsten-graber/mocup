#ifndef _ROS_mocup_msgs_Gps_h
#define _ROS_mocup_msgs_Gps_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace mocup_msgs
{

  class Gps : public ros::Msg
  {
    public:
      std_msgs::Header header;
      float utc;
      float latitude;
      float longitude;
      float altitude;
      float v_n;
      float v_e;
      float v_d;
      float truecourse;
      float groundspeed;
      uint8_t signalquality;
      uint8_t numberofsatellites;
      float pdop;
      enum { NO_FIX =  0 };
      enum { DEAD_RECKONING_ONLY =  1 };
      enum { FIX_2D =  2 };
      enum { FIX_3D =  3 };
      enum { GPS_DEAD_RECKONING_COMBINED =  4 };
      enum { TIME_ONLY_FIX =  5 };

    Gps():
      header(),
      utc(0),
      latitude(0),
      longitude(0),
      altitude(0),
      v_n(0),
      v_e(0),
      v_d(0),
      truecourse(0),
      groundspeed(0),
      signalquality(0),
      numberofsatellites(0),
      pdop(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->utc);
      offset += serializeAvrFloat64(outbuffer + offset, this->latitude);
      offset += serializeAvrFloat64(outbuffer + offset, this->longitude);
      union {
        float real;
        uint32_t base;
      } u_altitude;
      u_altitude.real = this->altitude;
      *(outbuffer + offset + 0) = (u_altitude.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_altitude.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_altitude.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_altitude.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->altitude);
      union {
        float real;
        uint32_t base;
      } u_v_n;
      u_v_n.real = this->v_n;
      *(outbuffer + offset + 0) = (u_v_n.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_v_n.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_v_n.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_v_n.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->v_n);
      union {
        float real;
        uint32_t base;
      } u_v_e;
      u_v_e.real = this->v_e;
      *(outbuffer + offset + 0) = (u_v_e.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_v_e.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_v_e.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_v_e.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->v_e);
      union {
        float real;
        uint32_t base;
      } u_v_d;
      u_v_d.real = this->v_d;
      *(outbuffer + offset + 0) = (u_v_d.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_v_d.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_v_d.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_v_d.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->v_d);
      union {
        float real;
        uint32_t base;
      } u_truecourse;
      u_truecourse.real = this->truecourse;
      *(outbuffer + offset + 0) = (u_truecourse.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_truecourse.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_truecourse.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_truecourse.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->truecourse);
      union {
        float real;
        uint32_t base;
      } u_groundspeed;
      u_groundspeed.real = this->groundspeed;
      *(outbuffer + offset + 0) = (u_groundspeed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_groundspeed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_groundspeed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_groundspeed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->groundspeed);
      *(outbuffer + offset + 0) = (this->signalquality >> (8 * 0)) & 0xFF;
      offset += sizeof(this->signalquality);
      *(outbuffer + offset + 0) = (this->numberofsatellites >> (8 * 0)) & 0xFF;
      offset += sizeof(this->numberofsatellites);
      union {
        float real;
        uint32_t base;
      } u_pdop;
      u_pdop.real = this->pdop;
      *(outbuffer + offset + 0) = (u_pdop.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pdop.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pdop.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pdop.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pdop);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->utc));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->latitude));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->longitude));
      union {
        float real;
        uint32_t base;
      } u_altitude;
      u_altitude.base = 0;
      u_altitude.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_altitude.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_altitude.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_altitude.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->altitude = u_altitude.real;
      offset += sizeof(this->altitude);
      union {
        float real;
        uint32_t base;
      } u_v_n;
      u_v_n.base = 0;
      u_v_n.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_v_n.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_v_n.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_v_n.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->v_n = u_v_n.real;
      offset += sizeof(this->v_n);
      union {
        float real;
        uint32_t base;
      } u_v_e;
      u_v_e.base = 0;
      u_v_e.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_v_e.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_v_e.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_v_e.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->v_e = u_v_e.real;
      offset += sizeof(this->v_e);
      union {
        float real;
        uint32_t base;
      } u_v_d;
      u_v_d.base = 0;
      u_v_d.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_v_d.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_v_d.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_v_d.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->v_d = u_v_d.real;
      offset += sizeof(this->v_d);
      union {
        float real;
        uint32_t base;
      } u_truecourse;
      u_truecourse.base = 0;
      u_truecourse.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_truecourse.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_truecourse.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_truecourse.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->truecourse = u_truecourse.real;
      offset += sizeof(this->truecourse);
      union {
        float real;
        uint32_t base;
      } u_groundspeed;
      u_groundspeed.base = 0;
      u_groundspeed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_groundspeed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_groundspeed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_groundspeed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->groundspeed = u_groundspeed.real;
      offset += sizeof(this->groundspeed);
      this->signalquality =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->signalquality);
      this->numberofsatellites =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->numberofsatellites);
      union {
        float real;
        uint32_t base;
      } u_pdop;
      u_pdop.base = 0;
      u_pdop.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pdop.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pdop.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pdop.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pdop = u_pdop.real;
      offset += sizeof(this->pdop);
     return offset;
    }

    const char * getType(){ return "mocup_msgs/Gps"; };
    const char * getMD5(){ return "3f5e56232ece86600d7d2c2e1299259c"; };

  };

}
#endif