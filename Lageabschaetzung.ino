#ifndef LAGEABSCHAETZUNG_H
#define LAGEABSCHAETZUNG_H

#include <SerialUSB.h>

#include "Pico_comm.h"

class Lageabschaetzung
{
  private:
    double angle_cam_deg;
    uint16_t angle_cam_minutes;
    unsigned long pre_interval;

    double COMP_FILTER_FACTOR;

    bool heading_initialized;
    
    double get_angle_diff(double angle_gyro, double angle_cam);

  public:
    double heading_deg;

    double get_heading_deg(double gyro_z);
    void parse_cam_angle_packet(uint8_t *PL);

    void set_angle_cam_deg(double angle_cam_deg);
    double get_angle_cam_deg();

    void set_comp_filter_factor(double factor);

    Lageabschaetzung();
    ~Lageabschaetzung();
};

#endif
