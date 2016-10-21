/*
 * Point.hpp
 *
 *  Created on: Oct 21, 2016
 *      Author: siri
 */

#ifndef POINT_HPP_
#define POINT_HPP_

class Point
{
public:
  //! Latitude (WGS-84).
  fp64_t lat;
  //! Longitude (WGS-84).
  fp64_t lon;
  //! Height (WGS-84).
  fp32_t z;
  // north-velocity (x_dot)
  fp32_t vx;
  // east-velocity (y_dot)
  fp32_t vy;
  // down-velocity (z_dot)
  fp32_t vz;

  fp32_t speed;

  Point(){
    lat = 0;
    lon = 0;
    z = 0;
    vx = vy = vz = 0;
    speed = 0;
  };

  Point(fp64_t lat_, fp64_t lon_, fp32_t z_) {
    this->lat = lat_;
    this->lon = lon_;
    this->z = z_;
    this->vx = 0;
    this->vy = 0;
    this->vz = 0;
    this->speed = 0;
  }

  Point(fp64_t lat_, fp64_t lon_, fp32_t z_, fp32_t vx_, fp32_t vy_, fp32_t vz_) {
    this->lat = lat_;
    this->lon = lon_;
    this->z = z_;
    this->vx = vx_;
    this->vy = vy_;
    this->vz = vz_;
    this->speed = sqrt(vx_*vx_+vy_*vy_+vz_*vz_);
  }
};



#endif /* POINT_HPP_ */
