/*
 * Beacon.hpp
 *
 *  Created on: Oct 5, 2016
 *      Author: siri
 */

#ifndef BEACON_HPP_
#define BEACON_HPP_

#include <DUNE/DUNE.hpp>
#include "Point.hpp"
#include "Dryden.hpp"
#include <limits.h>

class Beacon
{

private:
  DUNE::IMC::Target target;
  Dryden WindSimulator;
  Point CARP;
  Point estimated_hitpoint;
  fp32_t mass;
  fp32_t b;
  fp32_t g;
  double t;

public:
  Beacon()
  {   //THESE CONSTANTS MUST BE CHANGABLE IN INI FILE
      //fp32_t rho = 1.341; // air density at -10 degrees celsius, according to Simen Fuglaas
      fp32_t rho = 1.246; // air density at 10 degrees celsius, according to wikipedia
      fp32_t C_D = 0.39;   // drag coefficient of our cone
      fp32_t r = 0.045;    // 0.045 meters in radius.
      fp32_t A = M_PI*r*r;
      b = 0.5*C_D*rho*A;  //damping constant
      mass = 0.104;         // mass
      g = 9.81;           //gravity constant
      CARP = Point();
      WindSimulator = Dryden();
      t = 0;
  };

  // Used to calculate the error of the dropped target, through simulation from the actual drop point.
  // Needs proper simulation with more realistic wind, e.g. turbulence.
  fp64_t calculate_target_deviation(DUNE::IMC::EstimatedState state, fp32_t dt, int counter_max)
  {
    double r_bn[] = {state.phi, state.theta, state.psi};
    // Body to NED rotation matrix
    DUNE::Math::Matrix R_bn =  DUNE::Math::Matrix(r_bn,3, 1).toQuaternion().toDCM();
    // NED to body Rotation matrix
    DUNE::Math::Matrix R_nb = transpose(R_bn);
    // Prepare body wind in NED frame
//    WindSimulator.setSteadyWind( steady_wind );
    double steady_wind[] = {3.0,-2.0,0.0};
    double gust_wind[] = {0.0,0.0,0.0};
    double wind[] = {0.0, 0.0, 0.0};
    DUNE::Math::Matrix steady_wind_matrix = DUNE::Math::Matrix(steady_wind,3,1);
    DUNE::Math::Matrix gust_wind_matrix = DUNE::Math::Matrix(gust_wind,3,1);
    DUNE::Math::Matrix wind_matrix = steady_wind_matrix + R_bn*gust_wind_matrix;
    WindSimulator.initialize(steady_wind_matrix,gust_wind_matrix,R_bn);

    fp64_t x = state.x, y=state.y, z=state.height - state.z, vx=state.vx, vy=state.vy, vz = state.vz;
    int counter = 0;
    while(z > target.z and counter < counter_max)
    {
      counter ++;
      double speed = sqrt(vx*vx+vy*vy+vz*vz);
      wind_matrix = WindSimulator.update(z,speed,dt);
      fp32_t v_rel_abs = sqrt(((vx - wind[0]) * (vx - wind[0])) + ((vy - wind[1]) * (vy - wind[1])) + ((vz - wind[2]) * (vz-wind[2])));
      x += vx*dt;
      y += vy*dt;
      z -= vz*dt;
      vx += -b / mass * (vx - wind_matrix(0)) * v_rel_abs * dt;
      vy += -b / mass * (vy - wind_matrix(1)) * v_rel_abs * dt;
      vz += (-b / mass * (vz - wind_matrix(2)) * v_rel_abs + g) * dt;
    }
    //time to reach ground
    estimated_hitpoint.lat = state.lat;
    estimated_hitpoint.lon = state.lon;
    estimated_hitpoint.z = target.z;
    WGS84::displace(x, y, &estimated_hitpoint.lat, &estimated_hitpoint.lon);

    fp64_t target_deviation = WGS84::distance(target.lat,target.lon,target.z,estimated_hitpoint.lat,estimated_hitpoint.lon,estimated_hitpoint.z);

    return target_deviation;
  }

  void calculate_CARP_speed(double speed, double release_height, DUNE::IMC::EstimatedStreamVelocity wind, fp32_t dt, int counter_max)
  {
    fp64_t x = 0, y = 0, z = 0, vx = 0, vy = 0, vz = 0, x_copy = 0, y_copy = 0;
    fp32_t wind_speed = sqrt(wind.x * wind.x + wind.y * wind.y);
    vx = -wind.x / wind_speed * speed;
    vy = -wind.y / wind_speed * speed;
    vz = 0;
    x_copy = x;
    y_copy = y;
    z = release_height;
    int counter = 0;
    while(z > target.z and counter < counter_max)
    {
      counter ++;
      fp32_t v_rel_abs = sqrt(((vx - wind.x) * (vx - wind.x)) + ((vy - wind.y) * (vy - wind.y)) + ((vz - wind.z) * (vz-wind.z)));
      x += vx*dt;
      y += vy*dt;
      z -= vz*dt;
      vx += -b / mass * (vx - wind.x) * v_rel_abs * dt;
      vy += -b / mass * (vy - wind.y) * v_rel_abs * dt;
      vz += (-b / mass * (vz - wind.z) * v_rel_abs + g) * dt;
    }
    //time to reach ground
    t = counter * dt;
    // Calculate position of release in (x,y)-plane
    double dx = x - x_copy;
    double dy = y - y_copy;
    CARP.speed = speed;
    CARP.lat = target.lat;
    CARP.lon = target.lon;

    WGS84::displace(-dx,-dy, &CARP.lat, &CARP.lon);

    CARP.z = target.z + release_height;
    CARP.vx = vx / wind_speed * speed;
    CARP.vy = vy / wind_speed * speed;
    CARP.vz = 0;
  };

  void calculate_CARP_velocity(fp64_t vx, fp64_t vy, fp64_t vz, double release_height, DUNE::IMC::EstimatedStreamVelocity wind, fp32_t dt, int counter_max)
  {
    CARP.speed = sqrt(vx * vx + vy * vy + vz * vz);
    CARP.vx = vx;
    CARP.vy = vy;
    CARP.vz = 0;

    fp64_t x = 0, y = 0, z = 0, x_copy = 0, y_copy = 0;
    z = release_height;

    int counter = 0;
    while(z > target.z and counter < counter_max)
    {
      counter ++;
      fp32_t v_rel_abs = sqrt(((vx - wind.x) * (vx - wind.x)) + ((vy - wind.y) * (vy - wind.y)) + ((vz - wind.z) * (vz - wind.z)));
      x += vx * dt;
      y += vy * dt;
      z -= vz * dt;
      vx += -b / mass * (vx - wind.x) * v_rel_abs* dt;
      vy += -b / mass * (vy - wind.y) * v_rel_abs * dt;
      vz += (-b / mass * (vz - wind.z) * v_rel_abs + g) * dt;
    }

    //time to reach ground
    t = counter * dt;

    // Calculate position of release in (x,y)-plane
    double dx = x - x_copy;
    double dy = y - y_copy;
    CARP.lat = target.lat;
    CARP.lon = target.lon;

    WGS84::displace(-dx, -dy, &CARP.lat, &CARP.lon);

    CARP.z = release_height;
  };

  void optimal_CARP(double release_height, DUNE::IMC::EstimatedStreamVelocity wind, DUNE::IMC::EstimatedState estate, fp32_t dt,
      int counter_max, double rad_of_circle, int num_of_points, Matrix weight_velocity, double weight_position, double glide_time)
  {
    double best_sum = 10000;
    double test_sum = 0;
    Point optimal_carp;
    Matrix V_new;
    double v_now[] = {estate.vx, estate.vy, 0};
    Matrix V_now = Matrix(v_now, 3, 1);
    double speed = sqrt(estate.vx * estate.vx + estate.vy * estate.vy);
    //Speed minus speed lost while gliding (t*C_d*A*rho)
    double v_tot[] = {speed - glide_time * .1 * (2 * .10 * 3.14 + .80 * 2 * 2) * 1.225 , 0, 0};
    Matrix V_tot = Matrix(v_tot, 3, 1);

    double angle;
    double lat = estate.lat, lon = estate.lon, height = estate.height;
    WGS84::displace(estate.x, estate.y, estate.z, &lat, &lon, &height);

    double angle_to_target = atan2(target.lon - lon, target.lat - lat); //lat lon switch?
    double ned_speed_angle = atan2(estate.vy, estate.vx);

    //Turn v_body about z axis for accurate weighing (mirror about angle to target)
    double a = 2*(angle_to_target - ned_speed_angle);
    double rot_v_about_z[] = {cos(a), -sin(a), 0, sin(a),
        cos(2*(angle_to_target + ned_speed_angle)), 0, 0, 0, 1};
    V_now = Matrix(rot_v_about_z,3,3) * V_now ;

    for(int i = -1 * floor(num_of_points / 2.0); i < ceil(num_of_points / 2.0); i++){
      //Rotate into currently tested CARP velocity
      angle = (angle_to_target) + i * rad_of_circle / num_of_points;
      double rotz[] = {cos(angle), -sin(angle), 0, sin(angle), cos(angle), 0, 0, 0, 1};
      V_new = Matrix(rotz, 3, 3) * V_tot;
      calculate_CARP_velocity(V_new.element(0,0), V_new.element(1,0), V_new.element(2,0), release_height, wind, dt, counter_max);

      //Test if currently best
      test_sum =   (weight_velocity * abs(V_new - V_now)).element(0, 0) + weight_position *  WGS84::distance(CARP.lat,CARP.lon  , release_height, lat,lon, release_height);

      if(test_sum < best_sum){
        best_sum = test_sum;
        optimal_carp = CARP;
      }
    }
    CARP = optimal_carp;
  };

  Point get_CARP()
  {
    return CARP;
  };
  double get_t(){
    return t;
  };
  DUNE::IMC::Target get_target()
  {
    return target;
  };
  void set_CARP(Point CARP_)
  {
    this->CARP = CARP_;
  };
  void set_target(DUNE::IMC::Target target_)
  {
    this->target = target_;
  };

  fp64_t get_velocity_size_deviation(fp64_t speed)
  {
    printf("Speed: %f, Carp speed: %f\n", speed, CARP.speed);
    return (speed - CARP.speed);
  }
  fp64_t get_2D_velocity_angle_deviation(DUNE::IMC::EstimatedState state)
  {
    fp64_t ideal_angle = atan2(CARP.vy,CARP.vx);
    fp64_t real_angle = atan2(state.vy,state.vx);
    return ideal_angle - real_angle;
  }
};



#endif /* BEACON_HPP_ */
