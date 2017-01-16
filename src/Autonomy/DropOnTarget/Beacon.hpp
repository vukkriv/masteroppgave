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

  fp64_t estimated_carp_error(DUNE::IMC::EstimatedStreamVelocity wind, DUNE::IMC::EstimatedState state, fp64_t time_to_drop, fp32_t dt, fp64_t counter_max,
      fp64_t opt_rads, fp64_t opt_points, Matrix weight_velocity, double weight_position, double glide_time)
  {
    // Put Coordinates in order
    DUNE::IMC::EstimatedState sim_state = state;
    WGS84::displace(sim_state.x, sim_state.y, sim_state.z, &sim_state.lat, &sim_state.lon, &sim_state.height);
    optimal_CARP(sim_state.height,wind,state,dt,counter_max,opt_rads,opt_points,weight_velocity,weight_position,glide_time);
    sim_state.x = 0.0; sim_state.y = 0.0; sim_state.z = 0.0;
    fp32_t current_distance = WGS84::distance(sim_state.lat,sim_state.lon,sim_state.z,CARP.lat,CARP.lon,CARP.z);
    fp32_t previous_distance;
    int counter = 0;
    // Simulate further journey
    dt = 0.1;
    do
    {
      previous_distance = current_distance;
      WGS84::displace(
          (sim_state.vx)*dt,
          (sim_state.vy)*dt,
          (sim_state.vz)*dt,
          &sim_state.lat,
          &sim_state.lon,
          &sim_state.height);
      current_distance = WGS84::distance(sim_state.lat,sim_state.lon,sim_state.height,CARP.lat,CARP.lon,CARP.z);
      counter += 1;
    }
    while(current_distance < previous_distance);

    printf("Estimated CARP error: counter = %d current distance = %f\n",counter, current_distance);

    return current_distance;
  }

  // Used to calculate the error of the dropped target, through simulation from the actual drop point.
  // Needs proper simulation with more realistic wind, e.g. turbulence.

  fp32_t calculate_carp_deviation(DUNE::IMC::EstimatedState closest_state, fp64_t carp_dev[])
  {
    printf("State: x = %f, y = %f, z = %f\n\tlat = %f lon = %f height = %f\n", closest_state.x,closest_state.y,closest_state.z,closest_state.lat,closest_state.lon,closest_state.height);
    WGS84::displace(closest_state.x,closest_state.y,closest_state.z,&closest_state.lat,&closest_state.lon,&closest_state.height);
    printf("State: x = %f, y = %f, z = %f\n\tlat = %f lon = %f height = %f\n", closest_state.x,closest_state.y,closest_state.z,closest_state.lat,closest_state.lon,closest_state.height);
    WGS84::displacement(CARP.lat, CARP.lon,CARP.z,closest_state.lat,closest_state.lon,closest_state.height,&carp_dev[0],&carp_dev[1],&carp_dev[2]);
    printf("State: x = %f, y = %f, z = %f\n\tlat = %f lon = %f height = %f\n", closest_state.x,closest_state.y,closest_state.z,closest_state.lat,closest_state.lon,closest_state.height);
    return WGS84::distance(CARP.lat,CARP.lon,CARP.z,closest_state.lat,closest_state.lon,closest_state.height);
  }

  fp64_t calculate_target_deviation(DUNE::IMC::EstimatedStreamVelocity wind_in, DUNE::IMC::EstimatedState state, fp64_t time_to_drop, fp32_t dt, int counter_max, fp64_t target_dev[], fp64_t carp_dev[])
  {
//    // Clear wind storage
//    std::ofstream ofs;
//    ofs.open("/home/siri/uavlab/results/wind.txt", std::ofstream::out | std::ofstream::trunc);
//    ofs.close();

    // Make Rotation matrix and calculatie wind
    double r_bn[] = {state.phi, state.theta, state.psi};
    // Body to NED rotation matrix
    DUNE::Math::Matrix R_bn =  DUNE::Math::Matrix(r_bn,3, 1).toQuaternion().toDCM();
    // NED to body Rotation matrix
    DUNE::Math::Matrix R_nb = transpose(R_bn);
    // Prepare body wind in NED frame
    double steady_wind[3];
    steady_wind[0] = wind_in.x;
    steady_wind[1] = wind_in.y;
    steady_wind[2] = wind_in.z;
    DUNE::Math::Matrix steady_wind_matrix = DUNE::Math::Matrix(steady_wind,3,1);
    DUNE::Math::Matrix wind_matrix;

    // Put coordinates in order
    fp64_t present_lat = state.lat;
    fp64_t present_lon = state.lon;
    fp32_t present_height = state.height;
    WGS84::displace(state.x,state.y,state.z, &present_lat,&present_lon,&present_height);
    Point drop_position;
    drop_position.lat = state.lat;
    drop_position.lon = state.lon;
    drop_position.z = state.height;
    WGS84::displace(state.x, state.y, state.z, &drop_position.lat, &drop_position.lon, &drop_position.z);

    // Simulate Drop position
    WGS84::displace( //Displacing forward (drop time) times speed because of delay
        (state.vx) * (time_to_drop),
        (state.vy) * (time_to_drop),
        (state.vz) * (time_to_drop),
        &drop_position.lat, &drop_position.lon, &drop_position.z);
    drop_position.vx = state.vx;
    drop_position.vy = state.vy;
    drop_position.vz = state.vz;

    fp64_t x = 0.0, y=0.0, z=drop_position.z, vx=drop_position.vx, vy=drop_position.vy, vz = drop_position.vz;
    int counter = 0;
    double speed = sqrt(vx*vx+vy*vy+vz*vz);
    WindSimulator.initialize(steady_wind_matrix,R_bn,z-target.z);
    wind_matrix = WindSimulator.update(z-target.z,speed,dt);

    // Simulate fall
    while(z > target.z and counter < counter_max)
    {
      counter ++;
      speed = sqrt(vx*vx+vy*vy+vz*vz);
      fp32_t v_rel_abs = sqrt(((vx - wind_matrix(0)) * (vx - wind_matrix(0))) + ((vy - wind_matrix(1)) * (vy - wind_matrix(1))) + ((vz - wind_matrix(2)) * (vz-wind_matrix(2))));
      x += vx*dt;
      y += vy*dt;
      z -= vz*dt;
      vx += -b / mass * (vx - wind_matrix(0)) * v_rel_abs * dt;
      vy += -b / mass * (vy - wind_matrix(1)) * v_rel_abs * dt;
      vz += (-b / mass * (vz - wind_matrix(2)) * v_rel_abs + g) * dt;
//      if (counter < 100000)
//        printf("Z: %f Target.z: %f Height for Wind: %f\n",z,target.z,z-target.z);
      wind_matrix = WindSimulator.update(z-target.z,speed,dt);
    }
    //time to reach ground
    estimated_hitpoint.lat = drop_position.lat;
    estimated_hitpoint.lon = drop_position.lon;
    estimated_hitpoint.z = target.z;

    WGS84::displace(x, y, &estimated_hitpoint.lat, &estimated_hitpoint.lon);
    WGS84::displacement(target.lat, target.lon,target.z,estimated_hitpoint.lat,estimated_hitpoint.lon,estimated_hitpoint.z,&target_dev[0],&target_dev[1],&target_dev[2]);
    WGS84::displacement(CARP.lat, CARP.lon,CARP.z,drop_position.lat,drop_position.lon,drop_position.z,&carp_dev[0],&carp_dev[1],&carp_dev[2]);

    fp64_t target_deviation = WGS84::distance(target.lat,target.lon,target.z,estimated_hitpoint.lat,estimated_hitpoint.lon,estimated_hitpoint.z);

    return target_deviation;
  }

  void calculate_CARP_speed(double speed, double release_height, DUNE::IMC::EstimatedStreamVelocity wind, fp32_t dt, int counter_max)
  {
    fp64_t x = 0, y = 0, z = 0, vx = 0, vy = 0, vz = 0, x_copy = 0, y_copy = 0;
    fp32_t wind_speed = sqrt(wind.x * wind.x + wind.y * wind.y);

    fp32_t wind_speed_release = sqrt(pow(wind.x,2)+pow(wind.y,2)+pow(wind.z,2));
    fp32_t wind_speed_height;// = m_windspeed_20_feet*pow((height_in*m_convertMeters2Feet)/20,(1/7));
    fp32_t wind_height[3];
    wind_height[0] = wind.x;
    wind_height[1] = wind.y;
    wind_height[2] = wind.z;

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
      wind_speed_height = wind_speed_release*pow(z/release_height,1/7);
      wind_height[0] = wind_speed_height/wind_speed_release*wind.x;
      wind_height[1] = wind_speed_height/wind_speed_release*wind.y;
      wind_height[2] = wind_speed_height/wind_speed_release*wind.z;
      fp32_t v_rel_abs = sqrt(((vx - wind_height[0]) * (vx - wind_height[0])) + ((vy - wind_height[1]) * (vy - wind_height[1])) + ((vz - wind_height[2]) * (vz-wind_height[2])));
      x += vx*dt;
      y += vy*dt;
      z -= vz*dt;
      vx += -b / mass * (vx - wind_height[0]) * v_rel_abs * dt;
      vy += -b / mass * (vy - wind_height[1]) * v_rel_abs * dt;
      vz += (-b / mass * (vz - wind_height[2]) * v_rel_abs + g) * dt;
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

    fp32_t wind_speed_release = sqrt(pow(wind.x,2)+pow(wind.y,2)+pow(wind.z,2));
    fp32_t wind_speed_height;// = m_windspeed_20_feet*pow((height_in*m_convertMeters2Feet)/20,(1/7));
    fp32_t wind_height[3];
    wind_height[0] = wind.x;
    wind_height[1] = wind.y;
    wind_height[2] = wind.z;

    fp64_t x = 0, y = 0, z = 0, x_copy = 0, y_copy = 0;
    z = release_height;

    int counter = 0;
    while(z > target.z and counter < counter_max)
    {
      counter ++;
      wind_speed_height = wind_speed_release*pow(z/release_height,1/7);
      wind_height[0] = wind_speed_height/wind_speed_release*wind.x;
      wind_height[1] = wind_speed_height/wind_speed_release*wind.y;
      wind_height[2] = wind_speed_height/wind_speed_release*wind.z;
      fp32_t v_rel_abs = sqrt(((vx - wind_height[0]) * (vx - wind_height[0])) + ((vy - wind_height[1]) * (vy - wind_height[1])) + ((vz - wind_height[2]) * (vz-wind_height[2])));
      x += vx*dt;
      y += vy*dt;
      z -= vz*dt;
      vx += -b / mass * (vx - wind_height[0]) * v_rel_abs * dt;
      vy += -b / mass * (vy - wind_height[1]) * v_rel_abs * dt;
      vz += (-b / mass * (vz - wind_height[2]) * v_rel_abs + g) * dt;
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
    double v_tot[] = {speed - glide_time*b , 0, 0};
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
