/*
 * Beacon.hpp
 *
 *  Created on: Oct 5, 2016
 *      Author: siri
 */

#ifndef BEACON_HPP_
#define BEACON_HPP_

#include <DUNE/DUNE.hpp>
#include "Dryden.hpp"
#include <limits.h>

struct BeaconPoint
{
  fp64_t lat;
  fp64_t lon;
  fp32_t z;
  fp64_t vx;
  fp64_t vy;
  fp64_t vz;
};


class Beacon
{

private:
  BeaconPoint target;
  Dryden WindSimulator;
  BeaconPoint CARP;
  BeaconPoint estimated_hitpoint;
  BeaconPoint release_point;
  fp32_t mass;
  fp32_t b;
  fp32_t g;
  fp32_t opt_rads;
  int opt_points;
  Matrix w_vel;
  fp32_t w_pos;
  fp32_t glide_time;
  fp32_t drop_time;
  fp32_t dt;
  int counter_max;
  fp64_t t;

public:
  Beacon()
  {
    //fp32_t rho = 1.341; // air density at -10 degrees celsius, according to Simen Fuglaas
    fp32_t rho = 1.246; // air density at 10 degrees celsius, according to wikipedia
    fp32_t C_D = 0.39;   // drag coefficient of our cone
    fp32_t r = 0.045;    // 0.045 meters in radius.
    fp32_t A = M_PI*r*r;
    b = 0.5*C_D*rho*A;  //damping constant
    mass = 0.104;         // mass
    g = 9.81;           //gravity constant
    CARP = BeaconPoint();
    WindSimulator = Dryden();
    t = 0;
    opt_rads = 1;
    opt_points = 1;
    w_vel = 1.0;
    w_pos = 1.0;
    glide_time = 1.0;
    drop_time = 1.0;
    counter_max = 10000;
    dt = 1.0;
  };
  Beacon(fp32_t opt_rads_in, int opt_points_in, Matrix w_vel_in, fp32_t w_pos_in, fp32_t glide_time_in, fp32_t drop_time_in, fp32_t dt_in)
  {   //THESE CONSTANTS MUST BE CHANGABLE IN INI FILE
      //fp32_t rho = 1.341; // air density at -10 degrees celsius, according to Simen Fuglaas
      fp32_t rho = 1.246; // air density at 10 degrees celsius, according to wikipedia
      fp32_t C_D = 0.39;   // drag coefficient of our cone
      fp32_t r = 0.045;    // 0.045 meters in radius.
      fp32_t A = M_PI*r*r;
      b = 0.5*C_D*rho*A;  //damping constant
      mass = 0.104;         // mass
      g = 9.81;           //gravity constant
      CARP = BeaconPoint();
      WindSimulator = Dryden();
      t = 0;
      opt_rads = opt_rads_in;
      opt_points = opt_points_in;
      w_vel = w_vel_in;
      w_pos = w_pos_in;
      glide_time = glide_time_in;
      drop_time = drop_time_in;
      counter_max = 1000;
      dt = dt_in;
  };

  BeaconPoint get_CARP()
  {
    return CARP;
  };

  void set_target(DUNE::IMC::Target target_in)
  {
    target.lat = target_in.lat;
    target.lon = target_in.lon;
    target.z = target_in.z;
  };

  void load_target_error(fp32_t target_error_placekeeper[])
  {
    WGS84::displacement(target.lat,target.lon,target.z,estimated_hitpoint.lat,estimated_hitpoint.lon,estimated_hitpoint.z,
        &target_error_placekeeper[0],&target_error_placekeeper[1],&target_error_placekeeper[2]);
  }

  void load_CARP_error(fp32_t CARP_error_placekeeper[])
  {
    WGS84::displacement(CARP.lat,CARP.lon,CARP.z,release_point.lat,release_point.lon,release_point.z,
        &CARP_error_placekeeper[0],&CARP_error_placekeeper[1],&CARP_error_placekeeper[2]);
  }

  fp64_t get_target_error( void )
  {
    return WGS84::distance(target.lat,target.lon,target.z,estimated_hitpoint.lat,estimated_hitpoint.lon,estimated_hitpoint.z);
  }

  fp64_t get_CARP_error( void )
  {
    return WGS84::distance(CARP.lat,CARP.lon,CARP.z,release_point.lat,release_point.lon,release_point.z);
  }

  void calculate_CARP(fp32_t vx, fp32_t vy, fp32_t vz, fp32_t release_height, DUNE::IMC::EstimatedStreamVelocity wind)
  {
    CARP.vx = vx;
    CARP.vy = vy;
    CARP.vz = vz;
    fp32_t wind_speed_release = sqrt(pow(wind.x,2)+pow(wind.y,2)+pow(wind.z,2));
    fp32_t wind_speed_height;
    fp32_t wind_height[3];
    wind_height[0] = wind.x;
    wind_height[1] = wind.y;
    wind_height[2] = wind.z;
    fp32_t speed_rel;

    fp32_t x = 0, y = 0;
    fp32_t z = release_height;

    int counter = 0;
    while(z > target.z and counter < counter_max)
    {
      counter ++;
      wind_speed_height = wind_speed_release*pow(z/release_height,1/7);
      wind_height[0] = wind_speed_height/wind_speed_release*wind.x;
      wind_height[1] = wind_speed_height/wind_speed_release*wind.y;
      wind_height[2] = wind_speed_height/wind_speed_release*wind.z;
      speed_rel = sqrt(((vx - wind_height[0]) * (vx - wind_height[0])) + ((vy - wind_height[1]) * (vy - wind_height[1])) + ((vz - wind_height[2]) * (vz-wind_height[2])));
      x += vx*dt;
      y += vy*dt;
      z -= vz*dt;
      vx += -b / mass * (vx - wind_height[0]) * speed_rel * dt;
      vy += -b / mass * (vy - wind_height[1]) * speed_rel * dt;
      vz += (-b / mass * (vz - wind_height[2]) * speed_rel + g) * dt;
    }

    //time to reach ground
    t = counter * dt;

    // Calculate position of release in (x,y)-plane
    CARP.lat = target.lat;
    CARP.lon = target.lon;

    WGS84::displace(-x, -y, &CARP.lat, &CARP.lon);

    CARP.z = release_height;
  };

  void optimal_CARP(double release_height, DUNE::IMC::EstimatedStreamVelocity wind, DUNE::IMC::EstimatedState estate)
  {

    fp64_t best_sum = 10000;
    fp64_t test_sum = 0;
    BeaconPoint optimal_carp;
    Matrix V_new;
    //Assuming horizontal velocity
    fp64_t v_now[] = {estate.vx, estate.vy, 0};
    Matrix V_now = Matrix(v_now, 3, 1);
    fp64_t speed = sqrt(estate.vx * estate.vx + estate.vy * estate.vy);
    //Speed minus speed lost while gliding (t*C_d*A*rho)
    fp64_t v_tot[] = {speed - glide_time*b , 0, 0};
    Matrix V_tot = Matrix(v_tot, 3, 1);

    fp64_t angle;

    //Get current lat, lon, height
    fp64_t lat = estate.lat, lon = estate.lon;
    fp32_t height = estate.height;
    WGS84::displace(estate.x, estate.y, estate.z, &lat, &lon, &height);

    fp64_t angle_to_target = atan2(target.lon - lon, target.lat - lat); //lat lon switch?
    fp64_t ned_speed_angle = atan2(estate.vy, estate.vx);

    //Turn v_body about z axis for accurate weighing (mirror about angle to target)
    fp64_t a = 2*(angle_to_target - ned_speed_angle);
    fp64_t rot_v_about_z[] = {cos(a), -sin(a), 0, sin(a),
        cos(2*(angle_to_target + ned_speed_angle)), 0, 0, 0, 1};
    V_now = Matrix(rot_v_about_z,3,3) * V_now ;

    //Optimizing over all optimization points
    for(int i = -1 * floor(opt_points / 2.0); i < ceil(opt_points / 2.0); i++)
    {
      //Rotate into currently tested CARP velocity
      angle = (angle_to_target) + i * opt_rads / opt_points;
      fp64_t rotz[] = {cos(angle), -sin(angle), 0, sin(angle), cos(angle), 0, 0, 0, 1};
      V_new = Matrix(rotz, 3, 3) * V_tot;
      calculate_CARP(V_new.element(0,0), V_new.element(1,0), V_new.element(2,0), release_height, wind);

      //Test if currently best
      test_sum =   (w_vel * abs(V_new - V_now)).element(0, 0) + w_pos *  WGS84::distance(CARP.lat,CARP.lon  , release_height, lat,lon, release_height);

      if(test_sum < best_sum){
        best_sum = test_sum;
        optimal_carp = CARP;
      }
    }
    CARP = optimal_carp;
  };

  void set_release_point(IMC::EstimatedState activate_drop_state)
  {
    release_point.lat = activate_drop_state.lat;
    release_point.lon = activate_drop_state.lon;
    release_point.z = activate_drop_state.height;
    release_point.vx = activate_drop_state.vx;
    release_point.vy = activate_drop_state.vy;
    release_point.vz = activate_drop_state.vz;
    WGS84::displace(activate_drop_state.x + activate_drop_state.vx*drop_time,
        activate_drop_state.y + activate_drop_state.vy*drop_time,
        activate_drop_state.z + activate_drop_state.vz*drop_time,
                    &release_point.lat,&release_point.lon,&release_point.z);
  }

  void calculate_estimated_hitpoint(IMC::EstimatedState state, IMC::EstimatedStreamVelocity wind)
  {
    // Make Rotation matrix and calculatie wind
    fp64_t r_bn[] = {state.phi,
                    state.theta,
                    state.psi};
    // Body to NED rotation matrix
    Matrix R_bn = Matrix(r_bn,3, 1).toQuaternion().toDCM();
    // NED to body Rotation matrix
    Matrix R_nb = transpose(R_bn);
    // Prepare body wind in NED frame
    fp64_t steady_wind[3];
    steady_wind[0] = wind.x;
    steady_wind[1] = wind.y;
    steady_wind[2] = wind.z;
    Matrix steady_wind_matrix = Matrix(steady_wind,3,1);
    Matrix wind_matrix;

    estimated_hitpoint = release_point;

    fp64_t x = 0.0, y=0.0, z=release_point.z;

    int counter = 0;
    fp64_t speed = sqrt(estimated_hitpoint.vx*estimated_hitpoint.vx
                        +estimated_hitpoint.vy*estimated_hitpoint.vy
                        +estimated_hitpoint.vz*estimated_hitpoint.vz);
    WindSimulator.initialize(steady_wind_matrix,R_bn,state.height);
    wind_matrix = WindSimulator.update(speed,dt);

    // Simulate fall
    while(z > target.z and counter < counter_max)
    {
      counter ++;
      speed = sqrt(estimated_hitpoint.vx*estimated_hitpoint.vx
                  +estimated_hitpoint.vy*estimated_hitpoint.vy
                  +estimated_hitpoint.vz*estimated_hitpoint.vz);
      fp32_t v_rel_abs = sqrt(((estimated_hitpoint.vx - wind_matrix(0)) * (estimated_hitpoint.vx - wind_matrix(0)))
                      + ((estimated_hitpoint.vy - wind_matrix(1)) * (estimated_hitpoint.vy - wind_matrix(1)))
                      + ((estimated_hitpoint.vz - wind_matrix(2)) * (estimated_hitpoint.vz-wind_matrix(2))));
      x += estimated_hitpoint.vx*dt;
      y += estimated_hitpoint.vy*dt;
      z -= estimated_hitpoint.vz*dt;
      estimated_hitpoint.vx += -b / mass * (estimated_hitpoint.vx - wind_matrix(0)) * v_rel_abs * dt;
      estimated_hitpoint.vy += -b / mass * (estimated_hitpoint.vy - wind_matrix(1)) * v_rel_abs * dt;
      estimated_hitpoint.vz += (-b / mass * (estimated_hitpoint.vz - wind_matrix(2)) * v_rel_abs + g) * dt;
      wind_matrix = WindSimulator.update(speed,dt);
    }

    estimated_hitpoint.z = target.z;
    WGS84::displace(x, y, &estimated_hitpoint.lat, &estimated_hitpoint.lon);
  }

  fp64_t estimated_carp_error(DUNE::IMC::EstimatedStreamVelocity wind, DUNE::IMC::EstimatedState state)
  {
    // Put Coordinates in order
    DUNE::IMC::EstimatedState sim_state = state;
    WGS84::displace(sim_state.x, sim_state.y, sim_state.z, &sim_state.lat, &sim_state.lon, &sim_state.height);
    optimal_CARP(sim_state.height,wind,state);
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

    printf("Estimated CARP error: current distance = %f\n",current_distance);
    return current_distance;
  }


//  fp64_t calculate_target_deviation(DUNE::IMC::EstimatedStreamVelocity wind_in, DUNE::IMC::EstimatedState state, fp64_t target_dev[], fp64_t carp_dev[])
//  {
//    // Clear wind storage
//    std::ofstream ofs;
//    ofs.open("/home/siri/uavlab/results/wind.txt", std::ofstream::out | std::ofstream::trunc);
//    ofs.close();
//
//    // Make Rotation matrix and calculatie wind
//    double r_bn[] = {state.phi, state.theta, state.psi};
//    // Body to NED rotation matrix
//    DUNE::Math::Matrix R_bn =  DUNE::Math::Matrix(r_bn,3, 1).toQuaternion().toDCM();
//    // NED to body Rotation matrix
//    DUNE::Math::Matrix R_nb = transpose(R_bn);
//    // Prepare body wind in NED frame
//    double steady_wind[3];
//    steady_wind[0] = wind_in.x;
//    steady_wind[1] = wind_in.y;
//    steady_wind[2] = wind_in.z;
//    DUNE::Math::Matrix steady_wind_matrix = DUNE::Math::Matrix(steady_wind,3,1);
//    DUNE::Math::Matrix wind_matrix;
//
//    // Put coordinates in order
//    fp64_t present_lat = state.lat;
//    fp64_t present_lon = state.lon;
//    fp32_t present_height = state.height;
//    WGS84::displace(state.x,state.y,state.z, &present_lat,&present_lon,&present_height);
//    Point drop_position;
//    drop_position.lat = state.lat;
//    drop_position.lon = state.lon;
//    drop_position.z = state.height;
//    WGS84::displace(state.x, state.y, state.z, &drop_position.lat, &drop_position.lon, &drop_position.z);
//
//    // Simulate Drop position
//    WGS84::displace( //Displacing forward (drop time) times speed because of delay
//        (state.vx) * (drop_time),
//        (state.vy) * (drop_time),
//        (state.vz) * (drop_time),
//        &drop_position.lat, &drop_position.lon, &drop_position.z);
//    drop_position.vx = state.vx;
//    drop_position.vy = state.vy;
//    drop_position.vz = state.vz;
//
//    fp64_t x = 0.0, y=0.0, z=drop_position.z, vx=drop_position.vx, vy=drop_position.vy, vz = drop_position.vz;
//    int counter = 0;
//    double speed = sqrt(vx*vx+vy*vy+vz*vz);
//    WindSimulator.initialize(steady_wind_matrix,R_bn,state.height);
//    wind_matrix = WindSimulator.update(speed,dt);
//
//    // Simulate fall
//    while(z > target.z and counter < counter_max)
//    {
//      counter ++;
//      speed = sqrt(vx*vx+vy*vy+vz*vz);
//      fp32_t v_rel_abs = sqrt(((vx - wind_matrix(0)) * (vx - wind_matrix(0))) + ((vy - wind_matrix(1)) * (vy - wind_matrix(1))) + ((vz - wind_matrix(2)) * (vz-wind_matrix(2))));
//      x += vx*dt;
//      y += vy*dt;
//      z -= vz*dt;
//      vx += -b / mass * (vx - wind_matrix(0)) * v_rel_abs * dt;
//      vy += -b / mass * (vy - wind_matrix(1)) * v_rel_abs * dt;
//      vz += (-b / mass * (vz - wind_matrix(2)) * v_rel_abs + g) * dt;
//      wind_matrix = WindSimulator.update(speed,dt);
//    }
//    //time to reach ground
//    estimated_hitpoint.lat = drop_position.lat;
//    estimated_hitpoint.lon = drop_position.lon;
//    estimated_hitpoint.z = target.z;
//
//    WGS84::displace(x, y, &estimated_hitpoint.lat, &estimated_hitpoint.lon);
//    WGS84::displacement(target.lat, target.lon,target.z,estimated_hitpoint.lat,estimated_hitpoint.lon,estimated_hitpoint.z,&target_dev[0],&target_dev[1],&target_dev[2]);
//    WGS84::displacement(CARP.lat, CARP.lon,CARP.z,drop_position.lat,drop_position.lon,drop_position.z,&carp_dev[0],&carp_dev[1],&carp_dev[2]);
//
//    fp64_t target_deviation = WGS84::distance(target.lat,target.lon,target.z,estimated_hitpoint.lat,estimated_hitpoint.lon,estimated_hitpoint.z);
//
//    return target_deviation;
//  }
};



#endif /* BEACON_HPP_ */
