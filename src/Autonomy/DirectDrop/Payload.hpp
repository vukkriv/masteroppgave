/*
 * Payload.hpp
 *
 *  Created on: Feb 3, 2017
 *      Author: siri
 */

#ifndef PAYLOAD_HPP_
#define PAYLOAD_HPP_

#include <DUNE/DUNE.hpp>

using DUNE_NAMESPACES;

struct Point
{
  fp64_t lat;
  fp64_t lon;
  fp32_t height;
  fp64_t vx;
  fp64_t vy;
  fp64_t vz;
};

struct Circle
{
  fp64_t centre_lat;
  fp64_t centre_lon;
  fp32_t height;
  fp32_t radius;
};

struct State
{
  fp32_t x; //North position
  fp32_t y; //East position
  fp32_t z; //Down position
  fp32_t vx; //North linear velocity
  fp32_t vy; //East linear velocity
  fp32_t vz; //Down linear velocity
};

State operator*(State lhs, const fp32_t rhs)
{
  lhs.x*=rhs;
  lhs.y*=rhs;
  lhs.z*=rhs;
  lhs.vx*=rhs;
  lhs.vy*=rhs;
  lhs.vz*=rhs;
  return lhs;
}

State operator*(const fp32_t lhs, State rhs)
{
  rhs.x*=lhs;
  rhs.y*=lhs;
  rhs.z*=lhs;
  rhs.vx*=lhs;
  rhs.vy*=lhs;
  rhs.vz*=lhs;
  return rhs;
}

State operator+(State lhs, const State rhs)
{
  lhs.x += rhs.x;
  lhs.y += rhs.y;
  lhs.z += rhs.z;
  lhs.vx += rhs.vx;
  lhs.vy += rhs.vy;
  lhs.vz += rhs.vz;
  return lhs;
}

class Payload
{
private:
  Point received_target;
  Circle CARP_circle;
  fp32_t CARP_speed;
  Point estimated_hitpoint;
  Point release_state;
  fp32_t mass;
  fp32_t g;
  fp32_t rho;
  fp32_t C_D;
  fp32_t A;
  fp32_t dt;

  fp32_t glide_time;
  fp32_t drop_time;


public:
  Payload( void )
  {
    //rho = 1.341;       // air density at -10 degrees celsius, according to Simen Fuglaas
    rho = 1.246;         // air density at 10 degrees celsius, according to wikipedia
    C_D = 0.46;          // drag coefficient of our cone (Physics of Hockey, Haché 2002)
    A = 0.0036;          // Area of payload perpendicular to the wind (side of hockey puck, measured)
    mass = 0.312;               // mass in kilograms (measured)
    g = 9.81;                   // gravity constant
    dt = 0.01;                  // time step in seconds
    CARP_speed = 18.0;          // Speed of the UAV at the drop point
    glide_time = 0.0;
    drop_time = 0.0;
    dt = 0.01;
  }

  Payload( fp32_t glide_time_in, fp32_t drop_time_in, fp32_t dt_in )
  {
    //rho = 1.341;       // air density at -10 degrees celsius, according to Simen Fuglaas
    rho = 1.246;         // air density at 10 degrees celsius, according to wikipedia
    C_D = 0.46;          // drag coefficient of our cone (Physics of Hockey, Haché 2002)
    A = 0.0036;          // Area of payload perpendicular to the wind (side of hockey puck, measured)
    mass = 0.312;               // mass in kilograms (measured)
    g = 9.81;                   // gravity constant
    CARP_speed = 18.0;          // Speed of the UAV at the drop point
    glide_time = glide_time_in;
    drop_time = drop_time_in;
    dt = dt_in;
  }

  State fallingObjectFunction( State state, EstimatedStreamVelocity wind )
  {
    State state_dot;
    fp32_t V_a = sqrt(pow(state.vx-wind.x,2)+pow(state.vy-wind.y,2)+pow(state.vz-wind.z,2));
    state_dot.x = state.vx;
    state_dot.y = state.vy;
    state_dot.z = state.vz;
    state_dot.vx = -0.5/mass*C_D*A*rho*V_a*(state.vx-wind.x);
    state_dot.vy = -0.5/mass*C_D*A*rho*V_a*(state.vy-wind.y);
    state_dot.vz = -g-0.5/mass*C_D*A*rho*V_a*(state.vz-wind.z);
    return state_dot;
  }

  State RK4_fallingObject( State state_in, EstimatedStreamVelocity wind )
  {
    State k1,k2,k3,k4;
    k1 = fallingObjectFunction( state_in, wind );
    k2 = fallingObjectFunction( state_in + dt*0.5*k1, wind );
    k3 = fallingObjectFunction( state_in + dt*0.5*k2, wind );
    k4 = fallingObjectFunction( state_in + dt*k3, wind );
    return state_in + dt/6*(k1+2*k2+2*k3+k4);
  }

  Circle get_CARP_circle( void )
  {
    return CARP_circle;
  }

  fp32_t get_CARP_speed( void )
  {
    return CARP_speed;
  }

  fp32_t get_target_error( void )
  {
    return WGS84::distance(received_target.lat,received_target.lon,received_target.height,
        estimated_hitpoint.lat,estimated_hitpoint.lon,estimated_hitpoint.height);
  }

  void set_target( IMC::Target target_in)
  {
    received_target.lat = target_in.lat;
    received_target.lon = target_in.lon;
    received_target.height = target_in.z;
  }

  void load_target_error( fp32_t target_error_placekeeper[])
  {
    WGS84::displacement(received_target.lat,received_target.lon,received_target.height,
        estimated_hitpoint.lat,estimated_hitpoint.lon,estimated_hitpoint.height,
        &target_error_placekeeper[0],&target_error_placekeeper[1],&target_error_placekeeper[2]);
  }

  void load_CARP_centre_displacement( fp32_t placekeeper[] )
  {
    WGS84::displacement(received_target.lat,received_target.lon,received_target.height,
        CARP_circle.centre_lat,CARP_circle.centre_lon,CARP_circle.height,
        &placekeeper[0],&placekeeper[1],&placekeeper[2]);
  }

  void load_release_displacement( fp32_t placekeeper[] )
  {
    WGS84::displacement(received_target.lat,received_target.lon,received_target.height,
        release_state.lat,release_state.lon,release_state.height,
        &placekeeper[0],&placekeeper[1],&placekeeper[2]);
  }

  void calculate_CARP_circle( EstimatedStreamVelocity wind_in, fp32_t desired_speed, fp32_t drop_height)
  {
    CARP_speed = desired_speed;
//    printf("CARP_speed: %f\n", CARP_speed);
    CARP_circle.height = drop_height;
//    printf("CARP height: %f\n", CARP_circle.height);
    fp32_t wind_speed = sqrt(pow(wind_in.x,2)+pow(wind_in.y,2));

    //Assumption: Horizontal drop
    //Calculate point on the circle with the wind
    State falling_object_with_wind;
    falling_object_with_wind.x = 0.0;
    falling_object_with_wind.y = 0.0;
    falling_object_with_wind.z = CARP_circle.height;
    falling_object_with_wind.vx = wind_in.x/wind_speed*CARP_speed + wind_in.x;
    falling_object_with_wind.vy = wind_in.y/wind_speed*CARP_speed + wind_in.y;
    falling_object_with_wind.vz = 0.0;
//    printf("Start velocity: vx %f vy %f vz %f\n", falling_object_with_wind.vx, falling_object_with_wind.vy, falling_object_with_wind.vz);
    int counter = 0;
    while( falling_object_with_wind.z > received_target.height && counter < 10000)
    {
      counter ++;
      falling_object_with_wind = RK4_fallingObject( falling_object_with_wind, wind_in );
    }

    //calculate point on the circle against the wind
    State falling_object_against_wind;
    falling_object_against_wind.x = 0.0;
    falling_object_against_wind.y = 0.0;
    falling_object_against_wind.z = CARP_circle.height;
    falling_object_against_wind.vx = -wind_in.x/wind_speed*CARP_speed + wind_in.x;
    falling_object_against_wind.vy = -wind_in.y/wind_speed*CARP_speed + wind_in.y;
    falling_object_against_wind.vz = 0.0;
    counter = 0;
//    printf("Start velocity: vx %f vy %f vz %f\n", falling_object_against_wind.vx, falling_object_against_wind.vy, falling_object_against_wind.vz);
    while( falling_object_against_wind.z > received_target.height && counter < 10000)
    {
      counter ++;
      falling_object_against_wind = RK4_fallingObject( falling_object_against_wind, wind_in );
    }

    //calculate centre of circle and construct circle
    CARP_circle.centre_lat = received_target.lat;
    CARP_circle.centre_lon = received_target.lon;
    WGS84::displace((-falling_object_with_wind.x-falling_object_against_wind.x)/2,
        (-falling_object_with_wind.y-falling_object_against_wind.y)/2,&CARP_circle.centre_lat,&CARP_circle.centre_lon);

    //calculate the radius
    CARP_circle.radius = 0.5*sqrt(pow(-falling_object_with_wind.x + falling_object_against_wind.x,2)
                              + pow(-falling_object_with_wind.y + falling_object_against_wind.y,2));
  }

  void calculate_CARP_circle( EstimatedStreamVelocity wind_in, fp64_t speed, Point UAV)
    {
      //Assumption: Horizontal drop
      //Calculate point on the circle with the wind
      fp32_t wind_speed = sqrt(pow(wind_in.x,2)+pow(wind_in.y,2));
      CARP_circle.height = UAV.height;
      CARP_speed = speed;
      State falling_object_with_wind;
      falling_object_with_wind.x = 0.0;
      falling_object_with_wind.y = 0.0;
      falling_object_with_wind.z = UAV.height;
      falling_object_with_wind.vx = UAV.vx;
      falling_object_with_wind.vy = UAV.vy;
      falling_object_with_wind.vz = 0.0;
      int counter = 0;
      while( falling_object_with_wind.z > received_target.height && counter < 10000)
      {
        counter ++;
        falling_object_with_wind = RK4_fallingObject( falling_object_with_wind, wind_in );
      }

      //calculate point on the circle against the wind
      State falling_object_against_wind;
      falling_object_against_wind.x = 0.0;
      falling_object_against_wind.y = 0.0;
      falling_object_against_wind.z = UAV.height;
      falling_object_against_wind.vx = 2*wind_in.x-UAV.vx;
      falling_object_against_wind.vy = 2*wind_in.y-UAV.vy;
      falling_object_against_wind.vz = 0.0;
      counter = 0;
      while( falling_object_against_wind.z > received_target.height && counter < 10000)
      {
        counter ++;
        falling_object_against_wind = RK4_fallingObject( falling_object_against_wind, wind_in );
      }
      //calculate centre of circle and construct circle
      CARP_circle.centre_lat = received_target.lat;
      CARP_circle.centre_lon = received_target.lon;
      WGS84::displace((-falling_object_with_wind.x-falling_object_against_wind.x)/2,
          (-falling_object_with_wind.y-falling_object_against_wind.y)/2,&CARP_circle.centre_lat,&CARP_circle.centre_lon);

      //calculate the radius
      CARP_circle.radius = 0.5*sqrt(pow(-falling_object_with_wind.x + falling_object_against_wind.x,2)
                                + pow(-falling_object_with_wind.y + falling_object_against_wind.y,2));
    }

  void calculate_estimated_hitpoint( EstimatedStreamVelocity release_wind, Point release_state )
  {
    //Declaring and initializing variables
    this->release_state = release_state;
    State falling_object;
    falling_object.x = 0.0;
    falling_object.y = 0.0;
    falling_object.z = release_state.height;
    falling_object.vx = release_state.vx;
    falling_object.vy = release_state.vy;
    falling_object.vz = release_state.vz;

    //Make estimated hit point
    estimated_hitpoint.lat = release_state.lat;
    estimated_hitpoint.lon = release_state.lon;
    estimated_hitpoint.height = release_state.height;
    estimated_hitpoint.vx = release_state.vx;
    estimated_hitpoint.vy = release_state.vy;
    estimated_hitpoint.vz = release_state.vz;
    printf("Distance from drop position to CARP center: %f\n", WGS84::distance(estimated_hitpoint.lat,estimated_hitpoint.lon,estimated_hitpoint.height,
        CARP_circle.centre_lat,CARP_circle.centre_lon,CARP_circle.height));
    int counter = 0;
    while( falling_object.z > received_target.height && counter < 10000)
    {
      counter++;
      falling_object = RK4_fallingObject( falling_object, release_wind );
    }
    printf("Counter: %d\n", counter);
    WGS84::displace(falling_object.x,falling_object.y,&estimated_hitpoint.lat,&estimated_hitpoint.lon);
    estimated_hitpoint.height = falling_object.z;
//    printf("Falling object: x: %f, y: %f, z: %f. Target z: %f\n\n", falling_object.x,falling_object.y,falling_object.z,received_target.height);
//    printf("Distance between estimated hit point and target: %f\n",
//        WGS84::distance(received_target.lat,received_target.lon,received_target.height,estimated_hitpoint.lat,estimated_hitpoint.lon,estimated_hitpoint.height));
  }
};






#endif /* PAYLOAD_HPP_ */
