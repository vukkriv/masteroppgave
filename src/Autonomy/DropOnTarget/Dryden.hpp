/*
 * Dryden.hpp
 *
 *  Created on: Nov 16, 2016
 *      Author: siri
 */

#ifndef DRYDEN_HPP_
#define DRYDEN_HPP_

#include <DUNE/DUNE.hpp>

using DUNE_NAMESPACES;

class Dryden
{
  // All sizes should use imperial units
private:
  // Steady wind in feet/s in NED frame
  Matrix m_steady_wind_matrix;
  // Gust wind in feet/s in Body frame
  Matrix m_gust_wind_matrix;
  // Rotation matrix Body to NED frame
  Matrix m_R_bn;
  // White Noise
  Matrix m_white_noise_matrix;
  //Random Number Generator
  Random::Generator* gen;
  // height in feet
  fp32_t m_height;
  // wind speed in feet/s
  fp32_t m_windspeed_20_feet;
  // vehicle speed in feet/s
  fp32_t m_speed;
  fp32_t m_L[3];
  fp32_t m_sigma_w;
  fp32_t m_sigma_u;
  fp32_t m_sigma_v;
  fp32_t m_convertMeters2Feet;
  Matrix m_A;
  Matrix m_B;
  Matrix m_C;
  Matrix m_Y;

public:
  Dryden()
  {
    srand(time(0));
    m_convertMeters2Feet = 3.2808399;
    double steady_wind[] = {0.0,0.0,0.0};
    m_steady_wind_matrix = Matrix(steady_wind,3,1);
    double gust_wind[] = {0.0,0.0,0.0};
    m_gust_wind_matrix = Matrix(gust_wind,3,1);
    double y[] = {0.0,0.0,0.0,0.0,0.0};
    m_Y = Matrix(y,5,1);
    double euler_angles[] = {0.0,0.0,0.0};
    m_R_bn = Matrix(euler_angles,3,1).toQuaternion().toDCM();
    m_L[0] = m_height/pow(0.177+0.000823*m_height,1.2);
    m_L[1] = m_height/pow(0.177+0.000823*m_height,1.2);
    m_L[2] = m_height;
    double white_noise[] = {0.0,0.0,0.0};
    m_white_noise_matrix = Matrix(white_noise,3,1);
    m_speed = 17.0*m_convertMeters2Feet;
    m_height = 100*m_convertMeters2Feet;
    m_windspeed_20_feet = sqrt(pow(steady_wind[0],2)+pow(steady_wind[1],2)+pow(steady_wind[2],2));
    m_sigma_w = 0.1*m_windspeed_20_feet;
    m_sigma_u = m_sigma_w/pow((0.177+0.000823*m_height),0.4);
    m_sigma_v = m_sigma_w/pow((0.177+0.000823*m_height),0.4);
  }

  double gaussian(double mu, double sigma)
  {
    double x, y, r2;
    do
    {
      x = -1 + 2 * double(rand()/RAND_MAX);
      y = -1 + 2 * double(rand()/RAND_MAX);
      r2 = x * x + y * y;
    }
    while (r2 > 1.0 || r2 == 0);

    double ans = y * std::sqrt(-2.0 * std::log(r2) / r2);
    return mu + sigma*ans;
  }

  void initialize( Matrix steady_wind_in, Matrix gust_wind_in, Matrix rotation_matrix )
  {
    m_steady_wind_matrix = steady_wind_in;
    m_windspeed_20_feet = sqrt(pow(m_steady_wind_matrix(0),2)+pow(m_steady_wind_matrix(1),2)+pow(m_steady_wind_matrix(2),2));
//    printf("Steady wind: %f %f %f\n",m_steady_wind_matrix(0),m_steady_wind_matrix(1),m_steady_wind_matrix(2));
    m_gust_wind_matrix = gust_wind_in;
//    printf("gust_wind wind: %f %f %f\n",m_gust_wind_matrix(0),m_gust_wind_matrix(1),m_gust_wind_matrix(2));
    m_R_bn = rotation_matrix;
//    printf("rotation_matrix: %f %f %f"
//                            "%f %f %f"
//                            "%f %f %f\n",rotation_matrix(0),rotation_matrix(1),rotation_matrix(2),rotation_matrix(3),rotation_matrix(4),rotation_matrix(5),rotation_matrix(6),rotation_matrix(7),rotation_matrix(8));
    gen = Random::Factory::create(Random::Factory::c_default, -1);
  }

  Matrix update(fp32_t height_in, fp32_t speed_in,fp32_t DT)
  {
    m_height = height_in*m_convertMeters2Feet;
    m_speed = speed_in*m_convertMeters2Feet;
    m_L[0] = m_height/pow(0.177+0.000823*m_height,1.2);
    m_L[1] = m_height/pow(0.177+0.000823*m_height,1.2);
    m_L[2] = m_height;
    double white_noise[] = {0.1*gen->gaussian(),0.1*gen->gaussian(),0.1*gen->gaussian()};
//    printf("White Noise: %f %f %f\n", white_noise[0],white_noise[1],white_noise[2]);
    m_white_noise_matrix = Matrix(white_noise,3,1);
    m_sigma_w = 0.1*m_windspeed_20_feet;
    m_sigma_u = m_sigma_w/pow((0.177+0.000823*m_height),0.4);
    m_sigma_v = m_sigma_w/pow((0.177+0.000823*m_height),0.4);
    double a[] = {-m_speed/m_L[0],0.0,            0.0,                      0.0,            0.0,
                  0.0,        -(2*m_speed)/m_L[1],-pow(m_speed,2)/pow(m_L[1],2),0.0,            0.0,
                  0.0,        1.0,            0.0,                      0.0,            0.0,
                  0.0,        0.0,            0.0,                      -(2*m_speed)/m_L[2],-pow(m_speed,2)/pow(m_L[2],2),
                  0.0,        0.0,            0.0,                      1.0,            0.0};
    m_A = Matrix(a,5,5);
    double b[] = {1.0,  0.0,  0.0,
                  0.0,  1.0,  0.0,
                  0.0,  0.0,  0.0,
                  0.0,  0.0,  1.0,
                  0.0,  0.0,  0.0};
    m_B = Matrix(b,5,3);
    Matrix y1 = m_A*m_Y + m_B*m_white_noise_matrix;
    Matrix y2 = m_A*(m_Y+DT/2*y1) + m_B*m_white_noise_matrix;
    Matrix y3 = m_A*(m_Y+DT/2*y2) + m_B*m_white_noise_matrix;
    Matrix y4 = m_A*(m_Y+DT*y3) + m_B*m_white_noise_matrix;
    m_Y = m_Y + DT/6*(y1 + 2*y2 + 2*y3 + y4);
    double c[] = {(sqrt(2)*m_speed*m_sigma_u*sqrt(m_L[0]/m_speed))/(m_L[0]*sqrt(M_PI)), 0.0,  0.0,  0.0,  0.0,
                  0.0,  (sqrt(3.0)*m_speed*m_sigma_v*sqrt(m_L[1]/m_speed)), (pow(m_speed,2.0)*m_sigma_v*sqrt(m_L[1]/m_speed))/(pow(m_L[1],2)*sqrt(M_PI)),0.0,0.0,
                  0.0,  0.0,  0.0,  (sqrt(3.0)*m_speed*m_sigma_w*sqrt(m_L[2]/m_speed))/(m_L[2]*sqrt(M_PI)),(pow(m_speed,2)*m_sigma_w*sqrt(m_L[2]/m_speed))/(pow(m_L[2],2)*sqrt(M_PI))};
    m_C = Matrix(c,3,5);
    m_gust_wind_matrix = m_C*m_Y;
//    printf("Gust wind matrix: %f, %f, %f\n",m_gust_wind_matrix(0),m_gust_wind_matrix(1),m_gust_wind_matrix(2));
    return m_steady_wind_matrix + m_R_bn*m_gust_wind_matrix;
  }
};



#endif /* DRYDEN_HPP_ */
