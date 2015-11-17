//***************************************************************************
// Copyright 2007-2015 Universidade do Porto - Faculdade de Engenharia      *
// Laboratório de Sistemas e Tecnologia Subaquática (LSTS)                  *
//***************************************************************************
// This file is part of DUNE: Unified Navigation Environment.               *
//                                                                          *
// Commercial Licence Usage                                                 *
// Licencees holding valid commercial DUNE licences may use this file in    *
// accordance with the commercial licence agreement provided with the       *
// Software or, alternatively, in accordance with the terms contained in a  *
// written agreement between you and Universidade do Porto. For licensing   *
// terms, conditions, and further information contact lsts@fe.up.pt.        *
//                                                                          *
// European Union Public Licence - EUPL v.1.1 Usage                         *
// Alternatively, this file may be used under the terms of the EUPL,        *
// Version 1.1 only (the "Licence"), appearing in the file LICENCE.md       *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// http://ec.europa.eu/idabc/eupl.html.                                     *
//***************************************************************************
// Author: Jon-Håkon Bøe Røli                                               *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>
#include <DUNE/Control/DiscretePID.hpp>
#include <DUNE/Control/BasicAutopilot.hpp>

// Local headers.
#include <cmath>

namespace Control
{
  namespace SynchronisedControl
  {
    using DUNE_NAMESPACES;

    //! Controllable loops.
    static const uint32_t c_controllable = IMC::CL_PATH;
    //! Required loops.
    static const uint32_t c_required = IMC::CL_SPEED;

    struct Arguments
    {
      //!Delta is the tuning parameter of the LOS controller in xy and z direction
//    Matrix delta;

      //!PID syn controller parameters:
      Matrix Kp;
      Matrix Ki;
      Matrix Kd;
      Matrix maxV;
      Matrix maxInt;

      //! Disable heave flag, this will utilize new rate controller on some targets
      bool disable_heave;
    };

    struct Task: public BasicUAVAutopilot
    {
      //! Task arguments
      Arguments m_args;

      //! Desired velocity
      IMC::DesiredVelocity m_desired_velocity;
      //! DesiredPath
      IMC::DesiredPath m_path;

      //! Dummy position x8
      Matrix m_pos_est_x8 = Matrix(3,1,0.0);
      //! Dummy desired speed
      double m_des_speed;

      //! initialization done
      bool m_initialized_path = false;
      bool m_initialized_PID = false;

      double m_psi;
      double m_theta;

      //! Waypoint Matrices
//	  Matrix m_WP1 = Matrix(3,1,0.0);
//	  Matrix m_WP2 = Matrix(3,1,0.0);

//	  double m_alpha_k;
//	  double m_theta_k;

      //! Discrete velocity PID control x-direction
      DiscretePID m_PID_vx;
      //! Discrete velocity PID control y-direction
      DiscretePID m_PID_vy;
      //! Discrete velocity PID control z-direction
      DiscretePID m_PID_vz;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        BasicUAVAutopilot(name, ctx, c_controllable, c_required)
      {
/*        param("Delta", m_args.delta)
          .defaultValue("10,10")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Delta is the tuning parameter of the LOS controller in y and z direction");*/

          param("Disable Heave flag", m_args.disable_heave)
          .defaultValue("false")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Choose whether to disable heave flag. In turn, this will utilize new rate controller on some targets");

          param("Kp", m_args.Kp)
          .defaultValue("1.0, 1.0, 1.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("PID Kp");

             param("Ki", m_args.Ki)
          .defaultValue("1.0, 1.0, 1.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("PID Ki");

          param("Kd", m_args.Kd)
          .defaultValue("1.0, 1.0, 1.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("PID Kd");

          param("Max Vel", m_args.maxV)
          .defaultValue("1.0, 1.0, 1.0")
		  .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Maximum velocity");

          param("Max Integral", m_args.maxInt)
          .defaultValue("1.0, 1.0, 1.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Maximum integral y-direction");

          bind<IMC::DesiredPath>(this);
          bind<IMC::NavigationData>(this);
      }

      //! Consume Desired Path
      void
	  consume(const IMC::DesiredPath* msg)
      {
    	  m_path = *msg;
    	  m_initialized_path = false;
      }

      //!todo:above should become:
/*	  void
 	  consume(cont IMC::NetCatchManeuver* msg)
 	  {
 	  	  m_maneuver = *msg;
 	  	  m_initialized_path = false;
 	  }*/

      //! Consume Navigation Data
      void
	  consume(const IMC::NavigationData* msg)
      {
    	  if (msg->cog == 1) //todo change this into a proper vehicle check
    	  {
			  m_pos_est_x8(0) = msg->custom_x;
			  m_pos_est_x8(1) = msg->custom_y;
			  m_pos_est_x8(2) = msg->custom_z;

			  m_des_speed = msg->cyaw;
    	  }

    	  std::cout << "-->x8 position NED: " << m_pos_est_x8(0) << " " << m_pos_est_x8(1)<< " " << m_pos_est_x8(2) << "\n\n";
 	  }

      void
	  initpath(IMC::EstimatedState state) //in consume??
      {
    	  (void)state;

    	  //todo: get this info from netcatch maneuver in Neptus
		  m_psi = Angles::radians(45); //right hand rotation around z
		  m_theta = Angles::radians(0); //right hand rotation around y'

/* 		  //Way Point 1
		  WGS84::displacement(state.lat, state.lon, 0, m_path.end_lat, m_path.end_lon, 0, &m_WP1(0), &m_WP1(1));
		  m_WP1(2) = -m_path.end_z;

		  //Way point 2
		  m_WP2(0) = m_WP1(0)+100*cos(m_psi);
		  m_WP2(1) = m_WP1(1)+100*sin(m_psi);
		  m_WP2(2) = m_WP1(2)+0;

		  Matrix deltaWP = m_WP2 - m_WP1;
		  double deltaWP_NE = deltaWP.get(0,1,0,0).norm_2();

		  m_alpha_k =  atan2(deltaWP(1),deltaWP(0));
		  m_theta_k = -atan2(deltaWP_NE,deltaWP(2)) + Angles::radians(90);*/

		  //Dimension box todo: this info in maneuver msg
		  double l_box = 50;
		  double w_box = 15;
		  double h_box = 15;

		  //unit vectors body frame box
		  double xh_body[] = {1,0,0};
		  double yh_body[] = {0,1,0};
		  double zh_body[] = {0,0,1};

		  Matrix l_NED = Matrix(3,1,0.0);
		  Matrix w_NED = Matrix(3,1,0.0);
		  Matrix h_NED = Matrix(3,1,0.0);

		  l_NED = Rzyx(m_psi, m_theta, 0)*(Matrix(xh_body,3,1)*l_box);
		  w_NED = Rzyx(m_psi, m_theta, 0)*(Matrix(yh_body,3,1)*w_box);
		  h_NED = Rzyx(m_psi, m_theta, 0)*(Matrix(zh_body,3,1)*h_box);

/*		  std::cout << "l " << l_NED(0) << " " << l_NED(1) << " " << l_NED(2) <<"\n";
		  std::cout << "w " << w_NED(0) << " " << w_NED(1) << " " << w_NED(2) <<"\n";
		  std::cout << "h " << h_NED(0) << " " << h_NED(1) << " " << h_NED(2) <<"\n";*/

		  m_initialized_path = true;
		  inf("Finished path initialization");
      }

      Matrix
	  path_follow(Matrix pos_est)
      {
    	  (void)pos_est;

          /*Matrix eps = transpose(Rzyx(m_alpha_k,-m_theta_k,0))*(pos_est-m_WP1);

          double theta_d = m_theta_k + atan2(-eps(2),m_args.delta(1));
          double psi_d = m_alpha_k + atan2(-eps(1),m_args.delta(0));

		  Matrix u_d_p(3,1,0.0);
		  u_d_p(0) = m_path.speed;

          Matrix v_des_path = Rzyx(psi_d,-theta_d,0)*u_d_p;

          return v_des_path;*/

		  Matrix v_des_path_body = Matrix(3,1,0.0);
		  v_des_path_body(0) = m_des_speed;

		  return v_des_path_body;
      }

      void
      initPID()
      {
		m_PID_vy.setOutputLimits(-m_args.maxV(1), m_args.maxV(1));
		m_PID_vz.setOutputLimits(-m_args.maxV(2), m_args.maxV(2));

		m_PID_vy.setIntegralLimits(m_args.maxInt(1));
		m_PID_vz.setIntegralLimits(m_args.maxInt(2));

		m_PID_vy.setProportionalGain(m_args.Kp(1));
		m_PID_vz.setProportionalGain(m_args.Kp(2));

		m_PID_vy.setIntegralGain(m_args.Ki(1));
		m_PID_vz.setIntegralGain(m_args.Ki(2));

		m_PID_vy.setDerivativeGain(m_args.Kd(1));
		m_PID_vz.setDerivativeGain(m_args.Kd(2));

		m_initialized_PID = true;
		inf("Finished PID initialization");
      }

      Matrix
	  syn_x8(double timestep, Matrix pos_est)
      {
		  Matrix pos_est_x8_body = transpose(Rzyx(m_psi, m_theta, 0))*m_pos_est_x8;
		  Matrix pos_est_body = transpose(Rzyx(m_psi, m_theta, 0))*pos_est;

/*		  double Kp1 = 1.0;
		  double Kp2 = 1.0;

		  Matrix v_des_syn_body = Matrix(3,1,0.0);
		  v_des_syn_body(1) = (pos_est_x8_body(1)-pos_est_body(1))*Kp1;
		  v_des_syn_body(2) = (pos_est_x8_body(2)-pos_est_body(2))*Kp2;*/

    	  Matrix v_des_syn_body = Matrix(3,1,0.0);
          v_des_syn_body(1) = m_PID_vy.step(timestep, pos_est_x8_body(1) - pos_est_body(1));
          v_des_syn_body(2) = m_PID_vz.step(timestep, pos_est_x8_body(2) - pos_est_body(2));

		  std::cout << "-->   position body: " << pos_est_body(0) << " " << pos_est_body(1)<< " " << pos_est_body(2) << "\n";
    	  std::cout << "-->x8 position body: " << pos_est_x8_body(0) << " " << pos_est_x8_body(1)<< " " << pos_est_x8_body(2) << "\n\n";

		  return v_des_syn_body;
      }

      //!Return Rotation matrix Rzx'y''.
      Matrix Rzyx(double psi, double theta, double phi) const
      {
        double R_en_elements[] = {cos(psi)*cos(theta), -sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi),  sin(psi)*sin(phi) + cos(psi)*cos(phi)*sin(theta),
        						  sin(psi)*cos(theta),  cos(psi)*cos(phi) + sin(phi)*sin(theta)*sin(psi), -cos(psi)*sin(phi) + sin(theta)*sin(psi)*cos(phi),
								 -sin(theta),           cos(theta)*sin(phi),                               cos(theta)*cos(phi)								};
        return Matrix(R_en_elements,3,3);
      }

      //! Dispatch desired velocity
      void
      sendDesiredVelocity(Matrix v_des)
      {
    	m_desired_velocity.u = v_des(0);
        m_desired_velocity.v = v_des(1);
        m_desired_velocity.w = v_des(2);

        if (m_args.disable_heave)
          m_desired_velocity.flags = IMC::TranslationalSetpoint::FL_SURGE | IMC::TranslationalSetpoint::FL_SWAY;
        else
          m_desired_velocity.flags = IMC::TranslationalSetpoint::FL_SURGE | IMC::TranslationalSetpoint::FL_SWAY | IMC::TranslationalSetpoint::FL_HEAVE;

        m_desired_velocity.setSourceEntity(getEntityId());
        dispatch(m_desired_velocity);
      }

      virtual void
      onEstimatedState(const double timestep, const IMC::EstimatedState* state)
      {
    	  if (m_initialized_path == false)
    	  {
    		  initpath(*state);
    	  }

    	  if (m_initialized_PID == false)
    	  {
			  initPID();
	      }

		  //current estimated NED position
		  Matrix pos_est(3, 1, 0.0);
		  pos_est(0) = state->x;
		  pos_est(1) = state->y;
		  pos_est(2) = state->z;

		  Matrix v_des_path_body = path_follow(pos_est);
		  Matrix v_des_syn_body = syn_x8(timestep, pos_est);

		  Matrix v_des_body(3,1, 0.0);
		  v_des_body(0) = v_des_path_body(0);
		  v_des_body(1) = v_des_syn_body(1);
		  v_des_body(2) = v_des_syn_body(2);

		  Matrix v_des = Rzyx(m_psi, m_theta, 0)*v_des_body;
          sendDesiredVelocity(v_des);

		  std::cout << "speed_des= " << m_des_speed << "\n";
          std::cout << "x_est= " << pos_est(0) << " \tvx_des= " << v_des(0) << " \tvx_des_body= " << v_des_body(0) << "\n";
          std::cout << "y_est= " << pos_est(1) << " \tvy_des= " << v_des(1) << " \tvy_des_body= " << v_des_body(1) << "\n";
          std::cout << "z_est= " << pos_est(2) << " \tvz_des= " << v_des(2) << " \tvz_des_body= " << v_des_body(2) << "\n\n";
	  }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
      }

      //! Reserve entity identifiers.
      void
      onEntityReservation(void)
      {
          inf("Starting: %s", resolveEntity(getEntityId()).c_str());
      }

      //! Resolve entity names.
      void
      onEntityResolution(void)
      {
      }

      //! Acquire resources.
      void
      onResourceAcquisition(void)
      {
      }

      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
      }

      void
      onActivation(void)
      {
      }

      void
      onDeactivation(void)
      {
      }

    };
  }
}

DUNE_TASK
