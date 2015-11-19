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
    static const uint32_t c_required = IMC::CL_FORCE;

    struct Arguments
    {
      //!PID syn controller parameters:
      Matrix Kp;
      Matrix Ki;
      Matrix Kd;
      Matrix maxV;
      Matrix maxInt;

      //! Disable Z flag, this will utilize new rate controller on some targets
      bool disable_Z;
    };

    struct Task: public BasicUAVAutopilot
    {
      //! Task arguments
      Arguments m_args;

      //! m_NetRecovery
      IMC::NetRecovery m_NetRecovery;
      //! Desired force
      IMC::DesiredControl m_desired_force;
      //! NaviagationData x8
      IMC::NavigationData m_NavigationData;

      //! initialization done
      bool m_initialized_path;

      //! integration values PID controllers
      double m_v_int_value;
      Matrix m_pos_int_value;

      //! angles of orientation path
      double m_psi;
      double m_theta;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        BasicUAVAutopilot(name, ctx, c_controllable, c_required),
        m_initialized_path(false),
		m_v_int_value(0.0),
		m_pos_int_value(3,1,0.0)
      {
          param("Disable Z flag", m_args.disable_Z)
          .defaultValue("false")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Choose whether to disable Z flag. In turn, this will utilize new rate controller on some targets");

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

          bind<IMC::NetRecovery>(this);
          bind<IMC::NavigationData>(this);
      }

      //! Consume NetRecovery
      void
	  consume(const IMC::NetRecovery* msg)
      {
    	  m_NetRecovery = *msg;

    	  m_initialized_path = false;
      }

	  //! todo: needs to become msg from coordinator
      //! Consume Navigation Data
      void
	  consume(const IMC::NavigationData* msg)
      {
    	  if (msg->cog == 1) //todo change this into a proper vehicle check
    	  {
			  m_NavigationData = *msg;
    	  }
 	  }

      void
	  initpath(IMC::EstimatedState state)
	   {
    	  //start and end waypoints
		  Matrix WP1 = Matrix(3,1,0.0);
		  Matrix WP2 = Matrix(3,1,0.0);

		  //Way Point 1
		  WGS84::displacement(state.lat, state.lon, 0, m_NetRecovery.start_lat, m_NetRecovery.start_lon, 0, &WP1(0), &WP1(1));
		  WP1(2) = m_NetRecovery.z+m_NetRecovery.z_off;

		  //Way Point 2
		  WGS84::displacement(state.lat, state.lon, 0, m_NetRecovery.end_lat, m_NetRecovery.end_lon, 0, &WP2(0), &WP2(1));
		  WP2(2) = m_NetRecovery.z+m_NetRecovery.z_off;

		  m_psi = atan2(WP2(1)-WP1(1), WP2(0)-WP1(0)); //right hand rotation around z
		  m_theta = Angles::radians(0); //right hand rotation around y'

		  m_initialized_path = true;
		  inf("Finished path initialization");
	   }

	  //! todo: implement speedlimit
      Matrix
	  path_follow(double timestep, Matrix v_est, double des_speed)
      {
		  Matrix v_est_path = transpose(Rzyx(m_psi, m_theta, 0))*v_est;

		  m_v_int_value = m_v_int_value + (des_speed-v_est_path(0))*timestep;

		  Matrix F_des_path_path = Matrix(3,1,0.0);
		  F_des_path_path(0) = m_args.Kp(0)*(des_speed-v_est_path(0)) + m_args.Ki(0)*m_v_int_value;

		  debug("   speed des   : %f", des_speed);
		  debug("  v est path   : %f; %f; %f", v_est_path(0), v_est_path(1), v_est_path(2));

		  return F_des_path_path;
      }


	  //! todo: implement boundries
      Matrix
	  syn_x8(double timestep, Matrix pos_est, Matrix pos_est_x8, Matrix v_est, Matrix v_est_x8)
      {
		  Matrix pos_est_path = transpose(Rzyx(m_psi, m_theta, 0))*pos_est;
		  Matrix pos_est_x8_path = transpose(Rzyx(m_psi, m_theta, 0))*pos_est_x8;

		  Matrix v_est_path = transpose(Rzyx(m_psi, m_theta, 0))*v_est;
		  Matrix v_est_x8_path = transpose(Rzyx(m_psi, m_theta, 0))*v_est_x8;

		  m_pos_int_value = m_pos_int_value + (pos_est_x8_path-pos_est_path)*timestep;

		  Matrix F_des_syn_path = Matrix(3,1,0.0);
          F_des_syn_path(1) = m_args.Kp(1)*(pos_est_x8_path(1)-pos_est_path(1)) + m_args.Ki(1)*m_pos_int_value(1) + m_args.Kd(1)*(v_est_x8_path(1)-v_est_path(1));
          F_des_syn_path(2) = m_args.Kp(2)*(pos_est_x8_path(2)-pos_est_path(2)) + m_args.Ki(2)*m_pos_int_value(2) + m_args.Kd(2)*(v_est_x8_path(2)-v_est_path(2));

          debug("pos est path x8: %f; %f; %f", pos_est_x8_path(0), pos_est_x8_path(1), pos_est_x8_path(2));
		  debug("pos est path   : %f; %f; %f\n", pos_est_path(0), pos_est_path(1), pos_est_path(2));

		  return F_des_syn_path;
      }

      //!Return Rotation matrix Rzx'y''.
      Matrix Rzyx(double psi, double theta, double phi) const
      {
        double R_en_elements[] = {cos(psi)*cos(theta), -sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi),  sin(psi)*sin(phi) + cos(psi)*cos(phi)*sin(theta),
        						  sin(psi)*cos(theta),  cos(psi)*cos(phi) + sin(phi)*sin(theta)*sin(psi), -cos(psi)*sin(phi) + sin(theta)*sin(psi)*cos(phi),
								 -sin(theta),           cos(theta)*sin(phi),                               cos(theta)*cos(phi)								};
        return Matrix(R_en_elements,3,3);
      }

      //! Dispatch desired force
      void
      sendDesiredForce(Matrix F_des)
      {
    	m_desired_force.x =  F_des(0);
        m_desired_force.y =  F_des(1);
        m_desired_force.z =  F_des(2);

        if (m_args.disable_Z)
          m_desired_force.flags = IMC::DesiredControl::FL_X | IMC::DesiredControl::FL_Y;
        else
          m_desired_force.flags = IMC::DesiredControl::FL_X | IMC::DesiredControl::FL_Y | IMC::DesiredControl::FL_Z;

        m_desired_force.setSourceEntity(getEntityId());
        dispatch(m_desired_force);
      }

      virtual void
      onEstimatedState(const double timestep, const IMC::EstimatedState* state)
      {
    	  if (m_initialized_path == false)
    	  {
    		  initpath(*state);
    	  }

		  //current estimated NED position
		  Matrix pos_est(3, 1, 0.0);
		  pos_est(0) = state->x;
		  pos_est(1) = state->y;
		  pos_est(2) = state->z;

		  //current estimated NED velocity
		  Matrix v_est(3, 1, 0.0);
		  v_est(0) = state->vx;
		  v_est(1) = state->vy;
		  v_est(2) = state->vz;

		  //position x8
		  Matrix pos_est_x8(3, 1, 0.0);
		  pos_est_x8(0) = m_NavigationData.custom_x;
		  pos_est_x8(1) = m_NavigationData.custom_y;
		  pos_est_x8(2) = m_NavigationData.custom_z;

		  //desired speed x
		  double des_speed = m_NavigationData.cyaw;
		  //current velocity x8 x y
		  Matrix v_est_x8(3, 1, 0.0);
		  v_est_x8(0) = m_NavigationData.bias_psi;
		  v_est_x8(1) = m_NavigationData.bias_r;

		  Matrix F_des_path_path = path_follow(timestep, v_est, des_speed);
		  Matrix F_des_syn_path = syn_x8(timestep, pos_est, pos_est_x8, v_est, v_est_x8);

		  Matrix F_des_path(3,1, 0.0);
		  F_des_path(0) = F_des_path_path(0);
		  F_des_path(1) = F_des_syn_path(1);
		  F_des_path(2) = F_des_syn_path(2);

		  Matrix F_des = Rzyx(m_psi, m_theta, 0)*F_des_path;
          sendDesiredForce(F_des);

		  debug("angle path: %f", m_psi); 
		  debug("pos est x8: %f; %f; %f", pos_est_x8(0), pos_est_x8(1), pos_est_x8(2));
		  debug("pos est   : %f; %f; %f\n", pos_est(0), pos_est(1), pos_est(2));
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
