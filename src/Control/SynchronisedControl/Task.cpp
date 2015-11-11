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

// Local headers.
#include "/home/uavlab/uavlab/dune/src/DUNE/Control/BasicAutopilot.hpp"
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
      Matrix delta;

      //! Disable heave flag, this will utilize new rate controller on some targets
      bool disable_heave;
    };

    struct Task: public BasicUAVAutopilot
    {
      //! Task arguments
      Arguments m_args;

      //! Desired velocity
      IMC::TranslationalSetpoint m_desired_velocity;
      //! DesiredPath
      IMC::DesiredPath m_path;

      //! initialization done
      bool m_initialized = false;

      //! Waypoint Matrices
	  Matrix m_WP1;
	  Matrix m_WP2;

	  double m_alpha_k = 0;
	  double m_theta_k = 0;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        BasicUAVAutopilot(name, ctx, c_controllable, c_required)
      {
          param("Delta", m_args.delta)
          .defaultValue("10,10")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Delta is the tuning parameter of the LOS controller in y and z direction");

          param("Disable Heave flag", m_args.disable_heave)
          .defaultValue("false")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Choose whether to disable heave flag. In turn, this will utilize new rate controller on some targets");

          bind<IMC::DesiredPath>(this);
      }

      //! Consume Desired Path
      void
	  consume(const IMC::DesiredPath* msg)
      {
    	  m_path = *msg;
    	  m_initialized = false;
 	  }

      virtual void
      onEstimatedState(const double timestep, const IMC::EstimatedState* state)
      {
    	  (void)timestep;

    	  if (m_initialized == false)
    	  {
    		  m_WP1 = Matrix(3,1,0.0);
    		  m_WP2 = Matrix(3,1,0.0);

			  //Way Point 1
			  WGS84::displacement(state->lat, state->lon, 0, m_path.end_lat, m_path.end_lon, 0, &m_WP1(0), &m_WP1(1));
			  m_WP1(2) = -m_path.end_z;

			  //Way point 2 -> todo: get WP2 from DesiredPath
			  m_WP2(0) = m_WP1(0)-60;
			  m_WP2(1) = m_WP1(1)-30;
			  m_WP2(2) = m_WP1(2)-10;

			  Matrix deltaWP = m_WP2 - m_WP1;
			  double deltaWP_NE = deltaWP.get(0,1,0,0).norm_2();

			  m_alpha_k =  atan2(deltaWP(1),deltaWP(0));
			  m_theta_k = -atan2(deltaWP_NE,deltaWP(2)) + Angles::radians(90);

			  m_initialized = true;
			  inf("Path initialization done");
    	  }

		  //current estimated NED position
		  Matrix pos_est(3, 1, 0.0);
		  pos_est(0) = state->x;
		  pos_est(1) = state->y;
		  pos_est(2) = state->z;

          Matrix eps = transpose(Rzyx(0,-m_theta_k,m_alpha_k))*(pos_est-m_WP1);

          double theta_d = m_theta_k + atan2(-eps(2),m_args.delta(1));
          double psi_d = m_alpha_k + atan2(-eps(1),m_args.delta(0));

		  Matrix u_d_p(3,1,0.0);
		  u_d_p(0) = m_path.speed;

          Matrix v_des = Rzyx(0,-theta_d,psi_d)*u_d_p;
          sendDesiredVelocity(v_des);

          std::cout << "speed_des= " << u_d_p(0) << "\n";
          std::cout << "x_WP1= " << m_WP1(0) << "\tx_WP2= " << m_WP2(0) << "\tx_est= " << pos_est(0) << "\tvx_des= " << v_des(0) << "\n";
          std::cout << "y_WP1= " << m_WP1(1) << "\ty_WP2= " << m_WP2(1) << "\ty_est= " << pos_est(1) << "\tvy_des= " << v_des(1) << "\n";
          std::cout << "z_WP1= " << m_WP1(2) << "\tz_WP2= " << m_WP2(2) << "\tz_est= " << pos_est(2) << "\tvz_des= " << v_des(2) << "\n\n";
      }

      //Return Rotation matrix.
      Matrix Rzyx(double phi, double theta, double psi) const
      {
        double R_en_elements[] = {cos(psi)*cos(theta), (-sin(psi)*cos(phi))+(cos(psi)*sin(theta)*sin(psi)), (sin(psi)*sin(phi))+(cos(psi)*cos(phi)*sin(theta)),
        						  sin(psi)*cos(theta), (cos(psi)*cos(phi))+(sin(phi)*sin(theta)*sin(psi)), (-cos(psi)*sin(phi))+(sin(theta)*sin(psi)*cos(phi)),
								 -sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi)};

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
        spew("v_d: [%1.1f, %1.1f, %1.1f]", m_desired_velocity.u, m_desired_velocity.v, m_desired_velocity.w);
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
