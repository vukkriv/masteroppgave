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
      //! Reference latitude
      fp64_t ref_lat;
      //! Reference longitude
      fp64_t ref_lon;
      //! Reference height (above elipsoid)
      fp32_t ref_hae;

      //! Disable Z flag, this will utilize new rate controller on some targets
      bool disable_Z;
      //! Enables the synchronised x8 follow controller
      bool enable_syn;

      //!PID syn controller parameters:
      Matrix Kpp;
      Matrix Kip;
      Matrix Kpv;
      Matrix Kiv;
      Matrix maxV;
      Matrix maxInt;

      double max_veln;
	  double max_Fn;

      std::string m_copter_id;
      std::string m_aircraft_id;
    };

    struct Task: public BasicUAVAutopilot
    {
      //! Task arguments
      Arguments m_args;

      //! m_NetRecovery
      IMC::NetRecovery m_NetRecovery;
      //! Desired force
      IMC::DesiredControl m_desired_force;
      //! FormPos
      IMC::FormPos m_FormPosAircraft;
      IMC::FormPos m_FormPosCopter;
      IMC::DesiredPath m_path;

      //! initialization done
      bool m_initialized_path;

      //! integration values PID controllers
      Matrix m_v_int_value;
      Matrix m_pos_int_value;

      //! angles of orientation path
      double m_psi;
      double m_theta;

      Matrix m_box_min;
      Matrix m_box_max;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        BasicUAVAutopilot(name, ctx, c_controllable, c_required),

        m_initialized_path(false),
		m_v_int_value(3,1,0.0),
		m_pos_int_value(3,1,0.0),
        m_psi(0),
		m_theta(0),
        m_box_min(3,1,0.0),
		m_box_max(3,1,0.0)
      {
          param("Latitude", m_args.ref_lat)
          .defaultValue("-999.0")
          .units(Units::Degree)
          .description("Reference Latitude");

          param("Longitude", m_args.ref_lon)
          .defaultValue("0.0")
          .units(Units::Degree)
          .description("Reference Longitude");

          param("Height", m_args.ref_hae)
          .defaultValue("0.0")
          .units(Units::Meter)
          .description("Reference Height (above elipsoid)");

          param("Disable Z flag", m_args.disable_Z)
          .defaultValue("false")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Choose whether to disable Z flag. In turn, this will utilize new rate controller on some targets");

          param("Enable Syn Controller", m_args.enable_syn)
          .defaultValue("true")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Enables the synchronised x8 follow controller");

          param("Kp pos", m_args.Kpp)
          .defaultValue("1.0, 1.0, 1.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("PID Kp pos");

          param("Ki pos", m_args.Kip)
          .defaultValue("1.0, 1.0, 1.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("PID Ki pos");

          param("Kp vel", m_args.Kpv)
          .defaultValue("1.0, 1.0, 1.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("PID Kp vel");

          param("Ki vel", m_args.Kiv)
          .defaultValue("1.0, 1.0, 1.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("PID Ki vel");

          param("Max Vel", m_args.max_veln)
          .defaultValue("1.0")
		  .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Maximum normalized velocity");

          param("Max F", m_args.max_Fn)
          .defaultValue("1.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Maximum normalized force");

          param("Copter", m_args.m_copter_id);
          param("Aircraft", m_args.m_aircraft_id);

          bind<IMC::NetRecovery>(this);
          bind<IMC::DesiredPath>(this);
          bind<IMC::FormPos>(this);
      }

      //! Consume NetRecovery
      void
	  consume(const IMC::NetRecovery* msg)
      {
          inf("Got NetRecovery \nfrom '%s' at '%s'",
               resolveEntity(msg->getSourceEntity()).c_str(),
               resolveSystemId(msg->getSource()));

    	  m_NetRecovery = *msg;

    	  m_initialized_path = false;
      }

      void
	  consume(const IMC::FormPos* msg)
      {
    	  /*inf("Got FormPos \nfrom '%s' at '%s'",
               resolveEntity(msg->getSourceEntity()).c_str(),
               resolveSystemId(msg->getSource()));
    	  */
		  std::string src_entity  = resolveSystemId(msg->getSource());
		  if (src_entity == m_args.m_copter_id)
		  {
			  m_FormPosCopter = *msg;
		  }
		  else if (src_entity == m_args.m_aircraft_id)
		  {
			  m_FormPosAircraft = *msg;
		  }
 	  }

      //! Consume DesiredPath, contains the desired along-path velocity
      void
	  consume(const IMC::DesiredPath* msg)
      {
          /*inf("Got DesiredPath \nfrom '%s' at '%s'",
               resolveEntity(msg->getSourceEntity()).c_str(),
               resolveSystemId(msg->getSource()));
          */
    	  m_path = *msg;
      }


      void
	  initpath(void)
	   {
    	  //start and end waypoints
		  Matrix WP1 = Matrix(3,1,0.0);
		  Matrix WP2 = Matrix(3,1,0.0);

		  //Way Point 1
		  WGS84::displacement(Angles::radians(m_args.ref_lat), Angles::radians(m_args.ref_lon), 0, m_NetRecovery.start_lat, m_NetRecovery.start_lon, 0, &WP1(0), &WP1(1));
		  WP1(2) = m_NetRecovery.z + m_NetRecovery.z_off;

		  //Way Point 2
		  WGS84::displacement(Angles::radians(m_args.ref_lat), Angles::radians(m_args.ref_lon), 0, m_NetRecovery.end_lat, m_NetRecovery.end_lon, 0, &WP2(0), &WP2(1));
		  WP2(2) = m_NetRecovery.z + m_NetRecovery.z_off;

		  debug("WP_start: [%f,%f,%f]",WP1(0),WP1(1),WP1(2));
		  debug("WP_end: [%f,%f,%f]",WP2(0),WP2(1),WP2(2));

		  m_psi = atan2(WP2(1)-WP1(1), WP2(0)-WP1(0)); //right hand rotation around z
		  m_theta = Angles::radians(0); //right hand rotation around y' (no height difference between way points)

		  m_box_min(0) = 0;
		  m_box_min(1) = -m_NetRecovery.lbox_width/2;
		  m_box_min(2) = -m_NetRecovery.lbox_height/2 + m_NetRecovery.z + m_NetRecovery.z_off;

		  Matrix deltaWP = WP2-WP1;
		  m_box_max(0) = deltaWP.norm_2();
		  m_box_max(1) = m_NetRecovery.lbox_width/2;
		  m_box_max(2) = m_NetRecovery.lbox_height/2 + m_NetRecovery.z + m_NetRecovery.z_off;

		  m_initialized_path = true;
		  inf("Finished path initialization");
	   }

      Matrix
	  pos_con(double timestep, Matrix pos_est_path, Matrix pos_ref_path, Matrix v_des_path)
      {
		  Matrix e_pos_est_path = pos_ref_path-pos_est_path;

		  m_pos_int_value = m_pos_int_value + e_pos_est_path*timestep;

		  Matrix v_des_syn_path = Matrix(3,1,0.0);

		  if (pos_est_path(0) > pos_ref_path(0))
 		  {
			  v_des_syn_path(0) = m_args.Kpp(0)*e_pos_est_path(0) + m_args.Kip(0)*m_pos_int_value(0);
		  }
		  else
		  {
			  v_des_syn_path(0) = v_des_path(0);
		  }

          v_des_syn_path(1) = m_args.Kpp(1)*e_pos_est_path(1) + m_args.Kip(1)*m_pos_int_value(1);
          v_des_syn_path(2) = m_args.Kpp(2)*e_pos_est_path(2) + m_args.Kip(2)*m_pos_int_value(2);

          if (v_des_syn_path.norm_2() > m_args.max_veln)
          {
        	  v_des_syn_path = abs(m_args.max_veln) * v_des_syn_path/v_des_syn_path.norm_2();
          }

		  return v_des_syn_path;
      }

	  //! todo: implement speedlimit
      Matrix
	  vel_con(double timestep, Matrix v_est, Matrix v_des)
      {
    	  Matrix e_v_est = v_des-v_est;

		  m_v_int_value = m_v_int_value + e_v_est*timestep;

		  Matrix F_des = Matrix(3,1,0.0);
		  F_des(0) = m_args.Kpv(0)*e_v_est(0) + m_args.Kiv(0)*m_v_int_value(0);
		  F_des(1) = m_args.Kpv(1)*e_v_est(1) + m_args.Kiv(1)*m_v_int_value(1);
		  F_des(2) = m_args.Kpv(2)*e_v_est(2) + m_args.Kiv(2)*m_v_int_value(2);

		  return F_des;

          if (F_des.norm_2() > m_args.max_Fn)
          {
        	  F_des = abs(m_args.max_Fn) * F_des/F_des.norm_2();
          }
      }

      //! Dispatch desired force
      void
      sendDesiredForce(Matrix F_des)
      {
    	m_desired_force.x =  F_des(0);
        m_desired_force.y =  F_des(1);
        m_desired_force.z =  F_des(2);

        if (m_args.disable_Z)
        {
          m_desired_force.flags = IMC::DesiredControl::FL_X | IMC::DesiredControl::FL_Y;
        }
        else
        {
          m_desired_force.flags = IMC::DesiredControl::FL_X | IMC::DesiredControl::FL_Y | IMC::DesiredControl::FL_Z;
        }

        m_desired_force.setSourceEntity(getEntityId());
        dispatch(m_desired_force);
      }

      //!Return Rotation matrix Rzx'y''.
      Matrix Rzyx(double psi, double theta, double phi) const
      {
        double R_en_elements[] = {cos(psi)*cos(theta), -sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi),  sin(psi)*sin(phi) + cos(psi)*cos(phi)*sin(theta),
        						  sin(psi)*cos(theta),  cos(psi)*cos(phi) + sin(phi)*sin(theta)*sin(psi), -cos(psi)*sin(phi) + sin(theta)*sin(psi)*cos(phi),
								 -sin(theta),           cos(theta)*sin(phi),                               cos(theta)*cos(phi)								};
        return Matrix(R_en_elements,3,3);
      }

	  Matrix saturate(Matrix ref_value, Matrix min_value, Matrix max_value)
      {
          for (int i = 0; i <= 2; i = i+1)
		  {
			  if (ref_value(i) < min_value(i))
			  {
				  ref_value(i) = min_value(i);
			  }
			  if (ref_value(i) > max_value(i))
			  {
				  ref_value(i) = max_value(i);
			  }
          }

    	  return ref_value;
      }

      virtual void
      onEstimatedState(const double timestep, const IMC::EstimatedState* state)
      {
    	  if (m_initialized_path == false)
    	  {
    		  initpath();
    	  }

		  //current estimated NED position
		  Matrix pos_est(3, 1, 0.0);
		  pos_est(0) = m_FormPosCopter.x;
		  pos_est(1) = m_FormPosCopter.y;
		  pos_est(2) = m_FormPosCopter.z;

		  //current estimated NED velocity
		  Matrix v_est(3, 1, 0.0);
		  v_est(0) = m_FormPosCopter.vx;
		  v_est(1) = m_FormPosCopter.vy;
		  v_est(2) = m_FormPosCopter.vz;

		  //position x8
		  Matrix pos_est_x8(3, 1, 0.0);
		  pos_est_x8(0) = m_FormPosAircraft.x;
		  pos_est_x8(1) = m_FormPosAircraft.y;
		  pos_est_x8(2) = m_FormPosAircraft.z;

		  //desired velocity in path frame
		  Matrix v_des_path(3, 1, 0.0);
		  v_des_path(0) = m_NetRecovery.speed;
		  v_des_path(1) = 0.0;
		  v_des_path(2) = 0.0;

		  Matrix v_des_syn_path = Matrix(3,1,0.0);
		  Matrix F_des = Matrix(3,1,0.0);
		  Matrix pos_est_path = transpose(Rzyx(m_psi, m_theta, 0))*pos_est;
		  Matrix pos_est_x8_path = transpose(Rzyx(m_psi, m_theta, 0))*pos_est_x8;
		  Matrix pos_ref_path = saturate(pos_est_x8_path, m_box_min, m_box_max);
		  pos_ref_path(0) = m_box_max(0);
		  Matrix v_est_path = transpose(Rzyx(m_psi, m_theta, 0))*v_est;

		  if (m_args.enable_syn == true)
		  {
			  v_des_syn_path = pos_con(timestep, pos_est_path, pos_ref_path, v_des_path);
			  Matrix v_des = Rzyx(m_psi, m_theta, 0)*v_des_syn_path;
			  F_des = vel_con(timestep, v_est, v_des);
		  }

		  sendDesiredForce(F_des);

          debug("pos ref path: %f; %f; %f", pos_ref_path(0), pos_ref_path(1), pos_ref_path(2));
		  debug("pos est path: %f; %f; %f", pos_est_path(0), pos_est_path(1), pos_est_path(2));
		  debug("  v des path: %f, %f, %f", v_des_syn_path(0), v_des_syn_path(1), v_des_syn_path(2));
		  debug("  v est path: %f; %f; %f\n", v_est_path(0), v_est_path(1), v_est_path(2));

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
