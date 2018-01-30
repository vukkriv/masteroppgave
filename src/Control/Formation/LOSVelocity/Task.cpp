//***************************************************************************
// Copyright 2007-2017 Universidade do Porto - Faculdade de Engenharia      *
// Laboratório de Sistemas e Tecnologia Subaquática (LSTS)                  *
//***************************************************************************
// This file is part of DUNE: Unified Navigation Environment.               *
//                                                                          *
// Commercial Licence Usage                                                 *
// Licencees holding valid commercial DUNE licences may use this file in    *
// accordance with the commercial licence agreement provided with the       *
// Software or, alternatively, in accordance with the terms contained in a  *
// written agreement between you and Faculdade de Engenharia da             *
// Universidade do Porto. For licensing terms, conditions, and further      *
// information contact lsts@fe.up.pt.                                       *
//                                                                          *
// Modified European Union Public Licence - EUPL v.1.1 Usage                *
// Alternatively, this file may be used under the terms of the Modified     *
// EUPL, Version 1.1 only (the "Licence"), appearing in the file LICENCE.md *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// https://github.com/LSTS/dune/blob/master/LICENCE.md and                  *
// http://ec.europa.eu/idabc/eupl.html.                                     *
//***************************************************************************
// Author: Mads Bornebusch                                                  *
//***************************************************************************

// ISO C++ 98 headers.
#include <cmath>

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Control
{
  namespace Formation
  {
    namespace LOSVelocity
    {
      using DUNE_NAMESPACES;

      struct Arguments
      {
        //! Look ahead distance for line-of-sight
        double lookahead;
        //! Relative damping ratio for velocity filter
        double zeta_vel;
        //! Natural frequency for velocity filter
        double omega_vel;
        //! Flag to use enclosure based steering instead of lookahead based steering
        bool use_enclosure;
        //! Name of task that sends out formation centroid Estimated Local State
        std::string centroid_els_entity_label;
        //! Use as heading reference
        bool use_heading; 
        //! Use referencemodel for velocity
        bool use_refmodel; 
        //! Flag to enable controller
        bool use_controller;
      };

      struct RefModel
      {
        Matrix A; 
        Matrix B; 
        Matrix C; 
        Matrix x; 
      }; 

      struct Task: public DUNE::Control::PathController
      {
        Arguments m_args;
        double m_lookahead;
        double m_zeta_vel;
        double m_omega_vel;
        double m_desired_speed; 
        double m_last_loop_time; 

        RefModel m_ref; 

        double m_airspeed;
        double m_W_x, m_W_y;

        IMC::EstimatedLocalState centroid_state; 
        IMC::EstimatedLocalState vehicle_state; 
        IMC::DesiredHeading heading; 

        std::string m_mode;

        Task(const std::string& name, Tasks::Context& ctx):
          DUNE::Control::PathController(name, ctx),
          m_lookahead(0.0),
          m_zeta_vel(0.0),
          m_omega_vel(0.0),
          m_desired_speed(0.0),
          m_last_loop_time(0.0),
          m_airspeed(0.0),
          m_W_x(0.0),
          m_W_y(0.0)
        {
          param("Lookahead", m_args.lookahead)
          .defaultValue("20.0")
          .description("Lookahead distance (or enclosure radius if enclosure based steering is chosen");

          param("Zeta (velocity filter rel. damping)", m_args.zeta_vel)
          .defaultValue("1.0")
          .description("Velocity filter relative damping ratio");

          param("Omega (velocity filter nat. freq.)", m_args.omega_vel)
          .defaultValue("4.0")
          .description("Velocity filter natural frequency");

          // TODO: Make this into a drop down list
          param("Use enclosure", m_args.use_enclosure)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .defaultValue("false")
          .description("Use enclosure based steering instead of lookahead based steering");

          param("EstimatedLocalState Entity Label", m_args.centroid_els_entity_label)
          .defaultValue("Formation Centroid")
          .description("Entity label for the centroid EstimatedLocalState");

          // TODO: Use heading
          param("Use heading", m_args.use_heading)
          .defaultValue("false")
          .description("Use as heading reference for formation");

          param("Use refmodel", m_args.use_refmodel)
          .defaultValue("false")
          .description("Use reference model for velocity");

          param("Use controller", m_args.use_controller)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .defaultValue("false")
          .description("Use this controller for maneuver");

          /*
          const std::string c_controller_profiles = "Normal,High Gain,Cruise";

          //! Vector of the controller profile strings;
          std::vector<std::string> m_cprofiles;

          // Setup cprofiles
          Utils::String::split(c_controller_profiles, ",", m_cprofiles);

          param("Maneuver Profile", m_args.maneuver_profile)
          .defaultValue(m_cprofiles[0])
          .values(c_controller_profiles)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .description("The profile to override if Maneuver Override is true");
          */

          //bind<IMC::IndicatedSpeed>(this);
          bind<IMC::EstimatedStreamVelocity>(this);
          bind<IMC::AutopilotMode>(this);
          bind<IMC::EstimatedLocalState>(this);
          bind<IMC::DesiredSpeed>(this); 
       }

        void
        onUpdateParameters(void)
        {
          PathController::onUpdateParameters();

          m_lookahead = m_args.lookahead;
          m_zeta_vel = m_args.zeta_vel;
          m_omega_vel = m_args.omega_vel;
        }

        void
        onPathActivation(void)
        {
          if (!m_args.use_controller)
            return;
          // Activate bank (roll) controller.
          //enableControlLoops(IMC::CL_ROLL);
          
          // Init refmodel on path activation
          initRefmodel(); 
          // Initialise with last time control loop was run
          m_last_loop_time = Clock::get(); 
          debug("Activation complete"); 
        }

        void
        onPathDeactivation(void)
        {
          if (!m_args.use_controller){
            // Deactivate bank (roll) controller.
            //disableControlLoops(IMC::CL_ROLL);
          }
        }
        virtual void
        onPathStartup(const IMC::EstimatedState& state, const TrackingState& ts)
        {
          (void)state;
          (void)ts;

          /*
          if (!m_args.use_controller){
            disableControlLoops(IMC::CL_ROLL);
          }
          else{
          // Activate controller
          enableControlLoops(IMC::CL_ROLL);
          }
          */
        }

        /*
        void
        consume(const IMC::IndicatedSpeed* airspeed)
        {
          m_airspeed = airspeed->value;
        }
        */

        void
        consume(const IMC::AutopilotMode* ap_mode)
        {
          m_mode = ap_mode->mode;
        }

        void
        consume(const IMC::EstimatedStreamVelocity* wind)
        {
          m_W_x = wind->x;
          m_W_y = wind->y;
        }

        void
        consume(const IMC::DesiredSpeed* speed)
        {
          m_desired_speed = speed->value; 
        }

         //! Consume Formation Position
        void
        consume(const IMC::EstimatedLocalState* msg)
        {
          // TODO: check if configured for formation
          /*if (!m_configured)
          {
            spew("Not configured!");
            return;
          }*/   
          
          
          if (msg->getSource() == this->getSystemId())
          {
            if (resolveEntity(msg->getSourceEntity()).c_str() == m_args.centroid_els_entity_label)
            {
              spew("Got EstimatedLocalState from '%s', '%s'  - updating centroid state", resolveSystemId(msg->getSource()),resolveEntity(msg->getSourceEntity()).c_str());
              //Update local centroid variable
              centroid_state = *msg;
              spew("Centroid state pos (x,y): (%f,%f)", centroid_state.state->x, centroid_state.state->y);
              
            }
            else
            {
              spew("Got EstimatedLocalState from '%s', '%s' - updating vehicle state", resolveSystemId(msg->getSource()),resolveEntity(msg->getSourceEntity()).c_str());
              // Update local vehicle state variable
              vehicle_state = *msg;
              spew("Vehicle state pos (x,y): (%f,%f)", vehicle_state.state->x, vehicle_state.state->y);
              
            }
          }
        }

        /*
        bool
        hasSpecificZControl(void) const
        {
          if(m_mode == "FBWA")
            return true;
          else
            return false;
        }
        */


        void
        step(const IMC::EstimatedState& state, const TrackingState& ts)
        {
          spew("Control step start");

          if (!m_args.use_controller)
            return;

          spew("Tracking state: %f, %f", ts.track_pos.x, ts.track_pos.y);
          
          // Vector from centroid to vehicle in NED frame
          double p_centroid_x = vehicle_state.state->x - centroid_state.state->x;
          double p_centroid_y = vehicle_state.state->y - centroid_state.state->y; 
          
          // Rotate centroid to vehicle vector from NED into path frame
          Angles::rotate(ts.track_bearing,false, p_centroid_x, p_centroid_y); 

          // Centroid tracking state positions
          double centroid_ts_x = ts.track_pos.x - p_centroid_x;
          double centroid_ts_y = ts.track_pos.y - p_centroid_y; 
          spew("Centroid TrackingState: %f, %f", centroid_ts_x, centroid_ts_y);


          // Lookahead distance on LOS line
          double Delta;

          // Calculate lookahed distance in two different ways for LOS and enclosure based steering
          if (m_args.use_enclosure)
          {
            // Calculate squared lookahead so that it depends on cross track error
            double Delta_sq = m_lookahead*m_lookahead - centroid_ts_y*centroid_ts_y;
            // Constrain value to not take sqrt of a negative number
            trimValue(Delta_sq, 0, m_lookahead*m_lookahead);
            // Set lookahead
            Delta = std::sqrt(Delta_sq);
          }
          else
          {
            // Set lookahead distance to be the user input value
            Delta = m_lookahead;
          }

          // Create the desired velocity vector in path frame
          double v_x = Delta;
          double v_y = -centroid_ts_y;
          spew("Formation desired speed (path frame) %f, %f",v_x,v_y); 

          // Rotate vector from path frame into NED
          Angles::rotate(ts.track_bearing,false,v_x,v_y);
          spew("Formation desired speed (NED frame) %f, %f",v_x,v_y); 

          // Rotate vector from NED frame into centroid frame
          Angles::rotate(centroid_state.state->psi, false, v_x,v_y); 
          spew("Formation desired speed (centroid frame) %f, %f", v_x,v_y); 

          // Calculate length of velocity vector
          double v_length = std::sqrt(v_x*v_x + v_y*v_y);

          // Get the desired velocity magnitude
          double v_set = m_desired_speed;

          // Calculate the scaled reference velocity (so it has the desired magnitude)
          Matrix v_ref = Matrix(2,1);
          v_ref(0) = v_set*v_x/v_length;
          v_ref(1) = v_set*v_y/v_length;
          spew("Formation desired speed (centroid frame scaled) %f, %f", v_ref(0),v_ref(1)); 

          // Converting wind to path reference frame
          //double w_x = m_W_x, w_y = m_W_y;
          //Angles::rotate(ts.track_bearing, false, w_x, w_y);


          // Create variable for reference velocity (output of controller)
          Matrix v_out = Matrix(2,1);
          Matrix a_out = Matrix(2,1);
          // Filter the velocity reference if filter is activated
          if (m_args.use_refmodel)
          {
            v_out = stepRefmodel(v_ref, Clock::get()-m_last_loop_time);
            a_out = m_ref.x.get(2,3,0,0);
          }
          else
          {
            v_out = v_ref; 
            a_out = Matrix(2,1,0.0); 
          }

          // Set the time the control loop was last run
          m_last_loop_time = Clock::get(); 

          // Create desired linear state message
          IMC::DesiredLinearState desLinState;

          // Only use for x,y. Send z directly, no acc (for now)
          desLinState.vx = v_out(0);
          desLinState.vy = v_out(1);
          // TODO: do something about z
          desLinState.vz = 0;

          // TODO: insert m_ref model values
          desLinState.ax = a_out(0);
          desLinState.ay = a_out(1);
          desLinState.az = 0;

          desLinState.flags = IMC::DesiredLinearState::FL_VX |
                              IMC::DesiredLinearState::FL_VY |
                              IMC::DesiredLinearState::FL_VZ |
                              IMC::DesiredLinearState::FL_AX |
                              IMC::DesiredLinearState::FL_AY |
                              IMC::DesiredLinearState::FL_AZ; // Keep flag just so it uses the other ones.

          // Send desired linear state message
          dispatch(desLinState);

        }

        // TODO: make an updateRefmodel that updates the matrices but not the state
        void
        initRefmodel()
        {
          double ones[] = {1.0, 1.0};
          double omega_vel_sq = m_omega_vel * m_omega_vel; 

          Matrix eye = Matrix(ones, 2);
          Matrix zero = Matrix(2,2, 0.0);
          Matrix A_11 = zero;
          Matrix A_12 = eye; 
          Matrix A_21 = -eye*omega_vel_sq;
          Matrix A_22 = -eye*2.0*m_zeta_vel*m_omega_vel; 

          Matrix A_1 = A_11.horzCat(A_12);
          Matrix A_2 = A_21.horzCat(A_22); 

          m_ref.A = A_1.vertCat(A_2); 
          m_ref.B = zero.vertCat(eye) * omega_vel_sq; 
          m_ref.C = Matrix(ones, 2).horzCat(Matrix(2,2, 0.0)); 

          m_ref.x = Matrix(4, 1, 0.0);
        }

        Matrix
        stepRefmodel(Matrix input, double deltaT)
        {
          m_ref.x = m_ref.x + m_ref.A * m_ref.x * deltaT + m_ref.B * input * deltaT; 
          return m_ref.C * m_ref.x; 
        }

      };
    }
  }
}

DUNE_TASK
