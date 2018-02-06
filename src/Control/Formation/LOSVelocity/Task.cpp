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

      enum GUIDANCE_TYPE
      {
        LOS_LOOKAHEAD = 0,
        ENCLOSURE = 1,
        PURE_PURSUIT = 2
      };

      // This should be kept the same order and length as GUIDANCE_TYPE
      const std::string c_guicance_algorithms = "LOS Lookahead,Enclosure,Pure pursuit";

      struct Arguments
      {
        //! Look ahead distance for line-of-sight
        double lookahead;
        //! Guidance algorithm selection
        std::string guidance_algorithm;
        //! Use referencemodel for velocity (velocity filter)
        bool use_refmodel;
        //! Natural frequency for velocity filter
        Matrix omega_vel;
        //! Relative damping ratio for velocity filter
        Matrix zeta_vel;
        //! Name of task that sends out formation centroid Estimated Local State
        std::string centroid_els_entity_label;
        //! Use height guidance
        bool control_height; 
        //! Flag to enable controller
        bool use_controller;
      };

      struct RefModel
      {
        Matrix A; 
        Matrix B; 
        Matrix Cvel;
        Matrix Cacc;
        Matrix x; 
      }; 

      struct Task: public DUNE::Control::PathController
      {
        Arguments m_args;
        double m_lookahead;
        Matrix m_zeta_vel;
        Matrix m_omega_vel;
        double m_desired_speed; 
        double m_last_loop_time;
        uint8_t m_guidance_algorithm;
        double m_tan_path_angle; 

        //! Vector of the guidance algorithm strings
        std::vector<std::string> m_guidance_string;

        RefModel m_ref; 

        double m_airspeed;
        double m_W_x, m_W_y;

        IMC::EstimatedLocalState m_centroid_state;
        IMC::EstimatedLocalState m_vehicle_state;
        IMC::DesiredHeading heading; 

        std::string m_mode;

        Task(const std::string& name, Tasks::Context& ctx):
          DUNE::Control::PathController(name, ctx),
          m_lookahead(0.0),
          m_desired_speed(0.0),
          m_last_loop_time(0.0),
          m_guidance_algorithm(LOS_LOOKAHEAD),
          m_airspeed(0.0),
          m_W_x(0.0),
          m_W_y(0.0)
        {
          // Setup guidance algorithm list
          Utils::String::split(c_guicance_algorithms, ",", m_guidance_string);

          param("Guidance algorithm", m_args.guidance_algorithm)
          .defaultValue(m_guidance_string[LOS_LOOKAHEAD])
          .values(c_guicance_algorithms)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .description("Selection of which guidance algorithm to use in the controller");

          param("Lookahead", m_args.lookahead)
          .defaultValue("8.0")
          .units(Units::Meter)
          .description("Lookahead distance (or enclosure radius if enclosure based steering is chosen");

          param("Use height guidance", m_args.control_height)
          .defaultValue("false")
          .description("Use height guidance to send a z-velocity as well");

          param("Use velocity filter", m_args.use_refmodel)
          .defaultValue("false")
          .description("Use reference model for velocity");

          param("Omega (velocity filter nat. freq.)", m_args.omega_vel)
          .defaultValue("4.0")
          .units(Units::RadianPerSecond)
          .description("Velocity filter natural frequency matrix. One value:uses on all. Two values: uses one on xy and one on z (if activated)");

          param("Zeta (velocity filter rel. damping)", m_args.zeta_vel)
          .defaultValue("1.0")
          .units(Units::None)
          .description("Velocity filter relative damping ratio matrix. One value:uses on all. Two values: uses one on xy and one on z (if activated)");

          param("EstimatedLocalState Entity Label", m_args.centroid_els_entity_label)
          .defaultValue("Formation Centroid")
          .description("Entity label for the centroid EstimatedLocalState");

          param("Use controller", m_args.use_controller)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .defaultValue("false")
          .description("Use this controller for maneuver");


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

          if (paramChanged(m_args.zeta_vel))
          {
            debug("Changing zeta:");
            m_zeta_vel = createDiagonalMatrix(m_args.zeta_vel);
          }
          if (paramChanged(m_args.omega_vel))
          {
            debug("Changing omega:");
            m_omega_vel = createDiagonalMatrix(m_args.omega_vel);
          }

          if (paramChanged(m_args.use_refmodel) && m_args.use_refmodel)
          {
            debug("Initialising RefModel (velocity filter) AND setting sate");
            initRefmodel();
          }
          else
          {
            debug("Initialising RefModel Matrices (velocity filter)");
            setRefmodelMatrices();
          }

          for (uint8_t i = 0; i < m_guidance_string.size(); i++)
          {
            if(strcmp(m_args.guidance_algorithm.c_str(), m_guidance_string[i].c_str()) == 0)
              m_guidance_algorithm = i;
          }
          debug("Guidance algorithm selected number %d: %s",m_guidance_algorithm, m_guidance_string[m_guidance_algorithm].c_str());
        }

        Matrix
        createDiagonalMatrix(Matrix mat_in)
        {
          Matrix mat = Matrix(3,3, 0.0);

          if (mat_in.size() >= 3)
          {
            mat(0,0) = mat_in(0);
            mat(1,1) = mat_in(1);
            mat(2,2) = mat_in(2);
            debug("Matrix 3x3 with 3 values and diagonal: %f, %f, %f", mat(0,0), mat(1,1), mat(2,2));

          }
          else if (mat_in.size() == 2)
          {
            mat(0,0) = mat_in(0);
            mat(1,1) = mat_in(0);
            mat(2,2) = mat_in(1);
            debug("Matrix 3x3 with 2 values and diagonal: %f, %f, %f", mat(0,0), mat(1,1), mat(2,2));
          }
          else if (mat_in.size() == 1)
          {
            mat(0,0) = mat_in(0);
            mat(1,1) = mat_in(0);
            mat(2,2) = mat_in(0);
            debug("Matrix 3x3 with 1 value and diagonal: %f, %f, %f", mat(0,0), mat(1,1), mat(2,2));
          }
          return mat;
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

          m_tan_path_angle = (ts.end.z - ts.start.z)/ts.track_length;

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
          spew("Got DesiredSpeed message with value %f", m_desired_speed);
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
              m_centroid_state = *msg;
              spew("Centroid state pos (x,y): (%f,%f)", m_centroid_state.state->x, m_centroid_state.state->y);
              
            }
            else
            {
              spew("Got EstimatedLocalState from '%s', '%s' - updating vehicle state", resolveSystemId(msg->getSource()),resolveEntity(msg->getSourceEntity()).c_str());
              // Update local vehicle state variable
              m_vehicle_state = *msg;
              spew("Vehicle state pos (x,y): (%f,%f)", m_vehicle_state.state->x, m_vehicle_state.state->y);
              
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
          trace("Control step start");

          if (!m_args.use_controller)
            return;

          trace("Tracking state: %f, %f, %f", ts.track_pos.x, ts.track_pos.y, ts.track_pos.z);
          trace("Vehicle state: %f, %f, %f", m_vehicle_state.state->x, m_vehicle_state.state->y, m_vehicle_state.state->z);
          trace("Centroid state: %f, %f, %f", m_centroid_state.state->x,m_centroid_state.state->y,m_centroid_state.state->z);

          // Vector from centroid to vehicle in NED frame
          double p_centroid_x = m_vehicle_state.state->x - m_centroid_state.state->x;
          double p_centroid_y = m_vehicle_state.state->y - m_centroid_state.state->y;
          double p_centroid_z = m_vehicle_state.state->z - m_centroid_state.state->z; 
          
          // Rotate centroid to vehicle vector from NED into path frame
          Angles::rotate(ts.track_bearing,false, p_centroid_x, p_centroid_y); 
          trace("Centroid state in path frame: %f, %f, %f", p_centroid_x,p_centroid_y,p_centroid_z);

          // Centroid tracking state positions
          double centroid_ts_x = ts.track_pos.x - p_centroid_x;
          double centroid_ts_y = ts.track_pos.y - p_centroid_y;
          trace("Centroid TrackingState: %f, %f", centroid_ts_x, centroid_ts_y);


          // Lookahead distance on LOS line (set to LOS guidance by default, changed below)
          double Delta = m_lookahead;

          // Calculate lookahed distance based on guidance algorithm selection
          switch (m_guidance_algorithm)
          {
            case LOS_LOOKAHEAD:
            {
              // Set lookahead distance to be the user input value
              Delta = m_lookahead;
              trace("Using LOS with lookahead (%d)", m_guidance_algorithm);
              break;
            }
            case ENCLOSURE:
            {
              // Calculate squared lookahead so that it depends on cross track error
              double Delta_sq = m_lookahead*m_lookahead - centroid_ts_y*centroid_ts_y;
              // Constrain value to not take sqrt of a negative number
              Delta_sq = trimValue(Delta_sq, 0.0, m_lookahead*m_lookahead);
              // Set lookahead
              Delta = std::sqrt(Delta_sq);
              trace("Using LOS with enclosure (%d)", m_guidance_algorithm);
              break;
            }
            case PURE_PURSUIT:
            {
              // Set lookahead to the remaining distance to the next waypoint
              Delta = ts.track_length - centroid_ts_x;
              trace("Using pure pursuit (%d)", m_guidance_algorithm);
              break;
            }
            default:
              break;
          }

          // Create the desired velocity vector in path frame
          double v_x = Delta;
          double v_y = -centroid_ts_y;
          double v_z = 0.0;

          if (m_args.control_height)
          {
            // Centroid height (using the estimated state)
            double centroid_height = -state.height - p_centroid_z;
            // Current height error from path
            double h_err = (-ts.start.z - m_tan_path_angle * trimValue(centroid_ts_x, 0.0, abs(centroid_ts_x))) - centroid_height;
            // Height error at the lookahead distance
            v_z = h_err - m_tan_path_angle * Delta;
            trace("Centroid and path heights: %f, %f", centroid_height, h_err+centroid_height);
          }
          trace("Formation desired speed (path frame) %f, %f, %f",v_x,v_y,v_z);

          // Rotate vector from path frame into NED
          Angles::rotate(ts.track_bearing,false,v_x,v_y);
          trace("Formation desired speed (NED frame) %f, %f",v_x,v_y);

          // Rotate vector from NED frame into centroid frame
          Angles::rotate(m_centroid_state.state->psi, false, v_x,v_y);
          trace("Formation desired speed (centroid frame) %f, %f", v_x,v_y);

          // Calculate length of velocity vector
          double v_length = std::sqrt(v_x*v_x + v_y*v_y + v_z*v_z);

          // Get the desired velocity magnitude
          double v_set = m_desired_speed;

          // Calculate the scaled reference velocity (so it has the desired magnitude)
          Matrix v_ref = Matrix(3,1);
          v_ref(0) = v_set*v_x/v_length;
          v_ref(1) = v_set*v_y/v_length;
          v_ref(2) = v_set*v_z/v_length;
          trace("Formation desired speed (centroid frame scaled) %f, %f, %f", v_ref(0),v_ref(1), v_ref(2));

          // Converting wind to path reference frame
          //double w_x = m_W_x, w_y = m_W_y;
          //Angles::rotate(ts.track_bearing, false, w_x, w_y);


          // Create variable for reference velocity (output of controller)
          Matrix v_out = Matrix(3,1);
          Matrix a_out = Matrix(3,1);
          // Filter the velocity reference if filter is activated
          if (m_args.use_refmodel)
          {
            stepRefmodel(v_ref, Clock::get()-m_last_loop_time);
            v_out = getRefmodelVel();
            a_out = getRefmodelAcc();
          }
          else
          {
            v_out = v_ref; 
            a_out = Matrix(3,1,0.0); 
          }

          // Set the time the control loop was last run
          m_last_loop_time = Clock::get(); 

          // Create desired linear state message
          IMC::DesiredLinearState desLinState;

          // Set velocities
          desLinState.vx = v_out(0);
          desLinState.vy = v_out(1);
          desLinState.vz = v_out(2);

          // Set Accelerations
          desLinState.ax = a_out(0);
          desLinState.ay = a_out(1);
          desLinState.az = a_out(2);

          desLinState.flags = IMC::DesiredLinearState::FL_VX |
                              IMC::DesiredLinearState::FL_VY |
                              IMC::DesiredLinearState::FL_VZ |
                              IMC::DesiredLinearState::FL_AX |
                              IMC::DesiredLinearState::FL_AY |
                              IMC::DesiredLinearState::FL_AZ; // Keep flag just so it uses the other ones.

          // Send desired linear state message
          dispatch(desLinState);

        }

        void
        setRefmodelMatrices()
        {
          double ones[] = {1.0, 1.0, 1.0};
          Matrix omega_vel_sq = m_omega_vel * m_omega_vel; 

          Matrix eye = Matrix(ones, 3);
          Matrix zero = Matrix(3,3, 0.0);
          Matrix A_11 = zero;
          Matrix A_12 = eye; 
          Matrix A_21 = -omega_vel_sq;
          Matrix A_22 = -2.0*m_zeta_vel*m_omega_vel; 

          Matrix A_1 = A_11.horzCat(A_12);
          Matrix A_2 = A_21.horzCat(A_22); 

          m_ref.A = A_1.vertCat(A_2); 
          Matrix B_1 = zero; 
          m_ref.B = B_1.vertCat(omega_vel_sq);
          Matrix C_1 = eye; 
          Matrix C_2 = zero; 
          m_ref.Cvel = C_1.horzCat(C_2);
          Matrix C_1_a = zero; 
          Matrix C_2_a = eye; 
          m_ref.Cacc = C_1_a.horzCat(C_2_a);

          debug("Refmodel matrices set:");
          debug("A:");
          printMatrix(m_ref.A);
          debug("B:");
          printMatrix(m_ref.B);
          debug("Cvel and Cacc:");
          printMatrix(m_ref.Cvel);
          printMatrix(m_ref.Cacc);
        }

        void
        initRefmodelstate()
        {
          m_ref.x = Matrix(6, 1, 0.0);
          m_ref.x(0) = m_centroid_state.state->u;
          m_ref.x(1) = m_centroid_state.state->v;
          m_ref.x(2) = m_centroid_state.state->w; 
          m_ref.x(3) = m_centroid_state.acc->x;
          m_ref.x(4) = m_centroid_state.acc->y;
          m_ref.x(5) = m_centroid_state.acc->z;

          debug("RefModel state set:");
          printMatrix(m_ref.x);
        }

        void
        printMatrix(Matrix m, DUNE::Tasks::DebugLevel dbg = DEBUG_LEVEL_DEBUG)
        {
          if (getDebugLevel() >= dbg)
          {
            printf("[DEBUG Matrix]\n");
            for (int i = 0; i < m.rows(); i++)
            {
              for (int j = 0; j < m.columns(); j++)
              {
                printf("%f ", m.element(i, j));
              }
              printf("\n");
            }
          }
        }

        void
        initRefmodel()
        {
         setRefmodelMatrices();  
         initRefmodelstate(); 
        }

        void
        stepRefmodel(Matrix input, double deltaT)
        {
          m_ref.x = m_ref.x + m_ref.A * m_ref.x * deltaT + m_ref.B * input * deltaT;
        }

        Matrix
        getRefmodelVel()
        {
          return m_ref.Cvel * m_ref.x;
        }

        Matrix
        getRefmodelAcc()
        {
          return m_ref.Cacc * m_ref.x;
        }

      };
    }
  }
}

DUNE_TASK
