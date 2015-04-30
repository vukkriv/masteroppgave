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
// https://www.lsts.pt/dune/licence.                                        *
//***************************************************************************
// Author: Kristian Klausen                                                 *
//***************************************************************************

// ISO C++ 98 headers.
#include <cmath>

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Control
{
  namespace Path
  {
    namespace Velocity
    {
      using DUNE_NAMESPACES;

      struct Arguments
      {
        bool use_controller;
        double max_speed;
        bool use_refmodel;
        double refmodel_max_speed;
        double refmodel_omega_n;
        double refmodel_xi;
        double Kp;
        bool use_altitude;
        bool disable_heave;
      };

      struct Task: public DUNE::Control::PathController
      {
        IMC::DesiredVelocity m_velocity;
        //! Task arguments.
        Arguments m_args;
        //! Reference model state
        Matrix m_refmodel_x;
        //! Reference model trans.matrix
        Matrix m_refmodel_A;
        //! Reference model input matrix
        Matrix m_refmodel_B;
        //! Desired speed profile
        double m_desired_speed;
        //! Current autopilot mode
        IMC::AutopilotMode m_autopilot_mode;


        Task(const std::string& name, Tasks::Context& ctx):
          DUNE::Control::PathController(name, ctx),
          m_desired_speed(0)
        {

          param("Velocity Controller", m_args.use_controller)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .defaultValue("false")
          .description("Enable Velocity Controller");

          param("Max Speed", m_args.max_speed)
          .defaultValue("5.0")
          .units(Units::MeterPerSecond)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Max speed of the vehicle");

          param("Use Reference Model", m_args.use_refmodel)
          .defaultValue("true")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .description("Enable Reference Model.");

          param("Reference Model - Max Speed", m_args.refmodel_max_speed)
          .defaultValue("3.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Max speed of the reference model.");

          param("Reference Model - Natural Frequency",m_args.refmodel_omega_n)
          .units(Units::RadianPerSecond)
          .defaultValue("0.1")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Natural frequency for the speed reference model");

          param("Reference Model - Relative Damping", m_args.refmodel_xi)
          .units(Units::None)
          .defaultValue("0.9")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Relative Damping Factor of the speed reference model");

          param("Velocity Controller - Kp", m_args.Kp)
          .units(Units::None)
          .defaultValue("0.1")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("P-Gain of the velocity controller");

          param("Use Altitude", m_args.use_altitude)
          .defaultValue("false")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Choose whether altitude is controlled or not (set to 0 if not).");

          param("Disable Heave flag", m_args.disable_heave)
          .defaultValue("false")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Choose whether to disable heave flag. In turn, this will utilize new rate controller on some targets");

        }

        void
        onUpdateParameters(void)
        {
          PathController::onUpdateParameters();

          // update desired speed to max speed
          m_desired_speed = m_args.refmodel_max_speed;

        }

        void
        onEntityReservation(void)
        {
          PathController::onEntityReservation();
        }


        //! Consumer for DesiredSpeed message.
        //! @param dspeed message to consume.
        void
        consume(const IMC::DesiredSpeed* dspeed)
        {
          // overloaded.
          // Update desired speed
          if (dspeed->value < m_args.refmodel_max_speed)
          {
            m_desired_speed = dspeed->value;
          }
          else
          {
            m_desired_speed = m_args.refmodel_max_speed;
            debug("Trying to set a speed above maximum speed. ");
          }

          PathController::consume(dspeed);
        }

        void
        initRefmodel(const IMC::EstimatedState& state)
        {
          // Convencience matrix
          double ones[] = {1.0, 1.0, 1.0};

          Matrix eye = Matrix(ones, 3);

          // Restart refmodel
          m_refmodel_x = Matrix(6, 1, 0.0);
          m_refmodel_x(0) = state.x;
          m_refmodel_x(1) = state.y;
          m_refmodel_x(2) = state.z;

          m_refmodel_x(3) = state.vx;
          m_refmodel_x(4) = state.vy;
          m_refmodel_x(5) = state.vz;

          // Set model
          Matrix A_12 = eye;
          Matrix A_11 = Matrix(3,3, 0.0);

          Matrix A_21 = -pow(m_args.refmodel_omega_n, 2)*eye;
          Matrix A_22 = -2*m_args.refmodel_omega_n * m_args.refmodel_xi * eye;

          Matrix A_1 = A_11.horzCat(A_12);
          Matrix A_2 = A_21.horzCat(A_22);

          m_refmodel_A = A_1.vertCat(A_2);

          m_refmodel_B = Matrix(3,3, 0.0).vertCat(eye) * pow(m_args.refmodel_omega_n,2);

        }

        virtual void
        onPathStartup(const IMC::EstimatedState& state, const TrackingState& ts)
        {

          //(void)ts;


          // Print end coordinates
          debug("End coordinates: [%f, %f, %f]", ts.end.x, ts.end.y, ts.end.z);

          // Restart ref model
          initRefmodel(state);

        }

        virtual void
        onPathActivation(void)
        {
          if (!m_args.use_controller)
          {
            debug("Path activated, but not active: Requesting deactivation");
            requestDeactivation();
            return;
          }
          // Activate velocity controller.
          enableControlLoops(IMC::CL_SPEED);

          // Activate height controller
          enableControlLoops(IMC::CL_ALTITUDE);
          inf("Vel-control activated.");

        }

        void
        step(const IMC::EstimatedState& state, const TrackingState& ts)
        {

          if (!m_args.use_controller)
            return;

          // Get current position
          Matrix x = Matrix(3, 1, 0.0);
          x(0) = state.x;
          x(1) = state.y;
          x(2) = state.z;
          trace("x:\t [%1.2f, %1.2f, %1.2f]",
              state.x, state.y, state.z);

          // Get target position
          Matrix x_d = Matrix(3, 1, 0.0);
          x_d(0) = ts.end.x;
          x_d(1) = ts.end.y;
          x_d(2) = -ts.end.z; // NEU?!
          trace("x_d:\t [%1.2f, %1.2f, %1.2f]",
              x_d(0), x_d(1), x_d(2));

          Matrix vel = Matrix(3,1, 0.0);

          if (m_args.use_refmodel)
          {
            // Update reference
            m_refmodel_x += ts.delta * (m_refmodel_A * m_refmodel_x + m_refmodel_B * x_d);

            // Saturate reference velocity
            vel = m_refmodel_x.get(3,5,0,0);

            // Set heave to 0 if not controlling altitude
            if (!m_args.use_altitude)
            {
              vel(2) = 0;
            }

            if( vel.norm_2() > m_args.refmodel_max_speed )
            {
              vel = m_args.refmodel_max_speed * vel / vel.norm_2();
              m_refmodel_x.put(3,0,vel);
            }
            spew("Vel norm: %f", m_refmodel_x.get(3,5,0,0).norm_2());

            // Print reference pos and vel
            trace("x_r:\t [%1.2f, %1.2f, %1.2f]",
                m_refmodel_x(0), m_refmodel_x(1), m_refmodel_x(2));
            trace("v_r:\t [%1.2f, %1.2f, %1.2f]",
                m_refmodel_x(3), m_refmodel_x(4), m_refmodel_x(5));

            // Head straight to reference
            //vel(0) = m_refmodel_x(3) - m_args.Kp * (state.x - m_refmodel_x(0));
            //vel(1) = m_refmodel_x(4) - m_args.Kp * (state.y - m_refmodel_x(1));
            //vel(2) = m_refmodel_x(5) - m_args.Kp * (state.z - m_refmodel_x(2));
            vel = m_refmodel_x.get(3,5,0,0) + m_args.Kp * (m_refmodel_x.get(0,2,0,0) - x);
          }
          else
          {
            // Head straight to target
            //vel(0) = - m_args.Kp * (state.x - x_d(0));
            //vel(1) = - m_args.Kp * (state.y - x_d(1));
            //vel(2) = - m_args.Kp * (state.z - x_d(2));
            vel = m_args.Kp * (x_d - x);
          }

          // Set heave to 0 if not controlling altitude
          if (!m_args.use_altitude)
          {
            vel(2) = 0;
          }

          // Saturate velocity
          if( vel.norm_2() > m_args.max_speed )
          {
            vel = m_args.max_speed * vel / vel.norm_2();
          }

          m_velocity.u = vel(0);
          m_velocity.v = vel(1);
          m_velocity.w = vel(2);

          // Print desired velocity
          trace("v_d:\t [%1.2f, %1.2f, %1.2f]",
              m_velocity.u, m_velocity.v, m_velocity.w);

          // Todo: Add seperate altitude controller.


          m_velocity.flags = IMC::DesiredVelocity::FL_SURGE | IMC::DesiredVelocity::FL_SWAY | IMC::DesiredVelocity::FL_HEAVE;

          if(m_args.disable_heave)
            m_velocity.flags = IMC::DesiredVelocity::FL_SURGE | IMC::DesiredVelocity::FL_SWAY;


          // Kristian Test: Dispatch linear setpoint for logging
          /*
          IMC::TranslationalSetpoint setpoint;
          setpoint.x = m_refmodel_x(0);
          setpoint.y = m_refmodel_x(1);
          setpoint.z = m_refmodel_x(2);
          setpoint.u = m_refmodel_x(3);
          setpoint.v = m_refmodel_x(4);
          setpoint.w = m_refmodel_x(5);
          dispatch(setpoint);
          */

          dispatch(m_velocity);
          spew("Sent vel data.");
        }
      };
    }
  }
}

DUNE_TASK
