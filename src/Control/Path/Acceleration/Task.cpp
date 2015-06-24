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
#include <queue>

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Control
{
  namespace Path
  {
    namespace Acceleration
    {
      using DUNE_NAMESPACES;
      using std::queue;

      struct Arguments
      {
        bool use_controller;
        double max_speed;
        double max_acc;
        bool use_refmodel;
        double refmodel_max_speed;
        double refmodel_max_acc;
        double refmodel_omega_n;
        double refmodel_xi;
        double Kp;
        double Kd;
        double Ki;
        bool use_altitude;
        bool disable_heave;
        double max_integral;
        bool reset_integral_on_path_activation;
        std::string controller_type;
      };

      enum ControllerType {
        CT_PID,
        CT_PID_INPUT
      };

      class ReferenceHistoryContainer
      {
      public:
        Matrix state;
        double timestamp;

        ReferenceHistoryContainer():state(9,1,0.0),timestamp(0) {};
        ReferenceHistoryContainer(Matrix state, double timestamp): state(state), timestamp(timestamp) {};
      };

      struct Task: public DUNE::Control::PathController
      {
        IMC::DesiredControl m_desired_control;
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
        //! Current integrator value
        Matrix m_integrator_value;
        //! Timestamp of previous step
        double m_timestamp_prev_step;
        //! Controller type
        ControllerType m_controllerType;
        //! Reference history for input shaper
        queue<ReferenceHistoryContainer> m_refhistory;


        Task(const std::string& name, Tasks::Context& ctx):
          DUNE::Control::PathController(name, ctx),
          m_desired_speed(0),
          m_integrator_value(3,1, 0.0),
          m_timestamp_prev_step(0.0),
          m_controllerType(CT_PID)
        {

          param("Acceleration Controller", m_args.use_controller)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .defaultValue("false")
          .description("Enable Acc Controller");

          param("Max Speed", m_args.max_speed)
          .defaultValue("5.0")
          .units(Units::MeterPerSecond)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .description("Max speed of the vehicle");

          param("Max Acceleration", m_args.max_acc)
          .defaultValue("3.0")
          .units(Units::MeterPerSquareSecond)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .description("Max acceleration of the vehicle");

          param("Use Reference Model", m_args.use_refmodel)
          .defaultValue("true")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .description("Enable Reference Model.");

          param("Reference Model - Max Speed", m_args.refmodel_max_speed)
          .defaultValue("3.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Max speed of the reference model.");

          param("Reference Model - Max Acceleration", m_args.refmodel_max_acc)
          .defaultValue("2")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Max acceleration of the reference model.");

          param("Reference Model - Natural Frequency",m_args.refmodel_omega_n)
          .units(Units::RadianPerSecond)
          .defaultValue("0.5")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Natural frequency for the speed reference model");

          param("Reference Model - Relative Damping", m_args.refmodel_xi)
          .units(Units::None)
          .defaultValue("0.9")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Relative Damping Factor of the speed reference model");

          param("Acceleration Controller - Kp", m_args.Kp)
          .units(Units::None)
          .defaultValue("1")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("P-Gain of the velocity controller");

          param("Acceleration Controller - Kd", m_args.Kd)
          .units(Units::None)
          .defaultValue("1")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("D-Gain of the velocity controller");

          param("Acceleration Controller - Ki", m_args.Ki)
          .units(Units::None)
          .defaultValue("0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("I-Gain of the velocity controller");

          param("Use Altitude", m_args.use_altitude)
          .defaultValue("false")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Choose whether altitude is controlled or not (set to 0 if not).");

          param("Disable Heave flag", m_args.disable_heave)
          .defaultValue("false")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Choose whether to disable heave flag. In turn, this will utilize new rate controller on some targets");

          param("Reset integrator on path activation", m_args.reset_integral_on_path_activation)
          .defaultValue("true")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .description("Choose whether to reset the integrator in a path activation. ");

          param("Acceleration Controller - Max Integral", m_args.max_integral)
          .defaultValue("1")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Max integral value");

          param("Controller Type", m_args.controller_type)
          .values("PID,PID-InputShaper")
          .defaultValue("PID")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .description("Sets type of controller to use");

        }

        void
        onUpdateParameters(void)
        {
          PathController::onUpdateParameters();

          // update desired speed to max speed
          m_desired_speed = m_args.refmodel_max_speed;

          // Set controller type
          if (m_args.controller_type == "PID")
            m_controllerType = CT_PID;
          else if (m_args.controller_type == "PID-InputShaper")
            m_controllerType = CT_PID_INPUT;


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
          Matrix zero = Matrix(3,3, 0.0);

          // Restart refmodel
          m_refmodel_x = Matrix(9, 1, 0.0);
          m_refmodel_x(0) = state.x;
          m_refmodel_x(1) = state.y;
          m_refmodel_x(2) = state.z;

          m_refmodel_x(3) = state.vx;
          m_refmodel_x(4) = state.vy;
          m_refmodel_x(5) = state.vz;

          m_refmodel_x(6) = 0.0;
          m_refmodel_x(7) = 0.0;
          m_refmodel_x(8) = 0.0;




          // Set model

          Matrix A_11 = zero;
          Matrix A_12 = eye;
          Matrix A_13 = zero;

          Matrix A_21 = zero;
          Matrix A_22 = zero;
          Matrix A_23 = eye;

          Matrix A_31 = -pow(m_args.refmodel_omega_n, 3)*eye;
          Matrix A_32 = -(2*m_args.refmodel_xi + 1) * pow(m_args.refmodel_omega_n, 2) * eye;
          Matrix A_33 = -(2*m_args.refmodel_xi + 1) *     m_args.refmodel_omega_n     * eye;


          Matrix A_1 = A_11.horzCat(A_12.horzCat(A_13));
          Matrix A_2 = A_21.horzCat(A_22.horzCat(A_23));
          Matrix A_3 = A_31.horzCat(A_32.horzCat(A_33));

          m_refmodel_A = A_1.vertCat(A_2.vertCat(A_3));

          m_refmodel_B = Matrix(6,3, 0.0).vertCat(eye) * pow(m_args.refmodel_omega_n,3);

        }

        virtual void
        onPathStartup(const IMC::EstimatedState& state, const TrackingState& ts)
        {

          //(void)ts;


          // Print end coordinates
          debug("End coordinates: [%f, %f, %f]", ts.end.x, ts.end.y, ts.end.z);

          // Restart ref model
          initRefmodel(state);

          // Reset integral
          if (m_args.reset_integral_on_path_activation)
            m_integrator_value = Matrix(3, 1, 0.0);


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
          enableControlLoops(IMC::CL_FORCE);

          // Activate height controller
          enableControlLoops(IMC::CL_ALTITUDE);
          inf("Accel-control activated.");

        }

        // Helpers
        Matrix
        getRefPos(void)
        {
          return m_refmodel_x.get(0,2, 0,0);
        }
        Matrix
        getRefVel(void)
        {
          return m_refmodel_x.get(3,5, 0,0);
        }
        Matrix
        getRefAcc(void)
        {
          return m_refmodel_x.get(6,8, 0,0);
        }

        void
        setRefPos(Matrix& pos)
        {
          m_refmodel_x.put(0,0,pos);
        }
        void
        setRefVel(Matrix& vel)
        {
          m_refmodel_x.put(3,0,vel);
        }
        void
        setRefAcc(Matrix& acc)
        {
          m_refmodel_x.put(6,0,acc);
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

          // Get current velocity
          Matrix curVel = Matrix(3, 1, 0.0);
          curVel(0) = state.vx;
          curVel(1) = state.vy;
          curVel(2) = state.vz;

          // Get target position
          Matrix x_d = Matrix(3, 1, 0.0);
          x_d(0) = ts.end.x;
          x_d(1) = ts.end.y;
          x_d(2) = -ts.end.z; // NEU?!
          trace("x_d:\t [%1.2f, %1.2f, %1.2f]",
              x_d(0), x_d(1), x_d(2));

          Matrix vel = Matrix(3,1, 0.0);
          Matrix desiredAcc = Matrix(3,1, 0.0);

          if (m_args.use_refmodel)
          {
            // Update reference
            m_refmodel_x += ts.delta * (m_refmodel_A * m_refmodel_x + m_refmodel_B * x_d);

            // Saturate reference velocity
            vel = getRefVel();

            // Set heave to 0 if not controlling altitude
            if (!m_args.use_altitude)
            {
              vel(2) = 0;
            }

            if( vel.norm_2() > m_args.refmodel_max_speed )
            {
              vel = m_args.refmodel_max_speed * vel / vel.norm_2();
              setRefVel(vel);
            }
            spew("Vel norm: %f", m_refmodel_x.get(3,5,0,0).norm_2());

            Matrix acc = getRefAcc();
            if (!m_args.use_altitude)
              acc(2) = 0.0;

            if (acc.norm_2() > m_args.refmodel_max_acc)
            {
              acc = m_args.refmodel_max_acc * acc / acc.norm_2();
              setRefAcc(acc);
            }

            // Print reference pos and vel
            trace("x_r:\t [%1.2f, %1.2f, %1.2f]",
                m_refmodel_x(0), m_refmodel_x(1), m_refmodel_x(2));
            trace("v_r:\t [%1.2f, %1.2f, %1.2f]",
                m_refmodel_x(3), m_refmodel_x(4), m_refmodel_x(5));


            // Integral effect
            m_integrator_value += ts.delta * (getRefPos() - x);
            // Negate altitude
            if (!m_args.use_altitude)
              m_integrator_value(2) = 0.0;

            // Constrain
            if (m_integrator_value.norm_2() > m_args.max_integral)
              m_integrator_value = m_args.max_integral * m_integrator_value / m_integrator_value.norm_2();

            desiredAcc = getRefAcc() + m_args.Kd * (getRefVel() - curVel) + m_args.Kp * (getRefPos() - x) + m_args.Ki * m_integrator_value;


            // Dispatch debug parcels
            IMC::ControlParcel parcel;
            Matrix parcel_p = m_args.Kp * (getRefPos() - x);
            Matrix parcel_d = m_args.Kd * (getRefVel() - curVel);
            Matrix parcel_i = m_args.Ki * m_integrator_value;

            if (!m_args.use_altitude)
            {
              parcel_p(2) = 0.0;
              parcel_d(2) = 0.0;
              parcel_i(2) = 0.0;
            }

            parcel.p = parcel_p.norm_2();
            parcel.d = parcel_d.norm_2();
            parcel.i = parcel_i.norm_2();

            dispatch(parcel);
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
            desiredAcc(2) = 0;
          }

          // Saturate acceleration
          if( desiredAcc.norm_2() > m_args.max_acc )
          {
            desiredAcc = m_args.max_acc * desiredAcc / desiredAcc.norm_2();
          }

          m_desired_control.x = desiredAcc(0);
          m_desired_control.y = desiredAcc(1);
          m_desired_control.z = desiredAcc(2);


          m_desired_control.flags = IMC::DesiredControl::FL_X | IMC::DesiredControl::FL_Y | IMC::DesiredControl::FL_Z;

          if(m_args.disable_heave)
            m_desired_control.flags = IMC::DesiredControl::FL_X | IMC::DesiredControl::FL_Y;


          dispatch(m_desired_control);

          // Dispatch linear setpoint for logging
          IMC::TranslationalSetpoint setpoint;
          setpoint.x = m_refmodel_x(0);
          setpoint.y = m_refmodel_x(1);
          setpoint.z = m_refmodel_x(2);
          setpoint.u = m_refmodel_x(3);
          setpoint.v = m_refmodel_x(4);
          setpoint.w = m_refmodel_x(5);
          dispatch(setpoint);


          spew("Sent acc data.");
        }
      };
    }
  }
}

DUNE_TASK
