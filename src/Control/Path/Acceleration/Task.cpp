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
#include <deque>

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
      using std::deque;

      struct Arguments
      {
        bool use_controller;
        double max_acc;
        double refmodel_max_acc;
        double refmodel_omega_n;
        double refmodel_xi;
        double Kp;
        double Kd;
        double Ki;
        bool use_altitude;
        double max_integral;
        bool reset_integral_on_path_activation;
        float copter_mass_kg;
        float suspended_rope_length;
        float suspended_load_mass_g;
        float ks[3];
        float pd;
        int tau_d_extra_ms;
        double Gd_extra;
        bool reset_to_state;
        bool enable_input_shaping;
        bool enable_delayed_feedback;
        bool enable_slung_control;
        bool enable_hold_position;
        bool enable_mass_division;
        bool enable_integrator;
        double ctrl_omega_b;
        double ctrl_xi;
        bool enable_wind_ff;
        bool enable_wind_square_vel;
        float wind_drag_coefficient;
        double prefilter_time_constant;
      };

      static const std::string c_parcel_names[] = {DTR_RT("PID"), DTR_RT("Beta-X"),
                                                   DTR_RT("Beta-Y"), DTR_RT("Beta-Z"),
                                                   DTR_RT("Alpha45-Phi"), DTR_RT("Alpha45-Theta"), "Delayed-x", "Delayed-y", "PID-X", "PID-Y", "PID-Z", "ERROR", "ERROR-X", "ERROR-Y", "ERROR-Z"};
      enum Parcel {
        PC_PID = 0,
        PC_BETA_X = 1,
        PC_BETA_Y = 2,
        PC_BETA_Z = 3,
        PC_ALPHA45_PHI = 4,
        PC_ALPHA45_THETA = 5,
        PC_DELAYED_X = 6,
        PC_DELAYED_Y = 7,
        PC_PID_X = 8,
        PC_PID_Y = 9,
        PC_PID_Z = 10,
        PC_ERROR = 11,
        PC_ERROR_X = 12,
        PC_ERROR_Y = 13,
        PC_ERROR_Z = 14
      };

      static const int NUM_PARCELS = 15;

      enum ControllerType {
        CT_PID,
        CT_PID_INPUT,
        CT_SUSPENDED,
        CT_SUSPENDED_INPUT
      };

      class ReferenceHistoryContainer
      {
      public:
        Matrix state;
        double timestamp;

        ReferenceHistoryContainer():state(9,1,0.0),timestamp(0) {};
        ReferenceHistoryContainer(Matrix state_, double timestamp_): state(state_), timestamp(timestamp_) {};
      };

      struct InputFilterConfig
      {
        double t1;
        double t2;
        float A1;
        float A2;
      };

      // Angle measurements in NED
      class LoadAngle
      {
      public:
        double phi;
        double theta;
        double dphi;
        double dtheta;
        double timestamp;
        LoadAngle(): phi(0.0), theta(0.0), dphi(0.0), dtheta(0.0), timestamp(0.0){};
      };

      class LoadAngleHistoryContainer
      {
      public:
        LoadAngle angle;
        double timestamp;

        LoadAngleHistoryContainer():timestamp(0.0) {};
        LoadAngleHistoryContainer(LoadAngle angle_, double timestamp_): angle(angle_), timestamp(timestamp_) {};
      };

      class ReferenceModel
      {
      public:

        ReferenceModel():
          A(9,9, 0.0),
          B(9,3, 0.0),
          x(9,1, 0.0),
          a_out(3,1, 0.0),
          prefilterState(3,1, 0.0),
          k1(0.0),k2(0.0),k3(0.0)
      {
          /* Intentionally Empty */
      }
        Matrix
        getPos(void) { return x.get(0,2, 0,0); }

        Matrix
        getVel(void) { return x.get(3,5, 0,0); }

        Matrix
        getAcc(void) { return x.get(6,8, 0,0); }

        Matrix
        getAOut(void) { return a_out; };

        void
        setPos(Matrix& pos) { x.put(0,0, pos); }

        void
        setVel(Matrix& vel) { x.put(3,0, vel); }

        void
        setAcc(Matrix& acc) { x.put(6,0, acc); }

        void
        setAOut(Matrix& a) { a_out.put(0, 0, a); }


      public:
        Matrix A;
        Matrix B;
        Matrix x;
        Matrix a_out;
        Matrix prefilterState;
        double k1,k2,k3;
      };

      class Reference
      {
      public:
        Reference():
          x(9,1, 0.0)
        {

        }

        Matrix
        getPos(void) const { return x.get(0,2, 0,0); }

        Matrix
        getVel(void) { return x.get(3,5, 0,0); }

        Matrix
        getAcc(void) { return x.get(6,8, 0,0); }

        void
        setPos(Matrix& pos) { x.put(0,0, pos); }

        void
        setVel(Matrix& vel) { x.put(3,0, vel); }

        void
        setAcc(Matrix& acc) { x.put(6,0, acc); }

        void
        setReference(Matrix& x_new) { x.put(0,0, x_new); }

      public:

        Matrix x;
      };


      class DelayedFeedbackState
      {
      public:

        DelayedFeedbackState():
          addPos(3,1, 0.0),
          addVel(3,1, 0.0),
          addAcc(3,1, 0.0)
          {
          /* Intentionally Empty */
          }

        Matrix addPos;
        Matrix addVel;
        Matrix addAcc;
      };

      class InputShapingFilterState
      {
      public:
        InputShapingFilterState():
          filteredRef(9, 1, 0.0)
        {
          /* Intentionally Empty */
        }

        Matrix filteredRef;
      };

      struct Task: public DUNE::Control::PathController
      {
        IMC::DesiredControl m_desired_control;
        //! Task arguments.
        Arguments m_args;
        //! Reference Model
        ReferenceModel m_refmodel;
        //! Current autopilot mode
        IMC::AutopilotMode m_autopilot_mode;
        //! Current integrator value
        Matrix m_integrator_value;
        //! Timestamp of previous step
        double m_timestamp_prev_step;
        //! Reference history for input shaper
        queue<ReferenceHistoryContainer> m_refhistory;
        //! Input shaper preferences
        InputFilterConfig m_input_cfg;
        //! Translational setpoint for logging
        IMC::DesiredLinearState m_setpoint_log;
        //! Previous controller output
        Matrix m_prev_controller_output;
        //! Last Estimated State received
        IMC::EstimatedState m_estate;
        //! Last angle measurements
        LoadAngle m_loadAngle;
        //! MassMatrix and stuff
        double m_massMatrix[25];
        double m_coreolisMatrix[25];
        Matrix m_MassMatrix;
        Matrix m_CoreolisMatrix;
        Matrix m_Gravity;
        Matrix m_alpha_45;
        //! Parcel array
        IMC::ControlParcel m_parcels[NUM_PARCELS];
        IMC::EulerAngles m_calculated_angles;
        //! AngleHistory
        deque<LoadAngleHistoryContainer> m_anglehistory;
        Matrix m_delayed_feedback_desired_pos;
        //! Delayed Feedback State
        DelayedFeedbackState m_delayed_feedback_state;
        //! Shaping Filter State
        InputShapingFilterState m_input_shaping_state;
        //! The current reference
        Reference m_reference;
        //! Current desired speed
        IMC::DesiredSpeed m_dspeed;
        //! Current desired Z
        IMC::DesiredZ m_dz;



        Task(const std::string& name, Tasks::Context& ctx):
          DUNE::Control::PathController(name, ctx),
          m_integrator_value(3,1, 0.0),
          m_timestamp_prev_step(0.0),
          m_prev_controller_output(3,1, 0.0),
          m_MassMatrix(5, 5, 0.0),
          m_CoreolisMatrix(5,5, 0.0),
          m_Gravity(5,5, 0.0),
          m_alpha_45(2,1, 0.0),
          m_delayed_feedback_desired_pos(3,1, 0.0)
        {

          param("Acceleration Controller", m_args.use_controller)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .defaultValue("false")
          .description("Enable Acc Controller");

          param("Max Acceleration", m_args.max_acc)
          .defaultValue("3")
          .units(Units::MeterPerSquareSecond)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Max acceleration of the vehicle");

          param("Ref - Max Acceleration", m_args.refmodel_max_acc)
          .defaultValue("1.5")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .description("Max acceleration of the reference model.");

          param("Ref - Natural Frequency",m_args.refmodel_omega_n)
          .units(Units::RadianPerSecond)
          .defaultValue("0.5")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .description("Natural frequency for the speed reference model");

          param("Ref - Relative Damping", m_args.refmodel_xi)
          .units(Units::None)
          .defaultValue("0.9")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .description("Relative Damping Factor of the speed reference model");

          param("Ref - Pre-filter Time Constant", m_args.prefilter_time_constant)
          .defaultValue("0.05")
          .minimumValue("0.01")
          .maximumValue("0.99")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Sets the pre-filter time constant. Higher value for slower transition. ");

          param("Ref - Reset to position on path startup", m_args.reset_to_state)
          .defaultValue("false")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Set to reset to state rather than previus ref_pos on change");

          param("Controller - Bandwidth", m_args.ctrl_omega_b)
          .defaultValue("0.7")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Controller bandwidth");

          param("Controller - Relative Damping", m_args.ctrl_xi)
          .defaultValue("1.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Controller damping");

          param("Controller - Kp", m_args.Kp)
          .units(Units::None)
          .defaultValue("1.152")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("P-Gain of the velocity controller");

          param("Controller - Kd", m_args.Kd)
          .units(Units::None)
          .defaultValue("2.24")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("D-Gain of the velocity controller");

          param("Controller - Ki", m_args.Ki)
          .units(Units::None)
          .defaultValue("0.08064")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("I-Gain of the velocity controller");

          param("Controller - Enable Integrator", m_args.enable_integrator)
          .defaultValue("true")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Enable or disable I-term in controller.");

          param("Controller - Max Integral", m_args.max_integral)
          .defaultValue("20")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Max integral value");

          param("Controller - Use Altitude", m_args.use_altitude)
          .defaultValue("true")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Choose whether altitude is controlled or not (set to 0 if not).");

          param("Controller - Reset integrator on path activation", m_args.reset_integral_on_path_activation)
          .defaultValue("false")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .description("Choose whether to reset the integrator in a path activation. ");

          param("Model - Copter Mass", m_args.copter_mass_kg)
          .defaultValue("2.5")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .units(Units::Kilogram)
          .description("Mass of the copter");

          param("Model - Suspended Load Mass", m_args.suspended_load_mass_g)
          .defaultValue("250")
          .units(Units::Gram)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Mass of suspended object. Used for various filters. ");

          param("Model - Suspended Rope Length", m_args.suspended_rope_length)
          .defaultValue("4.3")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Length of suspended rope. Used for various filters. ");

          param("Model - Wind Drag Coefficient", m_args.wind_drag_coefficient)
          .defaultValue("0.15")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_PLAN)
          .description("Coefficient to use in wind ff");

          param("Input Shaping - Enable", m_args.enable_input_shaping)
          .defaultValue("false")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .description("Enable or disable input shaping on reference signal");

          param("Delayed - Enable Feedback", m_args.enable_delayed_feedback)
          .defaultValue("false")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .description("Enable or disable delayed feedback for slung angle minimization");

          param("Delayed - Hold Position", m_args.enable_hold_position)
          .defaultValue("false")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .description("Enable or disable hold current position functionality");

          param("Delayed - pd", m_args.pd)
          .defaultValue("0.01")
          .units(Units::None)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Estimated angle-damping.  ");

          param("Delayed - tau_d extra", m_args.tau_d_extra_ms)
          .defaultValue("-600")
          .units(Units::Millisecond)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Extra delay taime  ");

          param("Delayed - Gd extra", m_args.Gd_extra)
          .defaultValue("1.0")
          .units(Units::None)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Estimated angle-damping.  ");

          param("Slung - Enable Control", m_args.enable_slung_control)
          .defaultValue("false")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .description("Enable or disable slung control compensation");

          char buf[10];
          for (int i = 0; i < 3; i++)
          {
            sprintf(buf, "%d", i+1);
            param("Slung - K" + std::string(buf), m_args.ks[i])
            .defaultValue("1")
            .units(Units::None)
            .visibility(Tasks::Parameter::VISIBILITY_USER)
            .description("Gain for suspended controller. 3 is overall beta-changer. ");
          }

          param("CtrlMisc - Enable Output Division By Mass", m_args.enable_mass_division)
          .defaultValue("true")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Enable or disable division by copter mass in final output");

          param("CtrlMisc - Enable Wind Feed Forward", m_args.enable_wind_ff)
          .defaultValue("true")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_PLAN)
          .description("Enable to feed-forward wind effects");

          param("CtrlMisc - Enable Square Wind Velocity", m_args.enable_wind_square_vel)
          .defaultValue("true")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Use the square of the velocity to calculate wind ff");

          bind<IMC::EulerAngles>(this);
          bind<IMC::EstimatedState>(this);
          bind<IMC::DesiredSpeed>(this);
          bind<IMC::DesiredZ>(this);

          // zero-initialize mass-matrices
          for (int i = 0; i < 25; i++)
          {
            m_massMatrix[i] = 0.0;
            m_coreolisMatrix[i] = 0.0;
          }

          // Default speed
          m_dspeed.speed_units = IMC::SUNITS_METERS_PS;
          m_dspeed.value = 1.5;




        }

        void
        onUpdateParameters(void)
        {
          PathController::onUpdateParameters();

          // Set input shaper preferences

          double pd =  0.001;
          double omega_n = std::sqrt(Math::c_gravity/m_args.suspended_rope_length);
          double xi = pd/(2*omega_n*(m_args.suspended_load_mass_g/1000.0));
          double omega_d = omega_n*std::sqrt(1 - std::pow(xi,2));
          double Td = 2*Math::c_pi/omega_d;

          //input shaper values
          double input_K = exp(-(xi*Math::c_pi)/sqrt(1-pow(xi,2)));

          m_input_cfg.A1 = 1/(1 + input_K);
          m_input_cfg.A2 = input_K/(1 + input_K);
          m_input_cfg.t1 = 0.0;
          m_input_cfg.t2 = Td/2;

          // Controller variables
          if (paramChanged(m_args.ctrl_omega_b) || paramChanged(m_args.ctrl_xi))
          {
            // Update controller parameters!
            double ctrl_omega_n = m_args.ctrl_omega_b/0.64;

            double vKp = m_args.copter_mass_kg * pow(ctrl_omega_n, 2);
            double vKd = 2.0 * m_args.ctrl_xi * ctrl_omega_n * m_args.copter_mass_kg;
            double vKi = (ctrl_omega_n / 10.0) * m_args.Kp;

            if (!m_args.enable_integrator)
              vKi = 0.0;




            IMC::EntityParameter Kp, Kd, Ki;
            Kp.name = "Controller - Kp";
            Kd.name = "Controller - Kd";
            Ki.name = "Controller - Ki";

            char buffer[32];

            snprintf(buffer, sizeof(buffer), "%g", vKp);
            Kp.value = std::string(buffer);

            snprintf(buffer, sizeof(buffer), "%g", vKd);
            Kd.value = std::string(buffer);

            snprintf(buffer, sizeof(buffer), "%g", vKi);
            Ki.value = std::string(buffer);

            MessageList<IMC::EntityParameter> msgList;
            msgList.push_back(Kp);
            msgList.push_back(Kd);
            msgList.push_back(Ki);

            IMC::SetEntityParameters toSet;
            toSet.name = getEntityLabel();
            toSet.params = msgList;

            dispatch(toSet, DF_LOOP_BACK);

          }


        }

        void
        onEntityReservation(void)
        {
          PathController::onEntityReservation();

          for (unsigned i = 0; i < NUM_PARCELS; ++i)
            m_parcels[i].setSourceEntity(reserveEntity(c_parcel_names[i] + " Parcel"));

          m_calculated_angles.setSourceEntity(reserveEntity("Calculated Suspended Angles"));

        }

        virtual bool
        hasSpecificZControl(void) const
        {
          return false;
        }

        void
        consume(const IMC::EstimatedState* estate)
        {
          m_estate = *estate;
        }

        void
        consume(const IMC::DesiredZ* zref)
        {
          trace("Got Desired Z. ");
          if (zref->z_units == IMC::Z_ALTITUDE || zref->z_units == IMC::Z_HEIGHT)
          {
            m_dz = *zref;
          }
        }

        void
        consume(const IMC::DesiredSpeed* dspeed)
        {

          // Sanity check, and set internal speed.
          if (dspeed->speed_units == IMC::SUNITS_METERS_PS &&
              dspeed->value > 0 && dspeed->value < 10)
          {
            m_dspeed.value = dspeed->value;
          }

          PathController::consume(dspeed);
        }

        //! Consumer for EulerAngles message
        void
        consume(const IMC::EulerAngles* eangles)
        {
          // Need to convert to NED-measurements. (Sensor is mounted on body)

          // update wire length.
          Matrix load_z_vect = Matrix(3,1,0);
          load_z_vect(2) = m_args.suspended_rope_length;

          Matrix loadRnl = Rzyx(m_estate.phi,m_estate.theta,m_estate.psi) *
              Rx(eangles->phi)*Ry(eangles->theta);

          Matrix loadPosNED = loadRnl * load_z_vect;

          // Insert new element at the end
          LoadAngle newLoadAngles;
          newLoadAngles.timestamp = eangles->time;
          double dt = newLoadAngles.timestamp - m_loadAngle.timestamp;



          //newLoadAngles.phi   = atan2( loadRnl(2,1),loadRnl(1,1) );
          newLoadAngles.phi   = asin(loadRnl(2,1));

          //newLoadAngles.theta = atan2( loadRnl(0,2),loadRnl(0,0) );
          newLoadAngles.theta = asin(loadRnl(0,2));

          newLoadAngles.phi   = -atan((double) (loadPosNED(1)/loadPosNED(2)));
         newLoadAngles.theta = atan((double) (loadPosNED(0)/loadPosNED(2)));

          spew("Calculations: (atan/acossin): phi: %f, %f. Theta: %f, %f", newLoadAngles.phi, asin(loadRnl(2,1)), newLoadAngles.theta, asin(loadRnl(0,2)));
          spew("Determinant: %f", loadRnl.det());
          /*
          char buf[1024];
          int counter = 0;
          for (int i = 0; i<3; i++)
          {
            counter += snprintf(&buf[counter-1], 1024-counter, "\n");
            for (int j = 0; j<3; j++)
            {
              counter += snprintf(&buf[counter-1], 1024-counter, "%.2f, ", loadRnl(i,j));
            }
          }
          inf("R is: %s", buf);
          */

          if (dt > 0.0 && dt < 1)
          {
            newLoadAngles.dphi   = (newLoadAngles.phi - m_loadAngle.phi) / dt;
            newLoadAngles.dtheta = (newLoadAngles.theta - m_loadAngle.theta) / dt;
          }
          else
          {
            debug("Angle DT is: %f", dt);
          }

          // store
          m_loadAngle = newLoadAngles;

          // Update system matrices with new values
          updateSystemMatrices();

          // Dispatch the new angels in logs
          m_calculated_angles.phi = newLoadAngles.phi;
          m_calculated_angles.theta = newLoadAngles.theta;
          m_calculated_angles.psi = newLoadAngles.dphi;
          m_calculated_angles.psi_magnetic = newLoadAngles.dtheta;

          dispatch(m_calculated_angles);

          spew("Got and processed new angle measurement");

          // store
          m_anglehistory.push_back(LoadAngleHistoryContainer(newLoadAngles, eangles->time));
        }


        void
        updateSystemMatrices()
        {
          m_MassMatrix = MassMatrix();
          m_CoreolisMatrix = CoreolisMatrix();
          m_Gravity = Gravity();
        }

        void
        initRefmodel(const IMC::EstimatedState& state)
        {
          // Convencience matrix
          double ones[] = {1.0, 1.0, 1.0};

          Matrix eye = Matrix(ones, 3);
          Matrix zero = Matrix(3,3, 0.0);


          // Restart refmodel
          if (m_args.reset_to_state || Clock::get() - m_timestamp_prev_step > 2.0)
          {
            m_refmodel.x = Matrix(9, 1, 0.0);
            m_refmodel.x(0) = state.x;
            m_refmodel.x(1) = state.y;
            m_refmodel.x(2) = state.z;

            m_refmodel.x(3) = state.vx;
            m_refmodel.x(4) = state.vy;
            m_refmodel.x(5) = state.vz;

            m_refmodel.x(6) = 0.0;
            m_refmodel.x(7) = 0.0;
            m_refmodel.x(8) = 0.0;

            m_refmodel.a_out(0) = 0.0;
            m_refmodel.a_out(1) = 0.0;
            m_refmodel.a_out(2) = 0.0;

            m_refmodel.prefilterState(0) = m_refmodel.x(0);
            m_refmodel.prefilterState(1) = m_refmodel.x(1);
            m_refmodel.prefilterState(2) = m_refmodel.x(2);

            // Consider using last setpoint as acc startup
            if (Clock::get() - m_timestamp_prev_step < 2.0)
            {
              m_refmodel.x(6) = m_prev_controller_output(0);
              m_refmodel.x(7) = m_prev_controller_output(1);
              m_refmodel.x(8) = m_prev_controller_output(2);
            }

            debug("Did a reference model reset on path switch. ");
          }


          m_refmodel.k3 =  (2*m_args.refmodel_xi + 1) *     m_args.refmodel_omega_n;
          m_refmodel.k2 = ((2*m_args.refmodel_xi + 1) * pow(m_args.refmodel_omega_n, 2)) /  m_refmodel.k3;
          m_refmodel.k1 =                               pow(m_args.refmodel_omega_n, 3)  / (m_refmodel.k3 * m_refmodel.k2);


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

          m_refmodel.A = A_1.vertCat(A_2.vertCat(A_3));

          m_refmodel.B = Matrix(6,3, 0.0).vertCat(eye) * pow(m_args.refmodel_omega_n,3);

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
          // If switch, and always if more than x seconds since last step.
          if (m_args.reset_integral_on_path_activation || Clock::get() - m_timestamp_prev_step > 4.0)
            m_integrator_value = Matrix(3, 1, 0.0);

          // If more than 4 seconds since last step, flush history
          if (Clock::get() - m_timestamp_prev_step > 4.0)
          {
            while(!m_anglehistory.empty()) m_anglehistory.pop_front();
          }

          // check  the latest timestamp.
          if (!m_refhistory.empty() && Clock::get() - m_refhistory.back().timestamp > 4.0)
          {
            // Stack Overflow suggests swapping with empty, but doing this for now.
            while(!m_refhistory.empty()) m_refhistory.pop();
          }

          // If more than 2 seconds since last step, restart suspended integrator
          if (Clock::get() - m_timestamp_prev_step > 4.0)
          {
            m_alpha_45 = Matrix(2,1, 0.0);
          }

          // Store current position as reference for delayed feedback hover control
          m_delayed_feedback_desired_pos = Matrix(3,1, 0.0);
          m_delayed_feedback_desired_pos(0) = state.x;
          m_delayed_feedback_desired_pos(1) = state.y;
          m_delayed_feedback_desired_pos(2) = state.z;


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


        void
        stepNewRefModel(const IMC::EstimatedState& state, const TrackingState& ts)
        {
          (void) state;

          float desiredZ = state.z;
          // Z ref handling
          if (m_dz.z_units == IMC::Z_HEIGHT)
          {
            desiredZ = state.height - m_dz.value;
          }
          else if(m_dz.z_units == IMC::Z_ALTITUDE)
          {
            desiredZ = state.z + state.alt - m_dz.value;
          }



          // Get target position
          TrackingState::Coord targetPosition = ts.end;

          if (m_args.enable_hold_position)
          {
            targetPosition = ts.start;
            // Hack
            targetPosition.z = -targetPosition.z;
          }
          Matrix x_d = Matrix(3, 1, 0.0);
          x_d(0) = targetPosition.x;
          x_d(1) = targetPosition.y;
          x_d(2) = desiredZ; // z is received as height. For copters, state.height will usually be zero.
          trace("x_d:\t [%1.2f, %1.2f, %1.2f]",
              x_d(0), x_d(1), x_d(2));

          trace("Using parameters k[1-3]: %.4f, %.4f, %.4f", m_refmodel.k1, m_refmodel.k2, m_refmodel.k3 );

          //double T = m_args.prefilter_time_constant;
          //m_refmodel.prefilterState += ts.delta * (-(1/T)*m_refmodel.prefilterState + (1/T)*x_d);

          double beta = m_args.prefilter_time_constant;
          m_refmodel.prefilterState = (beta) * m_refmodel.prefilterState + (1-beta) * x_d;

          // Step 1: V-part
          Matrix tau1 = m_refmodel.k1 * (m_refmodel.prefilterState - m_refmodel.getPos());

          // Set heave to 0 if not controlling altitude
          if (!m_args.use_altitude)
            tau1(2) = 0.0;

          if (tau1.norm_2() > m_dspeed.value)
          {
            tau1 = m_dspeed.value * tau1 / tau1.norm_2();
          }

          // Step 2: A-part
          Matrix tau2 = m_refmodel.k2 * (tau1 - m_refmodel.getVel());

          // Set heave to 0 if not controlling altitude
          if (!m_args.use_altitude)
            tau2(2) = 0.0;

          if (tau2.norm_2() > m_args.refmodel_max_acc)
          {
            tau2 = m_args.refmodel_max_acc * tau2 / tau2.norm_2();
          }

          // Step 3: J-part
          Matrix tau3 = m_refmodel.k3 * (tau2 - m_refmodel.getAcc());

          // Set heave to 0 if not controlling altitude
          if (!m_args.use_altitude)
            tau3(2) = 0.0;

          // Integrate
          m_refmodel.x += ts.delta * (m_refmodel.x.get(3,8,0,0).vertCat(tau3));

          // Set correct acc output
          Matrix acc = m_refmodel.getAcc();
          m_refmodel.setAOut(acc);

          // Log
          m_setpoint_log.x = m_refmodel.x(0);
          m_setpoint_log.y = m_refmodel.x(1);
          m_setpoint_log.z = m_refmodel.x(2);
          m_setpoint_log.vx = m_refmodel.x(3);
          m_setpoint_log.vy = m_refmodel.x(4);
          m_setpoint_log.vz = m_refmodel.x(5);
          m_setpoint_log.ax = m_refmodel.x(6);
          m_setpoint_log.ay = m_refmodel.x(7);
          m_setpoint_log.az = m_refmodel.x(8);

          // Print reference pos and vel
          trace("x_r:\t [%1.2f, %1.2f, %1.2f]",
              m_refmodel.x(0), m_refmodel.x(1), m_refmodel.x(2));
          trace("v_r:\t [%1.2f, %1.2f, %1.2f]",
              m_refmodel.x(3), m_refmodel.x(4), m_refmodel.x(5));
        }

        void
        stepRefModel(const IMC::EstimatedState& state, const TrackingState& ts)
        {
          (void) state;


          // Get target position
          TrackingState::Coord targetPosition = ts.end;

          if (m_args.enable_hold_position)
          {
            targetPosition = ts.start;
            // Hack
            targetPosition.z = -targetPosition.z;
          }
          Matrix x_d = Matrix(3, 1, 0.0);
          x_d(0) = targetPosition.x;
          x_d(1) = targetPosition.y;
          x_d(2) = -targetPosition.z - state.height; // z is received as height. For copters, state.height will usually be zero.
          trace("x_d:\t [%1.2f, %1.2f, %1.2f]",
              x_d(0), x_d(1), x_d(2));

          Matrix old_pos = m_refmodel.getPos();
          Matrix old_vel = m_refmodel.getVel();
          // Update reference
          //m_refmodel.x += ts.delta * (m_refmodel.A * m_refmodel.x + m_refmodel.B * m_refmodel.prefilterState);
          m_refmodel.x += ts.delta * (m_refmodel.A * m_refmodel.x + m_refmodel.B * x_d);

          // Saturate reference velocity
          Matrix vel = m_refmodel.getVel();
          Matrix acc = m_refmodel.getAcc();

          // Set heave to 0 if not controlling altitude
          if (!m_args.use_altitude)
          {
            vel(2) = 0.0;
            acc(2) = 0.0;
          }


          bool limited_vel = false;
          if (vel.norm_2() > m_dspeed.value)
          {
            limited_vel = true;
            vel = m_dspeed.value * vel / vel.norm_2();
            m_refmodel.setVel(vel);

            // Recalculate pos
            Matrix new_pos = old_pos + ts.delta * m_refmodel.getVel();

            m_refmodel.setPos(new_pos);
          }

          if (acc.norm_2() > m_args.refmodel_max_acc)
          {
            acc = m_args.refmodel_max_acc * acc / acc.norm_2();
            m_refmodel.setAcc(acc);
          }
          // Set A out
          m_refmodel.setAOut(acc);

          // Update A out if limited velocity
          if (limited_vel)
          {
            // numeric filter
            m_refmodel.a_out = (m_refmodel.getVel() - old_vel) / ts.delta;

            // Numeric filter with smoothing time constant.
            //m_refmodel.a_out += ts.delta * (-(1/T)*m_refmodel.a_out + (1/T)*(m_refmodel.getVel() - old_vel) / ts.delta);

          }

          m_setpoint_log.x = m_refmodel.x(0);
          m_setpoint_log.y = m_refmodel.x(1);
          m_setpoint_log.z = m_refmodel.x(2);
          m_setpoint_log.vx = m_refmodel.x(3);
          m_setpoint_log.vy = m_refmodel.x(4);
          m_setpoint_log.vz = m_refmodel.x(5);
          m_setpoint_log.ax = m_refmodel.x(6);
          m_setpoint_log.ay = m_refmodel.x(7);
          m_setpoint_log.az = m_refmodel.x(8);

          // Print reference pos and vel
          trace("x_r:\t [%1.2f, %1.2f, %1.2f]",
              m_refmodel.x(0), m_refmodel.x(1), m_refmodel.x(2));
          trace("v_r:\t [%1.2f, %1.2f, %1.2f]",
              m_refmodel.x(3), m_refmodel.x(4), m_refmodel.x(5));
        }

        void
        updateDelayedFeedbackState(double now)
        {
          // This function uses the angle-history to update the current additions to the reference signal

          // To be able to continiously update the parameters, they are re-calculated each step.
          double pd =  0.001;
          double omega_n = std::sqrt(Math::c_gravity/m_args.suspended_rope_length);
          double xi = pd/(2*omega_n*(m_args.suspended_load_mass_g/1000.0));
          double omega_d = omega_n*std::sqrt(1 - std::pow(xi,2));
          double Td = 2*Math::c_pi/omega_d;
          double tau_d = 0.325*Td + m_args.tau_d_extra_ms / 1000.0;
          double Gd    = 0.325 * m_args.Gd_extra;

          double pL = m_args.suspended_rope_length;

          trace("Angle history size: %lu", m_anglehistory.size());




          // check if we have angles far enough back
          while ( m_anglehistory.size() >=1 && now - m_anglehistory.front().timestamp >= tau_d)
          {

            LoadAngle oldAngle = m_anglehistory.front().angle;


            m_delayed_feedback_state.addPos(0)     =  sin(oldAngle.theta);
            m_delayed_feedback_state.addPos(1)     = -sin(oldAngle.phi);


            m_delayed_feedback_state.addVel(0)     =  cos(oldAngle.theta) * oldAngle.dtheta;
            m_delayed_feedback_state.addVel(1)     = -cos(oldAngle.phi)   * oldAngle.dphi;


            m_delayed_feedback_state.addAcc(0)     = -sin(oldAngle.theta) * oldAngle.dtheta;
            m_delayed_feedback_state.addAcc(1)     =  sin(oldAngle.phi)   * oldAngle.dphi;

            m_delayed_feedback_state.addPos = Gd*pL*m_delayed_feedback_state.addPos;
            m_delayed_feedback_state.addVel = Gd*pL*m_delayed_feedback_state.addVel;
            m_delayed_feedback_state.addAcc = Gd*pL*m_delayed_feedback_state.addAcc;


            trace("Delayed: Calculating");

            // if the next one is also valid, it is ok to delete this one. And we continue the loop
            if ( m_anglehistory.size() >= 2 && (now - m_anglehistory.at(1).timestamp >= tau_d))
            {
              m_anglehistory.pop_front();
            }
            else
              break;




          }

          // log
          m_parcels[PC_DELAYED_X].p = m_delayed_feedback_state.addPos(0);
          m_parcels[PC_DELAYED_X].d = m_delayed_feedback_state.addVel(0);
          m_parcels[PC_DELAYED_X].i = m_delayed_feedback_state.addAcc(0);

          m_parcels[PC_DELAYED_Y].p = m_delayed_feedback_state.addPos(1);
          m_parcels[PC_DELAYED_Y].d = m_delayed_feedback_state.addVel(1);
          m_parcels[PC_DELAYED_Y].i = m_delayed_feedback_state.addAcc(1);

          dispatch(m_parcels[PC_DELAYED_X]);
          dispatch(m_parcels[PC_DELAYED_Y]);

        }

        void
        clearDelayedFeedbackState()
        {
          // Nop for now
        }

        void
        updateInputShapingState(double now)
        {
          // Store current history
          m_refhistory.push(ReferenceHistoryContainer(m_refmodel.x, now));

          Matrix t2Ref = Matrix(9, 1, 0.0);
          Matrix new_ref = Matrix(9, 1, 0.0);
          // Peek first, to see if we have one far enough back in time
          if (now - m_refhistory.front().timestamp >= m_input_cfg.t2)
          {
            t2Ref = m_refhistory.front().state;
            m_refhistory.pop();

            new_ref = m_refmodel.x * m_input_cfg.A1 + t2Ref * m_input_cfg.A2;
          }
          else
          {
            // Use only the first refmodel
            new_ref = m_refhistory.front().state;
          }

          m_input_shaping_state.filteredRef = new_ref;
        }

        void
        clearInputShapingState()
        {
          // Nop for now
        }

        void
        updateReference(const IMC::EstimatedState& state, const TrackingState& ts, double now)
        {
          // Updates the reference model, and checks which modules are enabled.

          stepNewRefModel(state, ts);

          trace("Stepping with a td of %.3f, freq: %.3f", ts.delta, 1/ts.delta);

          // Set reference from reference model
          m_reference.setReference(m_refmodel.x);

          // Use numeric filtered version
          Matrix a_out = m_refmodel.a_out;
          m_reference.setAcc(a_out);


          if (m_args.enable_input_shaping)
          {
            // Update input shaping
            updateInputShapingState(now);

            m_reference.setReference(m_input_shaping_state.filteredRef);
          }

          if (m_args.enable_delayed_feedback)
          {
            // Update delayed feedback state
            updateDelayedFeedbackState(now);

            // Add new states
            Matrix newPos = m_reference.getPos() + m_delayed_feedback_state.addPos;
            Matrix newVel = m_reference.getVel() + m_delayed_feedback_state.addVel;
            Matrix newAcc = m_reference.getAcc() + m_delayed_feedback_state.addAcc;
            m_reference.setPos(newPos);
            m_reference.setVel(newVel);
            m_reference.setAcc(newAcc);


          }

          // Log
          m_setpoint_log.x = m_reference.x(0);
          m_setpoint_log.y = m_reference.x(1);
          m_setpoint_log.z = m_reference.x(2);
          m_setpoint_log.vx = m_reference.x(3);
          m_setpoint_log.vy = m_reference.x(4);
          m_setpoint_log.vz = m_reference.x(5);
          m_setpoint_log.ax = m_reference.x(6);
          m_setpoint_log.ay = m_reference.x(7);
          m_setpoint_log.az = m_reference.x(8);

          // Is dispatched later.
          //dispatch(m_setpoint_log);
        }

        void
        clearReferenceStates()
        {
          // Resets the reference states
          clearDelayedFeedbackState();
          clearInputShapingState();
        }

        void
        step(const IMC::EstimatedState& state, const TrackingState& ts)
        {

          if (!m_args.use_controller)
            return;

          double now = Clock::get();

          // Get current position
          Matrix curPos = Matrix(3, 1, 0.0);
          curPos(0) = state.x;
          curPos(1) = state.y;
          curPos(2) = state.z;
          trace("x:\t [%1.2f, %1.2f, %1.2f]",
              state.x, state.y, state.z);

          // Get current velocity
          Matrix curVel = Matrix(3, 1, 0.0);
          curVel(0) = state.vx;
          curVel(1) = state.vy;
          curVel(2) = state.vz;

          // Update reference mechanism
          updateReference(state, ts, now);

          // Prepare main controller container
          Matrix desiredAcc = Matrix(3,1, 0.0);

          // Main PID controller part
          // Define error signals
          Matrix error_p = m_reference.getPos() - curPos;
          Matrix error_d = m_reference.getVel() - curVel;
          Matrix refAcc  = m_reference.getAcc();

          // Integral effect
          m_integrator_value += ts.delta * error_p;
          // Negate altitude
          if (!m_args.use_altitude)
            m_integrator_value(2) = 0.0;

          // Constrain
          if (m_integrator_value.norm_2() > m_args.max_integral)
            m_integrator_value = m_args.max_integral * m_integrator_value / m_integrator_value.norm_2();

          desiredAcc = m_args.copter_mass_kg * refAcc + m_args.Kd * error_d + m_args.Kp * error_p + m_args.Ki * m_integrator_value;



          // Dispatch debug parcels
          IMC::ControlParcel parcel = m_parcels[PC_PID];
          Matrix parcel_p = m_args.Kp * error_p;
          Matrix parcel_d = m_args.Kd * error_d;
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

          IMC::ControlParcel errors = m_parcels[PC_ERROR];
          errors.p = parcel.p / m_args.Kp;
          errors.d = parcel.d / m_args.Kd;

          dispatch(errors);

          // Dispatch normed parcel
          dispatch(parcel);

          // Dispatch individual parcels
          for (int i = 0; i < 3; i++)
          {
            m_parcels[PC_PID_X + i].p = parcel_p(i);
            m_parcels[PC_PID_X + i].d = parcel_d(i);
            m_parcels[PC_PID_X + i].i = parcel_i(i);
            dispatch(m_parcels[PC_PID_X + i]);

            m_parcels[PC_ERROR_X + i].p = error_p(i);
            m_parcels[PC_ERROR_X + i].d = error_d(i);
            dispatch(m_parcels[PC_ERROR_X+i]);
          }

          IMC::RelativeState relstate;
          relstate.err_x = error_p(0); relstate.err_y = error_p(1); relstate.err_z = error_p(2);

          dispatch(relstate);


          if (m_args.enable_slung_control )
          {
            Matrix beta = Matrix(3, 1, 0.0);
            Matrix dalpha_45 = Matrix(2, 1, 0.0);

            double k2 = m_args.ks[1];
            double k1 = m_args.ks[0];
            double pd = m_args.pd;
            try{
              Matrix dTheta = Matrix(2,1, 0.0);
              dTheta(0) = m_loadAngle.dphi;
              dTheta(1) = m_loadAngle.dtheta;


              dalpha_45 = -pd *m_alpha_45 - C22()*m_alpha_45 -G2() + k2 * (dTheta - m_alpha_45) - M21() *( refAcc - k1*(error_d));
              dalpha_45 = M22_inv() * dalpha_45;


            }
            catch(...)
            {
              // don't do anything.
              war("Error when calculating dalpha_45");
            }

            // Add to normal output
            beta = C12() * m_alpha_45 + M12() * dalpha_45;

            // Store for logs
            for (int i = 0; i < 3; i++)
            {
              m_parcels[PC_BETA_X + i].p = beta(i);
              m_parcels[PC_BETA_X + i].d = desiredAcc(i);
              m_parcels[PC_BETA_X + i].i = m_args.Ki*m_integrator_value(i);
              dispatch(m_parcels[PC_BETA_X + i]);
            }

            desiredAcc = desiredAcc + m_args.ks[2] * beta;

            // integrate
            // and store
            m_parcels[PC_ALPHA45_PHI].p = m_alpha_45(0);
            m_parcels[PC_ALPHA45_PHI].d = dalpha_45(0);

            m_parcels[PC_ALPHA45_THETA].p = m_alpha_45(1);
            m_parcels[PC_ALPHA45_THETA].d = dalpha_45(1);

            m_alpha_45 = m_alpha_45 + ts.delta * dalpha_45;

            dispatch(m_parcels[PC_ALPHA45_PHI]);
            dispatch(m_parcels[PC_ALPHA45_THETA]);
          }

          // Apply wind feed forward
          if (m_args.enable_wind_ff)
          {

            Matrix wind_ff = Matrix(3,1, 0.0);

            wind_ff(0) = m_args.wind_drag_coefficient * state.vx;
            wind_ff(1) = m_args.wind_drag_coefficient * state.vy;
            wind_ff(2) = m_args.wind_drag_coefficient * state.vz;

            if (m_args.enable_wind_square_vel)
            {
              wind_ff(0) *= copysign(1.0, state.vx) * state.vx;
              wind_ff(1) *= copysign(1.0, state.vy) * state.vy;
              wind_ff(2) *= copysign(1.0, state.vz) * state.vz;
            }

            desiredAcc += wind_ff;

          }




          // Divide by mass
          if (m_args.enable_mass_division)
            desiredAcc = desiredAcc / m_args.copter_mass_kg;


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

          // store
          m_prev_controller_output = desiredAcc;

          m_desired_control.x = desiredAcc(0);
          m_desired_control.y = desiredAcc(1);
          m_desired_control.z = desiredAcc(2);


          m_desired_control.flags = IMC::DesiredControl::FL_X | IMC::DesiredControl::FL_Y | IMC::DesiredControl::FL_Z;

          if(!m_args.use_altitude)
            m_desired_control.flags = IMC::DesiredControl::FL_X | IMC::DesiredControl::FL_Y;


          // Hack test the transient of arducopter
          Matrix newDesiredAcc = Matrix(3, 1, 0.0);

          for (int i = 0; i < 1; ++i)
          {
            newDesiredAcc(i) = 1*copysign(1.0, -state.x + ts.end.x) + state.vx*9.81/15;
          }



          m_desired_control.x = desiredAcc(0);
          m_desired_control.y = desiredAcc(1);
          m_desired_control.z = desiredAcc(2);

          m_prev_controller_output = desiredAcc;

          dispatch(m_desired_control);

          // Dispatch linear setpoint for logging
          // At time of control dispatch
          dispatch(m_setpoint_log);


          spew("Sent acc data.");

          // Update step-time
          m_timestamp_prev_step = Clock::get();



        }
        Matrix MassMatrix()
        {
          float m_L = m_args.suspended_load_mass_g / 1000.0;
          float m_c = m_args.copter_mass_kg;
          float L   = m_args.suspended_rope_length;

          float phi_L = m_loadAngle.phi;
          float theta_L = m_loadAngle.theta;
          float epsilon = 0.001;


          m_massMatrix[0] = m_L+m_c;
          m_massMatrix[4] = L*m_L*cos(theta_L);
          m_massMatrix[6] = m_L+m_c;
          m_massMatrix[8] = -L*m_L*cos(phi_L)*cos(theta_L);
          m_massMatrix[9] = L*m_L*sin(phi_L)*sin(theta_L);
          m_massMatrix[12] = m_L+m_c;
          m_massMatrix[13] = -L*m_L*cos(theta_L)*sin(phi_L);
          m_massMatrix[14] = -L*m_L*cos(phi_L)*sin(theta_L);
          m_massMatrix[16] = -L*m_L*cos(phi_L)*cos(theta_L);
          m_massMatrix[17] = -L*m_L*cos(theta_L)*sin(phi_L);
          m_massMatrix[18] = epsilon*pow(sin(theta_L),2.0)+(L*L)*m_L*pow(cos(theta_L),2.0);
          m_massMatrix[20] = L*m_L*cos(theta_L);
          m_massMatrix[21] = L*m_L*sin(phi_L)*sin(theta_L);
          m_massMatrix[22] = -L*m_L*cos(phi_L)*sin(theta_L);
          m_massMatrix[24] = (L*L)*m_L;


          return Matrix(m_massMatrix, 5, 5);
        }

        Matrix CoreolisMatrix()
        {

          float m_L = m_args.suspended_load_mass_g / 1000.0;

          float L   = m_args.suspended_rope_length;

          float phi_L = m_loadAngle.phi;
          float theta_L = m_loadAngle.theta;
          float dphi_L = m_loadAngle.dphi;
          float dtheta_L = m_loadAngle.dtheta;

          float epsilon = 0.001;



          m_coreolisMatrix[16] = L*m_L*(dphi_L*cos(theta_L)*sin(phi_L)+dtheta_L*cos(phi_L)*sin(theta_L));
          m_coreolisMatrix[17] = -L*m_L*(dphi_L*cos(phi_L)*cos(theta_L)-dtheta_L*sin(phi_L)*sin(theta_L));
          m_coreolisMatrix[18] = dtheta_L*sin(theta_L*2.0)*(epsilon-(L*L)*m_L)*(1.0/2.0);
          m_coreolisMatrix[19] = (L*L)*dphi_L*m_L*sin(theta_L*2.0)*(1.0/2.0);
          m_coreolisMatrix[20] = -L*dtheta_L*m_L*sin(theta_L);
          m_coreolisMatrix[21] = L*m_L*(dphi_L*cos(phi_L)*sin(theta_L)+dtheta_L*cos(theta_L)*sin(phi_L));
          m_coreolisMatrix[22] = -L*m_L*(dtheta_L*cos(phi_L)*cos(theta_L)-dphi_L*sin(phi_L)*sin(theta_L));
          m_coreolisMatrix[23] = (L*L)*dphi_L*m_L*sin(theta_L*2.0)*(-1.0/2.0);

          return Matrix(m_coreolisMatrix, 5, 5);

        }
        Matrix Gravity()
        {
          float m_L = m_args.suspended_load_mass_g / 1000.0;
          float m_c = m_args.copter_mass_kg;
          float L   = m_args.suspended_rope_length;

          float phi_L = m_loadAngle.phi;
          float theta_L = m_loadAngle.theta;
          double g = Math::c_gravity;


          double T[5];
          T[0] = 0.0;
          T[1] = 0.0;
          T[2] = -g*(m_L+m_c);
          T[3] = L*g*m_L*cos(theta_L)*sin(phi_L);
          T[4] = L*g*m_L*cos(phi_L)*sin(theta_L);

          return Matrix(T, 5, 1);
        }

        Matrix M11()
        {
          return m_MassMatrix.get(0,2,0,2);
        }
        Matrix M12()
        {
          return m_MassMatrix.get(0,2,3,4);
        }
        Matrix M21()
        {
          return m_MassMatrix.get(3,4,0,2);
        }
        Matrix M22()
        {
          return m_MassMatrix.get(3,4,3,4);
        }
        Matrix M22_inv()
        {

          float m_L = m_args.suspended_load_mass_g / 1000.0;
          float L   = m_args.suspended_rope_length;

          //float phi_L = m_loadAngle.phi;
          float theta_L = m_loadAngle.theta;
          float epsilon = 0.001;


          double b[4];

          b[0] = 1.0/(epsilon*pow(sin(theta_L),2.0)+(L*L)*m_L*pow(cos(theta_L),2.0));
          b[1] = 0.0;
          b[2] = 0.0;
          b[3] = 1.0/(L*L)/m_L;

          return Matrix(b, 2, 2);
        }

        Matrix C12()
        {
          return m_CoreolisMatrix.get(0,2,3,4);
        }

        Matrix C22()
        {
          return m_CoreolisMatrix.get(3,4,3,4);
        }

        Matrix G2()
        {
          return m_Gravity.get(3,4,0,0);
        }



        //! @return  Rotation matrix.
        Matrix Rzyx(double phi, double theta, double psi) const
        {
          double R_en_elements[] = {cos(psi)*cos(theta), (-sin(psi)*cos(phi))+(cos(psi)*sin(theta)*sin(phi)), (sin(psi)*sin(phi))+(cos(psi)*cos(phi)*sin(theta)) ,
              sin(psi)*cos(theta), (cos(psi)*cos(phi))+(sin(phi)*sin(theta)*sin(psi)), (-cos(psi)*sin(phi))+(sin(theta)*sin(psi)*cos(phi)),
              -sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi)};
          return Matrix(R_en_elements,3,3);
        }
        Matrix Rx(double phi) const{
          double Rx_elements[] = {1.0, 0.0, 0.0, 0.0, cos(phi), -sin(phi), 0.0, sin(phi), cos(phi)};
          return Matrix(Rx_elements,3,3);
        }
        Matrix Ry(double theta) const{
          double Ry_elements[] = {cos(theta), 0.0, sin(theta), 0.0, 1.0, 0.0, -sin(theta), 0.0, cos(theta)};
          return Matrix(Ry_elements,3,3);
        }
      };
    }
  }
}

DUNE_TASK
