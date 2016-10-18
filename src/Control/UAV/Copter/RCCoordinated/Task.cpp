//***************************************************************************
// Copyright 2007-2016 Universidade do Porto - Faculdade de Engenharia      *
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
// Author: Kristian Klausen                                                 *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

// C++ Headers
#include <vector>

namespace Control
{
  namespace UAV
  {
    namespace Copter
    {
      namespace RCCoordinated
      {
        using DUNE_NAMESPACES;

        //! Controllable loops.
        static const uint32_t c_controllable = IMC::CL_PATH;
        //! Required loops.
        static const uint32_t c_required = IMC::CL_SPEED;

        struct Arguments
        {
          //! True if using controller
          bool use_controller;
          //! Max speeds
          double max_speed_xy;
          double max_speed_z;
          double max_yaw_rate;
          double max_acc;
          //! Entity to send centroid EstimatedLocalState
          std::vector<std::string> ent_centroid_elocalstate;

          // Refmodel parameters
          double refmodel_max_jerk;
          double refmodel_omega_n;
          double refmodel_xi;
        };

        enum PwmChannel {
           CH_X = 1,
           CH_Y = 0,
           CH_Z = 2,
           CH_YAW = 3,
           CH_TUNE = 5
         };

        class ReferenceModel
        {
        public:

          ReferenceModel():
            A(9,9, 0.0),
            B(9,3, 0.0),
            x(9,1, 0.0),
            k1(0.0),k2(0.0),k3(0.0)
        {
            /* Intentionally Empty */
        }
          Matrix
          getVel(void) { return x.get(0,2, 0,0); }

          Matrix
          getAcc(void) { return x.get(3,5, 0,0); }

          Matrix
          getJerk(void) { return x.get(6,8, 0,0); }

          void
          setVel(Matrix& pos) { x.put(0,0, pos); }

          void
          setAcc(Matrix& vel) { x.put(3,0, vel); }

          void
          setJerk(Matrix& acc) { x.put(6,0, acc); }

        public:
          Matrix A;
          Matrix B;
          Matrix x;

          double k1,k2,k3;
        };

        struct Task: public BasicUAVAutopilot
        {
          //! Task Arguments
          Arguments m_args;
          //! PWM inputs
          unsigned int m_pwm_inputs[8];
          //! Filter for centroid state
          SourceFilter* m_elocalstate_filter;
          //! Last received centroid elocal state
          IMC::EstimatedLocalState m_centroid_elstate;
          //! Current setpoint for centroid yaw
          double m_desired_yaw;
          //! Reference Model
          ReferenceModel m_refmodel;

          //! Constructor.
          //! @param[in] name task name.
          //! @param[in] ctx context.
          Task(const std::string& name, Tasks::Context& ctx):
            BasicUAVAutopilot(name, ctx, c_controllable, c_required),
            m_elocalstate_filter(NULL),
            m_desired_yaw(0)
          {
            param("RCCoordinated Controller", m_args.use_controller)
            .visibility(Tasks::Parameter::VISIBILITY_USER)
            .scope(Tasks::Parameter::SCOPE_MANEUVER)
            .defaultValue("false")
            .description("Enable RC Coordinated Controller");

            param("Max Speed - XY", m_args.max_speed_xy)
            .defaultValue("3")
            .visibility(Parameter::VISIBILITY_USER)
            .units(Units::MeterPerSecond);

            param("Max Speed - Z", m_args.max_speed_z)
            .defaultValue("5")
            .visibility(Parameter::VISIBILITY_USER)
            .units(Units::MeterPerSecond);

            param("Max Speed - Yaw", m_args.max_yaw_rate)
            .defaultValue("20")
            .visibility(Parameter::VISIBILITY_USER)
            .units(Units::DegreePerSecond);

            param("Max Acc", m_args.max_acc)
            .defaultValue("6")
            .visibility(Parameter::VISIBILITY_USER)
            .units(Units::MeterPerSquareSecond);

            param("Filter - Centroid EstimatedLocalState Entity", m_args.ent_centroid_elocalstate)
            .defaultValue("Formation Centroid");

            param("Ref - Max Jerk", m_args.refmodel_max_jerk)
            .defaultValue("8")
            .visibility(Tasks::Parameter::VISIBILITY_USER)
            .description("Max acceleration of the reference model.");

            param("Ref - Natural Frequency",m_args.refmodel_omega_n)
            .units(Units::RadianPerSecond)
            .defaultValue("1")
            .visibility(Tasks::Parameter::VISIBILITY_USER)
            .description("Natural frequency for the speed reference model");

            param("Ref - Relative Damping", m_args.refmodel_xi)
            .units(Units::None)
            .defaultValue("0.9")
            .visibility(Tasks::Parameter::VISIBILITY_USER)
            .description("Relative Damping Factor of the speed reference model");

            // Initialize
            m_centroid_elstate.clear();


            bind<IMC::PWM>(this);
            bind<IMC::EstimatedLocalState>(this);

            for (int i = 0; i < 8; i++)
            {
              m_pwm_inputs[i] = 1500;
            }
          }

          void
          onEntityResolution(void)
          {
            spew("Entity resolution.");

            // Process the systems allowed to define EstimatedLocalState
            m_elocalstate_filter = new Tasks::SourceFilter(*this, false, m_args.ent_centroid_elocalstate, "EstimatedLocalState");
          }

          void
          initRefmodel()
          {
            // Convencience matrix
            double ones[] = {1.0, 1.0, 1.0};

            Matrix eye = Matrix(ones, 3);
            Matrix zero = Matrix(3,3, 0.0);

            m_refmodel.x = Matrix(9, 1, 0.0);






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


          double
          pwmToValueDeadband(double val_min, double val_max, unsigned int rc_min, unsigned int rc_max, int inverse, double deadbandPercent, unsigned int rc_in)
          {
            unsigned int rc_trim = (rc_max + rc_min)/2;
            unsigned int dead_zone = (rc_max - rc_min) * deadbandPercent;

            unsigned int rc_trim_high = rc_trim + dead_zone;
            unsigned int rc_trim_low  = rc_trim - dead_zone;

            // prevent div by 0
            if ((rc_trim_low - rc_min) == 0 || (rc_max - rc_trim_high) == 0)
                return 0;

            // Sanity checks
            if ( rc_in > rc_max )
              rc_in = rc_max;
            if ( rc_in < rc_min )
              rc_in = rc_min;


            int reverse_mul = 1;
            if( inverse == 1 )
              reverse_mul = -1;


            if(rc_in > rc_trim_high) {
                return reverse_mul * ((double)val_max * (rc_in - (int)rc_trim_high)) / (rc_max  - rc_trim_high);
            }else if(rc_in < rc_trim_low) {
                return reverse_mul * ((double)val_min * ((int)rc_trim_low - rc_in)) / (rc_trim_low - rc_min);
            }else
                return 0.0;

          }

          void
          consume(const IMC::EstimatedLocalState* elstate)
          {
           // Only use local
           if (!elstate->getSource() != getSystemId())
             return;

           // Filter EstimatedLocalState by entities
           if (!m_elocalstate_filter->match(elstate))
             return;

           m_centroid_elstate = *elstate;
          }

          void
          consume(const IMC::PWM* pwm)
          {
            spew("Got pwm: %d, %d", pwm->id, pwm->duty_cycle);

            // Sanitycheck value
            if (pwm->duty_cycle < 900)
            {
              spew("Got invalid pwm value, ignoring. ");
              return;
            }


            if (pwm->id > 0 && pwm->id <=8 )
            {
              m_pwm_inputs[pwm->id - 1] = pwm->duty_cycle;
            }

            spew("Channel: %d, value: %f", pwm->id, pwmToValueDeadband(-4, 4, 900, 2100, 0, 0.1, pwm->duty_cycle));

          }

          //! On autopilot activation
          virtual void
          onAutopilotActivation(void)
          {
            if (!m_args.use_controller)
            {
              requestDeactivation();
              inf("Controller not enabled, deactivating. ");
            }
            else {
              inf("Controller enabled. ");
            }
          }

          // Called on activation
          virtual void
          reset(void)
          {
            if (m_centroid_elstate.state.isNull())
            {
              err("Not yet gotten centroid yaw. Setting desired yaw to 0");
              m_desired_yaw = 0;
            }
            else
              m_desired_yaw = m_centroid_elstate.state->psi;
            initRefmodel();
          }

          //! On autopilot deactivation
          //! Does nothing by default
          virtual void
          onAutopilotDeactivation(void)
          { }

          void
          stepNewRefModel(const IMC::EstimatedState& state, Matrix& desiredVel, const double timestep)
          {
            (void) state;


            Matrix v_d = desiredVel;

            //double T = m_args.prefilter_time_constant;
            //m_refmodel.prefilterState += ts.delta * (-(1/T)*m_refmodel.prefilterState + (1/T)*x_d);



            // Step 1: V-part
            Matrix tau1 = m_refmodel.k1 * (v_d - m_refmodel.getVel());



            if (tau1.norm_2() > m_args.max_acc)
            {
              tau1 = m_args.max_acc * tau1 / tau1.norm_2();
            }

            spew("Trying to reach acc: %.3f", m_args.max_acc);

            // Step 2: J-part
            Matrix tau2 = m_refmodel.k2 * (tau1 - m_refmodel.getAcc());


            // Actually jerk
            if (tau2.norm_2() > m_args.refmodel_max_jerk)
            {
              tau2 = m_args.refmodel_max_jerk * tau2 / tau2.norm_2();
            }

            // Step 3: dJ-part
            Matrix tau3 = m_refmodel.k3 * (tau2 - m_refmodel.getJerk());


            // Integrate
            m_refmodel.x += timestep * (m_refmodel.x.get(3,8,0,0).vertCat(tau3));


            // Print reference pos and vel
            trace("x_r:\t [%1.2f, %1.2f, %1.2f]",
                m_refmodel.x(0), m_refmodel.x(1), m_refmodel.x(2));
            trace("v_r:\t [%1.2f, %1.2f, %1.2f]",
                m_refmodel.x(3), m_refmodel.x(4), m_refmodel.x(5));
          }

          //! Step function
          virtual void
          onEstimatedState(const double timestep, const IMC::EstimatedState* msg)
          {
            //(void) timestep;
            (void) msg;

            // Send out new desired on each estate.

            // Calculate the velocity inputs
            Matrix vel = Matrix(3,1, 0.0);
            double yawrate = 0.0;


            vel(0) = pwmToValueDeadband(-m_args.max_speed_xy, m_args.max_speed_xy, 1100, 1900, 1, 0.07, m_pwm_inputs[CH_X]);
            vel(1) = pwmToValueDeadband(-m_args.max_speed_xy, m_args.max_speed_xy, 1100, 1900, 0, 0.07, m_pwm_inputs[CH_Y]);
            vel(2) = pwmToValueDeadband( m_args.max_speed_z, -m_args.max_speed_z,  1100, 1900, 0, 0.07, m_pwm_inputs[CH_Z]);

            yawrate = pwmToValueDeadband( -Angles::radians(m_args.max_yaw_rate), Angles::radians(m_args.max_yaw_rate),  1100, 1900, 0, 0.07, m_pwm_inputs[CH_YAW]);

            // Since we are not using the vel in z-axis from refmodel, remove it.
            Matrix refModelInputVel = vel;
            refModelInputVel(2) = 0.0;

            // Step ref model
            stepNewRefModel(*msg, refModelInputVel, timestep);


            IMC::DesiredLinearState desLinState;


            // Only use for x,y. Send z directly, no acc (for now)
            desLinState.vx = m_refmodel.x(0);
            desLinState.vy = m_refmodel.x(1);
            desLinState.vz = vel(2);

            desLinState.ax = m_refmodel.x(3);
            desLinState.ay = m_refmodel.x(4);
            desLinState.az = 0;

            desLinState.flags = IMC::DesiredLinearState::FL_VX |
                                IMC::DesiredLinearState::FL_VY |
                                IMC::DesiredLinearState::FL_VZ |
                                IMC::DesiredLinearState::FL_AX |
                                IMC::DesiredLinearState::FL_AY |
                                IMC::DesiredLinearState::FL_AZ; // Keep flag just so it uses the other ones.

            m_desired_yaw = m_desired_yaw + timestep * yawrate;

            IMC::DesiredHeading desYaw;
            desYaw.value = m_desired_yaw;



            dispatch(desLinState);
            dispatch(desYaw);
          }

          //! Update internal state with new parameter values.
          void
          onUpdateParameters(void)
          {
          }


        };
      }
    }
  }
}

DUNE_TASK
