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
          //! Entity to send centroid EstimatedLocalState
          std::vector<std::string> ent_centroid_elocalstate;
          //! Yaw controller gain
          double yaw_kp;
        };

        enum PwmChannel {
           CH_X = 1,
           CH_Y = 0,
           CH_Z = 2,
           CH_YAW = 3,
           CH_TUNE = 5
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

            param("Max Speed - Yaw", m_args.max_speed_z)
            .defaultValue("20")
            .visibility(Parameter::VISIBILITY_USER)
            .units(Units::DegreePerSecond);

            param("Filter - Centroid EstimatedLocalState Entity", m_args.ent_centroid_elocalstate)
            .defaultValue("Formation Centroid");

            param("Yaw Controller - Kp", m_args.yaw_kp)
            .defaultValue("1");


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

           // Filter EstimatedLocalState by entities
           if (!m_elocalstate_filter->match(elstate))
             return;

           m_centroid_elstate = *elstate;
          }

          void
          consume(const IMC::PWM* pwm)
          {
            spew("Got pwm: %d, %d", pwm->id, pwm->duty_cycle);



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
            m_desired_yaw = m_centroid_elstate.state->psi;
          }

          //! On autopilot deactivation
          //! Does nothing by default
          virtual void
          onAutopilotDeactivation(void)
          { }

          //! Step function
          virtual void
          onEstimatedState(const double timestep, const IMC::EstimatedState* msg)
          {
            (void) timestep;
            (void) msg;

            // Send out new desired on each estate.

            // Calculate the velocity inputs
            Matrix vel = Matrix(3,1, 0.0);
            double yawrate = 0.0;


            vel(0) = pwmToValueDeadband(-m_args.max_speed_xy, m_args.max_speed_xy, 1100, 1900, 1, 0.07, m_pwm_inputs[CH_X]);
            vel(1) = pwmToValueDeadband(-m_args.max_speed_xy, m_args.max_speed_xy, 1100, 1900, 0, 0.07, m_pwm_inputs[CH_Y]);
            vel(2) = pwmToValueDeadband( m_args.max_speed_z, -m_args.max_speed_z,  1100, 1900, 0, 0.07, m_pwm_inputs[CH_Z]);

            yawrate = pwmToValueDeadband( -Angles::radians(m_args.max_yaw_rate), Angles::radians(m_args.max_yaw_rate),  1100, 1900, 0, 0.07, m_pwm_inputs[CH_YAW]);

            IMC::DesiredLinearState desLinState;

            desLinState.vx = vel(0);
            desLinState.vy = vel(1);
            desLinState.vz = vel(2);

            desLinState.flags = IMC::DesiredLinearState::FL_VX |
                                IMC::DesiredLinearState::FL_VY |
                                IMC::DesiredLinearState::FL_VZ;

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
