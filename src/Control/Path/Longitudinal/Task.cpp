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
// Author: Sigurd Olav Nevstad                                              *
//***************************************************************************
// Input: Desired speed and desired path-angle/vertical-rate				*
// Output: Desired pitch and throttle										*
//***************************************************************************

// ISO C++ 98 headers.
#include <cmath>

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Control
{
  namespace Path
  {
    namespace Longitudinal
    {
      using DUNE_NAMESPACES;

      struct Arguments
      {
    	  bool use_controller; //Flag to enable controller
    	  double k_thr_p;
    	  double k_thr_i;
    	  double k_gamma_p;
    	  double k_thr_ph;
    	  double trim_pitch;
    	  double trim_throttle;

      };

      struct Task: public DUNE::Control::PathController
      {
        Arguments m_args;
        IMC::DesiredThrottle m_throttle;
        IMC::DesiredPitch m_pitch;

        double m_airspeed;
        double m_dspeed;
        double m_dvrate;
        double m_thr_i;
        double m_dz;

        Delta m_last_step;

        Task(const std::string& name, Tasks::Context& ctx):
          DUNE::Control::PathController(name, ctx),
          m_airspeed(0.0),
          m_dspeed(18.0),
          m_dvrate(0.0),
          m_thr_i(0.0),
          m_dz(0.0)

        {
            param("Use controller", m_args.use_controller)
			  .visibility(Tasks::Parameter::VISIBILITY_USER)
			  .scope(Tasks::Parameter::SCOPE_MANEUVER)
			  .defaultValue("false")
			  .description("Use this controller for maneuver");

            param("Throttle Integrator gain", m_args.k_thr_i)
               .defaultValue("2.0")
               .description("Throttle Integrator gain");

            param("Throttle Proportional gain", m_args.k_thr_p)
                           .defaultValue("12.0")
                           .description("Throttle Proportional gain");

            param("Throttle Proportional height gain", m_args.k_thr_ph)
                           .defaultValue("2.0")
                           .description("Throttle Proportional height gain");

            param("Gamma Proportional gain", m_args.k_gamma_p)
                           .defaultValue("1.5")
                           .description("Gamma Proportional gain");

            param("Trim pitch", m_args.trim_pitch)
                           .defaultValue("2.6585")
                           .description("Trim Pitch for level flight");

            param("Trim throttle", m_args.trim_throttle)
                           .defaultValue("44.0")
                           .description("Trim throttle for level flight");

          bind<IMC::IndicatedSpeed>(this);
          bind<IMC::DesiredVerticalRate>(this);
          bind<IMC::DesiredSpeed>(this);
          bind<IMC::DesiredZ>(this);

       }

        void
        onPathActivation(void)
        {
          if (!m_args.use_controller)
            return;
          // Activate controller
          enableControlLoops(IMC::CL_SPEED); //Throttle considered as cl speed atm.
          enableControlLoops(IMC::CL_PITCH);
        }

        bool
        hasSpecificZControl(void) const
        {
          return true;
        }

        void
        consume(const IMC::IndicatedSpeed* airspeed)
        {
          m_airspeed = airspeed->value;
        }

        void
		consume(const IMC::DesiredSpeed* d_speed)
		{
        	m_dspeed = d_speed->value;
		}
        void
        consume(const IMC::DesiredZ* d_z)
        {
        	m_dz = d_z->value;
        }

        void
		consume(const IMC::DesiredVerticalRate* d_vrate)
		{
        	m_dvrate = d_vrate->value;
		}

        void
        step(const IMC::EstimatedState& state, const TrackingState& ts)
        {
            if (!m_args.use_controller)
              return;

            double Vg = sqrt( (state.vx*state.vx) + (state.vy*state.vy) + (state.vz*state.vz) ); // Ground speed
            double h_dot = state.u*sin(state.theta) - state.v*sin(state.phi)*cos(state.theta) - state.w*cos(state.phi)*cos(state.theta);
            double gamma_now = asin(h_dot/Vg);

            double alpha_now = gamma_now - state.theta;

            double gamma_desired = asin(m_dvrate/Vg);
            double gamma_error = gamma_now - gamma_desired;
            double V_error =  m_dspeed - m_airspeed;
            double H_error = (m_dz - (state.height - state.z));

            //Throttle integrator
		    double timestep = m_last_step.getDelta();
		    m_thr_i = m_thr_i + timestep*V_error;
		    m_thr_i = trimValue(m_thr_i,-20,20); //Anti wind-up at 20 % throttle

		    //Calculate desired throttle and pitch
		    double throttle_desired = m_args.k_thr_p*V_error + m_args.k_thr_i *m_thr_i+ H_error*m_args.k_thr_ph + m_args.trim_throttle;
		    //double pitch_desired = gamma_desired + alpha_now-gamma_error*m_args.k_gamma_p;
		    double pitch_desired = gamma_desired + Angles::radians(m_args.trim_pitch)-gamma_error*m_args.k_gamma_p; //Backstepping,pitch_desired = gamma_desired + alpha_0

            m_throttle.value = throttle_desired;
            m_pitch.value = Angles::degrees(pitch_desired);

            spew("pitch desired er %f, og alpha_0 er: %f",m_pitch.value,Angles::degrees(alpha_now));

            dispatch(m_throttle);
            dispatch(m_pitch);
        }
      };
    }
  }
}

DUNE_TASK
