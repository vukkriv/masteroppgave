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

      };

      struct Task: public DUNE::Control::PathController
      {
        Arguments m_args;
        IMC::DesiredThrottle m_throttle;
        IMC::DesiredPitch m_pitch;

        double m_airspeed;
        double m_dspeed;
        double m_dvrate;

        Task(const std::string& name, Tasks::Context& ctx):
          DUNE::Control::PathController(name, ctx),
          m_airspeed(0.0),
          m_dspeed(18.0),
          m_dvrate(0.0)

        {
            param("Use controller", m_args.use_controller)
			  .visibility(Tasks::Parameter::VISIBILITY_USER)
			  .scope(Tasks::Parameter::SCOPE_MANEUVER)
			  .defaultValue("false")
			  .description("Use this controller for maneuver");

          bind<IMC::IndicatedSpeed>(this);
          bind<IMC::DesiredVerticalRate>(this);
          bind<IMC::DesiredSpeed>(this);

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
		consume(const IMC::DesiredVerticalRate* d_vrate)
		{
        	m_dvrate = d_vrate->value;
		}

        void
        step(const IMC::EstimatedState& state, const TrackingState& ts)
        {
            if (!m_args.use_controller)
              return;

            //TO-DO: Implement Controller
            inf("desired speed: %f, og dsired vrate: %f",m_dspeed,m_dvrate);
            m_throttle.value = 89;
            m_pitch.value = 4;

            dispatch(m_throttle);
            dispatch(m_pitch);
        }
      };
    }
  }
}

DUNE_TASK
