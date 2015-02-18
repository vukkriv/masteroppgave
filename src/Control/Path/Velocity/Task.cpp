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
        double max_speed;
      };

      struct Task: public DUNE::Control::PathController
      {
        IMC::DesiredVelocity m_velocity;
        //! Task arguments.
        Arguments m_args;

        Task(const std::string& name, Tasks::Context& ctx):
          DUNE::Control::PathController(name, ctx)
        {

          param("Max Speed", m_args.max_speed)
            .defaultValue("5.0")
            .units(Units::MeterPerSecond)
            .description("Max speed of the vehicle");

        }

        void
        onUpdateParameters(void)
        {
          PathController::onUpdateParameters();
        }

        void
        onEntityReservation(void)
        {
          PathController::onEntityReservation();
        }

        void
        onPathActivation(void)
        {
          // Activate velocity controller.
          enableControlLoops(IMC::CL_SPEED);
          inf("Vel-control activated.");
        }

        void
        step(const IMC::EstimatedState& state, const TrackingState& ts)
        {

          // Move at a rate towards the target

          // Head straight to target
          m_velocity.u = ts.end.x - ts.start.x;
          m_velocity.v = ts.end.y - ts.start.y;

          m_velocity.flags = IMC::DesiredVelocity::FL_SURGE | IMC::DesiredVelocity::FL_SWAY;

          dispatch(m_velocity);
          debug("Sent vel data.");
        }
      };
    }
  }
}

DUNE_TASK
