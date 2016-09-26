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
// If the vehicle is running a formation (copters) and recieves an abort, this
// task will generate a new plan to get away from dodge.

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Monitors
{
  namespace UAV
  {
    namespace FormationAbortPlan
    {
      using DUNE_NAMESPACES;

      struct Arguments
      {
        //! Horizontal deflection
        double deflection_horizontal;

        //! Vertical deflection
        double deflection_vertical;

        //! Use ardutracker
        bool use_ardutracker;

        //! State Reset timeout
        double state_reset_timeout;
      };

      // Holder class for criterias to be met to issue abort plan.
      struct AbortState
      {
        AbortState(): got_abort(false),
                      got_vs_error(false),
                      got_formcord_abort(false),
                      time_of_abort(0)
        {
          /* Intentionally empty. */
        };

        bool allConditionsMet(void) { return (got_abort && got_vs_error); };

        bool got_abort;
        bool got_vs_error;
        bool got_formcord_abort;

        double time_of_abort;

      };

      struct Task: public DUNE::Tasks::Task
      {
        //! Task arguments
        Arguments m_args;

        //! Last recieved FormCoord
        IMC::FormCoord m_prev_formcoord;

        //! Last recieved vehicle state
        IMC::VehicleState m_prev_vs;

        //! Is currently doing a formation
        bool m_is_formation_active;

        //! State
        AbortState m_abortState;



        //! Constructor.
        //! @param[in] name task name.
        //! @param[in] ctx context.
        Task(const std::string& name, Tasks::Context& ctx):
          DUNE::Tasks::Task(name, ctx),
          m_is_formation_active(false)
        {

          param("Deflection - Horizontal", m_args.deflection_horizontal)
          .defaultValue("2")
          .units(Units::Meter)
          .description("The deflection in horizontal direction from formation centroid to go to. ");

          param("Deflection - Vertical", m_args.deflection_vertical)
          .defaultValue("0")
          .units(Units::Meter)
          .description("The deflection in vertical direction from formation centroid to go to. ");

          param("Enable Ardutracker", m_args.use_ardutracker)
          .defaultValue("true")
          .description("Set to true to use ardutracker. Otherwise, uses copter acceleration controller. ");

          param("State Timeout", m_args.state_reset_timeout)
          .defaultValue("1")
          .description("Maximum time to wait for all criterias to be met. ");

          bind<IMC::Abort>(this);
          bind<IMC::FormCoord>(this);
          bind<IMC::VehicleState>(this);
        }

        //! Update internal state with new parameter values.
        void
        onUpdateParameters(void)
        {
        }

        //! Reserve entity identifiers.
        void
        onEntityReservation(void)
        {
        }

        //! Resolve entity names.
        void
        onEntityResolution(void)
        {
        }

        //! Acquire resources.
        void
        onResourceAcquisition(void)
        {
        }

        //! Initialize resources.
        void
        onResourceInitialization(void)
        {
        }

        //! Release resources.
        void
        onResourceRelease(void)
        {
        }

        void
        consume(const IMC::Abort* msg)
        {
          if (msg->getDestination() != getSystemId())
            return;

          if (!m_abortState.got_abort)
          {
            m_abortState.got_abort = true;
            m_abortState.time_of_abort = Clock::get();
            inf("Got abort, start checking conditions..");
          }
        }

        void
        consume(const IMC::FormCoord* msg)
        {
          if( msg->op == FormCoord::FCOP_START )
            m_is_formation_active = true;

          if( msg->op == FormCoord::FCOP_ABORT )
          {
            if (!m_abortState.got_abort)
            {
              m_abortState.got_abort = true;
              m_abortState.got_formcord_abort = true;

              m_abortState.time_of_abort = Clock::get();
              inf("Got formcoord abort, start checking conditions..");
            }
          }
        }

        void
        consume(const IMC::VehicleState* msg)
        {
          if (!m_abortState.got_abort)
            return;

          //! Check if got abort (either from normal abort or foormcoord. Both sets abort flag.
          if (msg->op_mode == VehicleState::VS_ERROR)
          {
            m_abortState.got_vs_error = true;
          }

        }

        // Reset abort state
        void
        resetAbortState(void)
        {
          m_abortState = AbortState();
          inf("Abort-state reset. ");
        }

        // Issue abort plan
        void
        executeAbortPlan(void)
        {
          inf("Executing abort plan. ");


          // Reset abortstate
          resetAbortState();
        }



        //! Main loop.
        void
        onMain(void)
        {
          while (!stopping())
          {
            waitForMessages(1.0);

            // Check for state reset
            if (m_abortState.got_abort && Clock::get() > m_abortState.time_of_abort + m_args.state_reset_timeout)
              resetAbortState();

            if (m_abortState.allConditionsMet())
              executeAbortPlan();
          }
        }
      };
    }
  }
}

DUNE_TASK
