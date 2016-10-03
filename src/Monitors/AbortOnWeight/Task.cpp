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

namespace Monitors
{
  namespace AbortOnWeight
  {
    using DUNE_NAMESPACES;


    struct Arguments
    {
      //! Weight to trigger on
      double weight_treshold;
      //! Time it needs to be above weight to trigger
      double trigger_minimum_time;
    };

    enum TriggerState
    {
      TS_INIT              = 0,
      TS_TRIGGER_WAIT      = 1,
      TS_ABORT_SENT        = 2
    };

    struct Task: public DUNE::Tasks::Task
    {
      //! Task Arguments
      Arguments m_args;
      //! Trigger State
      TriggerState m_tstate;
      //! Time of last weight crossed threshold
      double m_time_last_trigger;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_tstate(TS_INIT),
        m_time_last_trigger(0)
      {
        paramActive(Parameter::SCOPE_GLOBAL, Parameter::VISIBILITY_USER);

        param("Weight Threshold", m_args.weight_treshold)
        .minimumValue("0.0")
        .defaultValue("3.0")
        .units(Units::Kilogram)
        .visibility(Parameter::VISIBILITY_USER)
        .description("Weight to start the trigger sequence. ");

        param("Trigger Minimum Time", m_args.trigger_minimum_time)
        .minimumValue("0.0")
        .defaultValue("1.0")
        .units(Units::Second)
        .description("Minimum time to be over the trigger weight to trigger an abort. ");

        bind<IMC::Weight>(this);
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

      //! Consume weight
      void
      consume(const IMC::Weight* msg)
      {
        if (!isActive())
          return;


        switch(m_tstate)
        {
          case TS_INIT:
            if (msg->value > m_args.weight_treshold)
            {
              m_tstate = TS_TRIGGER_WAIT;
              m_time_last_trigger = Clock::get();
            }
            break;
          case TS_TRIGGER_WAIT:
          case TS_ABORT_SENT:
            if (msg->value < m_args.weight_treshold)
            {
              // Got below the threshold while waiting, got back to init state.
              m_tstate = TS_INIT;
            }
            break;
        }
      }

      //! Main loop.
      void
      onMain(void)
      {
        while (!stopping())
        {
          if (m_tstate == TS_INIT || m_tstate == TS_ABORT_SENT)
            waitForMessages(1.0);
          else
            waitForMessages(0.1);

          // Check the timers
          if (m_tstate == TS_TRIGGER_WAIT)
          {
            if (Clock::get() > m_time_last_trigger)
            {
              // Abort
              IMC::Abort abort;
              abort.setDestination(getSystemId());

              dispatch(abort);

              m_tstate = TS_ABORT_SENT;
            }
          }
        }
      }
    };
  }
}

DUNE_TASK
