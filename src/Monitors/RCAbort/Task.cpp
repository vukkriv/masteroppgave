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
// This task monitors a certain PWM value on a certain channel, and issues abort when
// the value reaches a certain threshold

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Monitors
{
  namespace RCAbort
  {
    using DUNE_NAMESPACES;

    struct Arguments
    {
      int channel;
      bool trigger_on_high;
      double threshold;
      double time_between_aborts;

    };

    struct Task: public DUNE::Tasks::Task
    {
      //! Task arguments
      Arguments m_args;

      //! Time last abort was sent.
      double m_time_prev_abort;

      //! Previous pwm value on channel
      int m_prev_duty_cycle;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_prev_duty_cycle(1500)
      {
        paramActive(Parameter::SCOPE_GLOBAL, Parameter::VISIBILITY_USER, true);

        param("Channel", m_args.channel)
        .minimumValue("1")
        .defaultValue("7")
        .maximumValue("12")
        .description("Channel [1-12] to trigger on. ");

        param("Trigger On High", m_args.trigger_on_high)
        .defaultValue("false")
        .description("Set to true to trigger on higher than treshold. Set to false to trigger on lower than threshold.");

        param("Threshold", m_args.threshold)
        .minimumValue("900")
        .defaultValue("1600")
        .maximumValue("2100")
        .description("PWM Value [900-2100] to trigger on. ");

        param("Time Between Aborts", m_args.time_between_aborts)
        .defaultValue("3")
        .units(Units::Second)
        .description("Time between consequtive aborts being sent .");

        // Initiate previous abort time;
        m_time_prev_abort = Clock::get();



        bind<IMC::PWM>(this);
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

      //! Consume PWM Values
      void
      consume(const IMC::PWM* pwm)
      {
        if (!isActive())
          return;

        //! Check for channel
        // ID is 1-indexed for both argument and message.
        if( (pwm->id ) == m_args.channel)
        {
          spew("Got PWM packet on correct channel. ");
          bool send_abort = false;

          spew("PWM VALUE: %d, Threshold: %f", pwm->duty_cycle, m_args.threshold);

          bool triggered = isTriggered(pwm->duty_cycle);
          bool wasTriggered = isTriggered(m_prev_duty_cycle);

          // Check for new trigger
          if (triggered && !wasTriggered)
            send_abort = true;

          // Check for old trigger, and time.
          if (triggered && Clock::get() > m_time_prev_abort + m_args.time_between_aborts)
            send_abort = true;

          if (send_abort)
          {
            IMC::Abort abort;
            abort.setDestination(getSystemId());
            dispatch(abort);

            m_time_prev_abort = Clock::get();

            err("RC Monitor dispatched Abort");
          }

          // Update prev duty cycle
          m_prev_duty_cycle = pwm->duty_cycle;
        }
      }

      bool
      isTriggered(int pwmValue)
      {
        if ( m_args.trigger_on_high && pwmValue > m_args.threshold)
          return true;
        if (!m_args.trigger_on_high && pwmValue < m_args.threshold && pwmValue > 1000)
          return true;

        return false;

      }

      //! Main loop.
      void
      onMain(void)
      {
        while (!stopping())
        {
          waitForMessages(1.0);
        }
      }
    };
  }
}

DUNE_TASK
