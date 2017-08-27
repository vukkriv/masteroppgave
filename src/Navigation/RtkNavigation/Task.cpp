//***************************************************************************
// Copyright 2007-2017 Universidade do Porto - Faculdade de Engenharia      *
// Laboratório de Sistemas e Tecnologia Subaquática (LSTS)                  *
//***************************************************************************
// This file is part of DUNE: Unified Navigation Environment.               *
//                                                                          *
// Commercial Licence Usage                                                 *
// Licencees holding valid commercial DUNE licences may use this file in    *
// accordance with the commercial licence agreement provided with the       *
// Software or, alternatively, in accordance with the terms contained in a  *
// written agreement between you and Faculdade de Engenharia da             *
// Universidade do Porto. For licensing terms, conditions, and further      *
// information contact lsts@fe.up.pt.                                       *
//                                                                          *
// Modified European Union Public Licence - EUPL v.1.1 Usage                *
// Alternatively, this file may be used under the terms of the Modified     *
// EUPL, Version 1.1 only (the "Licence"), appearing in the file LICENCE.md *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// https://github.com/LSTS/dune/blob/master/LICENCE.md and                  *
// http://ec.europa.eu/idabc/eupl.html.                                     *
//***************************************************************************
// Author: Kristian Klausen                                                 *
//***************************************************************************


/*
 * First version of this task will simply
 * try to accurately tag information,
 * and increase update-rate for log examination.
 *
 *
 * Intended functionality:
 *  - Consume RtkFix
 *  - Note time of arrival
 *  - At fixed interval, integrate forward to current time.
 *  - Dispatch as EstimatedLocalState
 *
 *
 *
 */
// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Navigation
{
  namespace RtkNavigation
  {
    using DUNE_NAMESPACES;

    struct Arguments
    {
      //! Constant time delay
      double receipt_delay;
      //! Output frequency
      double execution_frequency;
    };

    class RtkReceipt
    {
    public:
      //! The message received
      IMC::GpsFixRtk msg;
      //! (Estimated) time of validity
      double tov;
    };

    struct Task: public DUNE::Tasks::Periodic
    {
      //! Task arguments
      Arguments m_args;
      //! Last received message
      RtkReceipt m_rtkReceipt;
      //! Current message
      RtkReceipt m_currentRtk;
      //! Dispatched State
      IMC::EstimatedLocalState m_els;



      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Periodic(name, ctx)
      {
        param("Receipt Delay", m_args.receipt_delay)
        .minimumValue("0")
        .defaultValue("0.1")
        .description("Constant value to use for forward propagation. ");

        param("Output Frequency", m_args.execution_frequency)
        .minimumValue("0")
        .defaultValue("25")
        .description("Solution output frequency");


        m_els.clear();
        m_els.state.set(new IMC::EstimatedState);
        m_els.acc.set(new IMC::Acceleration);

        bind<IMC::GpsFixRtk>(this);
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

      //! Consume RtkFix
      void
      consume(const IMC::GpsFixRtk* msg)
      {
        bool valid = true;

        if (msg->validity & IMC::GpsFixRtk::RFV_VALID_BASE == 0)
        {
          trace("Message not valid due to invalid base.");
          valid = false;
        }
        if (msg->validity & IMC::GpsFixRtk::RFV_VALID_POS == 0)
        {
          trace("Message not valid due to invalid position. ");
          valid = false;
        }

        if (!valid)
          return;

        // Store message
        m_rtkReceipt.msg = *msg;
        m_rtkReceipt.tov = msg->getTimeStamp() - m_args.receipt_delay;

        // Set the newly received message to the current
        m_currentRtk = m_rtkReceipt;


      }


      void
      propagateForward(void)
      {
        // Integrate forward to the current time
        RtkReceipt old = m_currentRtk;

        double now = Clock::get();

        double deltat = now - old.tov;

        if (deltat < 0)
        {
          err("Invalid time delta. Using zero. ");
          deltat = 0;
        }

        m_currentRtk.msg.n += deltat * old.msg.v_n;
        m_currentRtk.msg.e += deltat * old.msg.v_e;
        m_currentRtk.msg.d += deltat * old.msg.v_d;

        m_currentRtk.tov += deltat;


      }

      void
      fillState(void)
      {
        // TODO: Fill up ELS
      }




      //! Main loop.
      void
      task(void)
      {
        propagateForward();
      }
    };
  }
}

DUNE_TASK
