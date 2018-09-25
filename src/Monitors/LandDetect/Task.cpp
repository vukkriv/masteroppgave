//***************************************************************************
// Copyright 2007-2018 Norwegian University of Science and Technology (NTNU)*
// Centre for Autonomous Marine Operations and Systems (AMOS)               *
// Department of Engineering Cybernetics (ITK)                              *
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
// Author: Kristoffer Gryte                                                       *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Monitors
{
  //! This task is responsible to monitor net landing
  //! based on information from the ardupilot 
  //! from the inertial sensors.
  namespace LandDetect
  {
    using DUNE_NAMESPACES;

    //! Task arguments.
    struct Arguments
    {
      //! Number of samples to average accelerations
      //! for innovation limits threshold checking.
      unsigned avg_samples_innov;
    };

    //! Collisions task.
    struct Task: public Tasks::Task
    {
      //! Task arguments.
      Arguments m_args;

      Task(const std::string& name, Tasks::Context& ctx):
        Tasks::Task(name, ctx)
      {
        // Definition of configuration parameters.
        param("Innovation Moving Average Samples", m_args.avg_samples_innov)
        .defaultValue("10")
        .minimumValue("5")
        .maximumValue("20")
        .description("Number of moving average samples to smooth accelerations");

        // Register consumers.
        //bind<IMC::EstimatedState>(this);
      }

      void
      onUpdateParameters(void)
      {
        //if (paramChanged(m_args.t_error))
          //m_twindow.setTop(m_args.t_error);
      }

      void
      onResourceInitialization(void)
      {
      }

      void
      onEntityResolution(void)
      {
      }

      void
      onResourceRelease(void)
      {
      }

      void
      consume(const IMC::EstimatedState* msg)
      {
        if (msg->getSource() != getSystemId())
          return;

        //m_depth = msg->depth;
      }


      void
      onMain(void)
      {
        while (!stopping())
        {
          waitForMessages(1.0);

          //if (getEntityState() == IMC::EntityState::ESTA_ERROR)
          //{
            //// Return to normal mode once counter overflows.
            //if (m_twindow.overflow())
              //setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
          //}
        }
      }
    };
  }
}

DUNE_TASK
