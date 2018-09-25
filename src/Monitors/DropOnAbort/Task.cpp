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
// Author: Kristian Klausen                                                 *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Monitors
{
  namespace DropOnAbort
  {
    using DUNE_NAMESPACES;

    struct Arguments
    {
      //! Drop duration, in seconds. Set to zero or negative to not send a duration.
      int drop_duration;
    };

    struct Task: public DUNE::Tasks::Task
    {
      //! Task arguments
      Arguments m_args;



      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx)
      {
        paramActive(Parameter::SCOPE_GLOBAL, Parameter::VISIBILITY_DEVELOPER, true);

        param("Drop Duration", m_args.drop_duration)
        .defaultValue("0")
        .description("Drop Duration. Set to zero or negative to use system default. ");

        bind<IMC::Abort>(this);
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
        this->setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
      }

      //! Consume abort
      void
      consume(const IMC::Abort* msg)
      {
        if (msg->getDestination() != getSystemId())
          return;

        if (isActive())
        {
          IMC::RemoteActions rac;

          rac.actions = "doDrop=1;";

          if (m_args.drop_duration > 0)
          {
            char buf[30];
            snprintf(buf, 30, "%d", m_args.drop_duration);
            rac.actions += "dropDuration=" + std::string(buf) + ";";

          }

          debug("Drop command issued.");
          dispatch(rac);
        }

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
