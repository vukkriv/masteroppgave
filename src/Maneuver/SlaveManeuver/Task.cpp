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
// Author: Jostein B. Moe                                                   *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Maneuver
{
  namespace SlaveManeuver
  {
    using DUNE_NAMESPACES;

    struct Task: public DUNE::Maneuvers::Maneuver
    {
      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Maneuvers::Maneuver(name, ctx)
      {
        bindToManeuver<Task, IMC::SlaveManeuver>();
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
      }

      void
      consume(const IMC::SlaveManeuver* maneuver)
      {
        (void) maneuver;
        inf("SlaveManeuver maneuver received");
        // First disable all
        setControl(0);
        // Enable control loops
        //(will enable the slave formation speed controller
        setControl(IMC::CL_SPEED);
      }
    };
  }
}

DUNE_TASK
