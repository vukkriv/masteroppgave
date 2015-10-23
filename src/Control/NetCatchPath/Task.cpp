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
// Author: Jostein B. Moe                                                   *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Control
{
  namespace NetCatchPath
  {
    using DUNE_NAMESPACES;

    //! %Task arguments.
    struct Arguments
    {

    };

    //! Controllable loops.
    static const uint32_t c_controllable = 1;
    //! Required loops.
    static const uint32_t c_required = 1;

    struct Task: public DUNE::Control::BasicUAVAutopilot
    {
      //! Task arguments.
      Arguments m_args;

      //! Last desired path received
      IMC::DesiredPath m_dp;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Control::BasicUAVAutopilot(name,ctx, c_controllable, c_required)
      {

        // Bind incoming IMC messages
        bind<IMC::DesiredPath>(this);        
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
        BasicUAVAutopilot::onResourceAcquisition();
      }

      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
        BasicUAVAutopilot::onResourceInitialization();
      }

      // consume desired path from NetCatchCoordinator
      void
      consume(const IMC::DesiredPath* dp)
      {
        trace("Got DesiredPath \nfrom '%s' at '%s'",
             resolveEntity(dp->getSourceEntity()).c_str(),
             resolveSystemId(dp->getSource()));

        m_dp = *dp;
      }
      //! Release resources.
      void
      onResourceRelease(void)
      {
        BasicUAVAutopilot::onResourceRelease();
      }

      void
      onEstimatedState(const double timestep, const IMC::EstimatedState* msg)
      {
          //calculate desired velocity to be dispatched to inner control loop
      }
    };
  }
}

DUNE_TASK
