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
  namespace AbortOnNavSourceChange
  {
    using DUNE_NAMESPACES;

    struct Arguments
    {
      //! Drop on flag
      std::string abort_on_loss_of;

      //! Only during maneuver execution
      bool only_in_maneuver;
    };

    typedef uint16_t Mask;

    struct Task: public DUNE::Tasks::Task
    {
      //! Task Arguments
      Arguments m_args;
      //! Last recieved nav source
      IMC::NavSources m_navsource;
      //! Last recieved vehicle state
      IMC::VehicleState m_vstate;
      //! Flags to require
      uint16_t m_required_flags;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_required_flags(0x0000)
      {
        paramActive(Parameter::SCOPE_GLOBAL, Parameter::VISIBILITY_USER, true);

        param("Abort On Loss Of", m_args.abort_on_loss_of)
        .defaultValue("RTK")
        .values("None,RTK")
        .description("If this is lost, then send abort. ");

        param("Only On Maneuver", m_args.only_in_maneuver)
        .defaultValue("true")
        .description("Only send abort if doing a maneuver. ");

        bind<IMC::NavSources>(this);
        bind<IMC::VehicleState>(this);
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        // Check flags
        m_required_flags = 0x00;
        if (m_args.abort_on_loss_of == "RTK")
          m_required_flags = IMC::NS_GNSS_RTK;

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
        setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
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

      //! Store vehicle state
      void
      consume(const IMC::VehicleState* vstate)
      {
        m_vstate = *vstate;
      }

      //! Consume nav sources
      void
      consume(const IMC::NavSources* msg)
      {
        if (!isActive())
          return;

        // Check for change regarding selected flags
        Mask was = m_navsource.mask & m_required_flags;
        Mask is  = msg->mask        & m_required_flags;


        // We only trigger on change, and only if we were having all flags.
        if (was != is && was == m_required_flags)
        {
          if (!m_args.only_in_maneuver || (m_vstate.op_mode == IMC::VehicleState::VS_MANEUVER))
          {
            inf("Sent abort due to loss of navsource. ");

            IMC::Abort abort;
            abort.setDestination(getSystemId());

            dispatch(abort);
          }
        }


        m_navsource = *msg;
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
