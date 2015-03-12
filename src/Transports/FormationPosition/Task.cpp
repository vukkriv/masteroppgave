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
// Author: Jon-Håkon Bøe Røli                                               *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Transports
{
  namespace FormationPosition
  {
    using DUNE_NAMESPACES;

    //! %Input type
    enum INPUT_TYPE
    {
      RTK = 0,
      ESTATE
    };

    struct Arguments
    {
      //! Input type
      std::string type;
    };

    struct Task: public DUNE::Tasks::Task
    {
      //! Task arguments.
      Arguments m_args;
      //! Sensor Type
      INPUT_TYPE m_type;
      //! Formation position to controller
      IMC::FormPos m_form_pos;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_type(RTK)
      {
        param("Input Type", m_args.type)
        .defaultValue("RTK")
        .values("RTK,EstimatedState")
        .description("Input Type - RTK or EstimatedState");

        // Bind to incoming IMC messages
        bind<IMC::RtkFix>(this);
        bind<IMC::EstimatedState>(this);
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        // Check input type
        if (m_args.type == "EstimatedState")
        {
          m_type = ESTATE;
        }
        else
        {
          m_type = RTK;
        }
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
      consume(const IMC::RtkFix* msg)
      {
        spew("Got RTK Fix");

        if (m_type == RTK)
        {
          switch (msg->type)
          {
            case IMC::RtkFix::RTK_NONE:
              break;
            case IMC::RtkFix::RTK_OBS:
              break;
            case IMC::RtkFix::RTK_FLOAT:
              break;
            case IMC::RtkFix::RTK_FIXED:
              m_form_pos.x = msg->n;
              m_form_pos.y = msg->e;
              m_form_pos.z = msg->d;
              m_form_pos.vx = msg->v_n;
              m_form_pos.vy = msg->v_e;
              m_form_pos.vz = msg->v_d;

              dispatch(m_form_pos);
              spew("Sent Formation Position");
          }
        }
      }

      void
      consume(const IMC::EstimatedState* msg)
      {
        spew("Got Estimated State");

        if (m_type == ESTATE)
        {
          m_form_pos.x = msg->x;
          m_form_pos.y = msg->y;
          m_form_pos.z = msg->z;
          m_form_pos.vx = msg->vx;
          m_form_pos.vy = msg->vy;
          m_form_pos.vz = msg->vz;

          dispatch(m_form_pos);
          spew("Sent Formation Position");
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
