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
// Author: Siri Mathisen                                                    *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Actuators
{
  namespace LogicOutput
  {
    namespace BBB
    {
      using DUNE_NAMESPACES;

      struct Arguments
      {
        //! Pin to control
        int pin;
        //! Initial value
        int init;
        //! Name.
        std::string name;
      };

      struct Task: public DUNE::Tasks::Task
      {
        IMC::PowerChannelState m_status;
        GPIO* m_out;
        //! Constructor.
        //! @param[in] name task name.
        //! @param[in] ctx context.
        Arguments m_args;
        Task(const std::string& name, Tasks::Context& ctx):
          DUNE::Tasks::Task(name, ctx), m_out(NULL)
        {
          param("Pin", m_args.pin)
          .defaultValue("2")
          .description("Pin to control.");

          param("Initial Output", m_args.init)
          .defaultValue("0")
          .description("Initial output on pin.");

          param("Name", m_args.name)
          .defaultValue("drop")
          .description("Name of PowerChannel.");

          bind<PowerChannelControl>(this);
        }

        //! Update internal state with new parameter values.
        void
        onUpdateParameters(void)
        {
          war("started act");
          m_status.name = m_args.name;
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
          m_out = new GPIO(m_args.pin);
          m_out->setDirection(GPIO::GPIO_DIR_OUTPUT);
          m_out->setValue(m_args.init);
          inf("Drop mechanism ready!");
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

        void
        consume(const IMC::PowerChannelControl* msg)
        {
          if(!msg->name.compare(m_args.name)){
            if(msg->op == 0 || msg->op == 1)
            {
              //Turn off/on the drop mechanism
              m_out->setValue(msg->op);

              if (msg->op == 0){
                m_status.state = IMC::PowerChannelState::PCS_OFF;
                dispatch(m_status);
              }
              if (msg->op == 1){
                m_status.state = IMC::PowerChannelState::PCS_ON;
                dispatch(m_status);
              }
            }
            else
            {
              war("Received a non drop op");
            }
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
}

DUNE_TASK
