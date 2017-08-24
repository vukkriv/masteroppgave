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
// Author: Krzysztof Cisek                                                  *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

// ISO Headers
#include <map>
#include <sstream>

namespace Sensors
{
  namespace UM100
  {
    using DUNE_NAMESPACES;

    static const unsigned int c_max_num_tags = 6;

    struct Arguments
    {
      // Serial port device.
      std::string uart_dev;
      // Serial port baud rate.
      unsigned uart_baud; //921600 for bespoon ekv boards

    };

    struct Task: public DUNE::Tasks::Task
    {
      // Device protocol handler.
      SerialPort* m_uart;
      // Task Arguments.
      Arguments m_args;

      std::string m_msg;

      // I/O Multiplexer.
      Poll m_poll;

      // Buffer
      char m_bfr[1024];

      // Address book map
      // sender id <-> output ID
      std::map<unsigned int, unsigned int> m_device_addressbook;

      // Array of outgoing messages
      IMC::BeaconDistance m_bdistance[c_max_num_tags];



      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_uart(NULL)
      {
        param("Serial Port - Device", m_args.uart_dev)
        .defaultValue("/dev/ttyUSB0")
        .description("Serial port device (used to communicate with the actuator)");

        param("Serial Port - Baud Rate", m_args.uart_baud)
        .defaultValue("921600")
        .description("Serial port baud rate");

        // Fill the address book with the first initial values
        // these will have the same outgoing ID every run.
        m_device_addressbook[2008] = 0;
        m_device_addressbook[3007] = 1;
        m_device_addressbook[3202] = 2;
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

        for( unsigned int i = 0; i < c_max_num_tags; ++i)
        {
          std::ostringstream ostr;
          ostr << "BeSpoon-" << i;
          m_bdistance[i].setSourceEntity(reserveEntity(ostr.str()));
        }
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
        m_uart = new SerialPort(m_args.uart_dev, m_args.uart_baud);
      }

      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
        setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);

        m_poll.add(*m_uart);
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
        if (m_uart != NULL)
        {
          m_poll.remove(*m_uart);
          delete m_uart;
        }
      }


      void
      process(const char* bfr)
      {



        float dist = 0;
        unsigned short int lqi = 0;
        unsigned short int src = 0;
        unsigned long int dlt = 0;

        // std::string data = bfr;

        //  spew("BFR|%s|", bfr);
        std::string data(bfr);
        std::size_t foundBegin = data.find("DLT");
        std::size_t foundEnd = data.find("\n");
        if ((foundBegin != std::string::npos)&&(foundEnd != std::string::npos))
        {
          data = data.substr(foundBegin, foundEnd-foundBegin);

          // spew("|%s|", data.c_str());
          //example data: DLT 54688 SRC 3007  LQI 99%  DIST 1.03m
          //sscanf(data.c_str(), "$%lu %hu %hu %f", &ts, &src, &lqi, &dist_origin);
          sscanf(data.c_str(), "DLT %lu SRC %hu LQI %hu%% DIST %f", &dlt, &src, &lqi, &dist);

          //dist_imc=dist*100; //dist_imc[cm]=dist_orign[m]*100

          // Check if the sender is not in the address book

          bool deviceInAddreessBook = false;

          // if cannot find the source in the map.
          if (m_device_addressbook.find(src) ==  m_device_addressbook.end())
          {
            // Try to add
            if (m_device_addressbook.size() < c_max_num_tags)
            {
              // Add
              unsigned int newId = m_device_addressbook.size();
              m_device_addressbook[src] = newId;
              deviceInAddreessBook = true;
              inf("Added new device: %d at id %d", src,m_device_addressbook[src] );
            }
            else
            {
              // We are full, do not use the address book
              deviceInAddreessBook = false;
            }
          }
          else
          {
            deviceInAddreessBook = true;
          }

          if (deviceInAddreessBook)
          {
            unsigned int id = m_device_addressbook[src];

            m_bdistance[id].dist = dist;
            m_bdistance[id].dqf = lqi;
            m_bdistance[id].dlt = dlt;
            m_bdistance[id].sender = src;
            dispatch(m_bdistance[id]);
          }

          // Send it on the global entity always anyway.
          IMC::BeaconDistance msg;
          msg.dist = dist;
          msg.dqf = lqi;
          msg.dlt = dlt;
          msg.sender = src;
          dispatch(msg);



          IMC::DevDataText raw;
          raw.value = data;
          dispatch(raw);

          debug("dlt: %lu src: %hu lqi: %hu dist[m]: %.2f", dlt, src, lqi, dist);
        }
      }



      void
      checkSerialPort(void)
      {
        if (m_poll.wasTriggered(*m_uart))
        {
          int rv = m_uart->readString(m_bfr, sizeof(m_bfr));

          if(rv > 0)
          {
            if(joinMessage(m_bfr, &m_msg))
            {
              spew("BFRS|%s|", m_msg.c_str());
              process(m_msg.c_str());

              m_msg.clear();
            }
          }
        }
      }


      bool
      joinMessage(char* part, std::string* msg)
      {
        std::string data(part);

        if(data.find('\n') != std::string::npos)
        {
          msg->append(data);
          //debug(DTR("full msg: %s"), msg->c_str());
          return true;
        }
        else
        {
          msg->append(data);
        }

        return false;
      }

      //! Main loop.
      void
      onMain(void)
      {
        while(!stopping()){
          if (m_poll.poll(1.0))
          {
            checkSerialPort();
            consumeMessages();
          }
        }
      }
    };
  }
}

DUNE_TASK
