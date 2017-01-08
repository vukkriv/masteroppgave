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

namespace Sensors
{
  namespace Pozyx
  {
    using DUNE_NAMESPACES
;
    struct Arguments
    {
      // Serial port device.
      std::string uart_dev;
      // Serial port baud rate.
      unsigned uart_baud; //115200 for pozyx&arduino uno

    };

    struct Task : public DUNE::Tasks::Task
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

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx) :
          DUNE::Tasks::Task(name, ctx), m_uart(NULL)
      {
        param("Serial Port - Device", m_args.uart_dev)
        .defaultValue("/dev/ttyACM3")
        .description("Serial port device (used to communicate with the actuator)");

        param("Serial Port - Baud Rate", m_args.uart_baud)
        .defaultValue("115200")
        .description("Serial port baud rate");
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

        IMC::BeaconDistance msg;

        //float dist = 0;
        unsigned long int dist = 0;
        float dist_f = 0.0;
        int rssi = 0;
        unsigned int src = 0;
        unsigned long int tsp = 0;

        // std::string data = bfr;

//
        //  spew("BFR|%s|", bfr);
        std::string data(bfr);
        std::size_t foundBegin = data.find("TSP");
        std::size_t foundEnd = data.find("\n");
        if ((foundBegin != std::string::npos)
            && (foundEnd != std::string::npos))
        {
          data = data.substr(foundBegin, foundEnd - foundBegin);

          // spew("|%s|", data.c_str());
          //example data pozyx:TSP 83104 SRC 6057 RSS -88 DST 2022
          //example data: DLT 54688 SRC 3007  LQI 99%  DIST 1.03m
          //sscanf(data.c_str(), "$%lu %hu %hu %f", &ts, &src, &lqi, &dist_origin);
          sscanf(data.c_str(), "TSP %lu SRC %x RSS %d DST %lu", &tsp, &src,
                 &rssi, &dist);

          //dist_imc=dist*100; //dist_imc[cm]=dist_orign[m]*100
          dist_f = (float)dist / 1000;
          msg.dist = dist_f;
          msg.rssi = rssi;
          msg.time = tsp;          // * 1000;
          msg.sender = src;
          dispatch(msg);

          IMC::DevDataText raw;
          raw.value = data;
          dispatch(raw);

          debug("stp: %lu src: %04x rssi: %d dist[m]: %.2f", tsp, src, rssi,
                dist_f);
          //debug("stp: %lu src: %hu rssi: %hu dist[m]: %.2f", tsp, src, rssi,dist);
        }
      }
      void
      checkSerialPort(void)
      {
        if (m_poll.wasTriggered(*m_uart))
        {
          int rv = m_uart->readString(m_bfr, sizeof(m_bfr));

          if (rv > 0)
          {
            if (joinMessage(m_bfr, &m_msg))
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

        if (data.find('\n') != std::string::npos)
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
        while (!stopping())
        {
          if (m_poll.poll(1.0))
          {
            checkSerialPort();
          }
        }
      }
    };
  }
}

DUNE_TASK
