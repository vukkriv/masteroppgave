//***************************************************************************
// Copyright 2007-2014 Universidade do Porto - Faculdade de Engenharia      *
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
// https://www.lsts.pt/dune/licence.                                        *
//***************************************************************************
// Author: Kristian Klausen                                                 *
//***************************************************************************

// ISO C++ 98 headers.
#include <vector>
#include <stdexcept>
#include <set>
#include <algorithm>
#include <cstddef>


// DUNE headers.
#include <DUNE/DUNE.hpp>

// Library headers
extern "C"
{
  #include <libswiftnav/sbp.h>
  #include <libswiftnav/sbp_messages.h>
}



namespace Sensors
{
  namespace PiksiBaseStation
  {
    using DUNE_NAMESPACES;

    const int PIKSI_MSG_INIT_BASELINE = 0x23;
    const int PIKSI_MSG_RESET_FILTERS = 0x22;

    //! %Task arguments.
    struct Arguments
    {
      //! Serial port device.
      std::string uart_dev;
      //! Serial port baud rate.
      unsigned uart_baud;
      //! Communication UDP Port
      uint16_t UDP_port;
      //! Todo: List of IPs to transmit to
      //! List of rovers.
      std::vector<std::string> rovers;
      //! Communications timeout
      uint8_t comm_timeout;
    };

    struct Task: public DUNE::Tasks::Task
    {
      //! Task arguments.
      Arguments m_args;
      //! Serial port handle.
      SerialPort* m_uart;
      //! Udp handle
      UDPSocket* m_udp;
      //! Set of rovers
      std::set<Address> m_rovers;
      uint8_t m_buf[512];
      //! Timestamp on previous local packet
      double m_last_pkt_time;
      //! If in error mode
      bool m_error_missing_data;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_uart(NULL),
        m_udp(NULL),
        m_last_pkt_time(0),
        m_error_missing_data(true)
      {


        param("Static Rover IPs", m_args.rovers)
        .description("List of <IPv4> destinations that will always receive base observations");

        param("Communication Timeout", m_args.comm_timeout)
        .defaultValue("5")
        .units(Units::Second)
        .description("Timeout for base and local communication.");

        param("UDP Port", m_args.UDP_port)
        .defaultValue("8890")
        .description("Communication port for UDP observations.");

        // Define configuration parameters.
        param("Serial Port - Device", m_args.uart_dev)
        .defaultValue("")
        .description("Serial port device used to communicate with the sensor");

        param("Serial Port - Baud Rate", m_args.uart_baud)
        .defaultValue("115200")
        .description("Serial port baud rate");

        m_last_pkt_time = Clock::get();

        setEntityState(IMC::EntityState::ESTA_ERROR, "Waiting for observations.");


      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {


        // Initialize set of rover IPs
        m_rovers.clear();
        for (unsigned int i = 0; i < m_args.rovers.size(); ++i)
          m_rovers.insert(Address(m_args.rovers[i].c_str()));

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

        // Open Serial connection
        openSerialConnection();

        // Bind local UDP port
        openUdpSocket();

      }

      void
      openUdpSocket(void)
      {
        try
        {
          m_udp = new UDPSocket();

          // Dont actually need to bind.
          //m_udp->bind(m_args.UDP_port);
        }
        catch (Exception& ex) {
          err(DTR("Error opening UDP socket: %s"), ex.what());
        }

      }

      void
      openSerialConnection(void)
      {
        try
        {
          m_uart = new SerialPort(m_args.uart_dev, m_args.uart_baud);
        }
        catch (Exception& ex)
        {
          err(DTR("Error opening serial device: %s"), ex.what());
        }
        catch (...)
        {
          err(DTR("Error opening serial device. "));
        }
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
        Memory::clear(m_uart);
        Memory::clear(m_udp);
      }

      bool
      pollUart(double timeout)
      {
        if (m_uart != NULL)
          return Poll::poll(*m_uart, timeout);

        return false;
      }

      int
      receiveUartData(uint8_t* buf, size_t blen)
      {
        if (m_uart)
        {
          try
          {
            return m_uart->read(buf, blen);
          }
          catch (std::runtime_error& e)
          {
            err(DTR("%s"), e.what());
            war(DTR("Connection lost, retrying..."));
            Memory::clear(m_uart);

            openSerialConnection();

            return 0;
          }
        }
        return 0;
      }

      //! Handle incomming serial data
      void
      handleSerialData(void)
      {
        // Simply forward to all udp clients, on local port.
        // Todo: Consider waiting for a number of bytes first.

        int counter = 0;
        while (pollUart(0.01) && counter < 50)
        {
          ++counter;
          int n = receiveUartData(m_buf, sizeof(m_buf));

          spew("Received %d bytes. Forwarding to rovers.", n);

          if (n < 0)
          {
            debug(DTR("Receive error"));
            break;
          }
          // Send to rovers
          std::set<Address>::iterator itr = m_rovers.begin();
          for (; itr != m_rovers.end(); ++itr)
          {
            try
            {
              m_udp->write(m_buf, n, *itr, m_args.UDP_port);
            }
            catch (...)
            { }
          }
        }

      }



      //! Main loop.
      void
      onMain(void)
      {
        while (!stopping())
        {
          // Handle data
          if (m_uart)
          {
            handleSerialData();
          }
          else
          {
            Time::Delay::wait(0.5);
            openSerialConnection();
          }
          double now = Clock::get();

          // Timeout of last packet of 1 second. and we think all is ok.
          if ((m_last_pkt_time + m_args.comm_timeout) < now)
          {
            if (getEntityState() == IMC::EntityState::ESTA_NORMAL)
            {
              war(DTR("Timeout of prev. packet received!"));
              setEntityState(IMC::EntityState::ESTA_ERROR, "Timeout of packet.");
            }
          }
          // Not timeout and we were in an error state
          else if (getEntityState() != IMC::EntityState::ESTA_NORMAL)
          {
            inf(DTR("Got packet, all is ok. "));
            setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
          }



          // Handle IMC messages from bus
          consumeMessages();
        }
      }

    };
  }
}

DUNE_TASK
