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

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Sensors
{
  namespace AT86RF233
  {
    using DUNE_NAMESPACES;


    //! %Task arguments.
    struct Arguments
    {
      //! Communications timeout
      uint8_t comm_timeout;
      //! TCP Port - Local Sensor
      uint16_t TCP_port;
      //! TCP Address - Local Sensor
      Address TCP_addr;
    };

    struct Task: public DUNE::Tasks::Task
    {

      //! Task arguments.
      Arguments m_args;
      //! Timestamp on prevoius local packet
      double m_last_pkt_time;
      uint8_t m_buf[512];
      //! TCP socket - Sensor
      TCPSocket* m_TCP_sock;
      Address m_TCP_addr;
      uint16_t m_TCP_port;


      bool m_error_missing_data;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_last_pkt_time(0),
        m_TCP_sock(NULL),
        m_TCP_port(0),
        m_error_missing_data(false)
      {

        param("TCP - Port", m_args.TCP_port)
        .defaultValue("9999")
        .description("Port for connection to Atmel device");

        param("TCP - Address", m_args.TCP_addr)
        .defaultValue("127.0.0.1")
        .description("Address for connection to Atmel device");


        param("Communication Timeout", m_args.comm_timeout)
        .defaultValue("5")
        .units(Units::Second)
        .description("Timeout for communication.");
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

      void
      openConnection(void)
      {
        try
        {
          m_TCP_sock = new TCPSocket;
          m_TCP_sock->connect(m_TCP_addr, m_TCP_port);
          inf(DTR("Local Piksi interface initialized"));
        }
        catch (...)
        {
          Memory::clear(m_TCP_sock);
          war(DTR("Local Connection failed, retrying..."));
          setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_COM_ERROR);
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
        m_TCP_addr = m_args.TCP_addr;
        m_TCP_port = m_args.TCP_port;

        openConnection();
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
        Memory::clear(m_TCP_sock);

      }

      bool
      poll(double timeout)
      {
        if (m_TCP_sock != NULL)
          return Poll::poll(*m_TCP_sock, timeout);

        return false;
      }

      int
      sendData(uint8_t* bfr, int size)
      {
        if (m_TCP_sock)
        {
          trace("Sending something");
          return m_TCP_sock->write((char*)bfr, size);
        }
        return 0;
      }

      int
      receiveData(uint8_t* buf, size_t blen)
      {
        if (m_TCP_sock)
        {
          try
          {
            return m_TCP_sock->read(buf, blen);
          }
          catch (std::runtime_error& e)
          {
            err("%s", e.what());
            war(DTR("Connection lost, retrying..."));
            Memory::clear(m_TCP_sock);

            m_TCP_sock = new Network::TCPSocket;
            m_TCP_sock->connect(m_TCP_addr, m_TCP_port);
            return 0;
          }
        }
        return 0;
      }

      //! Main loop.
      void
      onMain(void)
      {
        while (!stopping())
        {
          // Handle data
          if (m_TCP_sock)
          {
            handleInputData();
          }
          else
          {
            Time::Delay::wait(0.5);
            openConnection();
          }

          if (!m_error_missing_data)
          {

            if (getEntityState() != IMC::EntityState::ESTA_NORMAL)
            {
              setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
            }
          }

          // Handle IMC messages from bus
          consumeMessages();
        }
      }

      void
      handleInputData(void)
      {
        double now = Clock::get();
        int counter = 0;

        while (poll(0.01) && counter < 100)
        {
          counter++;

          int n = receiveData(m_buf, sizeof(m_buf));

          if (n < 0)
          {
            debug("Receive error");
            break;
          }

          now = Clock::get();

          /*
          // Do something with the data.
          */


          m_last_pkt_time = now;
        }



        if (now - m_last_pkt_time >= m_args.comm_timeout)
        {
          if (!m_error_missing_data)
          {
            setEntityState(IMC::EntityState::ESTA_ERROR, Status::CODE_MISSING_DATA);
            m_error_missing_data = true;
          }
        }
        else
          m_error_missing_data = false;
      }

    };
  }
}

DUNE_TASK
