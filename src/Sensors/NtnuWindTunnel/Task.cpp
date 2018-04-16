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
// Author: Kristoffer Gryte                                                 *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

#include<sstream>

namespace Sensors
{
  namespace NtnuWindTunnel
  {
    using DUNE_NAMESPACES;

    //! %Task arguments.
    struct Arguments
    {
      //! port from sitl
      uint16_t sitl_port_in;
      //! Port to sitl
      uint16_t sitl_port_out;
      //! Address of ardupilot sitl application
      Address sitl_addr;
      std::string parse_string;
    };

    // Interface between %DUNE, Ardupilot control and APM SITL.
    struct Task: public DUNE::Tasks::Task
    {
      //! Arguments
      Arguments m_args;

      //! Port from SITL.
      uint16_t m_sitl_port_in;
      //! Address of ardupilot sitl application
      Address m_sitl_addr;

      //! UDP Socket for input from apm.
      UDPSocket* m_udp_sock;
      //! Buffer
      uint8_t m_buf[512];

      IMC::Weight m_force; //abuse of notation...
      IMC::IndicatedSpeed m_airspeed;
      IMC::AssistedControlMode m_string;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx)
      {
        param("SITL - Port In", m_args.sitl_port_in)
        .defaultValue("5502")
        .description("Port for data from the sitl application");

        param("SITL - Address", m_args.sitl_addr)
        .defaultValue("127.0.0.1")
        .description("Address of the sitl application.");

        param("Parse string", m_args.parse_string)
        .defaultValue("127.0.0.1")
        .description("Address of the sitl application.");

        // Set OK status
        setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
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
        m_sitl_addr = m_args.sitl_addr;
        m_sitl_port_in = m_args.sitl_port_in;

        openConnection();
      }

      void
      openConnection()
      {
        Memory::clear(m_udp_sock);
        m_udp_sock = new UDPSocket;
        m_udp_sock->bind(m_sitl_port_in, "");
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
        Memory::clear(m_udp_sock);
      }

      int
      receiveData(uint8_t* buf, size_t blen)
      {
        if (m_udp_sock)
        {
          try
          {
            return m_udp_sock->read(buf, blen);
          }
          catch (...)
          {
            war(DTR("Connection lost, retrying..."));
            Memory::clear(m_udp_sock);

            openConnection();
            return 0;
          }
        }
        return 0;
      }

      bool
      poll(double timeout)
      {
        if (m_udp_sock != NULL)
        {
          /* spew("polling"); */
          return Poll::poll(*m_udp_sock, timeout);
        }
        return false;
      }

      //! Main loop.
      void
      onMain(void)
      {
        inf("Starting");
        while (!stopping())
        {
          //! \begin{Spaghetti-code}
          // Poll socket for incomming RC messages
          if (m_udp_sock != NULL)
          {
            while (poll(0.01))
            {

              /* spew(DTR("Got data from ardpilot SIL.")); */
              int n = receiveData(m_buf, sizeof(m_buf));
              debug("buf: %s", m_buf);
              /* debug("buf: %s", (char*)m_buf); */
              /* debug("n: %d", n); */
              double airspeed,dummy,force;
              /* uint8_t buf [1000]; */
              /* sprintf((char*)buf,"%5.2f\t%5.2f\t%5.2f\n",123.0,456.0,789.0); */
              sscanf((char*)m_buf, m_args.parse_string.c_str(), &airspeed,&dummy,&force);
              /* sscanf((char*)buf, m_args.parse_string.c_str(), &airspeed,&dummy,&force); */

              /* debug("Parse str: %s", m_args.parse_string.c_str()); */
              /* debug("airspeed: %f", airspeed); */
              /* debug("dummy: %f", dummy); */
              /* debug("force: %f", force); */

              m_airspeed.value = airspeed;
              m_force.value = force;
              std::string s((char*)m_buf);
              m_string.mode = s;
              
              dispatch(m_string);
              dispatch(m_airspeed);
              dispatch(m_force);
            } // end while
          }
          else
          {
            spew("No udp");
          }
          consumeMessages();
        }
      }
    };
  }
}

DUNE_TASK
