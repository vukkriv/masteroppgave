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

// Library headers
extern "C"
{
  #include <libswiftnav/sbp.h>
  #include <libswiftnav/sbp_messages.h>
}



namespace Sensors
{
  namespace Piksi
  {
    using DUNE_NAMESPACES;

    //! %Sensor type
    enum SENSOR_TYPE
    {
      ROVER = 0,
      ROVER_SINGLE,
      BASE
    };

    //! %Task arguments.
    struct Arguments
    {
      //! Communications timeout
      uint8_t comm_timeout;
      //! TCP Port - Local Sensor
      uint16_t local_TCP_port;
      //! TCP Address - Local Sensor
      Address local_TCP_addr;

      //! TCP Port - Base station
      uint16_t base_TCP_port;
      //! TCP Address - Sensor
      Address base_TCP_addr;

      //! Sensor type
      std::string type;

    };

    struct Task: public DUNE::Tasks::Task
    {
      //! Task arguments.
      Arguments m_args;
      //! Processing state of incoming sbp messages
      sbp_state_t m_sbp_state;
      uint8_t m_buf[512];
      //! Map of callback-nodes for swiftnav API
      typedef std::map<int, sbp_msg_callbacks_node_t> CallbackNodeMap;
      CallbackNodeMap m_nodemap;
      //! Timestamp on previous local packet
      double m_local_last_pkt_time;
      //! Timestamp on previous base packet
      double m_base_last_pkt_time;
      //! TCP socket - Sensor
      TCPSocket* m_local_TCP_sock;
      Address m_local_TCP_addr;
      uint16_t m_local_TCP_port;
      //! TCP socket - Base
      TCPSocket* m_base_TCP_sock;
      Address m_base_TCP_addr;
      uint16_t m_base_TCP_port;
      bool m_error_local_missing_data;
      bool m_error_base_missing_data;

      //! Sensor Type
      SENSOR_TYPE m_type;
      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_local_last_pkt_time(0),
        m_base_last_pkt_time(0),
        m_local_TCP_sock(NULL),
        m_local_TCP_port(0),
        m_base_TCP_sock(NULL),
        m_base_TCP_port(0),
        m_error_local_missing_data(false),
        m_error_base_missing_data(false),
        m_type(ROVER)
      {

        param("Type", m_args.type)
        .defaultValue("Rover")
        .values("Rover,Single Rover,Base")
        .description("Sensor Type - Rover or Base");

        param("Local TCP - Port", m_args.local_TCP_port)
        .defaultValue("8880")
        .description("Port for connection to local Piksi");

        param("Local TCP - Address", m_args.local_TCP_addr)
        .defaultValue("127.0.0.1")
        .description("Address for connection to local Piksi");

        param("Rover -- Base TCP - Port", m_args.base_TCP_port)
        .defaultValue("8880")
        .description("Port for connection to remote base-station Piksi");

        param("Rover -- Base TCP - Address", m_args.base_TCP_addr)
        .defaultValue("192.168.1.46")
        .description("Address for connection to remote base-station Piksi");

        param("Communication Timeout", m_args.comm_timeout)
        .defaultValue("5")
        .units(Units::Second)
        .description("Timeout for base and local communication.");

        // Init piksi interface
        sbp_state_init(&m_sbp_state);


        // Populate callback nodes.
        // values are copied to the map.
        sbp_msg_callbacks_node_t tmp;
        m_nodemap[SBP_GPS_TIME] = tmp;
        m_nodemap[SBP_POS_LLH]  = tmp;
        m_nodemap[SBP_BASELINE_NED] = tmp;
        m_nodemap[SBP_VEL_NED]   = tmp;
        m_nodemap[SBP_DOPS]     = tmp;

        // Register task as context
        sbp_state_set_io_context(&m_sbp_state, (void*) this);

        // Register callbacks
        sbp_register_callback(&m_sbp_state, SBP_BASELINE_NED, sbp_baseline_ned_callback, (void*)this, &m_nodemap[SBP_BASELINE_NED]);
        sbp_register_callback(&m_sbp_state, SBP_POS_LLH,      sbp_pos_llh_callback, (void*)this, &m_nodemap[SBP_POS_LLH]);
        sbp_register_callback(&m_sbp_state, SBP_VEL_NED,      sbp_vel_ned_callback, (void*)this, &m_nodemap[SBP_VEL_NED]);
        sbp_register_callback(&m_sbp_state, SBP_DOPS,         sbp_dops_callback, (void*)this, &m_nodemap[SBP_DOPS]);
        sbp_register_callback(&m_sbp_state, SBP_GPS_TIME,     sbp_gps_time_callback, (void*)this, &m_nodemap[SBP_GPS_TIME]);

      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        // Check type
        if (m_args.type == "Base")
          m_type = BASE;
        else if (m_args.type == "Single Rover")
          m_type = ROVER_SINGLE;
        else
          m_type = ROVER;
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
        m_local_TCP_addr = m_args.local_TCP_addr;
        m_local_TCP_port = m_args.local_TCP_port;

        m_base_TCP_addr = m_args.base_TCP_addr;
        m_base_TCP_port = m_args.base_TCP_port;
        openLocalConnection();
        openBaseConnection();
      }

      void
      openLocalConnection(void)
      {
        try
        {
          m_local_TCP_sock = new TCPSocket;
          m_local_TCP_sock->connect(m_local_TCP_addr, m_local_TCP_port);
          inf(DTR("Local Piksi interface initialized"));
        }
        catch (...)
        {
          Memory::clear(m_local_TCP_sock);
          war(DTR("Local Connection failed, retrying..."));
          setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_COM_ERROR);
        }
      }

      void
      openBaseConnection()
      {
        // Try to connect to base if Rover
        if (m_type == ROVER)
        {
          try
          {
            m_base_TCP_sock = new TCPSocket;
            m_base_TCP_sock->connect(m_base_TCP_addr, m_base_TCP_port);
            inf(DTR("Remote Base Piksi interface initialized"));
          }
          catch (...)
          {
            Memory::clear(m_base_TCP_sock);
            war(DTR("Base Connection failed, retrying..."));
            setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_COM_ERROR);
          }
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
        Memory::clear(m_local_TCP_sock);
        Memory::clear(m_base_TCP_sock);
      }

      bool
      local_poll(double timeout)
      {
        if (m_local_TCP_sock != NULL)
          return Poll::poll(*m_local_TCP_sock, timeout);

        return false;
      }

      bool
      base_poll(double timeout)
      {
        if (m_base_TCP_sock != NULL)
          return Poll::poll(*m_base_TCP_sock, timeout);

        return false;
      }

      static int
      sendData(uint8_t* bfr, int size, void* context)
      {
        Task* task = (Task*)(context);
        if (task->m_local_TCP_sock)
        {
          task->trace("Sending something");
          return task->m_local_TCP_sock->write((char*)bfr, size);
        }
        return 0;
      }

      static u32
      receiveData(u8* buf, u32 blen, void* context)
      {
        Task* task = (Task*)(context);
        if (task->m_local_TCP_sock)
        {
          try
          {
            return task->m_local_TCP_sock->read(buf, blen);
          }
          catch (std::runtime_error& e)
          {
            task->err("%s", e.what());
            task->war(DTR("Connection lost locally, retrying..."));
            Memory::clear(task->m_local_TCP_sock);

            task->m_local_TCP_sock = new Network::TCPSocket;
            task->m_local_TCP_sock->connect(task->m_local_TCP_addr, task->m_local_TCP_port);
            return 0;
          }
        }
        return 0;
      }

      int
      receiveBaseData(uint8_t* buf, size_t blen)
      {
        if (m_base_TCP_sock)
        {
          try
          {
            return m_base_TCP_sock->read(buf, blen);
          }
          catch (std::runtime_error& e)
          {
            err("%s", e.what());
            war(DTR("Connection lost to Base, retrying..."));
            Memory::clear(m_base_TCP_sock);

            m_base_TCP_sock = new Network::TCPSocket;
            m_base_TCP_sock->connect(m_base_TCP_addr, m_base_TCP_port);
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
          if (m_local_TCP_sock)
          {
            handleLocalPiksiData();
          }
          else
          {
            Time::Delay::wait(0.5);
            openLocalConnection();
          }
          if (m_type == ROVER)
          {
            if (m_base_TCP_sock)
            {
              handleBasePiksiData();
            }
            else
            {
              Time::Delay::wait(0.5);
              openBaseConnection();
            }
          }

          if (!m_error_local_missing_data && !m_error_base_missing_data)
          {
            setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
          }



          // Handle IMC messages from bus
          consumeMessages();
        }
      }

      void
      handleLocalPiksiData(void)
      {

        // Reads incoming data and processes it with Piksi API.
        // This uses the call-back functions.
        double now = Clock::get();
        int counter = 0;

        while (local_poll(0.01) && counter < 100)
        {
          counter++;

          now = Clock::get();
          int result = sbp_process(&m_sbp_state, receiveData);
          m_local_last_pkt_time = now;

          switch(result)
          {
            case SBP_OK:
            case SBP_OK_CALLBACK_EXECUTED:
              break;
            case SBP_OK_CALLBACK_UNDEFINED:
              debug("Unknown message. (NB: may be heartbeat).");
              break;
            case SBP_CRC_ERROR:
              debug("Received message CRC error.");
              break;
            default:
              spew("Unknown result from sbp process.");
          }


        }

        if (now - m_local_last_pkt_time >= m_args.comm_timeout)
        {
          if (!m_error_local_missing_data)
          {
            setEntityState(IMC::EntityState::ESTA_ERROR, Status::CODE_MISSING_DATA);
            war("Local Piksi Timeout.");
            m_error_local_missing_data = true;
          }
        }
        else
          m_error_local_missing_data = false;
      }

      void
      handleBasePiksiData(void)
      {
        // Reads incoming data from base.
        // If any, just write to base TCP port.


        double now = Clock::get();
        int counter = 0;

        while (base_poll(0.01) && counter < 100)
        {
          counter++;

          now = Clock::get();
          // Write to base socket
          if (m_base_TCP_sock && m_local_TCP_sock)
          {

            int n = receiveBaseData(m_buf, sizeof(m_buf));

            if (n < 0)
            {
              debug("Receive error");
              break;
            }
            // Forward to local Piksi
            int n2 = sendData(m_buf, n, (void*) this);
            trace("Sent %d bytes to local Piksi from base. ", n2);
            now = Clock::get();



          }
          m_base_last_pkt_time = now;

          }

        if (now - m_base_last_pkt_time >= m_args.comm_timeout)
        {
          if (!m_error_base_missing_data)
          {
            setEntityState(IMC::EntityState::ESTA_ERROR, Status::CODE_NOT_SYNCHED);
            m_error_base_missing_data = true;
          }
        }
        else
          m_error_base_missing_data = false;

      }

      void
      handleBaselineNed(sbp_baseline_ned_t& msg)
      {
        (void) msg;
        inf("Got baseline ned");
      }

      void
      handlePosllh(sbp_pos_llh_t& msg)
      {
        (void) msg;
        inf("Got Pos LLH");
      }

      void
      handleVelNed(sbp_vel_ned_t& msg)
      {
        (void) msg;
        inf("Got vel ned");
      }

      void
      handleDops(sbp_dops_t& msg)
      {
        (void) msg;
        inf("got dops");
      }
      void
      handleGpsTime(sbp_gps_time_t& msg)
      {
        (void) msg;
        inf("got gps time");
      }


      /* Callback-methods for Piksi interface */
      static void
      sbp_baseline_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context)
      {
        sbp_baseline_ned_t baseline_ned = *(sbp_baseline_ned_t *)msg;
        Sensors::Piksi::Task* task = (Sensors::Piksi::Task*)(context);

        (void) sender_id;
        (void) len;
        task->handleBaselineNed(baseline_ned);
      }

      static void
      sbp_pos_llh_callback(u16 sender_id, u8 len, u8 msg[], void *context)
      {
        sbp_pos_llh_t pos_llh = *(sbp_pos_llh_t *)msg;
        Sensors::Piksi::Task* task = (Sensors::Piksi::Task*)(context);

        (void) sender_id;
        (void) len;
        task->handlePosllh(pos_llh);
      }

      static void
      sbp_vel_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context)
      {
        sbp_vel_ned_t vel_ned = *(sbp_vel_ned_t *)msg;
        Sensors::Piksi::Task* task = (Sensors::Piksi::Task*)(context);

        (void) sender_id;
        (void) len;
        task->handleVelNed(vel_ned);
      }

      static void
      sbp_dops_callback(u16 sender_id, u8 len, u8 msg[], void *context)
      {
        sbp_dops_t dops = *(sbp_dops_t *)msg;
        Sensors::Piksi::Task* task = (Sensors::Piksi::Task*)(context);

        (void) sender_id;
        (void) len;
        task->handleDops(dops);
      }

      static void
      sbp_gps_time_callback(u16 sender_id, u8 len, u8 msg[], void *context)
      {
        sbp_gps_time_t gps_time = *(sbp_gps_time_t *)msg;
        Sensors::Piksi::Task* task = (Sensors::Piksi::Task*)(context);

        (void) sender_id;
        (void) len;
        task->handleGpsTime(gps_time);
      }

    };
  }
}

DUNE_TASK
