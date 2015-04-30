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
      ROVER = 0,  // Vehicle, calc and send BL etc
      STANDALONE, // Only broadcasts LLH
      BASE        // Only broadcasts observations
    };

    const int PIKSI_MSG_INIT_BASELINE = 0x23;
    const int PIKSI_MSG_RESET_FILTERS = 0x22;
    const int PIKSI_MSG_IAR_STATE     = 0x0019;

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
      //! UDP port to receive observations
      uint16_t UDP_port;

      //! Sensor type
      std::string type;

      //! Dispatch GpsFix or not
      bool dispatch_gpsfix;

      //! Baseline timeout
      uint8_t baseline_timeout;
      //! Observations timeout
      uint8_t observations_timeout;

      //! Apply low-pass smoothing to GPS data
      bool apply_gps_smoothing;
      //! Time constants for low-pass smoothing of GPS data
      double gps_smoothing_T;

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
      bool m_error_local_missing_data;
      //! TCP socket - Base
      TCPSocket* m_base_TCP_sock;
      Address m_base_TCP_addr;
      uint16_t m_base_TCP_port;
      //! Udp receiving socket
      UDPSocket* m_udp_socket;
      uint16_t m_udp_port;
      bool m_error_base_missing_data;
      //! Navdata from Piksi - Rover
      IMC::RtkFix m_rtk_fix;
      IMC::GpsFix m_gps_fix;
      uint16_t m_gps_week;
      double m_last_baseline_time;
      //! Piksi data scaling
      const double m_pos_scale;
      const double m_vel_scale;
      const double m_dop_scale;

      //! Sensor Type
      SENSOR_TYPE m_type;

      bool m_dispatch_gpsfix;


      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_local_last_pkt_time(0),
        m_base_last_pkt_time(0),
        m_local_TCP_sock(NULL),
        m_local_TCP_port(0),
        m_error_local_missing_data(false),
        m_base_TCP_sock(NULL),
        m_base_TCP_port(0),
        m_udp_socket(NULL),
        m_udp_port(0),
        m_error_base_missing_data(false),
        m_gps_week(0),
        m_last_baseline_time(0),
        m_pos_scale(1E-3), // Piksi sends positions in mm, scale to m
        m_vel_scale(1E-3), // Piksi sends velocities in mm/s, scale to m/s
        m_dop_scale(1E-2), // Piksi sends dop in 0.01, scale to 1
        m_type(ROVER),
        m_dispatch_gpsfix(true)
      {

        param("Type", m_args.type)
        .defaultValue("Rover")
        .values("Rover,Standalone,Base")
        .description("Sensor Type - Rover, Standalone or Base");

        param("Local TCP - Port", m_args.local_TCP_port)
        .defaultValue("8880")
        .description("Port for connection to local Piksi");

        param("Local TCP - Address", m_args.local_TCP_addr)
        .defaultValue("127.0.0.1")
        .description("Address for connection to local Piksi");

        param("Rover -- Base TCP - Port", m_args.base_TCP_port)
        .defaultValue("8881")
        .description("Port for connection to remote base-station Piksi");

        param("Rover -- Base TCP - Address", m_args.base_TCP_addr)
        .defaultValue("192.168.1.46")
        .description("Address for connection to remote base-station Piksi");

        param("UDP Port Observations", m_args.UDP_port)
        .defaultValue("8890")
        .description("Communication port for observations");

        param("Communication Timeout", m_args.comm_timeout)
        .defaultValue("5")
        .units(Units::Second)
        .description("Timeout for base and local communication.");

        param("Dispatch GpsFix", m_args.dispatch_gpsfix)
        .defaultValue("True")
        .visibility(Tasks::Parameter::VISIBILITY_USER)
        .description("Dispatch GpsFix or not.");

        param("Baseline Timeout", m_args.baseline_timeout)
        .defaultValue("1")
        .units(Units::Second)
        .description("Timeout for local baseline received.");

        param("Observations Timeout", m_args.observations_timeout)
        .defaultValue("1")
        .units(Units::Second)
        .description("Timeout for base observations received.");

        param("Use GPS Smoothing", m_args.apply_gps_smoothing)
        .defaultValue("true")
        .visibility(Tasks::Parameter::VISIBILITY_USER)
        .description("Apply low-pass smoothing to GPS data (pos and vel)");

        param("GPS Smoothing Time Constant", m_args.gps_smoothing_T)
        .defaultValue("2.0")
        .units(Units::Second)
        .visibility(Tasks::Parameter::VISIBILITY_USER)
        .description("Time constant used in low-pass smoothing of GPS data");

        // Bind to incoming IMC messages
        bind<IMC::RemoteActions>(this);

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
        m_nodemap[PIKSI_MSG_IAR_STATE] = tmp;

        // Register task as context
        sbp_state_set_io_context(&m_sbp_state, (void*) this);

        // Register callbacks
        sbp_register_callback(&m_sbp_state, SBP_BASELINE_NED, sbp_baseline_ned_callback, (void*)this, &m_nodemap[SBP_BASELINE_NED]);
        sbp_register_callback(&m_sbp_state, SBP_POS_LLH,      sbp_pos_llh_callback, (void*)this, &m_nodemap[SBP_POS_LLH]);
        sbp_register_callback(&m_sbp_state, SBP_VEL_NED,      sbp_vel_ned_callback, (void*)this, &m_nodemap[SBP_VEL_NED]);
        sbp_register_callback(&m_sbp_state, SBP_DOPS,         sbp_dops_callback, (void*)this, &m_nodemap[SBP_DOPS]);
        sbp_register_callback(&m_sbp_state, SBP_GPS_TIME,     sbp_gps_time_callback, (void*)this, &m_nodemap[SBP_GPS_TIME]);
        sbp_register_callback(&m_sbp_state, PIKSI_MSG_IAR_STATE,     sbp_iar_state_callback, (void*)this, &m_nodemap[PIKSI_MSG_IAR_STATE]);


      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        // Check type
        if (m_args.type == "Base")
          m_type = BASE;
        else if (m_args.type == "Standalone")
          m_type = STANDALONE;
        else
          m_type = ROVER;

        // Set gps dispatch
        m_dispatch_gpsfix = m_args.dispatch_gpsfix;


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
        //openBaseConnection();

        // Update port
        m_udp_port = m_args.UDP_port;
        openUdpConnection();


      }

      void
      openUdpConnection(void)
      {
        try
        {
          m_udp_socket = new UDPSocket();
          m_udp_socket->bind(m_udp_port);
          inf(DTR("Remote Base Piksi UDP interface initialized"));
        }
        catch (Exception &ex)
        {
          Memory::clear(m_udp_socket);
          war(DTR("UDP bind failed. %s"), ex.what());
          setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_COM_ERROR);
        }
      }

      void
      openLocalConnection(void)
      {
        try
        {
          m_local_TCP_sock = new TCPSocket;
          m_local_TCP_sock->connect(m_local_TCP_addr, m_local_TCP_port);
          m_local_TCP_sock->setNoDelay(true);
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

      void
      consume(const IMC::RemoteActions* action)
      {
        inf("Got an action: %s", action->actions.c_str());

        TupleList list = TupleList(action->actions);
        if(list.get("piksiInitZeroBaseline") == "1")
        {
          // Send message to init baseline
          inf("Resetting piksi baseline.");
          u16 sender_id = 99;
          sbp_send_message(&m_sbp_state, PIKSI_MSG_INIT_BASELINE, sender_id, 0, NULL, &sendLocalData);
        }
        if(list.get("piksiResetFilters") == "1")
        {
          // Send message to reset Filters¨
          inf("Resetting Piksi Filters");

          u8 value = 0x00;
          u16 sender_id = 99;
          sbp_send_message(&m_sbp_state, PIKSI_MSG_RESET_FILTERS, sender_id, 1, &value, &sendLocalData);

        }

        if(list.get("piksiResetIARs") == "1")
        {
          // Send message to reset IAR
          inf("Resetting Piksi IARs");


          u8 value = 0x01;
          u16 sender_id = 99;
          sbp_send_message(&m_sbp_state, PIKSI_MSG_RESET_FILTERS, sender_id, 1, &value, &sendLocalData);
        }


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

      bool
      udp_poll(double timeout)
      {
        if (m_udp_socket != NULL)
          return Poll::poll(*m_udp_socket, timeout);

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
      sendLocalData(u8* bfr, u32 size, void* context)
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
      receiveUdpData(uint8_t* buf, size_t blen)
      {
        if (m_udp_socket)
        {
          try
          {
            return m_udp_socket->read(buf, blen);
          }
          catch (std::runtime_error& e)
          {
            err("%s", e.what());
            war(DTR("Connection lost to Base, retrying..."));
            Memory::clear(m_udp_socket);


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

      Matrix
      lowPassSmoothing(Matrix output, Matrix input, double dt, double RC)
      {
        if (output.isZeroSized())
          return input;

        // Check valid inputs
        assert(input.size() == output.size());
        assert(dt > 0);
        assert(RC > 0);

        // Calculate alpha
        double alpha = dt/(RC + dt);

        // Apply smoothing to all data
        for (int i = 0; i < input.size(); i++)
        {
          output(i) += alpha*(input(i) - output(i));
        }
        return output;
      }

      double
      lowPassSmoothing(double output, double input, double dt, double RC)
      {
        // Check valid inputs
        assert(dt > 0);
        assert(RC > 0);

        // Calculate alpha
        double alpha = dt/(RC + dt);

        // Apply smoothing
        return output += alpha*(input - output);
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
            if (m_udp_socket)
            {
              handleObservationData();
            }
            else
            {
              Time::Delay::wait(0.5);
              openUdpConnection();
            }

            // Check if we should send "empty" RtkFix
            double now = Clock::get();
            if (now - m_last_baseline_time >= m_args.baseline_timeout)
            {
              if (now - m_base_last_pkt_time >= m_args.observations_timeout)
              {
                // We don't have a baseline solution
                m_rtk_fix.type = IMC::RtkFix::RTK_NONE;
              }
              else
              {
                // We don't have a baseline solution, but we are receving observation data from BASE
                m_rtk_fix.type = IMC::RtkFix::RTK_OBS;
              }

              // Set time for baseline so that we only send "empty" RtkFix every "baseline_timeout"
              m_last_baseline_time = Clock::get();

              // Send "empty" RtkFix"
              dispatch(m_rtk_fix);
              trace("Sent (empty) RTK Fix");
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
              trace("Unknown message. (NB: may be heartbeat).");
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
      handleObservationData(void)
      {
        // Reads incoming UDP data and forward to the local piksi.

        double now = Clock::get();
        int counter = 0;

        while (udp_poll(0.01) && counter < 100)
        {
          counter++;

          now = Clock::get();
          // Write to local socket
          if (m_udp_socket && m_local_TCP_sock)
          {

            int n = receiveUdpData(m_buf, sizeof(m_buf));

            if (n < 0)
            {
              debug("Receive error");
              break;
            }
            // Forward to local Piksi
            int n2 = sendData(m_buf, n, (void*) this);
            spew("Sent %d bytes to local Piksi from base. ", n2);
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
      handleBasePiksiData(void)
      {
        // Reads incoming data from base.
        // If any, just write to rover TCP port.


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
            spew("Sent %d bytes to local Piksi from base. ", n2);
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
        spew("Got baseline ned");

        m_last_baseline_time = Clock::get();

        m_rtk_fix.tow = (uint32_t)msg.tow;
        m_rtk_fix.n = m_pos_scale*(fp32_t)msg.n;
        m_rtk_fix.e = m_pos_scale*(fp32_t)msg.e;
        m_rtk_fix.d = m_pos_scale*(fp32_t)msg.d;
        m_rtk_fix.satellites = (uint8_t)msg.n_sats;
        switch (msg.flags)
        {
          case 0:
            m_rtk_fix.type = IMC::RtkFix::RTK_FLOAT;
            break;
          case 1:
            m_rtk_fix.type = IMC::RtkFix::RTK_FIXED;
            break;
        }

        dispatch(m_rtk_fix);
        spew("Sent RTK Fix");
        //m_rtk_fix.toText(std::cout);
      }

      void
      handlePosllh(sbp_pos_llh_t& msg)
      {
        spew("Got Pos LLH");

        if (m_dispatch_gpsfix)
        {
          // Check that GPS week has been set by a dops message
          if (m_gps_week > 0)
          {
            float prev_fix_utc_time = m_gps_fix.utc_time;
            // Convert from GPS week and TOW to UTC
            gps_to_ymdt(m_gps_week, msg.tow, &m_gps_fix.utc_year, &m_gps_fix.utc_month, &m_gps_fix.utc_day, &m_gps_fix.utc_time);
            m_gps_fix.validity |= IMC::GpsFix::GFV_VALID_DATE;
            m_gps_fix.validity |= IMC::GpsFix::GFV_VALID_TIME;

            if (m_args.apply_gps_smoothing && (m_gps_fix.validity & IMC::GpsFix::GFV_VALID_POS))
            {
              float dt = m_gps_fix.utc_time - prev_fix_utc_time;

              m_gps_fix.lat = lowPassSmoothing(m_gps_fix.lat, Angles::radians(msg.lat), dt, m_args.gps_smoothing_T);
              m_gps_fix.lon = lowPassSmoothing(m_gps_fix.lon, Angles::radians(msg.lon), dt, m_args.gps_smoothing_T);
              m_gps_fix.height = lowPassSmoothing(m_gps_fix.height, (fp32_t)msg.height, dt, m_args.gps_smoothing_T);
            }
            else
            {
              m_gps_fix.lat = Angles::radians(msg.lat);
              m_gps_fix.lon = Angles::radians(msg.lon);
              m_gps_fix.height = (fp32_t)msg.height;
            }
            m_gps_fix.validity |= IMC::GpsFix::GFV_VALID_POS;

            m_gps_fix.hacc = m_pos_scale*(fp32_t)msg.h_accuracy;
            m_gps_fix.validity |= IMC::GpsFix::GFV_VALID_HACC;

            m_gps_fix.vacc = m_pos_scale*(fp32_t)msg.v_accuracy;
            m_gps_fix.validity |= IMC::GpsFix::GFV_VALID_VACC;

            m_gps_fix.satellites = (uint8_t)msg.n_sats;
            m_gps_fix.type = IMC::GpsFix::GFT_STANDALONE;

            dispatch(m_gps_fix);
            spew("Sent GPS Fix");
            //m_gps_fix.toText(std::cerr);
          }
        }
      }

      void
      gps_to_ymdhms(uint32_t gps_week, uint32_t ITOW, uint16_t *year, uint8_t *month,
          uint8_t* day, uint8_t *hours, uint8_t *minutes, float *seconds)
      {
        *seconds = (float) (ITOW % 1000) / 1000;

        long gps_sec_since_1970 = 315964800 + 7 * 24 * 60 * 60 * gps_week + ITOW / 1000;

        time_t t = gps_sec_since_1970;

        struct tm* utc;

        utc = gmtime(&t);

        *year = utc->tm_year + 1900;
        *month = utc->tm_mon +1;
        *day = utc->tm_mday;
        *hours = utc->tm_hour;
        *minutes = utc->tm_min;
        *seconds += utc->tm_sec;
      }

      void
      gps_to_ymdt(uint32_t gps_week, uint32_t ITOW, uint16_t *year, uint8_t *month,
          uint8_t* day, float *time)
      {
        uint8_t hours;
        uint8_t minutes;
        float seconds;

        gps_to_ymdhms(gps_week, ITOW, year, month, day, &hours, &minutes, &seconds);

        *time = 3600 * hours + 60 * minutes + seconds;
      }

      void
      handleVelNed(sbp_vel_ned_t& msg)
      {
        trace("Got vel ned");
        static u32 prev_vel_ned;

        m_rtk_fix.v_n = m_vel_scale*(fp32_t)msg.n;
        m_rtk_fix.v_e = m_vel_scale*(fp32_t)msg.e;
        m_rtk_fix.v_d = m_vel_scale*(fp32_t)msg.d;

        if (m_dispatch_gpsfix)
        {
          if (m_args.apply_gps_smoothing && (m_gps_fix.validity & IMC::GpsFix::GFV_VALID_SOG))
          {
            double dt = msg.tow - prev_vel_ned;
            m_gps_fix.sog = lowPassSmoothing(m_gps_fix.sog, m_vel_scale*(fp32_t)(Math::norm(msg.n,msg.e)), dt, m_args.gps_smoothing_T);
            m_gps_fix.cog = std::atan2(msg.e,msg.n);
          }
          else
          {
            m_gps_fix.sog = m_vel_scale*(fp32_t)(Math::norm(msg.n,msg.e));
            m_gps_fix.cog = std::atan2(msg.e,msg.n);
          }
          m_gps_fix.validity |= IMC::GpsFix::GFV_VALID_COG;
          m_gps_fix.validity |= IMC::GpsFix::GFV_VALID_SOG;

          // GpsFix is only dispatched by handlePosllh
        }
        prev_vel_ned = msg.tow;

        // RtkFix is only dispatched by handleBaselineNed
      }

      void
      handleDops(sbp_dops_t& msg)
      {
        trace("GOT dops");

        if (m_dispatch_gpsfix)
        {
          m_gps_fix.hdop = m_dop_scale*(fp32_t)msg.hdop;
          m_gps_fix.validity |= IMC::GpsFix::GFV_VALID_HDOP;

          m_gps_fix.vdop = m_dop_scale*(fp32_t)msg.vdop;
          m_gps_fix.validity |= IMC::GpsFix::GFV_VALID_VDOP;

          // gps_fix is only dispatched by handlePosllh
        }
      }

      void
      handleGpsTime(sbp_gps_time_t& msg)
      {
        trace("Got GPS time");

        if (m_dispatch_gpsfix)
        {
          uint16_t gps_week = (uint16_t)msg.wn;

          if (m_gps_week == 0)
          {
            m_gps_week = gps_week;
            inf("GPS week number set");
          }
          else if (m_gps_week != gps_week)
          {
            m_gps_week = gps_week;
            war("GPS week number has changed and has been reset");
          }
        }
      }

      void
      handleIARState(uint32_t& iar)
      {
        trace("Got IAR-state: %u", iar);

        m_rtk_fix.iar_hyp = iar;
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

      static void
      sbp_iar_state_callback(u16 sender_id, u8 len, u8 msg[], void *context)
      {
        uint32_t *iar = (uint32_t *)msg;
        Sensors::Piksi::Task* task = (Sensors::Piksi::Task*)(context);

        (void) sender_id;
        (void) len;
        task->handleIARState(*iar);
      }

    };
  }
}

DUNE_TASK
