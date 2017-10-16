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
// Author: Frederik Leira                                                   *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

// MAVLink headers.
#include <mavlink/ardupilotmega/mavlink.h>

namespace Transports
{
  namespace MAVProxy
  {
    using DUNE_NAMESPACES;

    //! %Task arguments.
    struct Arguments
    {
      //! Communications timeout
      uint8_t comm_timeout;
      //! Use Ardupilot's waypoint tracker
      bool ardu_tracker;
      //! TCP Port
      uint16_t TCP_port;
      //! TCP Address
      Address TCP_addr;
      //! Telemetry Rate
      uint8_t trate;
      //! Convert MSL to WGS84 height
      bool convert_msl;
      //! Name of Target
      std::string target;
    };

    struct Task: public DUNE::Tasks::Task
    {
      //! Task arguments.
      Arguments m_args;
      //! Type definition for Arduino packet handler.
      typedef void (Task::* PktHandler)(const mavlink_message_t* msg);
      typedef std::map<int, PktHandler> PktHandlerMap;
      //! Arduino packet handling
      PktHandlerMap m_mlh;
      double m_last_pkt_time;
      uint8_t m_buf[512];
      //! Estimated state message.
      IMC::EstimatedState m_estate;
      //! GPS Fix message
      IMC::GpsFix m_fix;
      //! Localization origin (WGS-84)
      fp64_t m_ref_lat, m_ref_lon;
      fp32_t m_ref_hae;
      //! TCP socket
      Network::TCPSocket* m_TCP_sock;
      //! System ID
      uint8_t m_sysid;
      //! Last received position
      double m_lat, m_lon;
      //! Height received from the Ardupilot (Geoid/MSL).
      float m_hae_msl;
      //! Height offset to convert to WGS-84 ellipsoid.
      float m_hae_offset;
      //! Parser Variables
      mavlink_message_t m_msg;
      int m_mode;
      bool m_error_missing, m_esta_ext;
      //! Setup rate hack
      bool m_has_setup_rate;

      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_ref_lat(0.0),
        m_ref_lon(0.0),
        m_ref_hae(0.0),
        m_TCP_sock(NULL),
        m_sysid(1),
        m_lat(0.0),
        m_lon(0.0),
        m_hae_msl(0.0),
        m_hae_offset(0.0),
        m_mode(0),
        m_error_missing(false),
        m_esta_ext(false),
        m_has_setup_rate(false)
      {
        param("TCP - Port", m_args.TCP_port)
        .defaultValue("5760")
        .description("Port for connection to Ardupilot");

        param("TCP - Address", m_args.TCP_addr)
        .defaultValue("127.0.0.1")
        .description("Address for connection to Ardupilot");

        param("Telemetry Rate", m_args.trate)
        .defaultValue("10")
        .units(Units::Hertz)
        .description("Telemetry output rate from Ardupilot");

        param("Convert MSL to WGS84 height", m_args.convert_msl)
        .defaultValue("false")
        .description("Convert altitude extracted from the Ardupilot to WGS84 height");
 	
	param("Target System", m_args.target)
        .description("System to be tracked")
        .defaultValue("ntnu-ghost");

        // Setup packet handlers
        // IMPORTANT: set up function to handle each type of MAVLINK packet here
        m_mlh[MAVLINK_MSG_ID_ATTITUDE] = &Task::handleAttitudePacket;
        m_mlh[MAVLINK_MSG_ID_GLOBAL_POSITION_INT] = &Task::handlePositionPacket;
        m_mlh[MAVLINK_MSG_ID_GPS_RAW_INT] = &Task::handleRawGPSPacket;
        m_mlh[MAVLINK_MSG_ID_VFR_HUD] = &Task::handleHUDPacket;
        m_mlh[MAVLINK_MSG_ID_SYSTEM_TIME] = &Task::handleSystemTimePacket;

        //! Misc. initialization
        m_last_pkt_time = 0; //! time of last packet from Ardupilot
        m_estate.clear();
        m_estate.setSource(resolveSystemName(m_args.target)); //ntnu-ghost - 0x2cff
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

      void
      openConnection(void)
      {
        try
        {
          m_TCP_sock = new TCPSocket;
          m_TCP_sock->connect(m_args.TCP_addr, m_args.TCP_port);
          m_TCP_sock->setNoDelay(true);
          setupRate(m_args.trate);
          inf(DTR("Ardupilot interface initialized"));
        }
        catch (...)
        {
          Memory::clear(m_TCP_sock);
          war(DTR("Connection failed, retrying..."));
          setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_COM_ERROR);
        }
      }
      
      void
      setupRate(uint8_t rate)
      {
        uint8_t buf[512];
        mavlink_message_t msg;

        //! ATTITUDE and SIMSTATE messages
        mavlink_msg_request_data_stream_pack(255, 0, &msg,
                                             m_sysid,
                                             0,
                                             MAV_DATA_STREAM_EXTRA1,
                                             rate,
                                             1);

        uint16_t n = mavlink_msg_to_send_buffer(buf, &msg);
        sendData(buf, n);
        spew("ATTITUDE Stream setup to %d Hertz", rate);

        //! VFR_HUD message
        mavlink_msg_request_data_stream_pack(255, 0, &msg,
                                             m_sysid,
                                             0,
                                             MAV_DATA_STREAM_EXTRA2,
                                             rate,
                                             1);

        n = mavlink_msg_to_send_buffer(buf, &msg);
        sendData(buf, n);
        spew("VFR Stream setup to %d Hertz", rate);

        //! GLOBAL_POSITION_INT message
        mavlink_msg_request_data_stream_pack(255, 0, &msg,
                                             m_sysid,
                                             0,
                                             MAV_DATA_STREAM_POSITION,
                                             rate,
                                             1);

        n = mavlink_msg_to_send_buffer(buf, &msg);
        sendData(buf, n);
        spew("POSITION Stream setup to %d Hertz", rate);

        //! SYS_STATUS, POWER_STATUS, MEMINFO, MISSION_CURRENT,
        //! GPS_RAW_INT, NAV_CONTROLLER_OUTPUT and FENCE_STATUS messages
        mavlink_msg_request_data_stream_pack(255, 0, &msg,
                                             m_sysid,
                                             0,
                                             MAV_DATA_STREAM_EXTENDED_STATUS,
                                             (int)(rate/5),
                                             1);

        n = mavlink_msg_to_send_buffer(buf, &msg);
        sendData(buf, n);
        spew("STATUS Stream setup to %d Hertz", (int)(rate/5));

        //! AHRS, HWSTATUS, WIND, RANGEFINDER and SYSTEM_TIME messages
        mavlink_msg_request_data_stream_pack(255, 0, &msg,
                                             m_sysid,
                                             0,
                                             MAV_DATA_STREAM_EXTRA3,
                                             1,
                                             1);

        n = mavlink_msg_to_send_buffer(buf, &msg);
        sendData(buf, n);
        spew("AHRS-HWSTATUS-WIND Stream setup to 1 Hertz");
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
            handleArdupilotData();
          }
          else
          {
            Time::Delay::wait(0.5);
            openConnection();
          }
          // Handle IMC messages from bus
          consumeMessages();
        }
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
            m_TCP_sock->connect(m_args.TCP_addr, m_args.TCP_port);
            return 0;
          }
        }
        return 0;
      }

      void
      handleArdupilotData(void)
      {
        mavlink_status_t status;

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

          for (int i = 0; i < n; i++)
          {
            int rv = mavlink_parse_char(MAVLINK_COMM_0, m_buf[i], &m_msg, &status);
            if (status.packet_rx_drop_count)
            {
              switch(status.parse_state)
              {
                case MAVLINK_PARSE_STATE_IDLE:
                  spew("failed at state IDLE");
                  break;
                case MAVLINK_PARSE_STATE_GOT_STX:
                  spew("failed at state GOT_STX");
                  break;
                case MAVLINK_PARSE_STATE_GOT_LENGTH:
                  spew("failed at state GOT_LENGTH");
                  break;
                case MAVLINK_PARSE_STATE_GOT_SYSID:
                  spew("failed at state GOT_SYSID");
                  break;
                case MAVLINK_PARSE_STATE_GOT_COMPID:
                  spew("failed at state GOT_COMPID");
                  break;
                case MAVLINK_PARSE_STATE_GOT_MSGID:
                  spew("failed at state GOT_MSGID");
                  break;
                case MAVLINK_PARSE_STATE_GOT_PAYLOAD:
                  spew("failed at state GOT_PAYLOAD");
                  break;
                case MAVLINK_PARSE_STATE_GOT_CRC1:
                  spew("failed at state GOT_CRC1");
                  break;
                default:
                  spew("failed OTHER");
              }
            }
            if (rv)
            {
              switch ((int)m_msg.msgid)
              {
                default:
                  trace("UNDEF: %u", m_msg.msgid);
                  break;
                case MAVLINK_MSG_ID_GPS_RAW_INT:
                  spew("GPS_RAW");
                  break;
                case 27:
                  trace("IMU_RAW");
                  break;
                case MAVLINK_MSG_ID_ATTITUDE:
                  spew("ATTITUDE");
                  break;
                case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                  spew("GLOBAL_POSITION_INT");
                  break;
              }

              PktHandler h = m_mlh[m_msg.msgid];

              if (!h)
                continue;  // Ignore this packet (no handler for it)

              // Call handler
              (this->*h)(&m_msg);
              m_sysid = m_msg.sysid;

              m_last_pkt_time = now;
            }
          }
        }

        if (now - m_last_pkt_time >= m_args.comm_timeout)
        {
          if (!m_error_missing)
          {
            setEntityState(IMC::EntityState::ESTA_ERROR, Status::CODE_MISSING_DATA);
            m_error_missing = true;
            m_has_setup_rate = false;
            m_esta_ext = false;
          }
        }
        else
          m_error_missing = false;
      }

      void
      handleAttitudePacket(const mavlink_message_t* msg)
      {
        mavlink_attitude_t att;

        mavlink_msg_attitude_decode(msg, &att);
        m_estate.phi = att.roll;
        m_estate.theta = att.pitch;
        m_estate.psi = att.yaw;
        m_estate.p = att.rollspeed;
        m_estate.q = att.pitchspeed;
        m_estate.r = att.yawspeed;

        dispatch(m_estate);
      }

      void
      handlePositionPacket(const mavlink_message_t* msg)
      {
        mavlink_global_position_int_t gp;
        mavlink_msg_global_position_int_decode(msg, &gp);

        double lat = Angles::radians((double)gp.lat * 1e-07);
        double lon = Angles::radians((double)gp.lon * 1e-07);

        m_lat = (double)gp.lat * 1e-07;
        m_lon = (double)gp.lon * 1e-07;

        double d = WGS84::distance(m_ref_lat, m_ref_lon, m_ref_hae,
                                   lat, lon, getHeight());
        if (d > 1000.0)
        {
          if (m_args.convert_msl)
          {
            Coordinates::WMM wmm(m_ctx.dir_cfg);
            m_hae_offset = wmm.height(lat, lon);
          }
          else
          {
            m_hae_offset = 0;
          }

          m_estate.lat = lat;
          m_estate.lon = lon;
          m_estate.height = getHeight();

          m_estate.x = 0;
          m_estate.y = 0;
          m_estate.z = 0;

          m_ref_lat = lat;
          m_ref_lon = lon;
          m_ref_hae = getHeight();

        }
        else
        {
          WGS84::displacement(m_ref_lat, m_ref_lon, m_ref_hae,
                              lat, lon, getHeight(),
                              &m_estate.x, &m_estate.y, &m_estate.z);
        }

        m_estate.vx = 1e-02 * gp.vx;
        m_estate.vy = 1e-02 * gp.vy;
        m_estate.vz = -1e-02 * gp.vz;

        // Note: the following will yield body-fixed *ground* velocity
        // Maybe this can be fixed w/IAS readings (anyway not too important)
        BodyFixedFrame::toBodyFrame(m_estate.phi, m_estate.theta, m_estate.psi,
                                    m_estate.vx, m_estate.vy, m_estate.vz,
                                    &m_estate.u, &m_estate.v, &m_estate.w);

        m_estate.depth = -1;
        m_estate.alt = -1;
      }

      float
      getHeight(void)
      {
        return m_hae_msl + m_hae_offset;
      }

      void
      handleRawGPSPacket(const mavlink_message_t* msg)
      {
        mavlink_gps_raw_int_t gps_raw;

        mavlink_msg_gps_raw_int_decode(msg, &gps_raw);

        m_fix.cog = Angles::radians((double)gps_raw.cog * 0.01);
        m_fix.sog = (float)gps_raw.vel * 0.01;
        m_fix.hdop = (float)gps_raw.eph * 0.01;
        m_fix.vdop = (float)gps_raw.epv * 0.01;
        m_fix.lat = Angles::radians((double)gps_raw.lat * 1e-07);
        m_fix.lon = Angles::radians((double)gps_raw.lon * 1e-07);
        m_fix.height = (double)gps_raw.alt * 0.001;
        m_fix.satellites = gps_raw.satellites_visible;

        m_fix.validity = 0;
        if (gps_raw.fix_type > 1)
        {
          m_fix.validity |= IMC::GpsFix::GFV_VALID_POS;
          m_fix.type = IMC::GpsFix::GFT_STANDALONE;
        }
        else
          m_fix.type = IMC::GpsFix::GFT_DEAD_RECKONING;

        if (gps_raw.fix_type == 3)
        {
          m_fix.validity |= IMC::GpsFix::GFV_VALID_VDOP;
          m_fix.vdop = 5;
        }
      }

      void
      handleHUDPacket(const mavlink_message_t* msg)
      {
        mavlink_vfr_hud_t vfr_hud;
        mavlink_msg_vfr_hud_decode(msg, &vfr_hud);

        m_hae_msl = vfr_hud.alt;
      }

      void
      handleSystemTimePacket(const mavlink_message_t* msg)
      {
        mavlink_system_time_t sys_time;
        mavlink_msg_system_time_decode(msg, &sys_time);

        time_t t = sys_time.time_unix_usec / 1000000;
        struct tm* utc;
        utc = gmtime(&t);

        m_fix.utc_time = utc->tm_hour * 3600 + utc->tm_min * 60 + utc->tm_sec + (sys_time.time_unix_usec % 1000000) * 1e-6;
        m_fix.utc_year = utc->tm_year + 1900;
        m_fix.utc_month = utc->tm_mon +1;
        m_fix.utc_day = utc->tm_mday;

        if (m_fix.utc_year>2014)
          m_fix.validity |= (IMC::GpsFix::GFV_VALID_TIME | IMC::GpsFix::GFV_VALID_DATE);
        dispatch(m_fix);
      }
    };
  }
}

DUNE_TASK
