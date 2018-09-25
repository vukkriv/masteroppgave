//***************************************************************************
// Copyright 2007-2018 Norwegian University of Science and Technology (NTNU)*
// Centre for Autonomous Marine Operations and Systems (AMOS)               *
// Department of Engineering Cybernetics (ITK)                              *
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
// Author: RecepCetin                                                       *
//***************************************************************************
#include <vector>
#include <cmath>
#include <queue>
// DUNE headers.
#include <DUNE/DUNE.hpp>

// MAVLink headers.
#include <mavlink/ardupilotmega/mavlink.h>


namespace Sensors
{
namespace Vicon
{
using DUNE_NAMESPACES;



//! %Task arguments.
struct Arguments
{
	//! Communications timeout
	uint8_t comm_timeout;
	//! TCP Port
	uint16_t TCP_port;
	//! TCP Address
	Address TCP_addr;
	//! Telemetry Rate
	uint8_t trate;
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
	//! TCP socket
	Network::TCPSocket* m_TCP_sock;
	//! System ID
	uint8_t m_sysid;
	//! External control
	bool m_external;
	//! Parser Variables
	mavlink_message_t m_msg;
	bool m_error_missing, m_esta_ext;

	Task(const std::string& name, Tasks::Context& ctx):
		Tasks::Task(name, ctx),
		m_TCP_sock(NULL),
		m_sysid(1),
		m_external(true),
		m_error_missing(false),
		m_esta_ext(false)
	{

    param("TCP - Port", m_args.TCP_port)
    .defaultValue("5760")
    .description("Port for connection to Ardupilot");

    param("TCP - Address", m_args.TCP_addr)
    .defaultValue("127.0.0.1")
    .description("Address for connection to Ardupilot");


		// Setup packet handlers
		// IMPORTANT: set up function to handle each type of MAVLINK packet here
		m_mlh[MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE] = &Task::handleViconPacket;

		//! Misc. initialization
		m_last_pkt_time = 0; //! time of last packet from Ardupilot
	}

	void
	onResourceRelease(void)
	{
		Memory::clear(m_TCP_sock);
	}

	void
	onResourceAcquisition(void)
	{
		openConnection();
	}

	void
	onUpdateParameters(void)
	{

	}

	void
	openConnection(void)
	{
		try
		{
			m_TCP_sock = new TCPSocket;
			m_TCP_sock->connect(m_args.TCP_addr, m_args.TCP_port);
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

			if (!m_error_missing)
			{
				if (m_external)
				{
					if (!m_esta_ext)
					{
						setEntityState(IMC::EntityState::ESTA_NORMAL, "External Control");
						m_esta_ext = true;
					}
				}
				else// if (getEntityState() != IMC::EntityState::ESTA_NORMAL)
				{
					setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
					m_esta_ext = false;
				}
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
						debug("UNDEF: %u", m_msg.msgid);
						break;
					case 22:
						trace("PARAM_VALUE");
						break;
					case 104:
						trace("Vicon position estimate");
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
				m_esta_ext = false;
			}
		}
		else
			m_error_missing = false;
	}


	void
	handleViconPacket(const mavlink_message_t* msg)
	{
		mavlink_vicon_position_estimate_t viconPos;
		mavlink_msg_vicon_position_estimate_decode(msg, &viconPos);

		double tstamp = Clock::getSinceEpoch();

		IMC::ViconPositionEstimates est_pos;
		est_pos.x = viconPos.x;
		est_pos.y = viconPos.y;
		est_pos.z = viconPos.z;
		est_pos.phi = viconPos.roll;
		est_pos.theta = viconPos.pitch;
		est_pos.psi = viconPos.yaw;
		est_pos.setTimeStamp(tstamp);
		dispatch(est_pos);

	}


};
}
}


DUNE_TASK
