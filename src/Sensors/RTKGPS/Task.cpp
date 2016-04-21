//***************************************************************************
// Copyright 2007-2015 Universidade do Porto - Faculdade de Engenharia      *
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
// Author: Ricardo Martins                                                  *
//***************************************************************************

// ISO C++ 98 headers.
#include <cstring>
#include <algorithm>
#include <cstddef>

// DUNE headers.
#include <DUNE/DUNE.hpp>

// Local headers.
#include "Reader.hpp"

namespace Sensors
{
  //! Device driver for RTKLIB.
  namespace RTKGPS
  {
    using DUNE_NAMESPACES;

    struct Arguments
    {
      //! Serial port device.
      std::string uart_dev;
      //! Serial port baud rate.
      unsigned uart_baud;
      //! Input timeout in seconds.
      float inp_tout;
      //! Solution Format.
      std::string sol_format;
      bool ned_velocity_available;
      //! Nest identification
      std::string base_system_name;
      //! Nest position timeout
      float rtk_base_position_timeout;
      //! Max number of single solution from rtklib that should be ignored
      int max_number_of_ignored_rtklib_single_solution;

    };

    struct Task: public Tasks::Task
    {
      //! Serial port handle.
      IO::Handle* m_handle;
      //! GPS Fix message.
      IMC::GpsFix m_fix;
      //! RTK Fix message.
      IMC::GpsFixRtk m_rtkfix;
      //! GPS Uncertainty
      IMC::NavigationUncertainty m_gps_uncertainty;
      //! Task arguments.
      Arguments m_args;
      //! Input watchdog.
      Time::Counter<float> m_wdog;
      //! True if we have angular velocity.
      bool m_llh_output;
      //! True if we have euler angles.
      bool m_ned_output;
      //! Last initialization line read.
      std::string m_init_line;
      //! Reader thread.
      Reader* m_reader;
      //! ID of system to accept base location from
      unsigned m_base_sys_id;
      //! Input watchdog for nest position
      Time::Counter<float> m_rtk_wdog_base_available;
      //! Number of single rtklib solution after a fixed solution
      int m_single_solution_counter;

      Task(const std::string& name, Tasks::Context& ctx):
        Tasks::Task(name, ctx),
        m_handle(NULL),
        m_llh_output(false),
        m_ned_output(false),
        m_reader(NULL),
        m_base_sys_id(0)
      {
        // Define configuration parameters.
        param("Serial Port - Device", m_args.uart_dev)
        .defaultValue("")
        .description("Serial port device used to communicate with the sensor");

        param("Serial Port - Baud Rate", m_args.uart_baud)
        .defaultValue("4800")
        .description("Serial port baud rate");

        param("Input Timeout", m_args.inp_tout)
        .units(Units::Second)
        .defaultValue("4.0")
        .minimumValue("0.0")
        .description("Input timeout");

        param("Output Solution Format", m_args.sol_format)
        .defaultValue("llh")
        .description("Output Solution Format - llh or ned");

        param("Rtk Velocity Available", m_args.ned_velocity_available)
        .defaultValue("False")
        .description("True if an edited version of RTKlib is used, which also outputs velocity information. ");
        
        param("Base System Name", m_args.base_system_name)
        .defaultValue("ntnu-nest-02")
        .description("Base system name");

        param("RTK Base Position Timeout", m_args.rtk_base_position_timeout)
        .defaultValue("10");

        param("Max Single Ignored", m_args.max_number_of_ignored_rtklib_single_solution)
        .defaultValue("5")
        .description("The maximum number of single solution from rtklib that should not be dispatched");

        // Initialize messages.
        clearMessages();

        //! Initialize counter
        m_single_solution_counter = 0;

        bind<IMC::DevDataText>(this);
        bind<IMC::IoEvent>(this);
        bind<IMC::GpsFixRtk>(this);
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
      void
      onResourceAcquisition(void)
      {

        try
        {
          if (!openSocket())
            m_handle = new SerialPort(m_args.uart_dev, m_args.uart_baud);

          m_reader = new Reader(this, m_handle);
          m_reader->start();
        }
        catch (...)
        {
          err("Restart due to Resource Acquisition.");
          throw RestartNeeded(DTR(Status::getString(CODE_COM_ERROR)), 5);
        }
      }

      void
      onParametersUpdate(void)
      {
        if (paramChanged(m_args.base_system_name))
        {
          m_base_sys_id = resolveSystemName(m_args.base_system_name);

          if (m_base_sys_id == IMC::AddressResolver::invalid())
          {
            war("Unable to resolve system %s", m_args.base_system_name.c_str());
          }
        }

      }


      bool
      openSocket(void)
      {
        char addr[128] = {0};
        unsigned port = 0;

        if (std::sscanf(m_args.uart_dev.c_str(), "tcp://%[^:]:%u", addr, &port) != 2)
          return false;

        TCPSocket* sock = new TCPSocket;
        sock->connect(addr, port);
        m_handle = sock;
        return true;
      }

      void
      onResourceRelease(void)
      {
        if (m_reader != NULL)
        {
          m_reader->stopAndJoin();
          delete m_reader;
          m_reader = NULL;
        }

        Memory::clear(m_handle);
      }

      void
      onResourceInitialization(void)
      {
    	  std::cout << "SOLUTION FORMAT: " << m_args.sol_format;
    	  if(m_args.sol_format == "llh"){
    		  m_llh_output = true;
    	  }
    	  else if(m_args.sol_format == "ned")
    	  {
    		  m_ned_output = true;
    	  }
    	  else
    	  {
    		  err("No Solution Format Set");
    	  }


        setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
        m_wdog.setTop(m_args.inp_tout);
        m_rtk_wdog_base_available.setTop(m_args.rtk_base_position_timeout);

        m_base_sys_id = resolveSystemName(m_args.base_system_name);
      }

      void
      consume(const IMC::DevDataText* msg)
      {
        if (msg->getDestination() != getSystemId())
          return;

        if (msg->getDestinationEntity() != getEntityId())
          return;

        spew("%s", sanitize(msg->value).c_str());

        if (getEntityState() == IMC::EntityState::ESTA_BOOT)
          m_init_line = msg->value;
        else
          processSentence(msg->value);
      }

      void
      consume(const IMC::GpsFixRtk* msg)
      {


        if (msg->getSource() == m_base_sys_id
            && (msg->validity & IMC::GpsFixRtk::RFV_VALID_BASE))
        {
            spew("Received GpsFixRtk from %s",resolveSystemId(msg->getSource()));
            m_rtkfix.base_lat = msg->base_lat;
            m_rtkfix.base_lon = msg->base_lon;
            m_rtkfix.base_height = msg->base_height;
            m_rtkfix.validity |= IMC::GpsFixRtk::RFV_VALID_BASE;
            m_rtk_wdog_base_available.reset();
        }
        else
        {
          spew("Ignored GpsRtkFix from %s, validty: %d", resolveSystemId(msg->getSource()), msg->validity);
          spew("Got from: %u, expected: %u", msg->getSource(), m_base_sys_id);
        }
      }
      void
      consume(const IMC::IoEvent* msg)
      {
        if (msg->getDestination() != getSystemId())
          return;

        if (msg->getDestinationEntity() != getEntityId())
          return;

        if (msg->type == IMC::IoEvent::IOV_TYPE_INPUT_ERROR)
        {
          err("Restart due to IOEvent Input Error");
        	throw RestartNeeded(msg->error, 5);
        }
      }

      void
      clearMessages(void)
      {
	    m_rtkfix.clear();
        m_fix.clear();
      }

      //! Read decimal from input string.
      //! @param[in] str input string.
      //! @param[out] dst decimal.
      //! @return true if successful, false otherwise.
      template <typename T>
      bool
      readDecimal(const std::string& str, T& dst)
      {
        unsigned idx = 0;
        while (str[idx] == '0')
          ++idx;

        return castLexical(std::string(str, idx), dst);
      }

      //! Read number from input string.
      //! @param[in] str input string.
      //! @param[out] dst number.
      //! @return true if successful, false otherwise.
      template <typename T>
      bool
      readNumber(const std::string& str, T& dst)
      {
        return castLexical(str, dst);
      }

      //! Process sentence.
      //! @param[in] line line.
      void
      processSentence(const std::string& line)
      {
    	 // war("Processing Sentence");
    	 // int size = line.size();
    	 // printf("Line Length:%i\n",size);
    	 // std::cout << line;
    	 // std::cout << "\n";

    	  // Split sentence
		  std::vector<std::string> parts;
		  String::split(line, " ", parts);
		  /*
		   * parts[0]	GPS week wwww
		   * parts[1]	GPS TOW sssssss.sss
		   * 		      xyz		      llh           enu
		   * parts[2]	x-ecef(m)	 |	lat(deg)	|	e(m)
		   * parts[3]	y-ecef(m)	 |	lon(deg)	|	n(m)
		   * parts[4]	z-ecef(m)	 |	height(m)	|	u(m)
		   * parts[5]	Q=1:fix,2:float,3:sbas,4:dgps,5:single,6:ppp
		   * parts[6]	Number of Satelittes
		   * parts[7]	sdx(m)
		   * parts[8]	sdy(m)
		   * parts[9]	sdz(m)
		   * parts[10]	sdxy(m)
		   * parts[11]	sdyz(m)
		   * parts[12]	sdzx(m)
		   * parts[13]	Age of Differential
		   * parts[14]	Ambiguity Ratio
		   * if enu is the output format from rtklib
		   * parts[15] v_e (m/s)
		   * parts[16] v_n (m/s)
		   * parts[17] v_u (m/s)
		   */

		  // Ignore Init Sentence from RTKLIB
		  if(parts[0] == "%")
		  {
		    // std::cout << "Initial Sentence Ignored \n";
		    return;
		  }
		  // Remove Empty Cells in parts vector
		  size_t i = 0;
		  for(i = 0; i < parts.size(); i++)
		  {
		    if(parts[i].empty() )
		    {
		      parts.erase(parts.begin()+i);
		      i = i-1;
		    }
		  }

		  if(m_llh_output && parts.size()>=15)
		  {
		    setGpsFix(parts);
		  }
		  else if(m_ned_output && parts.size()>=18)
		  {
		    setRtkFix(parts);
		  }
		  else
		  {
		    err("No Solution Format Set");
		  }
		  m_wdog.reset();

      }
      void setGpsUncertainty(std::vector<std::string>& parts)
      {
        // Set gps position standard deviation
        if (readDecimal(parts[8],m_gps_uncertainty.x)
            && readDecimal(parts[7], m_gps_uncertainty.y)
            && readDecimal(parts[9], m_gps_uncertainty.z))
        {
          // Calculate position variance
          m_gps_uncertainty.x = m_gps_uncertainty.x*m_gps_uncertainty.x;
          m_gps_uncertainty.y = m_gps_uncertainty.y*m_gps_uncertainty.y;
          m_gps_uncertainty.z = m_gps_uncertainty.z*m_gps_uncertainty.z;
        }
        dispatch(m_gps_uncertainty);
      }
      void setGpsFix(std::vector<std::string>& parts)
      {

        //Set Type of Fix
        int Q;
        if(readNumber(parts[5], Q) )
        {
          //RTK Fix and Float flags not implemented in GpsFix

          if(Q == 1)
          {
            m_fix.type = IMC::GpsFix::GFT_DIFFERENTIAL; // Differential = FIX
          }
          else if(Q == 2)
          {
            m_fix.type = IMC::GpsFix::GFT_DEAD_RECKONING; // Dead-reckoning = FLOAT
          }
          else if(Q == 5)
          {
            m_fix.type = IMC::GpsFix::GFT_STANDALONE; // Single mode
          }
        }

        //Set UTC Year Month Day
        std::vector<std::string> UTC_YMD;
        String::split(parts[0], "/", UTC_YMD);

        if(readNumber(UTC_YMD[0],m_fix.utc_year)
            && readDecimal(UTC_YMD[1],m_fix.utc_month)
            && readDecimal(UTC_YMD[2],m_fix.utc_day) )
        {
          //inf("%d / %d / %d", m_fix.utc_year, m_fix.utc_month, m_fix.utc_day);
          m_fix.validity |= IMC::GpsFix::GFV_VALID_DATE;

        }

        //Set UTC time
        std::vector<std::string> UTC_Time;
        String::split(parts[1], ":", UTC_Time);
        fp32_t h;
        fp32_t m;
        fp32_t s;

        // [HH|MM|SS.FFF] UTC_Time

        if(UTC_Time.size()>=3 && readDecimal(UTC_Time[0],h)
            && readDecimal(UTC_Time[1],m)
            && readDecimal(UTC_Time[2],s))
        {

          m_fix.utc_time = (3600*h) + (60*m)+ s;
          m_fix.validity |= IMC::GpsFix::GFV_VALID_TIME;
        }

        // Set Longitude,Latitude and Height and set number of satellites
        if (readDecimal(parts[2], m_fix.lat)
            && readDecimal(parts[3], m_fix.lon)
            && readDecimal(parts[4], m_fix.height)
            && readDecimal(parts[6],m_fix.satellites))
        {
          // Convert coordinates to radians.
          m_fix.lat = Angles::radians(m_fix.lat);
          m_fix.lon = Angles::radians(m_fix.lon);
          m_fix.validity |= IMC::GpsFix::GFV_VALID_POS;
        }

        // Set gps uncertainty
        setGpsUncertainty(parts);

        dispatch(m_fix);
        trace("GpsFix Message Dispatched!");
      }

      void setRtkFix(std::vector<std::string>& parts)
      {
        //Set Type of Fix
        int Q;
        if(readNumber(parts[5], Q) )
        {
          if(Q == 1) // FIX
          {
            m_single_solution_counter = 0;
            m_rtkfix.type = IMC::GpsFixRtk::RTK_FIXED;
          }
          else if(Q == 2) // FLOAT
          {
            m_rtkfix.type = IMC::GpsFixRtk::RTK_FLOAT;
          }
          else
          {
            //! When the previous solution was a fixed check if the new solution is only a temporary single solution
            if (m_rtkfix.type == IMC::GpsFixRtk::RTK_FIXED && m_single_solution_counter < m_args.max_number_of_ignored_rtklib_single_solution)
            {
              m_single_solution_counter++;
              return;
            }
            m_rtkfix.type = IMC::GpsFixRtk::RTK_NONE;
          }

        }
        // Set North,East and Down and set number of satellites
        if (readDecimal(parts[3], m_rtkfix.n)
            && readDecimal(parts[2], m_rtkfix.e)
            && readDecimal(parts[4], m_rtkfix.d)
            && readDecimal(parts[6], m_rtkfix.satellites)
            && readDecimal(parts[14], m_rtkfix.iar_ratio))
        {
          // Invert Z axis
          m_rtkfix.d = -m_rtkfix.d;

          // Set validity flags
          m_rtkfix.validity |= IMC::GpsFixRtk::RFV_VALID_POS;
        }
        else
        {
          // Disable validity flags
          m_rtkfix.validity &= ~IMC::GpsFixRtk::RFV_VALID_POS;
        }
        // Set North,East and Down velocity
        if (m_args.ned_velocity_available
            && readDecimal(parts[16], m_rtkfix.v_n)
            && readDecimal(parts[15], m_rtkfix.v_e)
            && readDecimal(parts[17], m_rtkfix.v_d))
        {
          //Invert Z axis
          m_rtkfix.v_d = -m_rtkfix.v_d;

          // Set validity flags
          m_rtkfix.validity |= IMC::GpsFixRtk::RFV_VALID_VEL;
        }
        else
        {
          // Disable validity flags
          m_rtkfix.validity &= ~IMC::GpsFixRtk::RFV_VALID_VEL;
        }
        // Set time of week (TOW)
        readDecimal(parts[1],m_rtkfix.tow);

        // Sat validity flags
        m_rtkfix.validity |= IMC::GpsFixRtk::RFV_VALID_TIME;

        // Set gps uncertainty
        setGpsUncertainty(parts);

        dispatch(m_rtkfix);
        trace("RtkFix Message Dispatched!");
      }
       
      void
      onMain(void)
      {
        while (!stopping())
        {
          waitForMessages(1.0);

          if (m_wdog.overflow())
          {
            setEntityState(IMC::EntityState::ESTA_ERROR, Status::CODE_COM_ERROR);
            err("Restart due to watchdog overflow");
            throw RestartNeeded(DTR(Status::getString(CODE_COM_ERROR)), 5);
          }
          if (m_rtk_wdog_base_available.overflow() && (m_rtkfix.validity & IMC::GpsFixRtk::RFV_VALID_BASE))
          {
            m_rtkfix.validity &= ~IMC::GpsFixRtk::RFV_VALID_BASE;
            debug("Base Position Unavailable. Timeout");
          }
        }
      }
    };
  }
}

DUNE_TASK
