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
// Author: Jostein B. Moe                                                   *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace TestNetCatch
{
  namespace Maneuver
  {
    using DUNE_NAMESPACES;

    //! %Task arguments.
    struct Arguments
    {
    	uint16_t 	m_timeout;
    	fp64_t 		m_slat;
    	fp64_t 		m_slon;
    	fp64_t 		m_elat;
    	fp64_t 		m_elon;
    	fp32_t 		m_zref;
    	uint8_t 	m_zunits;
    	fp32_t 		m_lbox_height;
    	fp32_t 		m_lbox_width;
    	fp32_t 		m_speed;
    	fp32_t		m_macc;
    	fp32_t		m_zoff;
    };

    struct Task: public DUNE::Tasks::Task
    {
      //! Task arguments.
      Arguments m_args;
      bool maneuver_enabled;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        maneuver_enabled(false)
      {
          param("Timeout", m_args.m_timeout)
          .defaultValue("0.0");

          param("StartLat", m_args.m_slat)
          .defaultValue("0.0");

          param("StartLon", m_args.m_slon)
          .defaultValue("0.0");

          param("EndLat", m_args.m_elat)
          .defaultValue("0.0");

          param("EndLon", m_args.m_elon)
          .defaultValue("0.0");

          param("Z Reference", m_args.m_zref)
          .defaultValue("0.0");

          param("Z Units", m_args.m_zunits)
          .defaultValue("0.0");

          param("Landing Box Width", m_args.m_lbox_width)
          .defaultValue("0.0");

          param("Landing Box Height", m_args.m_lbox_height)
          .defaultValue("0.0");

          param("Speed", m_args.m_speed)
          .defaultValue("0.0");

          param("Max Acceleration", m_args.m_macc)
          .defaultValue("0.0");

          param("Altitude Offset", m_args.m_zoff)
          .defaultValue("0.0");

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
      }

      void
      sendManeuver(void)
      {
    	  IMC::NetRecovery msg;

    	  msg.timeout 	= m_args.m_timeout;
    	  msg.start_lat = Angles::radians(m_args.m_slat);
    	  msg.start_lon = Angles::radians(m_args.m_slon);
    	  msg.end_lon 	= Angles::radians(m_args.m_elon);
    	  msg.end_lat 	= Angles::radians(m_args.m_elat);
    	  msg.z			= m_args.m_zref;
    	  msg.z_units	= m_args.m_zunits;
    	  msg.lbox_width  = m_args.m_lbox_width;
    	  msg.lbox_height = m_args.m_lbox_height;
    	  msg.speed 	  = m_args.m_speed;
    	  msg.max_acc	  = m_args.m_macc;
    	  msg.z_off		  = m_args.m_zoff;

    	  dispatch(msg);
    	  inf("NetRecovery maneuver message dispatched");
    	  maneuver_enabled = true;
      }


      //! Main loop.
      void
      onMain(void)
      {
        while (!stopping())
        {
          waitForMessages(1.0);
          if (!maneuver_enabled)
        	  sendManeuver();
        }
      }
    };
  }
}

DUNE_TASK
