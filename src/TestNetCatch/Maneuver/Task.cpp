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
    	double 	m_start;
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
    	std::string m_copter_id;
    	std::string m_aircraft_id;
    };

    struct Task: public DUNE::Tasks::Periodic
    {
      //! Task arguments.
      Arguments m_args;
      bool maneuver_enabled;
      double m_timestamp_start;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Periodic(name, ctx),
        maneuver_enabled(false),
        m_timestamp_start(0.0)
      {
          param("Start time", m_args.m_start)
          .defaultValue("0.0");

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

          param("Copter", m_args.m_copter_id);
          param("Aircraft", m_args.m_aircraft_id);
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
    	  msg.aircraft	  = m_args.m_aircraft_id;
    	  msg.multicopters= m_args.m_copter_id;

    	  IMC::Goto goto_man;
    	  goto_man.lat = msg.start_lat;
    	  goto_man.lon = msg.start_lon;
    	  goto_man.z   = msg.z;
    	  goto_man.z_units = msg.z_units;
    	  goto_man.speed = msg.speed;
    	  goto_man.speed_units = IMC::SUNITS_METERS_PS;

    	  IMC::PlanManeuver* pman1 = new IMC::PlanManeuver();
    	  IMC::InlineMessage<IMC::Maneuver> pman1_inline;
    	  pman1_inline.set(goto_man);
    	  pman1->maneuver_id = "1";
    	  pman1->data = pman1_inline;

    	  IMC::PlanManeuver* pman2  = new IMC::PlanManeuver();
    	  IMC::InlineMessage<IMC::Maneuver> pman2_inline;
    	  pman2_inline.set(msg);
    	  pman2->maneuver_id = "2";
    	  pman2->data = pman2_inline;


        IMC::PlanSpecification plan;

        IMC::PlanTransition* transition = new IMC::PlanTransition;
        transition->source_man = "1";
        transition->dest_man   = "2";

        IMC::PlanTransition* transition2 = new IMC::PlanTransition;
        transition2->source_man = "2";
        transition2->dest_man   = "1";

        plan.description = "A net recovery test plan";
        plan.plan_id = "NetRecoveryTest";

        IMC::MessageList<IMC::PlanTransition> translist;
        translist.push_back(transition);
        translist.push_back(transition2);

        IMC::MessageList<IMC::PlanManeuver> manlist;
        manlist.push_back(pman1);
        manlist.push_back(pman2);

        plan.start_man_id = "1";
        plan.maneuvers = manlist;
        plan.transitions = translist;

        // Create plan
        IMC::PlanControl planCtrl;
        planCtrl.op = IMC::PlanControl::PC_START;
        planCtrl.type = IMC::PlanControl::PC_REQUEST;
        planCtrl.plan_id = "NetRecoveryTest";
        planCtrl.request_id = 10;

        // Apply custom maneuver to plan
        IMC::InlineMessage<IMC::Message> arg;
        arg.set(plan);
        planCtrl.arg = arg;
        debug("Custom plan activated");
        dispatch(planCtrl);

    	  //dispatch(msg);
    	  maneuver_enabled = true;
      }


      void
      task(void)
      {
    	  if (m_timestamp_start == 0.0)
    	  {
    		  m_timestamp_start = Clock::get();
    	  }
    	  double timeSinceStart =  Clock::get() - m_timestamp_start;
    	  if (!maneuver_enabled)

    		  war("Countdown (net recovery maneuver): %d", static_cast<int>(m_args.m_start - timeSinceStart + 0.5));

    	  if (timeSinceStart > m_args.m_start)
    	  {
			  if (!maneuver_enabled)
				  sendManeuver();
    	  }
      }
    };
  }
}

DUNE_TASK
