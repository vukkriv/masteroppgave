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

namespace Plan
{
  namespace NetRecovery
  {
    using DUNE_NAMESPACES;

    struct FixedWingLoiter
    {
      uint8_t altitude;
      fp32_t speed;
      fp32_t radius;
      uint16_t duration;
    };

    struct Arguments
    {
      FixedWingLoiter fw_loiter;
    };

    struct Task: public DUNE::Tasks::Task
    {
      Arguments m_args;

      //! Last plan control state
      IMC::PlanControlState m_last_pcs;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx)
      {
        param("Loiter Altitude", m_args.fw_loiter.altitude)
        .visibility(Parameter::VISIBILITY_USER)
        .defaultValue("50");

        param("Loiter Speed", m_args.fw_loiter.speed)
        .visibility(Parameter::VISIBILITY_USER)
        .defaultValue("20");

        param("Loiter Radius", m_args.fw_loiter.radius)
        .visibility(Parameter::VISIBILITY_USER)
        .defaultValue("100");

        param("Loiter Duration", m_args.fw_loiter.duration)
        .visibility(Parameter::VISIBILITY_USER)
        .defaultValue("0");


        bind<IMC::PlanDB>(this);
        bind<IMC::PlanControl>(this);
        bind<IMC::PlanControlState>(this);
        bind<IMC::PlanSpecification>(this);
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
      consume(const IMC::PlanDB* msg)
      {
        trace("Got PlanDB:\n from '%s' at '%s'\n to '%s' at '%d'\n",
              resolveEntity(msg->getSourceEntity()).c_str(),
              resolveSystemId(msg->getSource()),
              resolveSystemId(msg->getDestination()),
              msg->getDestinationEntity());

        //msg->toText(std::cout);
        // Ignore if not reply for this entity
        /*
        if (msg->getDestination() != getSystemId()
            || msg->getDestinationEntity() != getEntityId())
        {
          trace("Ignoring PlanDB with other destination");
          return;
        }
        */
        /*
        if (msg->type == IMC::PlanDB::DBT_REQUEST)
          trace("Plan DB request\n");
        if (msg->type == IMC::PlanDB::DBT_SUCCESS)
        {
          if (msg->op == IMC::PlanDB::DBOP_GET_INFO)
            trace("Plan DB get info\n");
          if (msg->op == IMC::PlanDB::DBOP_SET)
            trace("Plan DB set");
          if (msg->op == IMC::PlanDB::DBOP_GET_STATE)
            trace("Plan DB get state\n");
        }
        */
        if (   msg->getDestination() == getSystemId()
            && msg->getSource()      != getSystemId()
            && msg->type == IMC::PlanDB::DBT_REQUEST
            && msg->op   == IMC::PlanDB::DBOP_SET)
        {
          debug(
              "Neptus setting Plan in local DB, checking if there exists a NetRecovery maneuver in the plan");
          // Check if plan has NetRecovery maneuver
          try
          {
            const IMC::PlanSpecification* planspec =
                static_cast<const IMC::PlanSpecification*>(msg->arg.get());
            IMC::MessageList<IMC::PlanManeuver>::const_iterator it =
                planspec->maneuvers.begin();
            IMC::NetRecovery nr_maneuver;
            trace("Plan: '%s'",planspec->plan_id.c_str());
            for (; it != planspec->maneuvers.end(); it++)
            {
              trace("Maneuver: '%s'",(*it)->data->getName());
              if ( (*it)->data->getId() == nr_maneuver.getId() )
              {
                debug("NetRecovery maneuver set request dispatched from Neptus");
                IMC::Maneuver* man = (*it)->data.get();
                IMC::NetRecovery* nr = static_cast<IMC::NetRecovery*>(man);
                sendFixedWingPlan(planspec->plan_id,nr);
                debug("Sent FixedWing NetRecovery plan");
                break;
              }
            }
          }
          catch (std::exception& e)
          {
            err(DTR(
                "Plan maneuver request failed with uncaught exception: %s"),
                e.what());
          }
        }
      }

      void
      consume(const IMC::PlanControl* msg)
      {
        trace("Got PlanControl \nfrom '%s' at '%s'",
              resolveEntity(msg->getSourceEntity()).c_str(),
              resolveSystemId(msg->getSource()));
        //msg->toText(std::cout);

        // Check if system was destination
        if (msg->getDestination() != this->getSystemId())
        {
          trace("Ignored PlanControl with wrong destination '%s'",
                resolveSystemId(msg->getDestination()));
          return;
        }

        // Ignore if not a request
        if (msg->type != IMC::PlanControl::PC_REQUEST)
        {
          trace("Ignored PlanControl (not a request)");
          return;
        }

        if (msg->op == IMC::PlanControl::PC_LOAD)
        {

        }
        else if (msg->op == IMC::PlanControl::PC_START)
        {
          debug("PlanControl START");

        }
        else if (msg->op == IMC::PlanControl::PC_STOP)
        {
          debug("PlanControl STOP");
        }
      }

      void
      consume(const IMC::PlanControlState* pcs)
      {
        spew("Got PlanControlState \nfrom '%s' at '%s'",
              resolveEntity(pcs->getSourceEntity()).c_str(),
              resolveSystemId(pcs->getSource()));
        //pcs->toText(std::cout);

        // Check if we just stopped executing a plan
        if (m_last_pcs.state == IMC::PlanControlState::PCS_EXECUTING
            && pcs->state == IMC::PlanControlState::PCS_READY)
        {
          debug("Plan stopped");
          // Check if success
          if (pcs->last_outcome == IMC::PlanControlState::LPO_SUCCESS)
          {
            debug("Plan success");
          }
        }

        m_last_pcs = *pcs;
      }

      void
      consume(const IMC::PlanSpecification* spec)
      {
        spew("Got PlanControlState \nfrom '%s' at '%s'",
              resolveEntity(spec->getSourceEntity()).c_str(),
              resolveSystemId(spec->getSource()));
        trace("PlanDB destination \n '%s' at '%d'",
              resolveSystemId(spec->getDestination()),
              spec->getDestinationEntity());
        if (spec->getSource() != getSystemId()
            && spec->getDestination() == getSystemId())
        {
          trace("PlanSpecification from remote to me");
        }
        else
        {
          return;
        }
      }
      void
      sendFixedWingPlan(std::string name, IMC::NetRecovery* maneuver)
      {
        // Create plan set request
        IMC::PlanDB plan_db;
        plan_db.type = IMC::PlanDB::DBT_REQUEST;
        plan_db.op = IMC::PlanDB::DBOP_SET;
        plan_db.plan_id = name + "_fixedwing";
        plan_db.request_id = 0;


        // Add loiter as initial maneuver

        IMC::PlanManeuver loiter_man;
        loiter_man.maneuver_id = 1;

        IMC::Loiter man_loit;
        man_loit.type = IMC::Loiter::LT_CIRCULAR;
        man_loit.direction = IMC::Loiter::LD_CLOCKW;

        man_loit.z             = m_args.fw_loiter.altitude;
        man_loit.z_units       = IMC::Z_ALTITUDE;
        man_loit.speed         = m_args.fw_loiter.speed;
        man_loit.speed_units   = IMC::SUNITS_METERS_PS;
        man_loit.duration      = m_args.fw_loiter.duration;
        man_loit.radius        = m_args.fw_loiter.radius;

        //Should calculate based on a desired along-track position behind the virtual runway
        man_loit.lat = maneuver->start_lat;
        man_loit.lon = maneuver->start_lon;


        IMC::PlanManeuver* pman1 = new IMC::PlanManeuver();
        IMC::InlineMessage<IMC::Maneuver> pman1_inline;
        pman1_inline.set(man_loit);
        pman1->maneuver_id = "1";
        pman1->data = pman1_inline;

        IMC::PlanManeuver* pman2  = new IMC::PlanManeuver();
        IMC::InlineMessage<IMC::Maneuver> pman2_inline;
        pman2_inline.set(maneuver);
        pman2->maneuver_id = "2";
        pman2->data = pman2_inline;

        IMC::PlanSpecification plan_spec;
        plan_spec.description = "A fixed wing recovery plan";
        plan_spec.plan_id = plan_db.plan_id;

        IMC::PlanTransition* transition = new IMC::PlanTransition;
        transition->source_man = "1";
        transition->dest_man   = "2";
        transition->conditions = "ManeuverIsDone";

        IMC::PlanTransition* transition2 = new IMC::PlanTransition;
        transition2->source_man = "2";
        transition2->dest_man   = "1";
        transition2->conditions = "ManeuverIsDone";

        IMC::MessageList<IMC::PlanTransition> translist;
        translist.push_back(transition);
        translist.push_back(transition2);

        IMC::MessageList<IMC::PlanManeuver> manlist;
        manlist.push_back(pman1);
        manlist.push_back(pman2);

        plan_spec.start_man_id = "1";
        plan_spec.maneuvers = manlist;
        plan_spec.transitions = translist;

        // Set plan
        plan_db.arg.set(plan_spec);
        // Send set plan request
        dispatch(plan_db);

        // Create and send plan start request
        /*
        IMC::PlanControl plan_ctrl;
        plan_ctrl.type = IMC::PlanControl::PC_REQUEST;
        plan_ctrl.op = IMC::PlanControl::PC_START;
        plan_ctrl.plan_id = plan_spec.plan_id;
        plan_ctrl.request_id = 0;
        plan_ctrl.arg.set(plan_spec);
        dispatch(plan_ctrl);
        */
      }
      //! Main loop.
      void
      onMain(void)
      {
        while (!stopping())
        {
          waitForMessages(1.0);
        }
      }
    };
  }
}

DUNE_TASK
