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

    struct Task: public DUNE::Tasks::Task
    {
      //! Last plan control state
      IMC::PlanControlState m_last_pcs;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx)
      {
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
        trace("Got PlanDB \nfrom '%s' at '%s'",
              resolveEntity(msg->getSourceEntity()).c_str(),
              resolveSystemId(msg->getSource()));
        //msg->toText(std::cout);
        trace("PlanDB destination \n '%s' at '%d'",
              resolveSystemId(msg->getDestination()),
              msg->getDestinationEntity());
        if (msg->getSource() != getSystemId()
            && msg->getDestination() == getSystemId())
        {
          trace("PlanDB from remote to me");
        }
        else
        {
          return;
        }
        if (msg->type == IMC::PlanDB::DBT_SUCCESS
            && msg->op == IMC::PlanDB::DBOP_GET)
        {
          debug(
              "Got plan from DB, checking if Formation Controller is activated...");
          // Check if plan has activated Formation Controller
          bool is_formation_control = false;
          try
          {
            const IMC::SetEntityParameters* t_sep;
            const IMC::PlanSpecification* planspec =
                static_cast<const IMC::PlanSpecification*>(msg->arg.get());
            IMC::MessageList<IMC::PlanManeuver>::const_iterator it =
                planspec->maneuvers.begin();
            for (; it != planspec->maneuvers.end(); it++)
            {
              IMC::MessageList<IMC::Message>::const_iterator it_sa =
                  (*it)->start_actions.begin();
              for (; it_sa != (*it)->start_actions.end(); it_sa++)
              {
                t_sep = static_cast<const IMC::SetEntityParameters*>(*it_sa);
                if (t_sep->name.compare("Formation Controller") == 0)
                {
                  IMC::MessageList<IMC::EntityParameter>::const_iterator it_ep =
                      t_sep->params.begin();
                  for (; it_ep != t_sep->params.end(); it_ep++)
                  {
                    if ((*it_ep)->name.compare("Formation Controller") == 0
                        && (*it_ep)->value.compare("true") == 0)
                    {
                      is_formation_control = true;
                      break;
                    }
                  }
                }
                if (is_formation_control)
                  break;
              }
              if (is_formation_control)
                break;
            }
          }
          catch (std::exception& e)
          {
            err(DTR(
                "Plan specification request failed with uncaught exception: %s"),
                e.what());
          }

          //extract NetRecovery


          //sendFixedWingPlan();
          debug("Sent FixedWing NetRecovery plan");
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

        if (msg->op == IMC::PlanControl::PC_START)
        {
          debug("PlanControl START, requesting plan from DB...");
          // Request plan details from Plan Database to check if Formation Control is activated
          IMC::PlanDB plan_db;
          plan_db.type = IMC::PlanDB::DBT_REQUEST;
          plan_db.op = IMC::PlanDB::DBOP_GET;
          plan_db.plan_id = msg->plan_id;
          dispatch(plan_db);
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
      sendFixedWingPlan()//IMC::NetRecovery nr_maneuver)
      {
        // Create plan set request
        IMC::PlanDB plan_db;
        plan_db.type = IMC::PlanDB::DBT_REQUEST;
        plan_db.op = IMC::PlanDB::DBOP_SET;
        plan_db.plan_id = "formationPlan";
        plan_db.request_id = 0;

        // Create plan specification
        IMC::PlanSpecification plan_spec;
        plan_spec.plan_id = plan_db.plan_id;
        plan_spec.start_man_id = 1;
        plan_spec.description = "NetRecovery FixedWing plan";

        // Create plan maneuver
        IMC::PlanManeuver man_spec;
        man_spec.maneuver_id = 1;

        IMC::NetRecovery m_dummy;
        man_spec.data.set(m_dummy);

        // Create start actions
        IMC::SetEntityParameters eparam_start;
        eparam_start.name = "Speed";
        IMC::EntityParameter param_t;
        param_t.name = "Speed";
        param_t.value = 12.0;

        eparam_start.params.push_back(param_t);

        man_spec.start_actions.push_back(eparam_start);

        // Create end actions
        IMC::SetEntityParameters eparam_stop;
        eparam_start.name = "Speed";
        IMC::EntityParameter param_f;
        param_f.name = "Speed";
        param_f.value = 12.0;

        eparam_start.params.push_back(param_f);

        man_spec.end_actions.push_back(eparam_stop);

        plan_spec.maneuvers.push_back(man_spec);

        plan_db.arg.set(plan_spec);

        // Send set plan request
        dispatch(plan_db);

        // Create and send plan start request
        IMC::PlanControl plan_ctrl;
        plan_ctrl.type = IMC::PlanControl::PC_REQUEST;
        plan_ctrl.op = IMC::PlanControl::PC_START;
        plan_ctrl.plan_id = plan_spec.plan_id;
        plan_ctrl.request_id = 0;
        plan_ctrl.arg.set(plan_spec);
        dispatch(plan_ctrl);
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
