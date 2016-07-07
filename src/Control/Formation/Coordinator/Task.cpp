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
// Author: Jon-Håkon Bøe Røli                                               *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Control
{
  namespace Formation
  {
    namespace Coordinator
    {
      using DUNE_NAMESPACES;
      struct Task : public DUNE::Tasks::Task
      {
        //! Formation coordination message
        IMC::FormCoord m_form_coord;

        //! Last plan control state
        IMC::PlanControlState m_last_pcs;

        //! Last estimated state
        IMC::EstimatedState m_last_estate;

        //! Constructor.
        //! @param[in] name task name.
        //! @param[in] ctx context.
        Task(const std::string& name, Tasks::Context& ctx) :
            DUNE::Tasks::Task(name, ctx)
        {
          bind<IMC::PlanControl>(this);
          bind<IMC::PlanDB>(this);
          bind<IMC::Abort>(this);
          bind<IMC::PlanControlState>(this);
          bind<IMC::FormCoord>(this);
          bind<IMC::EstimatedState>(this);
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
          setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
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

          // Ignore if not reply for this entity
          if (msg->getDestination() != getSystemId()
              || msg->getDestinationEntity() != getEntityId())
          {
            trace("Ignoring PlanDB with other destination");
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

            if (!is_formation_control)
            {
              debug("Plan did not activate Formation Controller");
              return;
            }

            debug("Plan started with Formation Controller active!");
            // Notify vehicles in formation
            m_form_coord.type = IMC::FormCoord::FCT_REQUEST;
            m_form_coord.op = IMC::FormCoord::FCOP_START;
            m_form_coord.lat = m_last_estate.lat;
            m_form_coord.lon = m_last_estate.lon;
            m_form_coord.height = m_last_estate.height;
            dispatch(m_form_coord);
            debug("Sent Formation Start");
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
            // Notify vehicles in formation
            m_form_coord.type = IMC::FormCoord::FCT_REQUEST;
            m_form_coord.op = IMC::FormCoord::FCOP_STOP;
            dispatch(m_form_coord);
            debug("Sent Formation Stop");
          }
        }

        void
        consume(const IMC::Abort* msg)
        {
          // Ignore if this vehichle not the destination (inline with handling in Vehicles.Supervisor)
          if (msg->getDestination() != getSystemId())
            return;
          war("Received Abort!");
          // Notify vehicles in formation
          m_form_coord.type = IMC::FormCoord::FCT_REQUEST;
          m_form_coord.op = IMC::FormCoord::FCOP_ABORT;
          dispatch(m_form_coord);
          debug("Sent Formation Abort");
        }

        void
        consume(const IMC::PlanControlState* pcs)
        {
          trace("Got PlanControlState \nfrom '%s' at '%s'",
                resolveEntity(pcs->getSourceEntity()).c_str(),
                resolveSystemId(pcs->getSource()));
          //pcs->toText(std::cout);

          // Check if we just stopped executing a plan
          spew("Last state: %d. New state: %d. Outcome: %d. ", m_last_pcs.state, pcs->state, pcs->last_outcome);
          if (m_last_pcs.state == IMC::PlanControlState::PCS_EXECUTING
              && pcs->state == IMC::PlanControlState::PCS_READY)
          {
            debug("Plan stopped");
            // Check if success
            if (pcs->last_outcome == IMC::PlanControlState::LPO_SUCCESS
                || pcs->last_outcome == IMC::PlanControlState::LPO_FAILURE)
            {
              debug("Plan success");
              // Notify vehicles in formation
              m_form_coord.type = IMC::FormCoord::FCT_REQUEST;
              m_form_coord.op = IMC::FormCoord::FCOP_FINISHED;
              dispatch(m_form_coord);
              debug("Sent Formation Finished");
            }
          }

          m_last_pcs = *pcs;
        }

        void
        consume(const IMC::EstimatedState* estate)
        {
          spew("Saved EstimatedState \nfrom '%s' at '%s'",
               resolveEntity(estate->getSourceEntity()).c_str(),
               resolveSystemId(estate->getSource()));
          // Save to send ref LLH if starting formation
          m_last_estate = *estate;
        }

        void
        consume(const IMC::FormCoord* msg)
        {
          debug("Got FormCoord \nfrom '%s' at '%s'",
                resolveEntity(msg->getSourceEntity()).c_str(),
                resolveSystemId(msg->getSource()));
          // Ignored if sent by self
          if (msg->getSource() == getSystemId())
            return;

          if (msg->type == IMC::FormCoord::FCT_REQUEST)
          {
            debug("Received Formation Request");

            // Handle request
            switch (msg->op)
            {
              case IMC::FormCoord::FCOP_ABORT:
                {
                  war("Formation Abort!");
                  // Send abort
                  IMC::Abort plan_abort;
                  // Have to set destination, as Vehicle.Supervisor ignores if this destination is not this vehicle
                  plan_abort.setDestination(this->getSystemId());
                  dispatch(plan_abort);

                  break;
                }
              case IMC::FormCoord::FCOP_START:
                {
                  debug("Formation Start");
                  // Make, send and start formation plan
                  startFormationPlan();

                  break;
                }
              case IMC::FormCoord::FCOP_STOP:
                {
                  debug("Received Formation Stop");
                  // Send request to stop plan
                  IMC::PlanControl plan_ctrl;
                  plan_ctrl.type = IMC::PlanControl::PC_REQUEST;
                  plan_ctrl.op = IMC::PlanControl::PC_STOP;
                  dispatch(plan_ctrl);

                  break;
                }
              case IMC::FormCoord::FCOP_FINISHED:
                {
                  debug("Received Formation Finished");
                  // TODO: Notify plan success instead of just stopping (failure)
                  // Send request to stop plan
                  IMC::PlanControl plan_ctrl;
                  plan_ctrl.type = IMC::PlanControl::PC_REQUEST;
                  plan_ctrl.op = IMC::PlanControl::PC_STOP;
                  dispatch(plan_ctrl);
                  break;
                }
            }
          } // End handle request
          else
          {
            debug("Received Formation Reply");
            // TODO: Handle replies
          }
        }

        void
        startFormationPlan(void)
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
          plan_spec.description = "Plan activating FormationController";

          // Create plan maneuver
          IMC::PlanManeuver man_spec;
          man_spec.maneuver_id = 1;
          /*
           // Create custom maneuver (not supported?!)
           IMC::CustomManeuver c_man;
           c_man.name = "formationManeuver";
           */

          // Create some maneuver
          //IMC::Goto c_man;

          // Create a slave maneuver
          IMC::SlaveManeuver c_man;

          man_spec.data.set(c_man);

          // Create start actions
          IMC::SetEntityParameters eparam_start;
          eparam_start.name = "Formation Controller";
          IMC::EntityParameter param_t;
          param_t.name = "Formation Controller";
          param_t.value = "true";

          eparam_start.params.push_back(param_t);

          man_spec.start_actions.push_back(eparam_start);

          // Create end actions
          IMC::SetEntityParameters eparam_stop;
          eparam_start.name = "Formation Controller";
          IMC::EntityParameter param_f;
          param_f.name = "Formation Controller";
          param_f.value = "false";

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
}

DUNE_TASK
