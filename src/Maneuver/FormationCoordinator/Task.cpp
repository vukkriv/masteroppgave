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

namespace Maneuver
{
  namespace FormationCoordinator
  {
    using DUNE_NAMESPACES;

    struct Task: public DUNE::Tasks::Task
    {
      //! Formation coordination message
      IMC::FormCoord m_form_coord;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx)
      {
        //bind<IMC::PlanGeneration>(this);
        bind<IMC::PlanControl>(this);
        bind<IMC::PlanDB>(this);
        //bind<IMC::PlanManeuver>(this);
        //bind<IMC::PlanTransition>(this);
        bind<IMC::FormCoord>(this);
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

        // Ignore if not reply for this entity
        if (msg->getDestination() != getSystemId() || msg->getDestinationEntity() != getEntityId())
        {
          trace("Ignoring PlanDB with other destination");
          return;
        }

        if (msg->type == IMC::PlanDB::DBT_SUCCESS && msg->op == IMC::PlanDB::DBOP_GET)
        {
          debug("Got plan from DB, checking if Formation Controller is activated...");
          // Check if plan has activated Formation Controller
          bool is_formation_control = false;
          try
            {
              const IMC::SetEntityParameters* t_sep;
              const IMC::PlanSpecification* planspec = static_cast<const IMC::PlanSpecification*>(msg->arg.get());
              IMC::MessageList<IMC::PlanManeuver>::const_iterator it = planspec->maneuvers.begin();
              for (; it != planspec->maneuvers.end(); it++ )
              {
                IMC::MessageList<IMC::Message>::const_iterator it_sa = (*it)->start_actions.begin();
                for (; it_sa != (*it)->start_actions.end(); it_sa++ )
                {
                  t_sep = static_cast<const IMC::SetEntityParameters*>(*it_sa);
                  if (t_sep->name.compare("Formation Controller") == 0)
                  {
                    IMC::MessageList<IMC::EntityParameter>::const_iterator it_ep = t_sep->params.begin();
                    for (; it_ep != t_sep->params.end(); it_ep++ )
                    {
                      if ((*it_ep)->name.compare("Active") == 0 &&
                          (*it_ep)->value.compare("true") == 0)
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
              err(DTR("Plan specification request failed with uncaught exception: %s"), e.what());
            }

            if (!is_formation_control)
            {
              debug("Plan did not activate Formation Controller");
              return;
            }

            debug("Plan started with Formation Controller active!");
            // Notify vehicles in formation
            m_form_coord.flag = IMC::FormCoord::FC_START;
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
        msg->toText(std::cout);

        // Check if system was destination
        if (msg->getDestination() != this->getSystemId())
        {
          trace("Ignored PlanControl with wrong destination '%s'",
                resolveSystemId(msg->getDestination()));
          return;
        }

        // Check if it is a request
        if (msg->type != IMC::PlanControl::PC_REQUEST)
        {
          trace("Ignored PlanControl (not a request)");
          return;
        }

        if (msg->op == IMC::PlanControl::PC_START)
        {
          debug("PlanControl START, requesting plan from DB...");
          // Request plan details from Plan Database
          IMC::PlanDB plan_db;
          plan_db.type = IMC::PlanDB::DBT_REQUEST;
          plan_db.op = IMC::PlanDB::DBOP_GET;
          plan_db.plan_id = msg->plan_id;
          dispatch(plan_db);
        }
        else if (msg->op == IMC::PlanControl::PC_STOP)
        {
          debug("Plan stopped");
          // Notify vehicles in formation
          m_form_coord.flag = IMC::FormCoord::FC_ABORT;
          dispatch(m_form_coord);
          debug("Sent Formation Abort");
        }
      }

      void
      consume(const IMC::FormCoord* msg)
      {
        // Ignored if sent by self
        if (msg->getSource() == getSystemId())
          return;

        switch (msg->flag)
        {
          case IMC::FormCoord::FC_START:
          {
            debug("Received Formation Start");
            // TODO: Load and execute plan where Formation Controller is activated
            IMC::EntityParameter parm;
            parm.name = "Active";
            parm.value = "true";

            IMC::SetEntityParameters eparm;
            eparm.name = "Formation Controller";
            eparm.params.push_back(parm);

            dispatch(eparm);
            break;
          }
          case IMC::FormCoord::FC_ABORT:
          {
            war("Received Formation Abort");
            IMC::PlanControl plan_ctrl;
            plan_ctrl.type = IMC::PlanControl::PC_REQUEST;
            plan_ctrl.op = IMC::PlanControl::PC_STOP;
            dispatch(plan_ctrl);
            break;
          }
          case IMC::FormCoord::FC_FINISHED:
          {
            debug("Received Formation Finished");
            // TODO: Notify that plan finished
            break;
          }
        }
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
