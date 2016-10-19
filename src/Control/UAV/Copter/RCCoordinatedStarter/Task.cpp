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
// Author: Kristian Klausen                                                 *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Control
{
  namespace UAV
  {
    namespace Copter
    {
      namespace RCCoordinatedStarter
      {
        using DUNE_NAMESPACES;

        struct Arguments
        {
          bool auto_enable;
          //! True if trying to start on idle mode, after recieving a plan stop
          bool idle_enable;
        };

        struct Task: public DUNE::Tasks::Task
        {
          //! Task Arguments
          Arguments m_args;
          //! Last received plan control state.
          IMC::PlanControlState m_pcs;
          //! Last received autopilot mode
          IMC::AutopilotMode m_apmode;
          //! Is running
          bool m_fr_is_running;


          //! Time of last planStop
          double m_time_prev_stop;
          //! Time of last abort
          double m_time_prev_abort;

          //! Constructor.
          //! @param[in] name task name.
          //! @param[in] ctx context.
          Task(const std::string& name, Tasks::Context& ctx):
            DUNE::Tasks::Task(name, ctx),
            m_fr_is_running(false)
          {

            param("Automatic Enable", m_args.auto_enable)
            .defaultValue("false")
            .visibility(Parameter::VISIBILITY_USER)
            .description("Automatically enable if entering guided and still in service mode. ");

            param("Start on Idle after Plan Stop", m_args.idle_enable)
            .defaultValue("false")
            .visibility(Parameter::VISIBILITY_USER)
            .description("Starts the rc coordinated plan if doing an idle parameter just after a plan stop, with no aborts. ");

            bind<IMC::PlanControlState>(this);
            bind<IMC::AutopilotMode>(this);
            bind<IMC::RemoteActions>(this);
            bind<IMC::IdleManeuver>(this);
            bind<IMC::Abort>(this);

            setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_IDLE);
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
          consume(const IMC::Abort* abort)
          {
            if (abort->getDestination() == getSystemId())
              m_time_prev_abort = Clock::get();
          }

          void
          consume(const IMC::PlanControlState* pcs)
          {

            // Check for switch from executing to service
            if (m_pcs.state == IMC::PlanControlState::PCS_EXECUTING
                && pcs->state == IMC::PlanControlState::PCS_READY
                //&& pcs->last_outcome == IMC::PlanControlState::LPO_SUCCESS
                )
            {
              m_time_prev_stop = Clock::get();
              debug("Sensed a switch from PathControlState. ");
            }


            m_pcs = *pcs;

            if (pcs->plan_id == "CoordinatedFollowReference" && pcs->state == IMC::PlanControlState::PCS_EXECUTING)
              m_fr_is_running = true;
            else
              m_fr_is_running = false;
          }

          void
          consume(const IMC::RemoteActions* ra)
          {
            inf("GOt remote actin. ");
            inf("%s", ra->actions.c_str());

            if(strcmp("piksiResetIARs=1", ra->actions.c_str()) == 0)
            {
              inf("Starting follow reference based on piksiResetIARs.  ");
              generatePlan();
            }
          }

          void
          consume(const IMC::AutopilotMode* apmode)
          {

            if (apmode->mode == "GUIDED" && m_apmode.mode == "LOITER")
            {
              // If we are also in service, check if we should start.
              if (m_pcs.state == IMC::PlanControlState::PCS_READY
                  && m_args.auto_enable)
              {
                inf("Starting Follow Reference based on mode switch. ");
                generatePlan();
              }
            }

            // Check if disable
            if (apmode->mode != "GUIDED" && m_fr_is_running)
            {
              //m_ref.flags |= IMC::Reference::FLAG_MANDONE;
              //dispatch(m_ref);
            }

            m_apmode = *apmode;
          }

          void
          consume(const IMC::IdleManeuver* idle)
          {
            (void) idle;
            inf("Got Idle maneuver. ");
            if (m_args.idle_enable)
            {
              double now = Clock::get();
              inf("Checking if executing RCCoordinated..");
              // Start if not recieved any aborts the past 10 seconds, and only if just stopped a plan.
              if (now > m_time_prev_abort + 10.0
                  && now < m_time_prev_stop + 1.0)
              {
                inf("Starting Follow Reference based IdleManeuver. ");
                generatePlan();
              }

            }
          }

          void
          generatePlan(void)
          {
            // Create plan set request
            IMC::PlanDB plan_db;
            plan_db.type = IMC::PlanDB::DBT_REQUEST;
            plan_db.op = IMC::PlanDB::DBOP_SET;
            plan_db.plan_id = "CoordinatedFollowReference";
            plan_db.request_id = 0;

            // Create plan specification
            IMC::PlanSpecification plan_spec;
            plan_spec.plan_id = plan_db.plan_id;
            plan_spec.start_man_id = 1;
            plan_spec.description = "Plan activating CoordinatedFollowReference";

            // Create plan maneuver
            IMC::PlanManeuver man_spec;
            man_spec.maneuver_id = 1;

            // Create a follow reference maneuver
            IMC::FollowReference c_man;
            c_man.loiter_radius = 0.0;
            c_man.control_ent = getEntityId();
            c_man.control_src = getSystemId();
            c_man.timeout    = 2.0;

            man_spec.data.set(c_man);

            // Create start actions
            IMC::SetEntityParameters eparam_start;
            eparam_start.name = "Formation Controller";

            IMC::EntityParameter param_t;
            param_t.name = "Formation Controller";
            param_t.value = "true";
            eparam_start.params.push_back(param_t);


            IMC::SetEntityParameters coordinated_start;
            coordinated_start.name = "RCCoordinated Controller";

            IMC::EntityParameter param_ctrl_s;
            param_ctrl_s.name = "RCCoordinated Controller";
            param_ctrl_s.value = "true";
            coordinated_start.params.push_back(param_ctrl_s);

            man_spec.start_actions.push_back(eparam_start);
            man_spec.start_actions.push_back(coordinated_start);

            // Create end actions
            IMC::SetEntityParameters eparam_stop;
            eparam_stop.name = "Formation Controller";
            IMC::EntityParameter param_f;
            param_f.name = "Formation Controller";
            param_f.value = "false";

            // Enc coordniated
            IMC::SetEntityParameters coordinated_stop;
            coordinated_stop.name = "RCCoordinated Controller";

            IMC::EntityParameter param_ctrl_f;
            param_ctrl_f.name = "RCCoordinated Controller";
            param_ctrl_f.value = "false";
            coordinated_stop.params.push_back(param_ctrl_f);

            eparam_stop.params.push_back(param_f);
            coordinated_stop.params.push_back(param_ctrl_f);

            man_spec.end_actions.push_back(eparam_stop);
            man_spec.end_actions.push_back(coordinated_stop);

            plan_spec.maneuvers.push_back(man_spec);

            plan_db.arg.set(plan_spec);

            // Send set plan request
            dispatch(plan_db);

            // Set current pos as target.
            //setCurrentPosAsTarget();

            // Create and send plan start request
            IMC::PlanControl plan_ctrl;
            plan_ctrl.type = IMC::PlanControl::PC_REQUEST;
            plan_ctrl.op = IMC::PlanControl::PC_START;
            plan_ctrl.plan_id = plan_spec.plan_id;
            plan_ctrl.request_id = 0;
            plan_ctrl.arg.set(plan_spec);
            plan_ctrl.setDestination(this->getSystemId());
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
}

DUNE_TASK
