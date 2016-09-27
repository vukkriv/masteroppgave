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
// If the vehicle is running a formation (copters) and recieves an abort, this
// task will generate a new plan to get away from dodge.

// DUNE headers.
#include <DUNE/DUNE.hpp>

// DUNE USER headers
#include <USER/DUNE.hpp>

namespace Monitors
{
  namespace UAV
  {
    namespace FormationAbortPlan
    {
      using DUNE_NAMESPACES;

      struct Arguments
      {
        //! Horizontal deflection
        double deflection_horizontal;

        //! Vertical deflection
        double deflection_vertical;

        //! Use ardutracker
        bool use_ardutracker;

        //! Deflection speed
        float speed;

        //! State Reset timeout
        double state_reset_timeout;

        //! Entity to send centroid EstimatedLocalState
        std::vector<std::string> ent_centroid_elocalstate;
      };

      // Holder class for criterias to be met to issue abort plan.
      struct AbortState
      {
        AbortState(): got_abort(false),
                      got_vs_error(false),
                      got_formcord_abort(false),
                      got_vs_back_to_service(false),
                      got_idle_maneuver(false),
                      time_of_abort(0)
        {
          /* Intentionally empty. */
        };

        bool allConditionsMet(void) { return (got_abort && got_vs_error && got_vs_back_to_service && got_idle_maneuver); };

        bool got_abort;
        bool got_vs_error;
        bool got_formcord_abort;
        bool got_vs_back_to_service;
        bool got_idle_maneuver;

        double time_of_abort;

      };

      struct Location
      {
        Location(): lat(0.0), lon(0.0), hgt(0.0) {};
        Location(const EstimatedState* msg) {
          lat = msg->lat;
          lon = msg->lon;
          hgt = msg->height;
          WGS84::displace(msg->x, msg->y, msg->z, &lat, &lon, &hgt);
        }

        double lat, lon, hgt;
      };

      struct Task: public DUNE::Tasks::Task
      {
        //! Task arguments
        Arguments m_args;

        //! Last recieved FormCoord
        IMC::FormCoord m_prev_formcoord;

        //! Last recieved vehicle state
        IMC::VehicleState m_prev_vs;

        //! Is currently doing a formation
        bool m_is_formation_active;

        //! State
        AbortState m_abortState;

        //! Filter for centroid state
        SourceFilter* m_elocalstate_filter;
        //! Last received centroid elocal state
        IMC::EstimatedLocalState m_centroid_elstate;

        //! Current position in lstate
        IMC::EstimatedState m_estate;



        //! Constructor.
        //! @param[in] name task name.
        //! @param[in] ctx context.
        Task(const std::string& name, Tasks::Context& ctx):
          DUNE::Tasks::Task(name, ctx),
          m_is_formation_active(false),
          m_elocalstate_filter(NULL)
        {

          param("Deflection - Horizontal", m_args.deflection_horizontal)
          .defaultValue("4")
          .units(Units::Meter)
          .description("The deflection in horizontal direction from formation centroid to go to. ");

          param("Deflection - Vertical", m_args.deflection_vertical)
          .defaultValue("0")
          .units(Units::Meter)
          .description("The deflection in vertical direction from formation centroid to go to. ");

          param("Deflection - Speed", m_args.speed)
          .defaultValue("4")
          .units(Units::MeterPerSecond)
          .description("Speed to go to abort location. ");

          param("Enable Ardutracker", m_args.use_ardutracker)
          .defaultValue("true")
          .description("Set to true to use ardutracker. Otherwise, uses copter acceleration controller. ");



          param("State Timeout", m_args.state_reset_timeout)
          .defaultValue("2")
          .description("Maximum time to wait for all criterias to be met. ");

          param("Filter - Centroid EstimatedLocalState Entity", m_args.ent_centroid_elocalstate)
          .defaultValue("Formation Centroid");

          bind<IMC::Abort>(this);
          bind<IMC::FormCoord>(this);
          bind<IMC::VehicleState>(this);
          bind<IMC::EstimatedLocalState>(this);
          bind<IMC::EstimatedState>(this);
          bind<IMC::IdleManeuver>(this);

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
          // Process the systems allowed to define EstimatedLocalState
          m_elocalstate_filter = new Tasks::SourceFilter(*this, false, m_args.ent_centroid_elocalstate, "EstimatedLocalState");
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
        consume(const IMC::Abort* msg)
        {
          if (msg->getDestination() != getSystemId())
            return;

          if (!m_abortState.got_abort)
          {
            m_abortState.got_abort = true;
            m_abortState.time_of_abort = Clock::get();
            inf("Got abort, start checking conditions..");
          }
        }

        void
        consume(const IMC::FormCoord* msg)
        {
          if( msg->op == FormCoord::FCOP_START )
            m_is_formation_active = true;

          if( msg->op == FormCoord::FCOP_ABORT )
          {
            if (!m_abortState.got_abort)
            {
              m_abortState.got_abort = true;
              m_abortState.got_formcord_abort = true;

              m_abortState.time_of_abort = Clock::get();
              inf("Got formcoord abort, start checking conditions..");
            }
          }
        }

        void
        consume(const IMC::VehicleState* msg)
        {

          //! Check if got abort (either from normal abort or foormcoord. Both sets abort flag.
          if (!m_abortState.got_abort)
            return;

          // We need the sequence of events: abort -> error -> service.
          // Possible extension is to cancel abortPlan sequence if we drop back into error.

          if ( !m_abortState.got_vs_error
               && msg->op_mode == VehicleState::VS_ERROR )
          {
            debug("Got error state. ");
            m_abortState.got_vs_error = true;
          }

          if ( m_abortState.got_vs_error
               && !m_abortState.got_vs_back_to_service
               &&  msg->op_mode == VehicleState::VS_SERVICE)
          {
            debug("Got Back to Service. ");
            m_abortState.got_vs_back_to_service = true;
          }

        }

        void
        consume(const IMC::IdleManeuver* msg)
        {
          (void) msg;

          debug("Got Idle maneuver");

          // This actually comes right after abort..
          if (m_abortState.got_abort)
          {
            debug("Registered Idle Maneuver. ");
            m_abortState.got_idle_maneuver = true;
          }
        }

        void
        consume(const IMC::EstimatedLocalState* elstate)
        {

         // Filter EstimatedLocalState by entities
         if (!m_elocalstate_filter->match(elstate))
           return;

         m_centroid_elstate = *elstate;
        }

        void
        consume(const IMC::EstimatedState* msg)
        {
          m_estate = *msg;
        }

        // Reset abort state
        void
        resetAbortState(bool executed = false)
        {
          m_abortState = AbortState();

          if (!executed)
            war("Abort-state reset. ");
        }

        // Issue abort plan
        void
        executeAbortPlan(void)
        {
          inf("Executing abort plan. ");

          /*
           * Steps:
           * 1. Get distance from current pos to centroid.
           * 2. If distance is > x m (e.g 0.3) calculate bearing
           * 3. Go x m along that from current position.
           * 4. Generate plan with goto-maneuver at given point.
           *
           */


          Location cur_loc = Location(&m_estate);

          Location desired_location = cur_loc;



          if (m_centroid_elstate.state.isNull())
          {
            // Not recieved anything, just do height adjustment.
            desired_location.hgt += m_args.deflection_vertical;
          }
          else
          {
            Location centroid_loc = Location(m_centroid_elstate.state.get());

            double bearing, range;
            WGS84::getNEBearingAndRange(centroid_loc.lat, centroid_loc.lon,
                                        cur_loc.lat, cur_loc.lon,
                                        &bearing, &range);


            double dx = 0.0, dy = 0.0, dz = 0.0;

            dz = -m_args.deflection_vertical;

            if (range > 0.2)
            {
              dx = m_args.deflection_horizontal * std::cos(bearing);
              dy = m_args.deflection_horizontal * std::sin(bearing);
            }
            else
            {
              war("Very short range, just doing vertical");
            }

            WGS84_Accurate::displace(dx, dy, dz,
                                     &desired_location.lat, &desired_location.lon, &desired_location.hgt);

          }

          // Got desired location. Generating plan.
          generateAndStartPlan(desired_location);



          // Reset abortstate
          resetAbortState(true);
        }

        void
        generateAndStartPlan(Location location)
        {
          // Create plan set request
          IMC::PlanDB plan_db;
          plan_db.type = IMC::PlanDB::DBT_REQUEST;
          plan_db.op = IMC::PlanDB::DBOP_SET;
          plan_db.plan_id = "Formation Abort Plan";
          plan_db.request_id = 0;

          // Create plan specification
          IMC::PlanSpecification plan_spec;
          plan_spec.plan_id = plan_db.plan_id;
          plan_spec.start_man_id = 1;
          plan_spec.description = "Plan activating Formation Abort";

          // Create plan maneuver
          IMC::PlanManeuver man_spec;
          man_spec.maneuver_id = 1;

          // Create a goto maneuver
          IMC::Goto c_man;
          c_man.lat = location.lat;
          c_man.lon = location.lon;
          c_man.z = location.hgt;
          c_man.z_units = IMC::Z_HEIGHT;
          c_man.speed = m_args.speed;
          c_man.speed_units = IMC::SUNITS_METERS_PS;

          man_spec.data.set(c_man);

          // Make sure formation controller is off

          // Create start actions
          IMC::SetEntityParameters eparam_start;
          eparam_start.name = "Formation Controller";

          IMC::EntityParameter param_t;
          param_t.name = "Formation Controller";
          param_t.value = "false";
          eparam_start.params.push_back(param_t);


          bool doArduTracker = false;
          if (!m_ctx.profiles.isSelected("Simulation") && m_args.use_ardutracker)
            doArduTracker = true;



          IMC::SetEntityParameters coordinated_start;
          if (doArduTracker)
          {
            coordinated_start.name = "Autopilot";

            IMC::EntityParameter param_ctrl_s;
            param_ctrl_s.name = "Ardupilot Tracker";
            param_ctrl_s.value = "true";
            coordinated_start.params.push_back(param_ctrl_s);
          }
          else
          {
            coordinated_start.name = "Simple Acceleration Path Controller";

            IMC::EntityParameter param_ctrl_s;
            param_ctrl_s.name = "Acceleration Controller";
            param_ctrl_s.value = "true";
            coordinated_start.params.push_back(param_ctrl_s);
          }

          man_spec.start_actions.push_back(eparam_start);
          man_spec.start_actions.push_back(coordinated_start);

          // Create end actions
          IMC::SetEntityParameters eparam_stop;
          eparam_stop.name = "Formation Controller";

          IMC::EntityParameter param_f;
          param_f.name = "Formation Controller";
          param_f.value = "false";

          eparam_stop.params.push_back(param_f);

          // Enc coordniated
          IMC::SetEntityParameters coordinated_stop;
          if (doArduTracker)
          {
            coordinated_stop.name = "Autopilot";

            IMC::EntityParameter param_ctrl_f;
            param_ctrl_f.name = "Ardupilot Tracker";
            param_ctrl_f.value = "false";
            coordinated_stop.params.push_back(param_ctrl_f);
          }
          else
          {
            coordinated_stop.name = "Simple Acceleration Path Controller";

            IMC::EntityParameter param_ctrl_f;
            param_ctrl_f.name = "Acceleration Controller";
            param_ctrl_f.value = "false";
            coordinated_stop.params.push_back(param_ctrl_f);
          }



          man_spec.end_actions.push_back(eparam_stop);
          man_spec.end_actions.push_back(coordinated_stop);

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

            // Check for state reset
            if (m_abortState.got_abort && Clock::get() > m_abortState.time_of_abort + m_args.state_reset_timeout)
              resetAbortState();

            if (m_abortState.allConditionsMet())
              executeAbortPlan();
          }
        }
      };
    }
  }
}

DUNE_TASK
