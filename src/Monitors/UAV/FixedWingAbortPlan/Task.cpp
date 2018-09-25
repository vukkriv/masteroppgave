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
// Author: Kristian Klausen, Kristoffer Gryte                               *
//***************************************************************************
// If the vehicle is running a maneuver and recieves an abort, this
// task will generate a new plan to get away to a predefined (assumed safe)
// location.

// DUNE headers.
#include <DUNE/DUNE.hpp>

// DUNE USER headers
#include <USER/DUNE.hpp>

namespace Monitors
{
  namespace UAV
  {
    namespace FixedWingAbortPlan
    {
      using DUNE_NAMESPACES;

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

      struct Arguments
      {
        //double deflection_horizontal;
        //! Use ardutracker
        bool use_ardutracker;
        //! State Reset timeout
        double state_reset_timeout;
        
        //! Assumed safe location
        Location safe_loc;

        double speed;

        // Temporarily store lat,lon in degrees
        fp64_t lat_deg, lon_deg;
      };

      // Holder class for criterias to be met to issue abort plan.
      struct AbortState
      {
        AbortState(): got_abort(false),
                      got_vs_error(false),
                      //got_formcord_abort(false),
                      got_vs_back_to_service(false),
                      got_idle_maneuver(false),
                      time_of_abort(0)
        {
          /* Intentionally empty. */
        };

        bool allConditionsMet(void) { return (got_abort && got_vs_error && got_vs_back_to_service && got_idle_maneuver); };

        bool got_abort;
        bool got_vs_error;
        //bool got_formcord_abort;
        bool got_vs_back_to_service;
        bool got_idle_maneuver;

        double time_of_abort;
      };

      struct Task: public DUNE::Tasks::Task
      {
        //! Task arguments
        Arguments m_args;


        //! Current position in lstate
        IMC::EstimatedState m_estate;
        
        //! State
        AbortState m_abortState;

        //! Constructor.
        //! @param[in] name task name.
        //! @param[in] ctx context.
        Task(const std::string& name, Tasks::Context& ctx):
          DUNE::Tasks::Task(name, ctx)
          //m_is_formation_active(false),
          //m_elocalstate_filter(NULL)
        {
          param("Safe location, lat", m_args.lat_deg)
          .defaultValue("63.628744")
          .units(Units::Degree)
          .description("The latitude of the assumed safe location, that the UAV will fly to on an Abort message.");

          param("Safe location, lon", m_args.lon_deg)
          .defaultValue("9.727626")
          .units(Units::Degree)
          .description("The longitude of the assumed safe location, that the UAV will fly to on an Abort message.");

          param("Safe location, hgt", m_args.safe_loc.hgt)
          .defaultValue("100")
          .units(Units::Meter)
          .description("The WGS84 height of the assumed safe location, that the UAV will fly to on an Abort message.");

          param("State Timeout", m_args.state_reset_timeout)
          .defaultValue("2")
          .description("Maximum time to wait for all criterias to be met. ");

          param("Desired airspeed", m_args.speed)
          .defaultValue("18")
          .units(Units::MeterPerSecond)
          .description("Speed to go to abort location. ");

          param("Enable Ardutracker", m_args.use_ardutracker)
          .defaultValue("true")
          .description("Set to true to use ardutracker. Otherwise, uses copter acceleration controller. ");

          bind<IMC::Abort>(this);
          bind<IMC::IdleManeuver>(this);
          bind<IMC::VehicleState>(this);
          bind<IMC::EstimatedState>(this);
        }

        //! Update internal state with new parameter values.
        void
        onUpdateParameters(void)
        {
          m_args.safe_loc.lat = Angles::radians(m_args.lat_deg);
          m_args.safe_loc.lon = Angles::radians(m_args.lon_deg);
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
        consume(const IMC::Abort* msg)
        {
          debug("Got Abort message!");
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
        consume(const IMC::VehicleState* msg)
        {
          debug("Got VehicleState: %i", msg->op_mode);
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

          // This actually comes right after abort..
          if (m_abortState.got_abort)
          {
            debug("Registered Idle Maneuver. ");
            m_abortState.got_idle_maneuver = true;
          }
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

          // Got desired location. Generating plan.
          generateAndStartPlan(m_args.safe_loc);

           //Reset abortstate
          resetAbortState(true);
        }

        void
        generateAndStartPlan(Location location)
        {
          // Create plan set request
          IMC::PlanDB plan_db;
          plan_db.type = IMC::PlanDB::DBT_REQUEST;
          plan_db.op = IMC::PlanDB::DBOP_SET;
          plan_db.plan_id = "Abort Plan";
          plan_db.request_id = 0;

          // Create plan specification
          IMC::PlanSpecification plan_spec;
          plan_spec.plan_id = plan_db.plan_id;
          plan_spec.start_man_id = 1;
          plan_spec.description = "Plan activating Abort";

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

          IMC::SetEntityParameters coordinated_start;
          coordinated_start.name = "Autopilot";

          IMC::EntityParameter param_ctrl_s;
          param_ctrl_s.name = "Ardupilot Tracker";
          param_ctrl_s.value = "true";
          coordinated_start.params.push_back(param_ctrl_s);

          man_spec.start_actions.push_back(coordinated_start);

          // Enc coordniated
          IMC::SetEntityParameters coordinated_stop;
          
          coordinated_stop.name = "Autopilot";

          IMC::EntityParameter param_ctrl_f;
          param_ctrl_f.name = "Ardupilot Tracker";
          param_ctrl_f.value = "false";
          coordinated_stop.params.push_back(param_ctrl_f);

          man_spec.end_actions.push_back(coordinated_stop);

          plan_spec.maneuvers.push_back(man_spec);

          plan_db.arg.set(plan_spec);

          // Send set plan request
          
          debug("Dispatching plan request");
          dispatch(plan_db);

          // Create and send plan start request
          IMC::PlanControl plan_ctrl;
          plan_ctrl.type = IMC::PlanControl::PC_REQUEST;
          plan_ctrl.op = IMC::PlanControl::PC_START;
          plan_ctrl.plan_id = plan_spec.plan_id;
          plan_ctrl.request_id = 0;
          plan_ctrl.arg.set(plan_spec);
          plan_ctrl.setDestination(this->getSystemId());
          debug("Dispatching plan start request");
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
