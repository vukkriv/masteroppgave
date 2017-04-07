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
// Author: Siri Mathisen                                                    *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>
#include "Payload.hpp"

namespace Autonomy
{
  namespace DirectDrop
  {
    using DUNE_NAMESPACES;
    using namespace std;

    struct Arguments
    {
      bool    simulation;
      fp32_t  glide_time;
      fp32_t  drop_time;
      fp32_t  safe_altitude;
      fp32_t  drop_altitude;
      fp32_t  dt;
      fp32_t  desired_speed;
      fp32_t  max_current;
      fp32_t  decision_distance;
    };

    // The enums for the state machine that controls the drop maneuvre.
    enum Maneuver_states
    {
      IDLE=-1,
      STARTING,
      LOITER,
      DROP_POINT,
      SAFE_HEIGHT,
      GLIDE_BEFORE_DROP,
      GLIDE_AFTER_DROP,
      LAST_STATE
    };

    struct Task: public DUNE::Tasks::Task
    {
      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.

      //Variables
      bool                          m_is_ready;
      bool                          m_is_ready_for_target;
      bool                          m_is_flying;
      bool                          m_has_received_wind;
      bool                          m_has_target;
      bool                          m_continue_after_break;
      bool                          m_is_executing;

      int                           m_johansen_wind_est_ent;

      uint64_t                      m_t_dropped_object;
      uint64_t                      m_t_guided_started;
      uint64_t                      m_t_shut_down_motor;

      fp32_t                        m_distance_to_CARP_centre;
      fp32_t                        m_current;

      Maneuver_states               m_maneuver_state;

      Point                         m_UAV_state;
      Payload                       m_payload;
      Clock                         m_timer;
      Random::Generator*            m_random_generator;

      //Arguments from .ini-file
      Arguments                     m_args;

      //IMC messages
      Reference                     m_CARP_circle_centre_ref, m_loiter_ref, m_safe_height_ref;
      Target                        m_target;
      EstimatedStreamVelocity       m_wind;
      PowerChannelControl           m_pcc;
      DesiredSpeed                  m_dspeed;
      DesiredZ                      m_dheight;


      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_is_ready(false),
        m_is_ready_for_target(false),
        m_is_flying(false),
        m_has_received_wind(false),
        m_has_target(false),
        m_continue_after_break(false),
        m_is_executing(false),
        m_johansen_wind_est_ent(-1),
        m_t_dropped_object(0),
        m_t_guided_started(0),
        m_t_shut_down_motor(0),
        m_distance_to_CARP_centre(0.0),
        m_current(0.0),
        m_maneuver_state(IDLE),
        m_random_generator(0)
      {


        //Load parameters from .ini-file
        param("Simulation",m_args.simulation).
            defaultValue("false").
            description("True if this is supposed to be repeated indefinitely");
        param("Glide Time",m_args.glide_time).
            defaultValue("0.0").
            units(Units::Second).
            description("Glide time before turning motor back on again");
        param("Drop Time",m_args.drop_time).
            defaultValue("0.0").
            units(Units::Second).
            description("Time from drop signal is given before load is detached from UAV");
        param("Safe Altitude",m_args.safe_altitude).
            defaultValue("100").
            units(Units::Meter).
            description("Safe flying attitude: Is added on target height to get safe height");
        param("Drop Altitude",m_args.drop_altitude).
            defaultValue("100.0").
            units(Units::Meter).
            description("Drop altitude: Is added on target height to get drop height");
        param("Time Step",m_args.dt).
            defaultValue("0.01").
            units(Units::Second).
            description("Simulation time step");
        param("Desired Speed",m_args.desired_speed).
            defaultValue("18.0").
            units(Units::MeterPerSecond).
            description("Desired speed for all waypoints");
        param("Max Current",m_args.max_current).
            defaultValue("3.0").
            units(Units::Meter).
            description("Limit for motor current to count as off");
        param("Decision Distance",m_args.decision_distance).
            defaultValue("200.0").
            units(Units::Meter).
            description("Within this distance: Direct drop, else: Loiter");

        //IMC messages to receive
        bind<Target>(this);
        bind<EstimatedStreamVelocity>(this);
        bind<EstimatedState>(this);
        bind<PlanControlState>(this);
        bind<PlanControl>(this);
        bind<VehicleMedium>(this);
        bind<Current>(this);
        bind<EntityList>(this);
        bind<PowerChannelState>(this);
        bind<AutopilotMode>(this);
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        m_pcc.name = "drop";
        m_timer = Clock();
        m_random_generator = Random::Factory::create(Random::Factory::c_default, -1);
        m_payload = Payload(m_args.glide_time,m_args.drop_time,m_args.dt);
        m_dheight.z_units = IMC::Z_HEIGHT;
        m_dspeed.value = m_args.desired_speed;
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

      //Get estimated state
      void
      consume(const IMC::EstimatedState* msg)
      {
        if(m_has_received_wind && m_is_ready && m_is_flying && !m_is_ready_for_target)
        {
          m_is_ready_for_target = true;
          war("Ready for target coordinates");
        }
        m_UAV_state.lat = msg->lat;
        m_UAV_state.lon = msg->lon;
        m_UAV_state.z = msg->height;
        m_UAV_state.vx = msg->vx;
        m_UAV_state.vy = msg->vy;
        m_UAV_state.vz = msg->vz;
        WGS84::displace(msg->x,msg->y,msg->z,&m_UAV_state.lat,&m_UAV_state.lon,&m_UAV_state.z);
        if(m_has_target){
          m_distance_to_CARP_centre = WGS84::distance(m_UAV_state.lat,m_UAV_state.lon,m_UAV_state.z,
                     m_payload.get_CARP_circle().centre_lat,m_payload.get_CARP_circle().centre_lon,m_payload.get_CARP_circle().z);
        }
        runStateMachine();
      }

      //Get usefull entities
      void
      consume(const IMC::EntityList* msg)
      {
        if(m_johansen_wind_est_ent == -1)
        {
          TupleList tup(msg->list, "=", ";", true);
          m_johansen_wind_est_ent = tup.get("Wind Estimator", -1);
        }

      }

      //Get wind from estimator or ardupilot
      void
      consume(const IMC::EstimatedStreamVelocity* msg)
      {
//            war("Received wind: entity = %d", msg->getSourceEntity());
        if(msg->getSourceEntity() == m_johansen_wind_est_ent)
        {
          m_wind = *msg;
          m_has_received_wind = true;
        }
      }

      //Check how the drop mechanism is doing
      void
      consume(const IMC::PowerChannelState* msg)
      {
        war("We got a PowerChannelState msg");
        if(msg->state == IMC::PowerChannelState::PCS_ON)
        {
          m_pcc.op = 0;
          dispatch(m_pcc);
          fp32_t dt = m_timer.getMsec() - m_t_dropped_object;
          war("Time it took to drop the payload: %f", dt);
        }
      }

      void
      consume(const IMC::AutopilotMode* msg)
      {
        if(msg->mode == "GUIDED" && m_args.simulation)
        {
          int time_since_guided_started = m_timer.getMsec() - m_t_guided_started;
          if(time_since_guided_started > 5000)
          {
            stopExecution();
            m_continue_after_break = true;
            err("WatchDog!");
          }
        }
        else
          m_t_guided_started = m_timer.getMsec();
      }

      //Check plan control state
      void
      consume(const IMC::PlanControl* msg)
      {
        if (msg->op == IMC::PlanControl::PC_STOP)
        {
//          war("Stop request received");
          m_is_executing = false;
          if(m_continue_after_break)
          {
            war("StoppedExecution! And restarted...");
            m_maneuver_state = STARTING;
            war("Going to state STARTING");
            m_continue_after_break = false;
//            startExecution();
          }else
          {
            m_maneuver_state = IDLE;
            war("Going to state IDLE");
          }
        }
      }

      //Get battery current
      void
      consume(const IMC::Current* msg)
      {
        m_current = msg->value;
      }

      //Check if in air
      void
      consume(const IMC::VehicleMedium* vm)
      {
        m_is_flying = (vm->medium == IMC::VehicleMedium::VM_AIR);
      }

      //Check control state
      void
      consume(const IMC::PlanControlState* msg)
      {
        if(msg->state == PlanControlState::PCS_READY )
        {
          m_is_ready = true;
        }
      }

      //Consume target and calculate first CARP and start point
      void
      consume(const IMC::Target* msg)
      {
        if(m_is_ready_for_target){
          m_has_target = true;

          //Save target
          m_target = *msg;

          //Set target
          m_payload.set_target(m_target);
          inf("Target set to: %f %f %f", m_target.lat, m_target.lon, m_target.z);

          //Calculate CARP circle
          m_payload.calculate_CARP_circle(m_wind,m_args.desired_speed,m_args.drop_altitude);


          //Reference for CARP circle centre

          m_CARP_circle_centre_ref.lat = m_payload.get_CARP_circle().centre_lat;
          m_CARP_circle_centre_ref.lon = m_payload.get_CARP_circle().centre_lon;
          m_dheight.value = m_payload.get_CARP_circle().z;
          m_CARP_circle_centre_ref.z.set(m_dheight);
          m_CARP_circle_centre_ref.speed.set(m_dspeed);
          m_CARP_circle_centre_ref.radius = 0.0;
          m_CARP_circle_centre_ref.flags = Reference::FLAG_DIRECT | Reference::FLAG_LOCATION | Reference::FLAG_RADIUS | Reference::FLAG_SPEED | Reference::FLAG_Z;
          war("CARP ref height: %f", m_CARP_circle_centre_ref.z.get()->value);
          //Reference for loitering
          m_loiter_ref.lat = m_UAV_state.lat;
          m_loiter_ref.lon = m_UAV_state.lon;
          m_loiter_ref.radius = 2*m_args.decision_distance;
          m_loiter_ref.z.set(m_dheight);
          m_loiter_ref.speed.set(m_dspeed);
          m_CARP_circle_centre_ref.flags = Reference::FLAG_DIRECT | Reference::FLAG_LOCATION | Reference::FLAG_RADIUS | Reference::FLAG_SPEED | Reference::FLAG_Z;

          m_maneuver_state = STARTING;
          war("Going to state STARTING");


          if(!m_is_executing)
          {
          startExecution();
          }
        }
        else
        {
          war("New target received and ignored");
        }
      }

      /**************************************************************************************/
      /**************************** State Machine *******************************************/
      /**************************************************************************************/

      //Main state machine
      void
      runStateMachine(void)
      {
        switch(m_maneuver_state)
        {
          case IDLE:
//                inf("IDLE");
            break;
          case STARTING:
//                inf("STARTING");
            startManeuver();
            break;
          case LOITER:
//                inf("LOITER");
            loiter();
            break;
          case DROP_POINT:
//                inf("DROP_POINT");
            guidance();
            break;
          case GLIDE_BEFORE_DROP:
//                inf("GLIDE BEFORE");
            glideBeforeDrop();
            break;
          case GLIDE_AFTER_DROP:
//                inf("GLIDE AFTER");
            glideAfterDrop();
            break;
          case SAFE_HEIGHT:
//                inf("SAFE_HEIGHT");
            goToSafeHeight();
            break;
          default:
            return;
        }
      }

      void
      startManeuver( void )
      {
        startExecution();
        if (m_distance_to_CARP_centre >= m_args.decision_distance)
        {
          dispatch(m_CARP_circle_centre_ref);
          m_maneuver_state = DROP_POINT;
          war("Going to state DROP_POINT");
        }else
        {
          dispatch(m_loiter_ref);
          m_maneuver_state = LOITER;
          war("Going to state LOITER");
        }
      }
      // UAV Follows a truncated Dubins Path
      void
      loiter(void)
      {
//        dispatch(m_loiter_ref);
        if (m_distance_to_CARP_centre >= m_args.decision_distance)
        {
          dispatch(m_CARP_circle_centre_ref);
          m_maneuver_state = DROP_POINT;
          war("Going to state DROP_POINT");
        }
      }

      // UAV flies directly to the CARP, following different optimization strategies
      void guidance( void )
      {
//        dispatch(m_CARP_circle_centre_ref);
//            war("Distance to CARP centre: %f - Radius: %f", m_distance_to_CARP_centre, m_payload.get_CARP_circle().radius);
        if(m_distance_to_CARP_centre <= m_payload.get_CARP_circle().radius)
        {
          war("Distance to CARP centre: %f - Radius: %f", m_distance_to_CARP_centre, m_payload.get_CARP_circle().radius);
          change_to_FBWA();
          send_throttle_command(0.0);
          m_maneuver_state = GLIDE_BEFORE_DROP;
          war("Going to state GLIDE_BEFORE_DROP");
          m_t_shut_down_motor = m_timer.getMsec();
        }
      }

      // Before dropping the payload, the motor has to be turned off
      void glideBeforeDrop( void )
      {
        //Make sure motor is turned off
        int time_since_motor_shutdown = m_timer.getMsec() - m_t_shut_down_motor;
        if(m_current < m_args.max_current && (time_since_motor_shutdown > (m_args.glide_time*1000 - 50)))
        {
          drop();
          m_maneuver_state = GLIDE_AFTER_DROP;
          war("Going to state GLIDE_AFTER_DROP");
        }
      }

      // Drop once the motor is turned off
      void
      drop( void )
      {
        int time_since_motor_shutdown = m_timer.getMsec() - m_t_shut_down_motor;
        war("Signalling to drop: Time since motor shutdown signal was sent: %d", time_since_motor_shutdown);
        m_pcc.name = "drop";
        m_pcc.op = 1;

        m_t_dropped_object = m_timer.getMsec();
        dispatch(m_pcc);
        fp32_t release_dev[3];
        fp32_t carp_centre_dev[3];
        fp32_t target_dev[3];

        war("This is a simple drop!");
        war("Height: %f, ideal drop height: %f", m_UAV_state.z,m_payload.get_CARP_circle().z);

        m_payload.set_release_state(m_UAV_state);
        m_payload.calculate_estimated_hitpoint(m_wind);
        m_payload.load_release_displacement(release_dev);
        m_payload.load_CARP_centre_displacement(carp_centre_dev);
        m_payload.load_target_error(target_dev);

        std::ofstream myfile;
        myfile.open("/home/siri/uavlab/results/direct_drop_results.txt",std::ios::app);
        myfile << m_wind.x << "\t" << m_wind.y << "\t" << m_wind.z << "\t";
        myfile << m_UAV_state.vx << "\t" << m_UAV_state.vy << "\t" << m_UAV_state.vz << "\t";
        myfile << carp_centre_dev[0] << "\t" << carp_centre_dev[1] << "\t" << carp_centre_dev[2] << "\t" << m_payload.get_CARP_circle().radius << "\t";
        myfile << release_dev[0] << "\t" << release_dev[1] << "\t" << release_dev[2] << "\t";
        myfile << target_dev[0] << "\t" << target_dev[1] << "\t" << target_dev[2] << "\t"  << "\n\n";
        myfile.close();

      }

      // After the drop the UAV has to fly for a while with the motor turned off
      void
      glideAfterDrop(void)
      {
        int time_since_drop = m_timer.getMsec() - m_t_dropped_object;
        if(time_since_drop > 1000 * m_args.glide_time)
        {
          m_maneuver_state = SAFE_HEIGHT;
          war("Going to state SAFE_HEIGHT");
//          stopExecution();
          //Safe Height Reference
          m_safe_height_ref.lat = m_payload.get_CARP_circle().centre_lat;
          m_safe_height_ref.lon = m_payload.get_CARP_circle().centre_lon;
          m_dheight.value = m_target.z + m_args.safe_altitude;
          m_safe_height_ref.radius = 0.0;
          m_safe_height_ref.z.set(m_dheight);
          m_safe_height_ref.speed.set(m_dspeed);
          m_safe_height_ref.flags = Reference::FLAG_LOCATION | Reference::FLAG_DIRECT | Reference::FLAG_Z | Reference::FLAG_SPEED | Reference::FLAG_RADIUS;
          double vx_unit = m_UAV_state.vx / sqrt(m_UAV_state.vx * m_UAV_state.vx + m_UAV_state.vy * m_UAV_state.vy);
          double vy_unit = m_UAV_state.vy / sqrt(m_UAV_state.vx * m_UAV_state.vx + m_UAV_state.vy * m_UAV_state.vy);
          WGS84::displace(0.5*m_payload.get_CARP_circle().radius*vx_unit,0.5*m_payload.get_CARP_circle().radius*vy_unit, &m_safe_height_ref.lat,&m_safe_height_ref.lon);
          dispatch(m_safe_height_ref);
        }
      }

      // Go to a safe height after the drop. Repeat procedure
      void
      goToSafeHeight(void)
      {
//        if(m_autopilotmode != "FBWB")
//        {
//          war("Autopilotmode != FBWB");
//          if(m_autopilotmode == "GUIDED")
//          {
//            war("Autopilotmode == GUIDED");
//            stopExecution();
//            return;
//          }
//          if(!m_is_executing)
//          {
//            startExecution();
//          }
//          dispatch(m_ref);
//          return;
//        }

//            dispatch(m_safe_height_ref);

        fp32_t distance_to_ref = WGS84::distance(m_UAV_state.lat,m_UAV_state.lon,0.0,m_safe_height_ref.lat,m_safe_height_ref.lon,0.0);
        if(distance_to_ref < 50)
        {
          if(m_args.simulation)
          {
            fp32_t point_a, point_b;
            point_a = m_random_generator->uniform(-500.0,500.0);
            point_b = m_random_generator->uniform(-500.0,500.0);
            WGS84::displace(point_a,point_b,&m_target.lat,&m_target.lon);
            consume(&m_target);
            m_maneuver_state = STARTING;
            war("Going to state STARTING");
          }
          else
          {
            m_maneuver_state = IDLE;
            war("Going to state IDLE");
          }
          stopExecution();
        }
      }

      // Start FollowReference maneuver
      void
      startExecution(void)
      {
        m_is_executing = true;
        IMC::PlanControl startPlan;
        startPlan.type = IMC::PlanControl::PC_REQUEST;
        startPlan.op = IMC::PlanControl::PC_START;
        startPlan.plan_id = "direct_drop";
        IMC::FollowReference man;
        man.control_ent = getEntityId();
        man.control_src = getSystemId();
        man.altitude_interval = 1.0;
        man.timeout = 0.0;

        IMC::PlanSpecification spec;

        spec.plan_id = "direct_drop";
        spec.start_man_id = "follow_drop";
        IMC::PlanManeuver pm;
        pm.data.set(man);
        pm.maneuver_id = "follow_drop";
        pm.start_actions = maneuverParameters();
        spec.maneuvers.push_back(pm);
        startPlan.arg.set(spec);
        startPlan.request_id = 0;
        startPlan.flags = 0;

        dispatch(startPlan);
      }

      //Set controller (path or waypoint)
      IMC::MessageList<IMC::Message>
      maneuverParameters()
      {
        IMC::MessageList<IMC::Message> setEntityParameters;

        IMC::SetEntityParameters* sep = new IMC::SetEntityParameters();
        sep->name = "Path Control";
        IMC::MessageList<IMC::EntityParameter> entityParameters;
        IMC::EntityParameter* ep = new IMC::EntityParameter();
        ep->name = "Use controller";
        ep->value = "true";
        entityParameters.push_back(*ep);
        delete ep;
        sep->params = entityParameters;
        setEntityParameters.push_back(*sep);
        delete sep;

        sep = new IMC::SetEntityParameters();
        sep->name = "Height Control";
        entityParameters.clear();
        ep = new IMC::EntityParameter();
        ep->name = "Active";
        ep->value = "true";
        entityParameters.push_back(*ep);
        delete ep;
        sep->params = entityParameters;
        setEntityParameters.push_back(*sep);
        delete sep;

        sep = new IMC::SetEntityParameters();
        sep->name = "Autopilot";
        entityParameters.clear();
        ep = new IMC::EntityParameter();
        ep->name = "Ardupilot Tracker";
        ep->value = "false";
        entityParameters.push_back(*ep);
        delete ep;

        ep = new IMC::EntityParameter();
        ep->name = "Formation Flight";
        ep->value = "false";
        entityParameters.push_back(*ep);
        delete ep;
        sep->params = entityParameters;
        setEntityParameters.push_back(*sep);
        delete sep;

        return setEntityParameters;
      }

      // Stop ongoing FollowReference maneuver
      void
      stopExecution( void )
      {
        IMC::PlanControl stopPlan;
        stopPlan.type = IMC::PlanControl::PC_REQUEST;
        stopPlan.op = IMC::PlanControl::PC_STOP;
        stopPlan.plan_id = "drop_plan";
        if(m_maneuver_state != IDLE)
          m_continue_after_break = true;
        dispatch(stopPlan);
        m_is_executing = false;
      }

      void change_to_FBWA( void )
      {
        IMC::ControlLoops clloop;
        clloop.enable = true;
        clloop.mask = IMC::CL_PITCH | IMC::CL_THROTTLE;
        dispatch(clloop);
      }

      void send_throttle_command(fp64_t value)
      {
        DesiredThrottle dthrottle;
        dthrottle.value = value;
        dispatch(dthrottle);
      }

      //Check if speed direction is parallell to the wind
      bool
      isParallelish(IMC::EstimatedState estate, IMC::EstimatedStreamVelocity ewind, fp64_t percentAccurate)
      {
        double stateAngle = atan2(estate.vx, estate.vy);
        double windAngle = atan2(ewind.x, ewind.y);
        double diff = (Angles::normalizeRadian(stateAngle - windAngle));

        if(abs(diff) > Math::c_pi * (1 - percentAccurate/100.0))
          return true;
        return false;
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
