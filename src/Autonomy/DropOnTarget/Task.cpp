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
// Author: Siri Holthe Mathisen                                             *
//***************************************************************************

// DUNE headers.
#include "Beacon.hpp"

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Autonomy
{
  namespace DropOnTarget
  {
    using DUNE_NAMESPACES;
    using namespace std;

    struct Arguments
    {
      fp64_t accepted_distance_to_start_point;
      fp64_t distance_circle_to_CARP;
      fp64_t radius;
      fp64_t speed;
      fp32_t max_current;
      fp32_t glide_time;
      fp32_t drop_time;
      fp64_t drop_error;
      fp64_t percent_accurate;
      fp32_t release_height;
      fp64_t w_pos;
      fp64_t w_vel;
      fp64_t opt_circle;
      fp64_t dt;
      fp64_t altitude_accuracy;
      fp64_t connection_timeout;
      fp64_t safe_height;
      int counter_max;
      int opt_points;
      int opt_rate_inverse;
      std::string guidance_mode_input;
      bool bank;
      bool use_wind_est;
      bool simulation;
    };

    // The enums for the state machine that controls the drop maneuvre.
    enum States
    {
      IDLE=-1,
      STARTING,
      GOING_TO_START_POINT,
      GOING_TO_DROP_POINT,
      GOING_TO_SAFE_HEIGHT,
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
      //Check if we close to the present reference
      bool m_is_close_to_ref;
      //Check if we want to continue with the drop
      bool m_continue_state_machine;
      //Check if plan is running
      bool m_is_executing;
      //True if initiation is done
      bool m_is_initiated;
      //True while motor is off
      bool m_is_motor_off;
      //Check if motor shutdown counter has started
      bool m_is_motor_shutdown_counting;
      //True pre takeoff
      bool m_is_on_ground;
      //True while vehicle status is ready
      bool m_is_ready;
      //True if wind measurement message is received
      bool m_is_wind_received;

      //The current current running through the motor
      fp64_t m_current;
      //Previously measured distance to a point (CARP)
      fp64_t m_previous_distance;
      //Wind unit vector size in x and y
      fp64_t m_wind_unit_x, m_wind_unit_y;
      //Timer for drop reaction time
      uint64_t m_t_drop_reaction;
      //Time when glide started
      uint64_t m_t_shut_down_motor;
      //Time when guide mode ended
      uint64_t m_t_guided_started;


      //Wind estimation entity from ardupilot
      int m_ardupilot_wind_ent;
      //Counter for optimation rate
      int m_counter;
      //Wind estimation entity
      int m_wind_est_ent;
      //Random Number generator
      Random::Generator* m_gen;
      //The current state of the state machine
      States m_current_state;

      //The GPS beacon to be dropped
      Beacon m_beacon;
      //Timer for gliding
      Clock m_timer;
      //Weighing matrix for velocity
      Matrix m_W_vel;
      //End point
      SimpleState m_end_point;
      //Start point
      SimpleState m_start_point;
      //RTK Position
      SimpleState m_RTK_state;
      //Current Position
      SimpleState m_GPS_state;
      // Autopilot mode
      string m_autopilotmode;

      //Arguments from .ini-file
      Arguments m_args;

      //IMC messages
      IMC::Reference                m_ref;
      IMC::PowerChannelControl      m_pcc;
      IMC::Target                   m_target;
      IMC::EstimatedState           m_estate;
      IMC::EstimatedStreamVelocity  m_ewind;
      IMC::EstimatedStreamVelocity  m_carp_ewind;
      IMC::DesiredSpeed             m_dspeed;
      IMC::DesiredZ                 m_dheight;
      IMC::DesiredThrottle          m_dthrottle;
      IMC::Brake                    m_break;

      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_is_close_to_ref(false),
        m_continue_state_machine(false),
        m_is_executing(false),
        m_is_initiated(false),
        m_is_motor_off(false),
        m_is_motor_shutdown_counting(false),
        m_is_on_ground(false),
        m_is_ready(false),
        m_is_wind_received(false),
        m_current(250.0),
        m_previous_distance(INT_MAX),
        m_wind_unit_x(1.0),
        m_wind_unit_y(0.0),
        m_t_drop_reaction(0.0),
        m_t_shut_down_motor(0.0),
        m_t_guided_started(0.0),
        m_ardupilot_wind_ent(-1),
        m_counter(0),
        m_wind_est_ent(-1),
        m_gen(),
        m_current_state(IDLE)
      {

        //Load parameters from .ini-file

        param("Acceptance Proximity", m_args.accepted_distance_to_start_point)
        .defaultValue("15")
        .units(Units::Meter)
        .description("Acceptance circle around long stretch start point");

        param("Distance From Circle TO CARP", m_args.distance_circle_to_CARP)
        .defaultValue("100")
        .units(Units::Meter)
        .description("Distance From Dubins Path Circle TO CARP");

        param("Turn Radius", m_args.radius)
        .defaultValue("100")
        .units(Units::Meter)
        .description("Turn Radius of the UAV");

        param("Speed", m_args.speed)
        .defaultValue("17")
        .units(Units::MeterPerSecond)
        .description("Speed during mission");

        param("Max Drop Current", m_args.max_current)
        .defaultValue("1")
        .description("Maximum motor current allowed for drop");

        param("Glide Time", m_args.glide_time)
        .defaultValue("1")
        .description("Glide time needed to stop the motor");

        param("Drop Time", m_args.drop_time)
        .defaultValue(".6")
        .units(Units::Second)
        .description("Glide time needed to stop the motor");

        param("Drop Error", m_args.drop_error)
        .defaultValue("20")
        .units(Units::Meter)
        .description("Drop within circle of this radius");

        param("Percent Accurate", m_args.percent_accurate)
        .defaultValue("7")
        .units(Units::Meter)
        .description("Percent Accurate turning");

        param("Release Height", m_args.release_height)
        .defaultValue("50")
        .units(Units::Meter)
        .description("Drop release height");

        param("Position Weight", m_args.w_pos)
        .defaultValue("10")
        .description("Weighing of position during optimation");

        param("Velocity Weight", m_args.w_vel)
        .defaultValue("10")
        .description("Weighing of velocity during optimation");

        param("Optimize Rads", m_args.opt_circle)
        .defaultValue("3.1416")
        .description("How many radians to use during optimation");

        param("Time Step", m_args.dt)
        .defaultValue(".01")
        .units(Units::Second)
        .description("Time Step size for CARP calculation");

        param("Altitude Interval", m_args.altitude_accuracy)
        .defaultValue("5")
        .units(Units::Meter)
        .description("Altitude following accuracy");

        param("Connection Timeout", m_args.connection_timeout)
        .defaultValue("0.0")
        .units(Units::Second)
        .description("Response time before failure of followReference");

        param("Safe Height", m_args.safe_height)
        .defaultValue("100")
        .units(Units::Meter)
        .description("High enough to be safe");

        param("Max Counter", m_args.counter_max)
        .defaultValue("10000")
        .description("Max counter for simulation");

        param("Optimize Points", m_args.opt_points)
        .defaultValue("10")
        .description("How many points to check during optimation");

        param("Optimization Rate Inverse", m_args.opt_rate_inverse)
        .defaultValue("1")
        .description("Inverse of the optimation rate");

        param("Guidance Mode", m_args.guidance_mode_input)
        .values("OPTIMAL_LONG_STRETCH, LONG_STRETCH, OWSI")
        .description("Guidance mode for mission");

        param("Use Bank Controller", m_args.bank)
        .defaultValue("false")
        .description("Controller mode for mission");

        param("Use Wind Estimator", m_args.use_wind_est)
        .defaultValue("false")
        .description("Use new wind estimator");

        param("Simulation", m_args.simulation) //Write into report
        .defaultValue("false")
        .description("Test repeatedly on same target");

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
        bind<GpsFixRtk>(this);
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        m_pcc.name = "drop";
        m_timer = Clock();
        fp64_t W_vel[] = {m_args.w_vel, m_args.w_vel, m_args.w_vel};
        m_W_vel = Matrix(W_vel, 1, 3);
        m_autopilotmode = "MANUAL";
        m_gen = Random::Factory::create(Random::Factory::c_default, -1);
        m_beacon = Beacon(m_args.opt_circle,m_args.opt_points,m_W_vel,m_args.w_pos,m_args.glide_time,m_args.drop_time,m_args.dt);
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
        m_estate = *msg;
        m_GPS_state.lat = msg->lat;
        m_GPS_state.lon = msg->lon;
        m_GPS_state.z = msg->height;
        m_GPS_state.vx = msg->vx;
        m_GPS_state.vy = msg->vy;
        m_GPS_state.vz = msg->vz;
        WGS84::displace(msg->x,msg->y,msg->z,&m_GPS_state.lat,&m_GPS_state.lon,&m_GPS_state.z);
        if(m_is_wind_received && m_is_ready && !m_is_on_ground && !m_is_initiated)
        {
          m_is_initiated = true;
          war("Ready for target coordinates");
        }
        runStateMachine();
      }
      void
      consume(const IMC::GpsFixRtk* msg)
      {
        m_RTK_state.lat = msg->base_lat;
        m_RTK_state.lon = msg->base_lon;
        m_RTK_state.z = msg->base_height;
        m_RTK_state.vx = msg->v_n;
        m_RTK_state.vy = msg->v_e;
        m_RTK_state.vz = msg->v_d;
        WGS84::displace(msg->n,msg->e,msg->d,&m_RTK_state.lat,&m_RTK_state.lon,&m_RTK_state.z);
      }

      //Get usefull entities
      void
      consume(const IMC::EntityList* msg)
      {
        if(m_wind_est_ent == -1)
        {
          TupleList tup(msg->list, "=", ";", true);
          m_wind_est_ent = tup.get("Wind Estimator", -1);
          m_ardupilot_wind_ent = tup.get("Autopilot", -1);
        }
      }

      //Get wind from estimator or ardupilot
      void
      consume(const IMC::EstimatedStreamVelocity* msg)
      {
        if((msg->getSourceEntity() == m_wind_est_ent && m_args.use_wind_est) || (!m_args.use_wind_est && msg->getSourceEntity() == m_ardupilot_wind_ent))
        {
          m_ewind = *msg;
          m_is_wind_received = true;
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
          fp32_t dt = m_timer.getMsec() - m_t_drop_reaction;
          war("Time it took to drop the payload: %f", dt);
        }
      }

      void
      consume(const IMC::AutopilotMode* msg)
      {
        m_autopilotmode = msg->mode;
        if(m_autopilotmode == "GUIDED" && m_args.simulation)
        {
          int time_since_guided_started = m_timer.getMsec() - m_t_guided_started;
          if(time_since_guided_started > 5000)
          {
            stopExecution();
            m_continue_state_machine = true;
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
          if(m_continue_state_machine)
          {
            war("StoppedExecution! And restarted...");
            m_current_state = GOING_TO_START_POINT;
            m_continue_state_machine = false;
//            startExecution();
          }else
          {
            m_current_state = IDLE;
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
        m_is_on_ground = (vm->medium == IMC::VehicleMedium::VM_GROUND);
        if(m_is_on_ground)
          err("m_ground = true!");
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
        if(m_is_initiated){
          inf("Target received!");

          //Make sure to stop all actions if target is received in mission
          m_break.op=IMC::Brake::OP_STOP;
          dispatch(m_break);
          m_previous_distance = INT_MAX;

          //Save wind for non-optimized guidance mode
          m_carp_ewind = m_ewind;
          m_wind_unit_x = m_ewind.x / sqrt(m_ewind.x * m_ewind.x + m_ewind.y * m_ewind.y);
          m_wind_unit_y = m_ewind.y / sqrt(m_ewind.x * m_ewind.x + m_ewind.y * m_ewind.y);

          //Save target
          m_target = *msg;

          //Set target in Beacon class
          m_beacon.set_target(m_target);
          inf("Target set to: %f %f %f", m_target.lat, m_target.lon, m_target.z);

          //Calculate CARP using wind speed to determine direction (this point is used to calculate start point for long stretch and used as CARP in non-optimized guidance mode)
          m_beacon.calculate_CARP(-m_wind_unit_x * m_args.speed, -m_wind_unit_y * m_args.speed, 0, m_args.release_height+m_target.z, m_ewind);
//          inf("CARP has been calculated!");
//          inf("CARP: lat: %f, lon: %f, height: %f", m_beacon.get_CARP().lat, m_beacon.get_CARP().lon, m_beacon.get_CARP().z);

          //Reference for loitering start point
          m_dheight.z_units = IMC::Z_HEIGHT;
          m_ref.radius = m_args.radius;
          m_ref.lat = m_beacon.get_CARP().lat;
          m_ref.lon = m_beacon.get_CARP().lon;
          m_dheight.value = m_args.safe_height + m_target.z;
          m_ref.z.set(m_dheight);
          m_dspeed.value = m_args.speed;
          m_ref.speed.set(m_dspeed);
          m_ref.flags = Reference::FLAG_DIRECT | Reference::FLAG_LOCATION | Reference::FLAG_RADIUS | Reference::FLAG_SPEED | Reference::FLAG_Z;
          WGS84::displace(m_args.distance_circle_to_CARP*m_wind_unit_x,m_args.distance_circle_to_CARP*m_wind_unit_y,&m_ref.lat,&m_ref.lon);

          //Start point of long stretch
          m_start_point.lat = m_ref.lat;
          m_start_point.lon = m_ref.lon;
          m_start_point.z = m_ref.z.get()->value;

          //Displace the start point one radius to the side
          WGS84::displace(m_args.radius * m_wind_unit_y,-1 * m_args.radius * m_wind_unit_x ,&m_ref.lat, &m_ref.lon);

          m_current_state = STARTING;
          if(!m_is_executing)
          {
            startExecution();
          }
          dispatch(m_ref);
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
//        if(m_autopilotmode != "FBWB")
//        {
//          err("INFO");
//          cout << "Current State: "<< m_current_state << endl;
//          cout << "Autopilot mode: " << m_autopilotmode << endl;
//          cout << "Is executing: " << m_is_executing << endl;
//        }
//        inf("Drop time: %f, glide time: %f",m_args.drop_time, m_args.glide_time);
//        war("State: %d", m_current_state);
        switch(m_current_state)
        {
          case IDLE:
            break;
          case STARTING:
            startManeuver();
            break;
          case GOING_TO_START_POINT:
            goToStartPoint();
            break;
          case GOING_TO_DROP_POINT:
            guidance();
            break;
          case GLIDE_BEFORE_DROP:
            glideBeforeDrop();
            break;
          case GLIDE_AFTER_DROP:
            glideAfterDrop();
            break;
          case GOING_TO_SAFE_HEIGHT:
            goToSafeHeight();
            break;
          default:
            return;
        }
      }

      void
      startManeuver( void )
      {
        if(m_autopilotmode == "GUIDED")
        {
          war("Guided. State: %d", m_current_state);
        }
        if(m_autopilotmode != "FBWB")
        {
          dispatch(m_ref);
          return;
        }

        else
        {
          m_current_state = GOING_TO_START_POINT;
        }
      }
      // UAV Follows a truncated Dubins Path
      void
      goToStartPoint(void)
      {
        if(m_autopilotmode == "GUIDED")
        {
          war("Guided. State: %d", m_current_state);
        }
        if(m_autopilotmode != "FBWB")
        {
          consume(&m_target);
          return;
        }

        //check speed vector vs carp wind vector and closeness to start point
        if (WGS84::distance(m_start_point.lat, m_start_point.lon, 0, m_GPS_state.lat, m_GPS_state.lon, 0) < m_args.accepted_distance_to_start_point)
        {
          if(isParallelish(m_estate, m_carp_ewind, m_args.percent_accurate))
          {
            m_current_state = GOING_TO_DROP_POINT;
            setLongStretchReference();
          }
        }
      }

      // UAV flies directly to the CARP, following different optimization strategies
      void guidance( void )
      {
        if(m_autopilotmode == "GUIDED")
        {
          war("Guided. State: %d", m_current_state);
        }
        fp32_t distance_to_point = INT_MAX;
        distance_to_point = distanceByTime(m_GPS_state.lat, m_GPS_state.lon, m_GPS_state.z, m_args.glide_time + m_args.drop_time);
        fp64_t smallest_distance = distance_to_point;

        if("LONG_STRETCH" == m_args.guidance_mode_input)
        {

        }
        else if("OPTIMAL_LONG_STRETCH" == m_args.guidance_mode_input)
        {
          if(//distance_to_point < m_args.distance_circle_to_CARP/2 &&
              m_counter%m_args.opt_rate_inverse == 0)
          {
            updateOptimalCarp(m_GPS_state.z);
          }
        }
        else if("OWSI"== m_args.guidance_mode_input)
        {
          if(distance_to_point < 2*m_args.distance_circle_to_CARP/3 &&
              m_counter%m_args.opt_rate_inverse == 0)
          {
            if(m_beacon.estimated_carp_error(m_ewind, m_estate) > 0.5)
              updateOptimalCarp(m_GPS_state.z);
          }
        }
        m_counter++;
//        inf("Distance to point: %f. Max. Drop Error: %f", distance_to_point, m_args.drop_error);
        if((distance_to_point < m_args.drop_error && distance_to_point > m_previous_distance) || distance_to_point < 1.0)
        {
          m_counter = 0;
          IMC::ControlLoops clloop;
          clloop.enable = true;
          clloop.mask = IMC::CL_PITCH | IMC::CL_THROTTLE;
          dispatch(clloop);
          m_t_shut_down_motor = m_timer.getMsec();
          m_dthrottle.value = 0.0;
          dispatch(m_dthrottle);
          m_current_state = GLIDE_BEFORE_DROP;
        }else if(distance_to_point > m_args.drop_error && distance_to_point > smallest_distance + 10)
        {
          err("Missed!");
          m_current_state = IDLE;
          consume(&m_target);
        }
        m_previous_distance = distance_to_point;
        if(distance_to_point < smallest_distance)
          smallest_distance = distance_to_point;
      }


      // Before dropping the payload, the motor has to be turned off
      void glideBeforeDrop( void )
      {
        //Make sure motor is turned off
        dispatch(m_dthrottle);
        int time_since_motor_shutdown = m_timer.getMsec() - m_t_shut_down_motor;
        if(m_current < m_args.max_current && (time_since_motor_shutdown > (m_args.glide_time*1000 - 50)))
        {
          drop();
          m_current_state = GLIDE_AFTER_DROP;
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

        m_t_drop_reaction = m_timer.getMsec();
        dispatch(m_pcc);
        fp32_t target_dev[3];
        fp32_t carp_dev[3];
        fp32_t target_dist;
        fp32_t carp_dist;

        war("This is a simple drop!");
        fp32_t CARP_target[3];
        WGS84::displacement(m_target.lat,m_target.lon,m_target.z,m_beacon.get_CARP().lat,m_beacon.get_CARP().lon,m_beacon.get_CARP().z,
            &CARP_target[0],&CARP_target[1],&CARP_target[2]);
        fp32_t GSPpos_target[3];
        WGS84::displacement(m_target.lat,m_target.lon,m_target.z,m_GPS_state.lat,m_GPS_state.lon,m_GPS_state.z,
            &GSPpos_target[0],&GSPpos_target[1],&GSPpos_target[2]);
        fp32_t RTKpos_target[3];
        WGS84::displacement(m_target.lat,m_target.lon,m_target.z,m_RTK_state.lat,m_RTK_state.lon,m_RTK_state.z,
            &RTKpos_target[0],&RTKpos_target[1],&RTKpos_target[2]);
        war("CARP displacement from target: %f %f %f",CARP_target[0],CARP_target[1],CARP_target[2]);
        war("GPS position displacement from target: %f %f %f",GSPpos_target[0],GSPpos_target[1],GSPpos_target[2]);
        war("RTK position displacement from target: %f %f %f",RTKpos_target[0],RTKpos_target[1],RTKpos_target[2]);

        m_beacon.set_release_point(m_estate);
        m_beacon.calculate_estimated_hitpoint(m_estate,m_ewind);
        m_beacon.load_CARP_error(carp_dev);
        m_beacon.load_target_error(target_dev);
        target_dist = m_beacon.get_target_error();
        carp_dist = m_beacon.get_CARP_error();

        inf("Target distance: %f \t CARP distance: %f", target_dist, carp_dist);

        if(m_args.simulation)
        {
            ofstream myfile;
            myfile.open("/home/siri/uavlab/results/results.txt",std::ios::app);
            myfile << m_ewind.x << "\t" << m_ewind.y << "\t" << m_ewind.z << "\t";
            myfile << m_estate.vx << "\t" << m_estate.vy << "\t" << m_estate.vz << "\t";
            myfile << target_dist << "\t";
            myfile << target_dev[0] << "\t" << target_dev[1] << "\t" << target_dev[2] << "\t";
            myfile << carp_dist << "\t";
            myfile << carp_dev[0] << "\t" << carp_dev[1] << "\t" << carp_dev[2] << "\t"  << "\n\n";
            myfile.close();
        }
      }

      // After the drop the UAV has to fly for a while with the motor turned off
      void
      glideAfterDrop(void)
      {
        if(m_autopilotmode == "GUIDED")
        {
          war("Guided. State: %d", m_current_state);
        }
        int time_since_drop = m_timer.getMsec() - m_t_drop_reaction;
        if(time_since_drop > 1000 * m_args.glide_time)
        {
          m_current_state = GOING_TO_SAFE_HEIGHT;
          m_is_motor_off = false;
          m_is_motor_shutdown_counting = false;
//          stopExecution();
          //Safe Height Reference
          m_ref.lat = m_beacon.get_CARP().lat;
          m_ref.lon = m_beacon.get_CARP().lon;
          m_dspeed.value = m_args.speed;
          m_dheight.value = m_target.z + m_args.safe_height;
          m_ref.radius = 0.0;
          m_ref.z.set(m_dheight);
          m_ref.speed.set(m_dspeed);
          m_ref.flags = Reference::FLAG_LOCATION | Reference::FLAG_DIRECT | Reference::FLAG_Z | Reference::FLAG_SPEED | Reference::FLAG_RADIUS;
          double vx_unit = m_beacon.get_CARP().vx / sqrt(m_beacon.get_CARP().vx * m_beacon.get_CARP().vx + m_beacon.get_CARP().vy * m_beacon.get_CARP().vy);
          double vy_unit = m_beacon.get_CARP().vy / sqrt(m_beacon.get_CARP().vx * m_beacon.get_CARP().vx + m_beacon.get_CARP().vy * m_beacon.get_CARP().vy);
          WGS84::displace(0.5*m_args.distance_circle_to_CARP*vx_unit,0.5*m_args.distance_circle_to_CARP*vy_unit, &m_ref.lat,&m_ref.lon);
          dispatch(m_ref);
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

        if(m_autopilotmode == "GUIDED")
        {
          war("Guided. State: %d", m_current_state);
        }
        dispatch(m_ref);
        fp32_t distance_to_ref = WGS84::distance(m_GPS_state.lat,m_GPS_state.lon,0.0,m_ref.lat,m_ref.lon,0.0);
        if(distance_to_ref < 50)
        {
          if(m_args.simulation)
          {
            fp32_t point_a, point_b;
            point_a = m_gen->uniform(-200.0,200.0);
            point_b = m_gen->uniform(-200.0,200.0);
            WGS84::displace(point_a,point_b,&m_target.lat,&m_target.lon);
            consume(&m_target);
            m_current_state = GOING_TO_START_POINT;
          }
          else
          {
            m_current_state = IDLE;
          }
          stopExecution();
        }
      }

      void
      setLongStretchReference( void )
      {
        m_beacon.optimal_CARP(m_GPS_state.z, m_ewind, m_estate);

        m_ref.radius = 0.0; //Small radius to make UAV go straight: DANGER! DANGER! //20 is minimum
        m_ref.lat = m_beacon.get_CARP().lat;
        m_ref.lon = m_beacon.get_CARP().lon;

        m_dheight.value = m_args.release_height + m_target.z;
        m_dspeed.value = m_args.speed;
        m_ref.speed.set(m_dspeed);
        m_ref.z.set(m_dheight);

        m_ref.flags = Reference::FLAG_DIRECT | Reference::FLAG_SPEED | Reference::FLAG_LOCATION | Reference::FLAG_Z | Reference::FLAG_RADIUS;

        fp32_t disp[3];
        WGS84::displacement(m_GPS_state.lat,m_GPS_state.lon,m_GPS_state.z,
            m_beacon.get_CARP().lat,m_beacon.get_CARP().lon,m_beacon.get_CARP().z,&disp[0],&disp[1],&disp[2]);
        WGS84::displace(0.5*disp[0],0.5*disp[1], &m_ref.lat,&m_ref.lon);
        dispatch(m_ref);

      }

      void
      updateOptimalCarp(double height)
      {
        m_beacon.optimal_CARP(height, m_ewind, m_estate);

        double vx_unit = m_beacon.get_CARP().vx / sqrt(m_beacon.get_CARP().vx * m_beacon.get_CARP().vx + m_beacon.get_CARP().vy * m_beacon.get_CARP().vy);
        double vy_unit = m_beacon.get_CARP().vy / sqrt(m_beacon.get_CARP().vx * m_beacon.get_CARP().vx + m_beacon.get_CARP().vy * m_beacon.get_CARP().vy);

        m_ref.radius = 0.0; //Small radius to make UAV go straight: DANGER! DANGER! //20 is minimum
        m_ref.lat = m_beacon.get_CARP().lat;
        m_ref.lon = m_beacon.get_CARP().lon;

        m_dheight.value = m_args.release_height + m_target.z;
        m_dspeed.value = m_args.speed;
        m_ref.speed.set(m_dspeed);
        m_ref.z.set(m_dheight);

        m_ref.flags = Reference::FLAG_DIRECT | Reference::FLAG_SPEED | Reference::FLAG_LOCATION | Reference::FLAG_Z | Reference::FLAG_RADIUS;

        WGS84::displace(0.5*m_args.distance_circle_to_CARP*vx_unit,0.5*m_args.distance_circle_to_CARP*vy_unit, &m_ref.lat,&m_ref.lon);
        dispatch(m_ref);

      }

      // Start FollowReference maneuver
      void
      startExecution(void)
      {
        m_is_executing = true;
        IMC::PlanControl startPlan;
        startPlan.type = IMC::PlanControl::PC_REQUEST;
        startPlan.op = IMC::PlanControl::PC_START;
        startPlan.plan_id = "drop_plan";
        IMC::FollowReference man;
        man.control_ent = getEntityId();
        man.control_src = getSystemId();
        man.altitude_interval = m_args.altitude_accuracy;
        man.timeout = m_args.connection_timeout;

        IMC::PlanSpecification spec;
        spec.plan_id = "drop_plan";
        spec.start_man_id = "follow_drop";
        IMC::PlanManeuver pm;
        pm.data.set(man);
        pm.maneuver_id = "follow_drop";
        pm.start_actions = maneuverParameters(m_args.bank);
        spec.maneuvers.push_back(pm);
        startPlan.arg.set(spec);
        startPlan.request_id = 0;
        startPlan.flags = 0;
        dispatch(startPlan);
      }

      //Set controller (path or waypoint)
      IMC::MessageList<IMC::Message>
      maneuverParameters(bool bank)
      {
        IMC::MessageList<IMC::Message> setEntityParameters;

        IMC::SetEntityParameters* sep = new IMC::SetEntityParameters();
        sep->name = "Path Control";
        IMC::MessageList<IMC::EntityParameter> entityParameters;
        IMC::EntityParameter* ep = new IMC::EntityParameter();
        ep->name = "Use controller";
        ep->value = bank ? "true" : "false";
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
        ep->value = bank ? "true" : "false";
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
        ep->value = bank ? "false" : "true";
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
        if(m_current_state != IDLE)
          m_continue_state_machine = true;
        dispatch(stopPlan);
        m_is_executing = false;
      }

      //Check how far you are from the CARP in a certain number of seconds
      double
      distanceByTime(double lat, double lon, double height, double time){
        WGS84::displace( //Displacing forward (drop time) times speed because of delay
            (m_estate.vx) * (time),
            (m_estate.vy)* (time),
            (m_estate.vz)* (time),    //m_estate can be replaced with m_beacon.get_CARP.vx etc.
            &lat, &lon, &height);

        return WGS84::distance(m_beacon.get_CARP().lat, m_beacon.get_CARP().lon, (double)m_beacon.get_CARP().z, lat,  lon, height);
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
