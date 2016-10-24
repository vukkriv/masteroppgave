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
// Author: Siri Holthe Mathisen                                             *
//***************************************************************************

// DUNE headers.
#include "Beacon.hpp"
#include "Enums.hpp"

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
      double accepted_distance_to_start_point;
      fp64_t start_point_distance_from_carp;
      fp64_t end_point_distance_from_carp;
      fp64_t radius;
      double speed;
      fp32_t max_current;
      fp32_t glide_time;
      fp32_t drop_time;
      fp64_t drop_error;
      fp64_t percent_accurate;
      fp32_t release_height;
      uint16_t altitude_accuracy;
      uint16_t connection_timeout;
      uint16_t increments_input;
      std::string guidance_mode_input;
      uint16_t safe_height;
      bool bank;
      int counter_max;
      double dt;
      bool use_wind_est;
      double opt_circle;
      int opt_points;
      bool direct_to_opt;
      bool optimize_once;
      int opt_rate_inverse;
      double w_pos;
      double w_vel;
      bool repeated_testing;
      double OWSI_min_distance;
      double accepted_path_const_horizontal;
      double accepted_path_const_vertical;
      double accepted_speed_const;
      double accepted_course_const;
      double accepted_wind_difference;
    };

    struct Task: public DUNE::Tasks::Task
    {
      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.

      //Variables

      //True if initiation is done
      bool m_initiated;
      //True if wind measurement message is received
      bool m_wind_received;
      //True if target message is received
      bool m_target_received;
      //True while a follow reference plan is running
      bool m_follow_reference_is_running;
      //True while vehicle status is blocked
      bool m_isBlocked;
      //True while vehicle status is ready
      bool m_isReady;
      //True while motor is off
      bool m_isGliding;
      //True pre takeoff
      bool m_ground;
      //True until new estimated state message is processed
      bool m_new_EstimatedState;
      //True until new follow reference state message is processed
      bool m_new_FollowRefState;
      //True when long stretch end point is ready
      bool m_hasPoint;
      //Wind unit vector size in x and y
      double m_wind_unit_x, m_wind_unit_y;
      //The current current running through the motor
      double m_current;
      //Cross track error
      double m_ct_error;
      //LOS course error
      double m_course_error;
      //Height used for prev CARP optimatzion (OWSI)
      double m_opt_height;
      //Speed used for prev CARP optimatzion (OWSI)
      double m_opt_speed;
      //How many increments remain in guidance mode incremented long stretch
      int m_increments;
      //Wind estimation entity
      int m_wind_est_ent;
      //Wind estimation entity from ardupilot
      int m_ardupilot_wind_ent;
      //Fail or success when plan is finished
      uint8_t m_stop_type;
      //Previously measured distance to a point (CARP)
      fp64_t m_previous_distance;
      //Smallest distance measured to CARP
      fp64_t m_smallest_distance;
      //Time when glide started
      uint64_t m_t_start_glide;
      //Timer for drop reaction time
      uint64_t m_drop_reaction_time;
      //The GPS beacon to be dropped
      Beacon m_beacon;
      //The current state of the state machine
      States m_current_state;
      //The chosen guidance mode
      Modes m_guidance_mode;
      //Timer for gliding
      Clock timer;
      //End point
      Point m_end_point;
      //Start point
      Point m_start_point;
      //Counter for optimation rate
      int counter;
      //Weighing matrix for velocity
      Matrix m_W_vel;

      //Arguments from .ini-file
      Arguments m_args;

      //IMC messages
      IMC::Reference m_ref[LAST_STATE];
      IMC::PowerChannelControl m_pcc;
      IMC::Target m_target;
      IMC::EstimatedState m_estate;
      IMC::EstimatedStreamVelocity m_ewind;
      IMC::EstimatedStreamVelocity m_opt_wind;
      IMC::EstimatedStreamVelocity m_carp_ewind;
      IMC::FollowRefState m_follow_ref;
      IMC::DesiredSpeed m_dspeed;
      IMC::DesiredZ m_desired_height;
      IMC::Brake m_break;
      IMC::Distance m_drop_error_lat;
      IMC::Distance m_drop_error_lon;
      IMC::Distance m_drop_error_height;

      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx)
      {
        //Load parameters from .ini-file
        param("Altitude Interval", m_args.altitude_accuracy)
            .defaultValue("5")
            .units(Units::Meter)
            .description("Altitude following accuracy");

        param("Connection Timeout", m_args.connection_timeout)
        .defaultValue("10")
        .units(Units::Meter)
        .description("Response time before failure of followReference");

        param("Course Error Const", m_args.accepted_course_const)
        .defaultValue(".01")
        .description("Accepted course error / distance");

        param("Wind Difference Const", m_args.accepted_wind_difference)
        .defaultValue(".1")
        .description("Accepted wind error");

        param("Path Error Horizontal Const", m_args.accepted_path_const_horizontal)
        .defaultValue(".2")
        .units(Units::Meter)
        .description("Accepted horizontal path error / distance");

        param("Path Error Vertical Const", m_args.accepted_path_const_vertical)
        .defaultValue(".1")
        .units(Units::Meter)
        .description("Accepted vertical path error / distance");

        param("Speed Error Constant", m_args.accepted_speed_const)
        .defaultValue(".1")
        .units(Units::Meter)
        .description("Accepted speed error / distance");

        param("OWSI Min Distance", m_args.OWSI_min_distance)
        .defaultValue("5")
        .units(Units::Meter)
        .description("Closest distance to CARP for optimation with OWSI");

        param("Direct To Optimal", m_args.direct_to_opt)
        .defaultValue("10")
        .units(Units::Meter)
        .description("True to go directly to target");

        param("Max Drop Current", m_args.max_current)
        .defaultValue("1")
        .description("Maximum motor current allowed for drop");

        param("Glide Time", m_args.glide_time)
        .defaultValue("1")
        .description("Glide time needed to stop the motor");

        param("Drop Time", m_args.drop_time)
        .defaultValue(".6")
        .description("Glide time needed to stop the motor");

        param("Step Size", m_args.dt)
        .defaultValue(".01")
        .description("Step size for CARP calculation");

        param("Max Counter", m_args.counter_max)
        .defaultValue("10000")
        .description("Max counter for simulation");

        param("Distance To LSSP", m_args.accepted_distance_to_start_point)
        .defaultValue("15")
        .units(Units::Meter)
        .description("Acceptance circle around long stretch start point");

        param("End Point Distance From CARP", m_args.end_point_distance_from_carp)
        .defaultValue("100")
        .units(Units::Meter)
        .description("Distance from CARP to LSEP");

        param("Start Point Distance From CARP", m_args.start_point_distance_from_carp)
        .defaultValue("100")
        .units(Units::Meter)
        .description("Distance from CARP to LSSP");

        param("Percent Accurate", m_args.percent_accurate)
        .defaultValue("7")
        .units(Units::Meter)
        .description("Percent Accurate turning");

        param("Optimize Rads", m_args.opt_circle)
        .defaultValue("3.1416")
        .description("How many radians to use during optimation");

        param("Optimize Points", m_args.opt_points)
        .defaultValue("10")
        .description("How many points to check during optimation");

        param("Radius", m_args.radius)
        .defaultValue("100")
        .units(Units::Meter)
        .description("Radius of the turns");

        param("Release Height", m_args.release_height)
        .defaultValue("50")
        .units(Units::Meter)
        .description("Drop release height");

        param("Speed", m_args.speed)
        .defaultValue("17")
        .units(Units::Meter)
        .description("Speed during mission");

        param("Drop Error", m_args.drop_error)
        .defaultValue("20")
        .units(Units::Meter)
        .description("Drop within circle of this radius");

        param("Safe Height", m_args.safe_height)
        .defaultValue("100")
        .units(Units::Meter)
        .description("High enough to be safe");

        param("Increments", m_args.increments_input)
        .defaultValue("4")
        .description("Increments for INCREMENTED_LONG_STRETCH");

        param("Guidance Mode", m_args.guidance_mode_input)
        .values("INCREMENTED_LONG_STRETCH, OPTIMAL_LONG_STRETCH, LONG_STRETCH, OWSI")
        .description("Guidance mode for mission");

        param("Use Bank Controller", m_args.bank)
        .defaultValue("false")
        .description("Controller mode for mission");

        param("Use Wind Estimator", m_args.use_wind_est)
        .defaultValue("false")
        .description("Use new wind estimator");

        param("Optimize Once", m_args.optimize_once)
        .defaultValue("true")
        .description("Optimize once (true) or continously (false)");

        param("Repeated Testing", m_args.repeated_testing) //Write into report
        .defaultValue("false")
        .description("Test repeatedly on same target");


        param("Optimation Rate Inverse", m_args.opt_rate_inverse)
        .defaultValue("1")
        .description("Inverse of the optimation rate");

        param("Position Weight", m_args.w_pos)
        .defaultValue("10")
        .description("Weighing of position during optimation");

        param("Velocity Weight", m_args.w_vel)
        .defaultValue("10")
        .description("Weighing of velocity during optimation");


        //IMC messages to receive
        bind<Target>(this);
        bind<EstimatedStreamVelocity>(this);
        bind<EstimatedState>(this);
        bind<FollowRefState>(this);
        bind<PlanControlState>(this);
        bind<PlanControl>(this);
        bind<VehicleMedium>(this);
        bind<Current>(this);
        bind<EntityList>(this);
        bind<PathControlState>(this);
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        //Initiate variables
        m_initiated = false;
        m_target_received = false;
        m_wind_received = false;
        m_follow_reference_is_running = false;
        m_isBlocked = false;
        m_isReady = false;
        m_ground = true;
        m_new_EstimatedState = false;
        m_new_FollowRefState = false;
        m_hasPoint = false;
        m_isGliding = false;
        m_current_state = IDLE;
        m_beacon = Beacon();
        m_increments = m_args.increments_input;
        m_previous_distance = INT_MAX; //A lot.
        m_smallest_distance = INT_MAX; //A lot
        m_current = 250;
        m_stop_type = IMC::PlanControl::PC_REQUEST;
        m_pcc.name = "drop";
        m_wind_est_ent = -1;
        counter = 0;
        m_ct_error = 0;
        m_opt_height = m_args.release_height;
        m_opt_speed = m_args.speed;

        m_guidance_mode = guidance_mode_string_to_enum((char*)m_args.guidance_mode_input.c_str());

        double W_vel[] = {m_args.w_vel, m_args.w_vel, m_args.w_vel};
        m_W_vel = Matrix(W_vel, 1, 3);
      }

      //! Reserve entity identifiers.
      void
      onEntityReservation(void)
      {
        m_drop_error_lat.setSourceEntity(reserveEntity("Drop error lat"));
        m_drop_error_lon.setSourceEntity(reserveEntity("Drop error lon"));
        m_drop_error_height.setSourceEntity(reserveEntity("Drop error height"));
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

      //Stopping motor and starting glide
      void
      startGlide()
      {
        m_break.op=IMC::Brake::OP_START;
        dispatch(m_break);
        m_isGliding = true;
        inf("GLIDING");
        m_previous_distance = INT_MAX;
        m_t_start_glide = timer.getMsec();
      }

      //Starting glide mode
      void
      goToGlideMode()
      {
        m_t_start_glide = timer.getMsec();
        m_hasPoint = false;
        //IMPLEMENT LATER: m_stop_type = IMC::PlanControl::PC_SUCCESS;
        m_current_state = GLIDE_MODE;
        m_previous_distance = INT_MAX;
      }

      //Guidance strategy consisting of going to a start point and then turn against the wind towards the CARP
      void
      guidanceLongStretch(void)
      {
        if(!m_hasPoint)
        {
          m_hasPoint = true;
          makePointFromCarp(GOING_TO_DROP_POINT, true, false, m_wind_unit_x, m_wind_unit_y, 1, m_target.z + m_args.release_height);
        }

        if(m_new_EstimatedState)
        {
          m_new_EstimatedState = false;
          double lat, lon, height, distance_to_point = INT_MAX;
          getCurrentLatLonHeight(&lat, &lon, &height);

          if(!m_isGliding)
          {
            distance_to_point = distanceByTime(lat, lon, height, m_args.glide_time + m_args.drop_time);

            if(isCloseToPoint(distance_to_point))
            {
              startGlide();
            }
            if(!m_isGliding){
              m_previous_distance = distance_to_point;
            }
          }

          else
          {
            distance_to_point = distanceByTime(lat, lon, height, m_args.drop_time);

            if(isCloseToPoint(distance_to_point) && motorShutDownTimeHasRun())
            {
              drop(distance_to_point,lat, lon, height);
              goToGlideMode();
            }
            else
            {
              if(distance_to_point < m_smallest_distance)
              {
                m_smallest_distance = distance_to_point;
              }

              m_previous_distance = distance_to_point;
            }
          }
          checkForFailure();
        }
      }

      void
      updateOpt(double height)
      {
        m_opt_wind = m_ewind;
        m_hasPoint = true;
        double dt;
        uint64_t start = timer.getMsec();
        m_beacon.optimal_CARP(height, m_ewind, m_estate, m_args.dt,
                              m_args.counter_max, m_args.opt_circle, m_args.opt_points, m_W_vel, m_args.w_pos, m_args.glide_time);
        dt = timer.getMsec() - start;
        war("TIME FOR OPT: %f sec", dt);

        double vx_unit = m_beacon.get_CARP().vx / sqrt(m_beacon.get_CARP().vx * m_beacon.get_CARP().vx + m_beacon.get_CARP().vy * m_beacon.get_CARP().vy);
        double vy_unit = m_beacon.get_CARP().vy / sqrt(m_beacon.get_CARP().vx * m_beacon.get_CARP().vx + m_beacon.get_CARP().vy * m_beacon.get_CARP().vy);

        makeStraightLineOverCarp(vx_unit,  vy_unit);
      }

      //Guidance strategy consisting of going to a start point and then turn against the wind towards the CARP but using incremented waypoints to get there
      void
      guidanceIncrementedLongStretch(void)
      {
        if(!m_hasPoint)
        {
          m_hasPoint = true;
          makePointFromCarp(GOING_TO_DROP_POINT, true, false, -m_wind_unit_x, m_wind_unit_y, 1, m_target.z + m_args.release_height);
          dispatch_reference();
        }

        if(m_new_EstimatedState)
        {
          //Init vars and distances
          double lat = m_estate.lat, lon = m_estate.lon, height = m_estate.height, distance_to_point = INT_MAX, distance_to_carp = INT_MAX, distance_to_end = INT_MAX;
          WGS84::displace(m_estate.x, m_estate.y, m_estate.height, &lat, &lon, &height);
          distance_to_carp = WGS84::distance(m_beacon.get_CARP().lat, m_beacon.get_CARP().lon, 0, lat,  lon, 0);
          distance_to_point = WGS84::distance(m_ref[GOING_TO_DROP_POINT].lat, m_ref[GOING_TO_DROP_POINT].lon, 0, lat,  lon, 0);
          distance_to_end = WGS84::distance(m_end_point.lat, m_end_point.lon, 0, lat,  lon, 0);

          if(distance_to_point < m_args.start_point_distance_from_carp/m_args.increments_input ) //TEST AND PUT IN INI FILE
            m_hasPoint = false;

          if(distance_to_carp < m_smallest_distance)
          {
            m_smallest_distance = distance_to_carp;
          }

          //check for carp prox
          if(((distance_to_carp < m_args.drop_error) && distance_to_carp > m_previous_distance) || distance_to_carp < 1)
          {
            //IMPLEMENT NEW DROP LATER
            oldDrop();
            m_hasPoint = false;
            //m_stop_type = IMC::PlanControl::PC_SUCCESS;
            m_current_state = GOING_TO_SAFE_HEIGHT;
          }

          //Check if near end point of long stretch
          if(distance_to_end < m_args.end_point_distance_from_carp/5)
          {
            war("FAILURE: CARP missed! Was as close as: %f", m_smallest_distance);

            m_smallest_distance = INT_MAX;
            m_previous_distance = INT_MAX;

            m_hasPoint = false;
            //m_stop_type = IMC::PlanControl::PC_FAILURE;
            m_current_state = GOING_TO_SAFE_HEIGHT;
          }
          m_previous_distance = distance_to_carp;
        }
      }

      //Guidance strategy consisting of repeatedly optimizing the CARP and the straight line above it
      void
      guidanceOptLongStretch(void)
      {
        if(m_new_EstimatedState)
        {
          m_new_EstimatedState = false;
          double lat, lon, height, distance_to_point = INT_MAX;
          getCurrentLatLonHeight(&lat, &lon, &height);

          if(!m_isGliding)
          {
            if((!m_hasPoint || !m_args.optimize_once) && (counter%m_args.opt_rate_inverse == 0))
            {
              updateOpt(height);
            }

            distance_to_point = distanceByTime(lat, lon, height, m_args.glide_time + m_args.drop_time);

            if((distance_to_point < m_args.drop_error && distance_to_point > m_previous_distance) || distance_to_point < 1)
            {
              startGlide();
            }
            else
            {
              m_previous_distance = distance_to_point;
            }
          }
          else
          {
            distance_to_point = distanceByTime(lat, lon, height, m_args.drop_time);

            if(isCloseToPoint(distance_to_point) && motorShutDownTimeHasRun())
            {
              drop(distance_to_point,lat, lon, height);;
              goToGlideMode();
            }
            m_previous_distance = distance_to_point;
          }

          if(distance_to_point < m_smallest_distance && m_isGliding)
          {
            m_smallest_distance = distance_to_point;
          }
        }
        counter++;
        checkForFailure();
      }

      //To check whether success is improbable
      bool
      OWSI_update(double distance, double height, double speed){
        return (distance > m_args.OWSI_min_distance) && (sqrt(abs((m_ewind.x - m_opt_wind.x) * (m_ewind.x - m_opt_wind.x) + (m_ewind.y - m_opt_wind.y) * (m_ewind.y - m_opt_wind.y) + (m_ewind.z - m_opt_wind.z) * (m_ewind.z - m_opt_wind.z))) > m_args.accepted_wind_difference
            || (abs(m_ct_error) / distance > m_args.accepted_path_const_horizontal)
            || (abs(height - m_opt_height) / distance > m_args.accepted_path_const_vertical)
            || (abs(speed - m_opt_speed) / distance > m_args.accepted_speed_const)
            || (abs(m_course_error) / distance > m_args.accepted_course_const));//ENDRE DETTE I RAPPORTEN
      }

      //Guidance strategy consisting of optimizing the CARP and the straight line above it if success is improbable
      void
      guidanceOWSI(void)
      {
        if(m_new_EstimatedState)
        {
          m_new_EstimatedState = false;
          double lat, lon, height, distance_to_point = INT_MAX, speed = sqrt(m_estate.vy * m_estate.vy + m_estate.vx * m_estate.vx);
          getCurrentLatLonHeight(&lat, &lon, &height);

          if(!m_isGliding)
          {
            if(OWSI_update(WGS84::distance(m_beacon.get_CARP().lat, m_beacon.get_CARP().lon, 0, lat,  lon, 0), height, speed))
            {
              updateOpt(height);
              m_opt_height = m_beacon.get_CARP().z;
              m_opt_speed = speed;
            }

            distance_to_point = distanceByTime(lat, lon, height, m_args.glide_time + m_args.drop_time);

            if((distance_to_point < m_args.drop_error && distance_to_point > m_previous_distance) || distance_to_point < 1)
            {
              startGlide();
            }
            else
            {
              m_previous_distance = distance_to_point;
            }
          }
          else
          {
            distance_to_point = distanceByTime(lat, lon, height, m_args.drop_time);

            if(isCloseToPoint(distance_to_point) && motorShutDownTimeHasRun())
            {
              drop(distance_to_point,lat, lon, height);;
              goToGlideMode();
            }
            m_previous_distance = distance_to_point;
          }

          if(distance_to_point < m_smallest_distance && m_isGliding)
          {
            m_smallest_distance = distance_to_point;
          }
        }
        counter++;
        checkForFailure();
      }

      //Guidance state machine
      void
      runGuidanceStateMachine(void)
      {
        switch(m_guidance_mode)
        {
          case LONG_STRETCH:
            guidanceLongStretch();
            break;

          case INCREMENTED_LONG_STRETCH:
            guidanceIncrementedLongStretch();
            break;

          case OPTIMAL_LONG_STRETCH:
            guidanceOptLongStretch();
            break;
          case OWSI:
            guidanceOWSI();
            break;
          default:
            war("Current guidance law not available");
            break;
        }
      }

      //Final mode of state machine
      void
      goToSafeHeight(void)
      {
        if(!m_hasPoint)
        {
          m_hasPoint = true;
          inf("Going to safe height!");
          makePointFromCarp(GOING_TO_SAFE_HEIGHT, true, false, 0,0,100, m_target.z + m_args.safe_height);

          dispatch_reference();
        }

        if(m_new_FollowRefState)
        {
          m_new_FollowRefState = false;
          if (m_follow_ref.proximity & IMC::FollowRefState::PROX_Z_NEAR)
          {
            m_current_state = IDLE;
            if(!m_args.repeated_testing){
              stopExecution(m_stop_type);
            }
            if(m_args.repeated_testing){
              consume(&m_target);
            }
          }
        }
      }

      //State where the UAV goes to the start point
      void
      goToStartPoint(void)
      {
        double lat, lon, height;
        getCurrentLatLonHeight(&lat, &lon, &height);
        //check speed vector vs carp wind vector and closeness to start point
        if(m_new_FollowRefState)
        {
          m_new_FollowRefState = false;
          if ((m_follow_ref.proximity & IMC::FollowRefState::PROX_XY_NEAR)
              && (m_follow_ref.proximity & IMC::FollowRefState::PROX_Z_NEAR) &&
              WGS84::distance(m_start_point.lat, m_start_point.lon, 0, lat, lon, 0) < m_args.accepted_distance_to_start_point)
          {
            if(isParallelish(m_estate, m_carp_ewind, m_args.percent_accurate))
            {
              m_current_state = GOING_TO_DROP_POINT;
              m_follow_ref.proximity = 0;
              m_ref[m_current_state].setTimeStamp();
            }
          }
        }
      }

      //Glide mode for dropping with engine off
      void
      runGlideMode(void)
      {
        if((timer.getMsec() - m_t_start_glide) > 1000 * m_args.glide_time)
        {
          inf("GLIDE MODE finished");
          m_current_state = GOING_TO_SAFE_HEIGHT;
          m_break.op=IMC::Brake::OP_STOP;
          dispatch(m_break);
          m_isGliding = false;
        }
      }

      //Main state machine
      void
      runStateMachine(void)
      {
        if(m_follow_reference_is_running)
        {
          switch(m_current_state)
          {
            case GOING_TO_START_POINT:
              goToStartPoint();
              break;
            case GOING_TO_DROP_POINT:
              runGuidanceStateMachine();
              break;
            case GLIDE_MODE:
              runGlideMode();
              break;
            case GOING_TO_SAFE_HEIGHT:
              goToSafeHeight();
              break;
            default:
              war("Current reference point out of bounds");
              return;

          }
        }
      }

      //Consume target and calculate first CARP and start point
      void
      consume(const IMC::Target* msg)
      {
        if(m_initiated){
          inf("Target received!");

          //Make sure to stop all actions if target is received in mission
          m_break.op=IMC::Brake::OP_STOP;
          dispatch(m_break);
          m_isGliding = false;
          m_smallest_distance = INT_MAX;
          m_previous_distance = INT_MAX;
          m_hasPoint = false;

          //Save wind for non-optimized guidance mode
          m_carp_ewind = m_ewind;
          m_wind_unit_x = m_ewind.x / sqrt(m_ewind.x * m_ewind.x + m_ewind.y * m_ewind.y);
          m_wind_unit_y = m_ewind.y / sqrt(m_ewind.x * m_ewind.x + m_ewind.y * m_ewind.y);

          //Save target
          m_target = *msg;
          m_target_received = true;

          //Set target in Beacon class
          m_beacon.set_target(m_target);
          inf("Target set to: %f %f %f", m_beacon.get_target().lat, m_beacon.get_target().lon, m_beacon.get_target().z);

          //Calculate CARP using wind speed to determine direction (this point is used to calculate start point for long stretch and used as CARP in non-optimized guidance mode)
          m_beacon.calculate_CARP_velocity(-m_wind_unit_x * m_args.speed, -m_wind_unit_y * m_args.speed, 0, m_args.release_height + m_target.z, m_carp_ewind, m_args.dt, m_args.counter_max);
          inf("CARP has been calculated!");
          inf("CARP: lat: %f, lon: %f, height: %f", m_beacon.get_CARP().lat, m_beacon.get_CARP().lon, m_beacon.get_CARP().z);

          //Reference for loitering start point
          m_desired_height.z_units = IMC::Z_HEIGHT;
          makePointFromCarp(GOING_TO_START_POINT, true, false, -m_wind_unit_x, -m_wind_unit_y, m_args.radius, m_args.release_height + m_target.z);

          //Start point of long stretch
          m_start_point.lat = m_ref[GOING_TO_START_POINT].lat;
          m_start_point.lon = m_ref[GOING_TO_START_POINT].lon;
          m_start_point.z = m_ref[GOING_TO_START_POINT].z.get()->value;

          //Displace the start point one radius to the side
          WGS84::displace(
              m_args.radius * m_wind_unit_y,
              -1 * m_args.radius * m_wind_unit_x ,
              &m_ref[GOING_TO_START_POINT].lat, &m_ref[GOING_TO_START_POINT].lon);

          //Mission end point
          if(m_guidance_mode == INCREMENTED_LONG_STRETCH){
            m_end_point.z = m_args.release_height;
            m_end_point.lat = m_beacon.get_CARP().lat;
            m_end_point.lon = m_beacon.get_CARP().lon;

            //Displace away from CARP
            WGS84::displace(
                - m_args.end_point_distance_from_carp * m_wind_unit_x,
                - m_args.end_point_distance_from_carp * m_wind_unit_y,
                &m_end_point.lat, &m_end_point.lon);
          }
          if(m_args.direct_to_opt && (m_guidance_mode == OPTIMAL_LONG_STRETCH || m_guidance_mode == OWSI))
            m_current_state = GOING_TO_DROP_POINT;
          else
          {
            m_current_state = GOING_TO_START_POINT;
            inf("Start point has been calculated!");

            inf("Going to start point");
          }
          startExecution();
          dispatch_reference();
        }
        else
        {
          war("New target received and ignored");
        }
      }

      //Check if in air
      void
      consume(const IMC::VehicleMedium* vm)
      {
        m_ground = (vm->medium == IMC::VehicleMedium::VM_GROUND);
      }

      //Check control state
      void
      consume(const IMC::PlanControlState* msg)
      {
        if(msg->state == PlanControlState::PCS_READY && m_isBlocked )
        {
          m_isReady = true;
          m_isBlocked = false;
        }
        else if(msg->state == PlanControlState::PCS_BLOCKED){
          m_isBlocked = true;
        }
      }

      //Get estimated state
      void
      consume(const IMC::EstimatedState* msg)
      {
        m_estate = *msg;
        m_new_EstimatedState = true;

        if(m_wind_received && m_isReady && !m_ground && !m_initiated)
        {
          //m_initiated on first received EstimatedState and Estimated wind and not blocked and wind received
          m_initiated = true;
          setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
          war("Ready for target coordinates");
        }
        runStateMachine();
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
          m_wind_received = true;
        }
      }

      //Check how the drop mechanism is doing
      void
      consume(const IMC::PowerChannelState* msg)
      {
        if(msg->state == IMC::PowerChannelState::PCS_ON)
        {
          m_pcc.op = 0;
          dispatch(m_pcc);
          uint64_t dt = timer.getMsec() - m_drop_reaction_time;
          err("Delay from drop signal until dropped: %f", dt);
        }
      }

      //Get follow reference state
      void
      consume(const IMC::FollowRefState* msg)
      {
        m_follow_ref = *msg;
        m_new_FollowRefState = true;
        m_follow_ref.setTimeStamp();

        runStateMachine();

        dispatch_reference();
      }

      //Check plan control state
      void
      consume(const IMC::PlanControl* msg)
      {
        if (msg->op == IMC::PlanControl::PC_STOP)
        {
          inf("Stop request received");
          m_follow_reference_is_running = false;//Remove from rest of code?
        }
      }

      //Get battery current
      void
      consume(const IMC::Current* msg)
      {
        m_current = msg->value;
      }

      void
      consume(const IMC::PathControlState* msg){
        m_ct_error = msg->y;
        m_course_error = msg->course_error;
      }

      // Start FollowReference maneuver
      void
      startExecution(void)
      {
        if(!m_follow_reference_is_running)
        {
          inf("Starting followReference plan...");
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

          m_follow_reference_is_running = true;
          dispatch(startPlan);
        }
      }

      // Stop ongoing FollowReference maneuver
      void
      stopExecution(uint8_t stop_type)
      {
        if(m_follow_reference_is_running)
        {
          inf("Stopping drop plan...");
          IMC::PlanControl stopPlan;
          stopPlan.type = stop_type;
          stopPlan.op = IMC::PlanControl::PC_STOP;
          stopPlan.plan_id = "drop_plan";

          m_follow_reference_is_running = false;
          m_hasPoint = false;
          m_increments = m_args.increments_input;

          dispatch(stopPlan);
        }
      }

      //Make an enum from string
      Modes
      guidance_mode_string_to_enum(char* guidance_mode_input)
      {
        if(strcmp("INCREMENTED_LONG_STRETCH" , guidance_mode_input) == 0)
          return INCREMENTED_LONG_STRETCH;
        else if(strcmp("OPTIMAL_LONG_STRETCH" , guidance_mode_input) == 0)
          return OPTIMAL_LONG_STRETCH;
        else if(strcmp("LONG_STRETCH" , guidance_mode_input) == 0)
          return LONG_STRETCH;
        else if(OWSI)
          return OWSI;
        return  LONG_STRETCH;
      }

      //Check if speed direction is parallell to the wind
      bool
      isParallelish(IMC::EstimatedState estate, IMC::EstimatedStreamVelocity ewind, fp64_t percentAccurate)
      {
        double stateAngle = atan2(estate.vx, estate.vy);
        double windAngle = atan2(ewind.x, ewind.y);

        double diff = (Angles::normalizeRadian(stateAngle - windAngle));

        //inf("State angle is: %f and wind angle is %f", stateAngle, windAngle);
        //inf("Difference is: %f and has to be bigger than %f", diff, Math::c_pi * (1 - percentAccurate/100.0));
        if(abs(diff) > Math::c_pi * (1 - percentAccurate/100.0))
          return true;

        return false;
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

      //Drop without checking if motor is off
      void
      oldDrop(void)
      {
        m_pcc.op = 1;
        dispatch(m_pcc);
        //Wait before sending 0? TEST IT!
        m_pcc.op = 0;
        dispatch(m_pcc);
      }

      //Drop beacon
      void
      drop(double distance_to_point, double lat, double lon, double height)
      {
        if(m_current < m_args.max_current and m_isGliding)
        {
          inf("Current is OK!");
          m_pcc.name = "drop";
          m_pcc.op = 1;

          m_drop_reaction_time = timer.getMsec();
          dispatch(m_pcc);

//          WGS84::displace(m_estate.vx*m_args.drop_time, m_estate.vy*m_args.drop_time, m_estate.vz*m_args.drop_time, &lat, &lon, &height);
//          double x1=0, y1=0, z1=0, x2=0, y2=0, z2=0;
//          WGS84::toECEF(lat, lon, height, &x1, &y1, &z1); //ECEF position in drop_time seconds
//          WGS84::toECEF(m_beacon.get_CARP().lat, m_beacon.get_CARP().lon, m_beacon.get_CARP().z, &x2, &y2, &z2);
//
//          double rotz[] = {cos(m_estate.psi), -sin(-m_estate.psi), 0, sin(-m_estate.psi), cos(-m_estate.psi), 0, 0, 0, 1};
//          double pos[] = {x1 - x2, y1 - y2, z1 - z2};
//          Matrix Pos = Matrix(rotz, 3, 3) * Matrix(pos,3,1);
//
//          inf("Dropped %f meters away from optimum!", distance_to_point);
//          inf("at %f %f %f meters away from optimum in body!", Pos.element(0,0),Pos.element(1,0),Pos.element(2,0));
//          inf("With a speed vector in [N: %f], [E: %f], [D: %f] away from optimum", m_estate.vx - m_beacon.get_CARP().vx, m_estate.vy - m_beacon.get_CARP().vy, m_estate.vz - m_beacon.get_CARP().vz);
//
//          m_drop_error_lat.value = Pos.element(0,0);
//          m_drop_error_lon.value = Pos.element(1,0);
//          m_drop_error_height.value = Pos.element(2,0);
//          dispatch(m_drop_error_lat);
//          dispatch(m_drop_error_lon);
//          dispatch(m_drop_error_height);
        }
        else{
          err("Current is NOT OK!");
        }
      }

      //Make a point depending on CARP
      void
      makePointFromCarp(int state, bool directFlag, bool startFlag, double x_unit_speed, double y_unit_speed, double radius, double height){
        m_ref[state].radius = radius; //Small radius to make UAV go straight: DANGER! DANGER! //20 is minimum
        m_ref[state].lat = m_beacon.get_CARP().lat;
        m_ref[state].lon = m_beacon.get_CARP().lon;

        m_desired_height.value = height;
        m_ref[state].speed.set(m_dspeed);
        m_ref[state].z.set(m_desired_height);

        if(directFlag){
          m_ref[state].flags = Reference::FLAG_LOCATION | Reference::FLAG_RADIUS | Reference::FLAG_Z  | Reference::FLAG_DIRECT;

          WGS84::displace(
              - m_args.end_point_distance_from_carp * x_unit_speed,
              - m_args.end_point_distance_from_carp * y_unit_speed,
              &m_ref[state].lat, &m_ref[state].lon);
        }
        else if(startFlag){
          m_ref[state].flags = Reference::FLAG_LOCATION | Reference::FLAG_RADIUS | Reference::FLAG_Z  | Reference::FLAG_START_POINT;

          WGS84::displace(
              m_args.start_point_distance_from_carp * x_unit_speed,
              m_args.start_point_distance_from_carp * y_unit_speed,
              &m_ref[state].lat, &m_ref[state].lon);
        }
        else{
          m_ref[state].flags = Reference::FLAG_LOCATION | Reference::FLAG_RADIUS | Reference::FLAG_Z;

          WGS84::displace(
              - m_args.end_point_distance_from_carp * x_unit_speed,
              - m_args.end_point_distance_from_carp * y_unit_speed,
              &m_ref[state].lat, &m_ref[state].lon);
        }
      }

      //Get current lat lon and height
      void
      getCurrentLatLonHeight(fp64_t *lat, fp64_t *lon, fp64_t *height){
        *lat = m_estate.lat, *lon = m_estate.lon, *height = m_estate.height;
        WGS84::displace(m_estate.x, m_estate.y, m_estate.z, lat, lon, height);
      }

      //Check how far you are from the CARP in a certain number of seconds
      double
      distanceByTime(double lat, double lon, double height, double time){
        WGS84::displace( //Displacing forward (drop time) times speed because of delay
            m_estate.vx * (time),
            m_estate.vy * (time),
            m_estate.vz * (time),    //m_estate can be replaced with m_beacon.get_CARP.vx etc.
            &lat, &lon, &height);

        return WGS84::distance(m_beacon.get_CARP().lat, m_beacon.get_CARP().lon, (double)m_beacon.get_CARP().z, lat,  lon, height);
      }

      //Make two points over the CARP together making a line
      void
      makeStraightLineOverCarp(double vx_unit, double vy_unit){

        //For testing
        //vx_unit = 0;
        //vy_unit = 0;

        //Make and dispatch point A in a line for LOS to follow
        makePointFromCarp(GOING_TO_DROP_POINT, false, true, -vx_unit, -vy_unit, 0, m_target.z + m_args.release_height);
        dispatch_reference();

        //Make and dispatch point B in a line for LOS to follow
        makePointFromCarp(GOING_TO_DROP_POINT, false, false, -vx_unit, -vy_unit, 0, m_target.z + m_args.release_height);
        dispatch_reference();
      }

      //Check if you are close to the CARP
      bool
      isCloseToPoint(double distance_to_point){
        return (((distance_to_point < m_args.drop_error) && (distance_to_point > m_previous_distance)) || distance_to_point < m_args.speed/(10));
      }

      //Check if the motor has been off for a certain time
      bool
      motorShutDownTimeHasRun(){
        return (m_t_start_glide - timer.getMsec()) > m_args.glide_time * 1000;
      }

      //Check if you have passed the CARP
      void
      checkForFailure(){
        if(m_new_FollowRefState && (m_follow_ref.proximity & IMC::FollowRefState::PROX_XY_NEAR)
            && (m_follow_ref.proximity & IMC::FollowRefState::PROX_Z_NEAR)
            && (m_ref[m_current_state].getTimeStamp() + 10000) < m_follow_ref.getTimeStamp())
        {
          m_new_FollowRefState = false;
          war("FAILURE: CARP missed! Was as close as: %f", m_smallest_distance);

          m_break.op=IMC::Brake::OP_STOP;
          dispatch(m_break);
          m_isGliding = false;
          m_smallest_distance = INT_MAX;
          m_previous_distance = INT_MAX;
          m_t_start_glide = timer.getMsec(); //=0?

          //m_stop_type = IMC::PlanControl::PC_FAILURE;
          m_current_state = GOING_TO_SAFE_HEIGHT;
        }
        else
        {
          m_new_FollowRefState = false;
        }
      }

      //Dispatch current reference
      void
      dispatch_reference(void)
      {
        if(m_current_state > IDLE and m_current_state < LAST_STATE)
        {
          dispatch(m_ref[m_current_state]);
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
