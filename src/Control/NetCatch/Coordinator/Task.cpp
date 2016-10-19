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

// USER headers.
#include <USER/DUNE.hpp>

#include <vector>
#include <queue>

namespace Control
{
  namespace NetCatch
  {
    namespace Coordinator
    {
      using DUNE_NAMESPACES;
      //! %Task arguments.
      struct Arguments
      {
        //! Enable coordinated catch
        bool enable_coord;
        //! Enable catch-state
        bool enable_catch;
        //! Stop and hold at end of runway
        bool enable_stop_endRunway;
        //! Enable this path controller or not
        bool use_controller;
        //! Enable mean window for aircraft states
        bool use_mean_window_aircraft;
        //! Disable Z flag, this will utilize new rate controller on some targets
        bool disable_Z;

        // Max vel approach
        double max_vy_app;
        // Max pos cross-track approach
        double max_py_app;
        // Max pos along-track approach
        double max_px_app;
        //! Moving mean window size
        double mean_ws;
        //! Desired velocity of net at impact
        double m_ud_impact;
        //! Desired time to accelerate to desired velocity of net at impact
        double m_td_acc;
        //! Desired offset-cross track
        double m_crosstrack_offset;
        //! Desired collision radius
        double m_coll_r;
        //! Desired radius to confirm recovery
        double m_coll_eps;
        //! Radius to stop at end of runway
        double m_endCatch_radius;

        //!position Controller parameters
        Matrix Kp;
        Matrix Ki;
        Matrix Kd;

        double max_norm_v;
        double max_integral;

        //! Maximum cross-track error aircraft
        Matrix eps_ct_a;
        //! Maximum cross-track error net
        Matrix eps_ct_n;

        //! Ref-model paramters
        bool refmodel_use;
        double refmodel_w0;
        double refmodel_xi;
        double refmodel_max_v;
        double refmodel_max_a;

        std::string centroid_els_entity_label;

        float print_frequency;
      };

      static const std::string c_desired_names[] = {"Reference","Desired"};
      enum DesiredEntites
      {
        D_REFERENCE = 0,
        D_DESIRED = 1
      };
      static const int NUM_DESIRED = 2;

      struct VirtualRunway
      {
        fp64_t lat_start, lon_start, height_start;
        fp64_t lat_end, lon_end, height_end;
        double z_off_a;

        fp32_t box_height, box_width, box_length;

        //! Course of runway
        double alpha;
        //! Pitch of runway
        double theta;

        Matrix start_NED;
        Matrix end_NED;
      };

      struct ControlParam
      {
        Matrix Kp;
        Matrix Ki;
        Matrix Kd;
      };
      struct Vehicles
      {
        unsigned int no_vehicles;
        std::string aircraft;
        std::vector<std::string> copters;
      };

      enum Vehicle
      {
        FIXEDWING = 0, CENTROID, INVALID = -1
      };

      class ReferenceModel
      {
      public:

        ReferenceModel():
          x(9,1, 0.0),
          k1(0.0),k2(0.0),k3(0.0)
      {
          /* Intentionally Empty */
      }
        Matrix
        getPos(void) { return x.get(0,2, 0,0); }

        Matrix
        getVel(void) { return x.get(3,5, 0,0); }

        Matrix
        getAcc(void) { return x.get(6,8, 0,0); }

        void
        setPos(Matrix& pos) { x.put(0,0, pos); }

        void
        setVel(Matrix& vel) { x.put(3,0, vel); }

        void
        setAcc(Matrix& acc) { x.put(6,0, acc); }



      public:
        Matrix x;
        double k1,k2,k3;
      };

      static const char * NetRecoveryLevelEnumStrings[] =
      { "INITIALIZE",
        "STANDBY",
        "APPROACH",
        "START",
        "CATCH",
        "END",
        "ABORT",
        "STOP"};

      static const std::string c_parcel_names[] = { "PID-ALONG-SURGE",   "PID-CROSS-Y",   "PID-CROSS-Z",
                                                    "ERROR-ALONG-SURGE", "ERROR-CROSS-Y", "ERROR-CROSS-Z",};
      enum Parcel
      {
        PC_PID_ALONG_SURGE = 0,
        PC_PID_CROSS_Y = 1,
        PC_PID_CROSS_Z = 2,
        PC_ERROR_ALONG_SURGE = 3,
        PC_ERROR_CROSS_Y = 4,
        PC_ERROR_CROSS_Z = 5
      };

      static const int NUM_PARCELS = 6;

      struct Task : public DUNE::Control::PeriodicUAVAutopilot
      {
        //! Task arguments.
        Arguments m_args;

        //! Current state
        IMC::NetRecoveryState::NetRecoveryLevelEnum m_curr_state;

        //! Reference and desired linear state, the reference message only sent for logging
        IMC::DesiredLinearState m_desired_linear[NUM_DESIRED];
        //! Reference and desired heading, the reference message only sent for logging
        IMC::DesiredHeading m_desired_heading[NUM_DESIRED];

        //! Current desired Z
        IMC::DesiredZ m_dz;

        //!position Controller parameters (3x3)
        Matrix Kp;
        Matrix Ki;
        Matrix Kd;

        IMC::ControlParcel m_parcels[NUM_PARCELS];

        VirtualRunway m_runway;
        Vehicles m_vehicles;
        ControlParam m_control;

        //! Localization origin (WGS-84)
        fp64_t m_ref_lat, m_ref_lon;
        fp32_t m_ref_hae;
        bool m_ref_valid;

        //! Ready to read state-messages
        bool m_coordinatorEnabled;
        //! Vehicles initialized
        std::vector<bool> m_initialized;
        //! Vehicles connected
        std::vector<bool> m_connected;
        //! All parameters well defined
        bool m_initializedCoord;

        //! Last EstimatedLocalState received
        std::vector<IMC::EstimatedLocalState> m_estate;

        //! Last positions in NED
        std::vector<Matrix> m_p;
        //! Last velocities in NED
        std::vector<Matrix> m_v;

        //! Cross track errors
        std::vector<Matrix> m_p_path;
        std::vector<std::queue<Matrix> > m_cross_track_window;
        std::vector<Matrix> m_p_path_mean;
        //! Cross track errors derivative
        std::vector<Matrix> m_v_path;
        std::vector<std::queue<Matrix> > m_cross_track_d_window;
        std::vector<Matrix> m_v_path_mean;

        //! Position difference along path. Aka difference between centroid net and fixed-wing.
        double m_delta_p_path_x;
        double m_delta_p_path_x_mean;
        //! Velocity difference along path
        double m_delta_v_path_x;
        double m_delta_v_path_x_mean;

        Matrix m_p_ref_path;
        Matrix m_v_ref_path;

        Matrix m_a_des_path;

        Matrix m_p_int_value;

        //! Radius to start net-catch (calculate based on net-acceleration)
        double m_startCatch_radius;

        //! Timeout for operation
        uint16_t m_timeout;
        //! Along-track velocity reference value
        fp32_t m_u_ref;
        //! Desired along-track velocity
        fp32_t m_ud;
        //! Desired along-track acceleration
        fp32_t m_ad;

        //! Control loops last reference
        uint32_t m_scope_ref;

        //! To print the control-loop frequency
        double m_time_end;

        //! Time between executions of the main control-loop.
        double m_time_diff;

        //! Reference model state
        ReferenceModel m_refmodel;

        //centroid heading
        double m_centroid_heading;

        //! Controllable loops.
        static const uint32_t c_controllable = IMC::CL_PATH;
        //! Required loops.
        static const uint32_t c_required = IMC::CL_SPEED;


        //! Constructor.
        //! @param[in] name task name.
        //! @param[in] ctx context.
        Task(const std::string& name, Tasks::Context& ctx):
          PeriodicUAVAutopilot(name, ctx, c_controllable, c_required),
          m_curr_state(IMC::NetRecoveryState::NR_INIT),
          m_ref_lat(0.0),
          m_ref_lon(0.0),
          m_ref_hae(0.0),
          m_ref_valid(false),
          m_coordinatorEnabled(false),
          m_initializedCoord(false),
          m_delta_p_path_x(0),
          m_delta_p_path_x_mean(0),
          m_delta_v_path_x(0),
          m_delta_v_path_x_mean(0),
          m_p_ref_path(3, 1, 0.0),
          m_v_ref_path(3, 1, 0.0),
          m_a_des_path(3, 1, 0.0),
          m_p_int_value(3, 1, 0.0),
          m_startCatch_radius(0),
          m_timeout(0),
          m_u_ref(0.0),
          m_ud(0.0),
          m_ad(0.0),
          m_scope_ref(0),
          m_time_end(Clock::get()),
          m_time_diff(0),
          m_centroid_heading(0)
        {
          param("Path Controller", m_args.use_controller)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .defaultValue("false")
          .description("Enable Path Controller");

          param("Offset cross-track", m_args.m_crosstrack_offset)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .defaultValue("0.0")
          .description("Cross-track offset, subtract the offset from the y-position of the airplane in the path frame");

          param("Stop at end-of-runway", m_args.enable_stop_endRunway)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .defaultValue("false")
          .description("Enable stop at end of runway");

          param("Enable Mean Window Aircraft", m_args.use_mean_window_aircraft)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .defaultValue("false")
          .description("Use mean window on aircraft states");

          param("Disable Z flag", m_args.disable_Z)
          .defaultValue("false")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Choose whether to disable Z flag. In turn, this will utilize new rate controller on some targets");

          param("Coordinated Catch", m_args.enable_coord)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .defaultValue("false")
          .description("Flag to enable net catch with two multicopters");

          param("Enable Catch", m_args.enable_catch)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .defaultValue("false")
          .description("Flag to enable catch state of the state-machine");

          param("Mean Window Size", m_args.mean_ws)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .defaultValue("1.0")
          .description("Number of samples in moving average window");

          param("Maximum Cross-Track Error Aircraft", m_args.eps_ct_a)
          .units(Units::Meter);

          param("Maximum Cross-Track Error Net", m_args.eps_ct_n)
          .units(Units::Meter);

          param("Max vel y approach", m_args.max_vy_app)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .defaultValue("1");

          param("Max pos y approach", m_args.max_py_app)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .defaultValue("1");

          param("Max pos x approach", m_args.max_px_app)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .defaultValue("100.0")
          .units(Units::Meter)
          .description("Desired fixed-wing point to start approach measured along-track from the start point");

          param("Desired collision radius", m_args.m_coll_r)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .defaultValue("100.0")
          .units(Units::Meter)
          .description("Desired collision point measured along-track from the start point");

          param("Radius at recovery", m_args.m_coll_eps)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .defaultValue("0.1")
          .units(Units::Meter)
          .description("Desired radius to confirm recovery");

          param("Desired switching radius", m_args.m_endCatch_radius)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .defaultValue("10.0")
          .units(Units::Meter);

          param("Kp Position Control", m_args.Kp)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .defaultValue("1.0,1.0,1.0")
          .description("Position Controller tuning parameter Kp");

          param("Ki Position Control", m_args.Ki)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .defaultValue("0.0,0.0,0.0")
          .description("Position Controller tuning parameter Ki");

          param("Kd Position Control", m_args.Kd)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .defaultValue("0.0,0.0,0.0")
          .description("Position Controller tuning parameter Kd");

          param("Maximum Normalised Velocity", m_args.max_norm_v)
          .defaultValue("5.0")
          .description("Maximum Normalised Velocity of the Copter");

          param("Max Integral", m_args.max_integral)
          .defaultValue("1.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Max integral value");

          param("EstimatedLocalState Entity Label", m_args.centroid_els_entity_label)
          .defaultValue("Formation Centroid")
          .description("Entity label for the centroid EstimatedLocalState");

          param("ReferenceModel -- Activate", m_args.refmodel_use)
          .defaultValue("true")
          .description("To use the refmodel for zy or not. ");

          param("ReferenceModel -- Natural Frequency", m_args.refmodel_w0)
          .defaultValue("1.0")
          .units(Units::RadianPerSecond)
          .description("Reference model natural frequency. ");

          param("ReferenceModel -- Damping", m_args.refmodel_xi)
          .defaultValue("1.1")
          .units(Units::None)
          .description("Damping factor of the reference model. ");

          param("ReferenceModel -- Speed", m_args.refmodel_max_v)
          .defaultValue("4.0")
          .units(Units::MeterPerSecond)
          .description("Nominal max speed of the reference model setpoint. ");

          param("ReferenceModel -- Acceleration", m_args.refmodel_max_a)
          .defaultValue("5.0")
          .units(Units::MeterPerSquareSecond)
          .description("Nominal maximum acceleration during reference model usage. ");

          param("Print Frequency", m_args.print_frequency)
          .defaultValue("0.0")
          .units(Units::Second)
          .description("Frequency of pos.data prints. Zero => Print on every update.");

          // Bind incoming IMC messages
          bind<IMC::EstimatedLocalState>(this);
          bind<IMC::DesiredNetRecoveryPath>(this);
        }

        //! Reserve entity identifiers.
        void
        onEntityReservation(void)
        {
          debug("Reserve entities");
          for (unsigned i = 0; i < NUM_PARCELS; ++i)
            m_parcels[i].setSourceEntity(reserveEntity(c_parcel_names[i] + " Parcel" + this->getEntityLabel()) );         
          for (unsigned i = 0; i < NUM_DESIRED; ++i)
          {
            m_desired_linear[i].setSourceEntity(reserveEntity(c_desired_names[i] + " Linear " + this->getEntityLabel()));
            debug("Entity label '%s' for DesiredLinearState reserved",resolveEntity(m_desired_linear[i].getSourceEntity()).c_str());
            m_desired_heading[i].setSourceEntity(reserveEntity(c_desired_names[i] + " Heading " + this->getEntityLabel()));
            debug("Entity label '%s' for DesiredHeading reserved",resolveEntity(m_desired_heading[i].getSourceEntity()).c_str());
          }
          debug("Entities reserved");
        }

        //! Initialize resources.
        void
        onResourceInitialization(void)
        {
          initCoordinator();
        }

        //! Update internal state with new parameter values.
        void
        onUpdateParameters(void)
        {
          m_control.Kp = Matrix(3);
          m_control.Ki = Matrix(3);
          m_control.Kd = Matrix(3);
          m_control.Kp(0,0) = m_args.Kp(0);
          m_control.Kp(1,1) = m_args.Kp(1);
          m_control.Kp(2,2) = m_args.Kp(2);

          m_control.Ki(0,0) = m_args.Ki(0);
          m_control.Ki(1,1) = m_args.Ki(1);
          m_control.Ki(2,2) = m_args.Ki(2);

          m_control.Kd(0,0) = m_args.Kd(0);
          m_control.Kd(1,1) = m_args.Kd(1);
          m_control.Kd(2,2) = m_args.Kd(2);

          // Set reference model parameters
          m_refmodel.k3 =  (2*m_args.refmodel_xi + 1) *     m_args.refmodel_w0;
          m_refmodel.k2 = ((2*m_args.refmodel_xi + 1) * std::pow(m_args.refmodel_w0, 2)) /  m_refmodel.k3;
          m_refmodel.k1 =                               std::pow(m_args.refmodel_w0, 3)  / (m_refmodel.k3 * m_refmodel.k2);
        }

        void
        consume(const IMC::DesiredNetRecoveryPath* msg)
        {

          if (!isActive())
          {
            err(DTR("not active"));
            return;
          }

          trace("Got DesiredNetRecovery \nfrom '%s' at '%s'",
                resolveEntity(msg->getSourceEntity()).c_str(),
                resolveSystemId(msg->getSource()));
          inf("Initialize net recovery maneuver");

          debug("Start (ll): [%f,%f]", msg->start_lat, msg->start_lon);
          debug("End (ll): [%f,%f]", msg->end_lat, msg->end_lon);
          debug("z: %f", msg->z);
          debug("z_off: %f", msg->z);
          debug("box [h x w]=[%f x %f]=", msg->lbox_height, msg->lbox_width);
          debug("Speed: %f", msg->speed);
          debug("Max acceleration: %f", msg->max_acc);
          debug("\n\n");

          m_dz.value = msg->z;
          m_dz.z_units = msg->z_units;

          m_vehicles.aircraft = msg->aircraft;
          m_vehicles.no_vehicles = 2; //aircraft and virtual vehicle "centroid"

          m_runway.lat_start = msg->start_lat;
          m_runway.lon_start = msg->start_lon;
          m_runway.z_off_a = msg->z_off;
          m_runway.lon_end = msg->end_lon;
          m_runway.lat_end = msg->end_lat;

          m_runway.box_height = msg->lbox_height;
          m_runway.box_width = msg->lbox_width;

          m_u_ref = msg->speed;
          m_ad = msg->max_acc;

          m_coordinatorEnabled = true;
          inf("Maneuver enabled");
        }

        void
        consume(const IMC::EstimatedLocalState* el)
        {
          //this one should read only fixed-wing and centroid states

          if (!m_coordinatorEnabled)
            return;

          //local centroid messages
          if (el->getSource() == this->getSystemId() &&
              resolveEntity(el->getSourceEntity()).c_str() == m_args.centroid_els_entity_label)
            m_centroid_heading = el->state->psi;

          int s = getVehicle(el);
          if (s == INVALID)  //invalid vehicle
            return;


          m_estate[s] = *el;
          m_connected[s] = true;

          Matrix p = Matrix(3, 1, 0); //position in NED
          Matrix v = Matrix(3, 1, 0); //velocity in NED

          p(0) = el->state->x;
          p(1) = el->state->y;
          p(2) = el->state->z;
          m_p[s] = p;

          v(0) = el->state->vx;
          v(1) = el->state->vy;
          v(2) = el->state->vz;
          m_v[s] = v;

          static double last_print = 0.0;
          double now = Clock::get();

          if (!allConnected())
          {
            if (!m_args.print_frequency || !last_print
                || (now - last_print) > 1.0 / m_args.print_frequency)
            {

              if (!m_connected[FIXEDWING])
                war("FixedWing '%s' not connected",m_vehicles.aircraft.c_str());
              else if (!m_connected[CENTROID])
                war("Centroid not connected");
              else
                war("FixedWing and Centroid is not connected");
              last_print = now;
            }
            return;
          }

          //! consider doing the following only when reference has changed (check values)
          m_ref_lat = el->state->lat;
          m_ref_lon = el->state->lon;
          m_ref_hae = el->state->height;
          m_ref_valid = true;

          updateRunwayPath();

          //spew("calcpath");
          calcPathErrors(p, v, s);
          //spew("updateMean");
          updateMeanValues(s);
          //spew("switch state");

          m_initialized[s] = true;

          IMC::NetRecoveryState::NetRecoveryLevelEnum last_state = m_curr_state;
          switch (m_curr_state)
          {
            case IMC::NetRecoveryState::NR_INIT:
              {

                if (allInitialized())
                {
                  m_curr_state = IMC::NetRecoveryState::NR_STANDBY;

                  /**********************
                   //!!!!DEBUG!!!
                   ***********************/
                  //m_curr_state = IMC::NetRecoveryState::NR_STOP;
                  inf("Initialized, at standby");
                }
                else
                {
                  if (!m_args.print_frequency || !last_print
                      || (now - last_print) > 1.0 / m_args.print_frequency)
                  {

                    if (!m_initialized[FIXEDWING])
                      war("FixedWing '%s' not initialized",m_vehicles.aircraft.c_str());
                    else if (!m_initialized[CENTROID])
                      war("Centroid not initialized");
                    else
                      war("FixedWing and Centroid is not initialized");
                    last_print = now;
                  }
                }
                break;
              }
            case IMC::NetRecoveryState::NR_STANDBY:
              {
                updateMeanValues(s);
                if (aircraftApproaching() && m_args.enable_catch)
                {
                  updateStartRadius();
                  if (!startNetRecovery()) //aircraft should not be too close when starting approach
                  {
                    inf("Aircraft approaching");
                    m_curr_state = IMC::NetRecoveryState::NR_APPROACH; //requires that the net is standby at the start of the runway
                  }
                  else
                  {
                    war("Not able to recover, plane to close");
                    m_curr_state = IMC::NetRecoveryState::NR_STOP;
                  }
                }
                break;
              }
            case IMC::NetRecoveryState::NR_APPROACH:
              {
                updateMeanValues(s);
                updateStartRadius();
                if (startNetRecovery())
                {
                  inf("Start NetRecovery");
                  m_curr_state = IMC::NetRecoveryState::NR_START;
                }
                break;
              }
            case IMC::NetRecoveryState::NR_START:
              {
                updateMeanValues(s);
                m_ud = getPathVelocity(0, m_u_ref, m_ad, false);

                if (catched())
                {
                  inf("Fixed-wing catched");
                  m_curr_state = IMC::NetRecoveryState::NR_CATCH;
                }
                else if (aircraftPassed())
                {
                  war("Fixed-wing passed");
                  m_curr_state = IMC::NetRecoveryState::NR_STOP;
                }
                if (endAtRunway())
                {
                  inf("End at runway");
                  m_curr_state = IMC::NetRecoveryState::NR_END;
                }
                break;
              }
            case IMC::NetRecoveryState::NR_CATCH:
              {
                if (endAtRunway())
                {
                  inf("End at runway");
                  m_curr_state = IMC::NetRecoveryState::NR_END;
                }
                break;
              }
            case IMC::NetRecoveryState::NR_END:
              {
                m_curr_state = IMC::NetRecoveryState::NR_STOP;
                break;
              }
            case IMC::NetRecoveryState::NR_STOP:
              {
                //signal the supervisor that the maneuver is done
                break;
              }
            case IMC::NetRecoveryState::NR_ABORT:
              {
                break;
              }
          }
          sendCurrentState();
          reportCurrentPositionDifference();

          if (last_state != m_curr_state && m_curr_state == IMC::NetRecoveryState::NR_STANDBY)
            resetStates();

          if (last_state != m_curr_state)
            inf("Current state: %s",
                NetRecoveryLevelEnumStrings[static_cast<NetRecoveryState::NetRecoveryLevelEnum>(m_curr_state)]);
        }

        void
        initCoordinator()
        {
          if (m_initializedCoord)
          {
            inf("Coordinator already initialized");
            return;
          }
          unsigned int no_vehicles = 2;

          m_estate = std::vector<IMC::EstimatedLocalState>(no_vehicles);
          m_p = std::vector<Matrix>(no_vehicles);
          m_v = std::vector<Matrix>(no_vehicles);

          m_p_path = std::vector<Matrix>(no_vehicles);
          m_v_path = std::vector<Matrix>(no_vehicles);
          m_p_path_mean = std::vector<Matrix>(no_vehicles);
          m_v_path_mean = std::vector<Matrix>(no_vehicles);
          m_cross_track_window = std::vector<std::queue<Matrix> >(no_vehicles);
          m_cross_track_d_window = std::vector<std::queue<Matrix> >(no_vehicles);
          m_initialized = std::vector<bool>(no_vehicles);
          m_connected = std::vector<bool>(no_vehicles);

          for (unsigned int i = 0; i < no_vehicles; i++)
          {
            m_p[i] = Matrix(3, 1, 0);
            m_v[i] = Matrix(3, 1, 0);
            m_p_path[i] = Matrix(3, 1, 0);
            m_v_path[i] = Matrix(3, 1, 0);
            m_p_path_mean[i] = Matrix(3, 1, 0);
            m_v_path_mean[i] = Matrix(3, 1, 0);
            debug("m_p_path[%d]: Rows: %d, Cols: %d", i, m_p_path[i].rows(),
                  m_p_path[i].columns());
            debug("m_v_path[%d]: Rows: %d, Cols: %d", i, m_v_path[i].rows(),
                  m_v_path[i].columns());
            debug("m_p_path_mean[%d]: Rows: %d, Cols: %d", i,
                  m_p_path_mean[i].rows(), m_p_path_mean[i].columns());
            debug("m_v_path_mean[%d]: Rows: %d, Cols: %d", i,
                  m_v_path_mean[i].rows(), m_v_path_mean[i].columns());
            m_cross_track_window[i] = std::queue<Matrix>();
            m_cross_track_d_window[i] = std::queue<Matrix>();
            for (unsigned int j = 0; j < m_args.mean_ws; j++)
            {
              m_cross_track_window[i].push(Matrix(3, 1, 0));
              m_cross_track_d_window[i].push(Matrix(3, 1, 0));
            }
            m_initialized[i] = false;
            m_connected[i] = false;
          }
          m_runway.end_NED = Matrix(3, 1, 0);
          m_runway.start_NED = Matrix(3, 1, 0);

          m_curr_state = IMC::NetRecoveryState::NR_INIT;
          m_initializedCoord = true;
          inf("Coordinator initialized");
        }

        void
        initRefModel()
        {
          // Initializes and zeros the refmodel state to the current centroid IN PATH FRAME

          m_refmodel.x = Matrix(9, 1, 0.0);

          // Check if we have valid centroid pos
          if (m_connected[CENTROID])
          {
            inf("Reference model reset. ");
            Matrix p_net = getNetPosition(m_p_path);
            m_refmodel.x(0) = p_net(0);
            m_refmodel.x(1) = p_net(1);
            m_refmodel.x(2) = p_net(2);

            Matrix v_net = getNetVelocity(m_v_path);
            m_refmodel.x(3) = v_net(0);
            m_refmodel.x(4) = v_net(1);
            m_refmodel.x(5) = v_net(2);
          }
          else
          {
            err("Setting reference model state to zero, not connected to vehicle. ");
          }
        }

        //! Steps the reference model
        //! NB: Since the controller works in path frame,
        //!     the reference model expects the desiredPos to be in path-frame as well.
        void
        stepRefModel(Matrix desiredPos, double deltat)
        {

          if (desiredPos.size() < 3)
          {
            err("Invalid size of desiredPosition. ");
            return;
          }


          Matrix x_d = Matrix(3, 1, 0.0);
          x_d(0) = desiredPos(0);
          x_d(1) = desiredPos(1);
          x_d(2) = desiredPos(2);
          trace("x_d:\t [%1.2f, %1.2f, %1.2f]",
              x_d(0), x_d(1), x_d(2));

          trace("Using parameters k[1-3]: %.4f, %.4f, %.4f", m_refmodel.k1, m_refmodel.k2, m_refmodel.k3 );

          // Step 1: V-part
          Matrix tau1 = m_refmodel.k1 * (x_d - m_refmodel.getPos());

          // Set heave to 0 if not controlling altitude
          if (m_args.disable_Z)
            tau1(2) = 0.0;

          if (tau1.norm_2() > m_args.refmodel_max_v)
          {
            tau1 = m_args.refmodel_max_v * tau1 / tau1.norm_2();
          }

          spew("Trying to reach speed: %.3f", m_args.refmodel_max_v);

          // Step 2: A-part
          Matrix tau2 = m_refmodel.k2 * (tau1 - m_refmodel.getVel());

          // Set heave to 0 if not controlling altitude
          if (m_args.disable_Z)
            tau2(2) = 0.0;

          if (tau2.norm_2() > m_args.refmodel_max_a)
          {
            tau2 = m_args.refmodel_max_a * tau2 / tau2.norm_2();
          }

          // Step 3: J-part
          Matrix tau3 = m_refmodel.k3 * (tau2 - m_refmodel.getAcc());

          // Set heave to 0 if not controlling altitude
          if (m_args.disable_Z)
            tau3(2) = 0.0;

          // Integrate
          m_refmodel.x += deltat * (m_refmodel.x.get(3,8,0,0).vertCat(tau3));



          // Log
          /*
          m_setpoint_log.x = m_refmodel.x(0);
          m_setpoint_log.y = m_refmodel.x(1);
          m_setpoint_log.z = m_refmodel.x(2);
          m_setpoint_log.vx = m_refmodel.x(3);
          m_setpoint_log.vy = m_refmodel.x(4);
          m_setpoint_log.vz = m_refmodel.x(5);
          m_setpoint_log.ax = m_refmodel.x(6);
          m_setpoint_log.ay = m_refmodel.x(7);
          m_setpoint_log.az = m_refmodel.x(8);

          */
          // Print reference pos and vel
          trace("x_r:\t [%1.2f, %1.2f, %1.2f]",
              m_refmodel.x(0), m_refmodel.x(1), m_refmodel.x(2));
          trace("v_r:\t [%1.2f, %1.2f, %1.2f]",
              m_refmodel.x(3), m_refmodel.x(4), m_refmodel.x(5));

        }

        void
        updateRunwayPath()
        {
          WGS84::displacement(m_ref_lat, m_ref_lon, 0, m_runway.lat_start,
                              m_runway.lon_start, 0, &m_runway.start_NED(0),
                              &m_runway.start_NED(1));
          WGS84::displacement(m_ref_lat, m_ref_lon, 0, m_runway.lat_end,
                              m_runway.lon_end, 0, &m_runway.end_NED(0),
                              &m_runway.end_NED(1));


          m_runway.end_NED(2)   = m_estate[CENTROID].state->height;
          m_runway.start_NED(2) = m_estate[CENTROID].state->height;
          // Z ref handling
          if (m_dz.z_units == IMC::Z_HEIGHT)
          {
            m_runway.end_NED(2)   = m_estate[CENTROID].state->height - m_dz.value;
            m_runway.start_NED(2) = m_estate[CENTROID].state->height - m_dz.value;
          }
          else if(m_dz.z_units == IMC::Z_ALTITUDE)
          {
            //war("Altitude not supported, assuming HEIGHT for now");
            m_runway.end_NED(2)   = m_estate[CENTROID].state->height - m_dz.value;
            m_runway.start_NED(2) = m_estate[CENTROID].state->height - m_dz.value;
          }
          else
          {
            war("Unit not supported, assuming HEIGHT");
            m_runway.end_NED(2)   = m_estate[CENTROID].state->height - m_dz.value;
            m_runway.start_NED(2) = m_estate[CENTROID].state->height - m_dz.value;
          }

          Matrix deltaWP = m_runway.end_NED - m_runway.start_NED;

          m_runway.box_length = deltaWP.norm_2();

          double deltaWP_NE = deltaWP.get(0, 1, 0, 0).norm_2();

          m_runway.alpha  =  atan2(deltaWP(1), deltaWP(0));
          m_runway.theta  = -atan2(deltaWP_NE, deltaWP(2)) + Angles::radians(90);
          m_desired_heading[D_REFERENCE].value = m_runway.alpha;
          // might add a smoothing filter here
          m_desired_heading[D_DESIRED].value = m_desired_heading[D_REFERENCE].value;
          dispatch(m_desired_heading[D_REFERENCE]);
          dispatch(m_desired_heading[D_DESIRED]);
        }

        void
        calcPathErrors(Matrix p, Matrix v, int s)
        {
          Matrix R = transpose(Rzyx(0.0, -m_runway.theta, m_runway.alpha));
          Matrix eps = R * (p - m_runway.start_NED);
          Matrix eps_dot = R * v;

          m_p_path[s] = eps;
          m_v_path[s] = eps_dot;

          if (s == FIXEDWING)
          {
            m_p_path[s](1) = m_p_path[s](1) - m_args.m_crosstrack_offset;
            m_p_path[s](2) = m_p_path[s](2) + m_runway.z_off_a;
          }

          Matrix p_net = getNetPosition(m_p);
          Matrix v_net = getNetVelocity(m_v);

          Matrix delta_p_path = R * (p_net - m_p[FIXEDWING]);
          Matrix delta_v_path = R * (v_net - m_v[FIXEDWING]);

          // Calculate position of the net relative to the fixed-wing.
          m_delta_p_path_x = delta_p_path(0);
          m_delta_v_path_x = delta_v_path(0);
        }

        void
        updateMeanValues(int s)
        {
          if (m_cross_track_window[s].size() == 0)
          {
            err("Mean window queue empty");
            return;
          }
          Matrix weightedAvg = m_p_path[s] / m_args.mean_ws;
          Matrix firstWeightedAvg = m_cross_track_window[s].front();
          Matrix newMean = m_p_path_mean[s] + weightedAvg - firstWeightedAvg;

          m_p_path_mean[s] = newMean;
          m_cross_track_window[s].pop();
          m_cross_track_window[s].push(weightedAvg);
          if (m_cross_track_d_window[s].size() == 0)
          {
            debug("Mean window d queue empty");
            return;
          }

          Matrix weightedAvg_d = m_v_path[s] / m_args.mean_ws;
          Matrix firstWeightedAvg_d = m_cross_track_d_window[s].front();
          Matrix newMean_d = m_v_path_mean[s] + weightedAvg_d
              - firstWeightedAvg_d;
          m_v_path_mean[s] = newMean_d;

          m_cross_track_d_window[s].pop();
          m_cross_track_d_window[s].push(weightedAvg_d);

          m_delta_p_path_x_mean = m_delta_p_path_x;
          m_delta_v_path_x_mean = m_delta_v_path_x;
        }

        bool
        abort()
        {
          bool aircraftOff = false;
          bool netOff = false;
          if (std::abs(m_p_path_mean[FIXEDWING](1)) >= m_args.eps_ct_a(0)
              || std::abs(m_p_path_mean[FIXEDWING](2)) >= m_args.eps_ct_a(1))
          {
            aircraftOff = true;
          }
          Matrix p_n = getNetPosition(m_p_path);
          if (std::abs(p_n(1)) >= m_args.eps_ct_n(0)
              || std::abs(p_n(2)) >= m_args.eps_ct_n(1))
          {
            netOff = true;
          }

          return (aircraftOff || netOff);
        }

        Vehicle
        getVehicle(const EstimatedLocalState* el)
        {
          if (resolveSystemId(el->getSource()) == m_vehicles.aircraft)
          {
            return FIXEDWING;
          }
          else if(  el->getSource() == this->getSystemId() &&
                    resolveEntity(el->getSourceEntity()).c_str() == m_args.centroid_els_entity_label)
          {
            return CENTROID;
          }
          return INVALID;
        }

        bool
        allConnected()
        {
          for (unsigned int i = 0; i < m_vehicles.no_vehicles; i++)
          {
            if (!m_connected[i])
              return false;
          }
          return true;
        }

        bool
        allInitialized()
        {
          for (unsigned int i = 0; i < m_vehicles.no_vehicles; i++)
          {
            if (!m_initialized[i])
              return false;
          }
          return true;
        }

        bool
        aircraftApproaching()
        {
          if (   m_v_path[FIXEDWING](0) > 0
              && m_p_path[FIXEDWING](0) < 0
              && m_p_path[FIXEDWING](0) > -m_args.max_px_app
              && std::abs(m_p_path[FIXEDWING](1)) < m_args.max_py_app
              && std::abs(m_v_path[FIXEDWING](1)) < m_args.max_vy_app)
            return true;
          return false;
        }

        bool
        startNetRecovery()
        {
          //monitor the path-along distance between the net and the aircraft
          // when at a given boundary, start the net-catch mission
          // this requires that the net are stand-by at the first WP at the runway
          double delta_p = m_delta_p_path_x;
          if (m_args.use_mean_window_aircraft)
            delta_p = m_delta_p_path_x_mean;

          if (std::abs(delta_p) <= m_startCatch_radius)
          {
            return true;
          }
          return false;
        }

        bool
        catched()
        {
          //simulation: position check
          //real life: weight cell in combination with rotary sensor
          Matrix diff = m_p_path[CENTROID] - m_p_path[FIXEDWING];
          if (diff.norm_2() < m_args.m_coll_eps && m_v_path[FIXEDWING](0) > 0)
            return true;
          return false;
        }

        bool
        aircraftPassed()
        {
          Matrix diff = m_p_path[CENTROID] - m_p_path[FIXEDWING];
          if (diff(0) < -m_args.m_coll_eps)
            return true;
          return false;
        }

        bool
        endAtRunway()
        {
          Matrix p_to_end = m_runway.end_NED - m_runway.start_NED;
          double length_runway = p_to_end.norm_2();
          if (m_p_path[CENTROID](0) > length_runway)
            return true;
          return false;
        }

        void
        updateStartRadius()
        {
          double v_a = std::abs(m_v_path[FIXEDWING](0));
          m_startCatch_radius = calcStartRadius(v_a, 0, m_u_ref, m_ad,
                                                  m_args.m_coll_r);
          if (m_startCatch_radius == -1)
          {
            err("Unable to calculate the desired start-radius");
            return;
          }
        }

        // Gets the path ramp along-track velocity.
        // The function internally takes care of timing to produce the ramp
        // from when the ramp previously was reset.
        double
        getPathVelocity(double v0, double v_ref, double a_n, bool reset_ramp)
        {

          double deltaT = (v_ref - v0) / a_n;


          static bool s_rampEnabled = false;
          static double s_startTime = -1;
          static double s_deltaV = (v_ref - v0) / deltaT;



          if (reset_ramp)
          {
            s_startTime = -1;
            s_rampEnabled = false;
          }
          //when starting net-catch operation, this should be a ramp in velocity
          if (!reset_ramp)
            s_rampEnabled = true;
          if (s_rampEnabled && s_startTime == -1)
          {
            s_startTime = Clock::get();
            s_deltaV = (v_ref - v0) / deltaT;
          }
          double vel = v0;
          double deltaTime = Clock::get() - s_startTime;
          if (s_rampEnabled)
          {
            if (deltaTime < deltaT)
              vel = v0 + s_deltaV * deltaTime;
            else
              vel = v_ref;
          }
          else
          {
            vel = v0;
          }
          static double startPrint = 0;
          if (Clock::get() - startPrint > 1)
          {
            spew("getPathVelocity:\n");
            spew("\t u_d=%f \n  \t deltaTime=%f \n \t enabled=%d", vel, deltaTime,
                 s_rampEnabled);
            startPrint = Clock::get();
          }

          return vel;
        }

        double
        calcStartRadius(double v_a, double v0_n, double v_ref_n, double a_n,
            double r_impact)
        {
          // calculate the radius which the net-catch maneuver should start, based on the mean velocity of the airplane
          // and the ramp reference velocity and max velocity
          if (a_n == 0.0)
            return -1;

          double deltaT_n = (v_ref_n - v0_n) / a_n;
          double r_n_delta_t = (std::pow(v_ref_n, 2), std::pow(v0_n, 2))
              / (2 * a_n);
          double Delta_r_impact = r_impact - r_n_delta_t;
          //inf("Delta_r_impact=%f",Delta_r_impact);
          if (Delta_r_impact < 0)
          {
            err("Desired impact position should be at least %f m from the start of the runway",
                r_n_delta_t);
            Delta_r_impact = 0;
          }
          double r_start = std::abs(
              r_impact - v_a * (deltaT_n + Delta_r_impact / v_ref_n));

          static double startPrint = 0;
          if (Clock::get() - startPrint > 1)
          {
            spew("desiredStartRadius: %f", r_start);
            startPrint = Clock::get();
          }
          return r_start;
        }

        void
        reportCurrentPositionDifference(void)
        {

          if (!allConnected())
            return;

          IMC::GroupStreamVelocity diff;

          // Set difference state
          // z is calculated as height - z, go get a positive value upwards. (altitude).
          // So, a reported positive z means the fixed-wing is OVER the net.
          // For x,y, a positive value means the fixed-wing is on the North/east positive side of the net.
          diff.x = m_p_path[FIXEDWING](0) -  m_p_path[CENTROID](0);
          diff.y = m_p_path[FIXEDWING](1) -  m_p_path[CENTROID](1);

          diff.z =  (m_estate[FIXEDWING].state->height - m_estate[FIXEDWING].state->z)
                  - (m_estate[CENTROID].state->height  - m_estate[CENTROID].state->z);

          dispatch(diff);

        }

        void
        sendCurrentState()
        {
          IMC::NetRecoveryState state;
          state.flags = m_curr_state;	// Should use the IMC flags for state
          Matrix p_n = getNetPosition(m_p_path);
          Matrix v_n = getNetVelocity(m_v_path);
          state.x_n = p_n(0);
          state.y_n = p_n(1);
          state.z_n = p_n(2);

          state.vx_n = v_n(0);
          state.vy_n = v_n(1);
          state.vz_n = v_n(2);

          state.vx_n_d = m_ud;
          state.vy_n_d = m_v_ref_path(1);
          state.vz_n_d = m_v_ref_path(2);

          state.x_n_d = m_p_ref_path(0);
          state.y_n_d = m_p_ref_path(1);
          state.z_n_d = m_p_ref_path(2);

          state.x_a = m_p_path[FIXEDWING](0);
          state.y_a = m_p_path[FIXEDWING](1);
          state.z_a = m_p_path[FIXEDWING](2);

          state.vx_a = m_v_path[FIXEDWING](0);
          state.vy_a = m_v_path[FIXEDWING](1);
          state.vz_a = m_v_path[FIXEDWING](2);

          state.delta_p_p = m_delta_p_path_x;
          state.delta_v_p = m_delta_v_path_x;

          state.start_r = m_startCatch_radius;

          state.course_error_a = 0;
          state.course_error_n = 0;

          dispatch(state);
        }

        Matrix
        getNetPosition(std::vector<Matrix> p)
        {
          return p[CENTROID];
        }

        Matrix
        getNetVelocity(std::vector<Matrix> v)
        {
          return v[CENTROID];
        }

        //! Control loop in path-frame
        //! Main workhorse. Using desired along track velocity for the net,
        //! and the current position of the net and aircraft (in the virtual runway path frame)
        //! NB: Remember that optional offsets to the aircraft position is already added in p_a_path.
        Matrix
        getDesiredPathVelocity(double u_d_along_path, Matrix p_a_path,
            Matrix v_a_path, Matrix p_n_path, Matrix v_n_path, bool& validAccReturn, Matrix& accOut)
        {

          // Resulting desired velocity
          Matrix v_path = Matrix(3, 1, 0.0);

          // Constraints
          Matrix p_max_path = Matrix(3, 1, 0.0);
          p_max_path(0) = m_runway.box_length;
          p_max_path(1) = m_runway.box_width / 2;
          p_max_path(2) = m_runway.box_height / 2;

          // Set along-track (x) reference position.
          if (m_curr_state == IMC::NetRecoveryState::NR_CATCH
              || m_curr_state == IMC::NetRecoveryState::NR_END)
          {
            m_p_ref_path(0) = p_max_path(0);
          }
          else
          {
            m_p_ref_path(0) = 0;
          }

          // Set y,z refernce for position and velocity.
          if (m_curr_state != IMC::NetRecoveryState::NR_CATCH)
          {
            for (int i = 1; i <= 2; i = i + 1)
            {

              // Bounding box on y-z position. If within, use aircraft position and velocity.
              if (sqrt(pow(p_a_path(i), 2)) < p_max_path(i))
              {
                m_p_ref_path(i) = p_a_path(i);
                m_v_ref_path(i) = v_a_path(i);
              }
              else
              {
                m_p_ref_path(i) = p_a_path(i) / sqrt(pow(p_a_path(i), 2))
                    * p_max_path(i);
                m_v_ref_path(i) = 0;
              }
            }
          }
          else
          {
            m_v_ref_path(1) = 0;
            m_v_ref_path(2) = 0;

            // Todo: Why is not m_p_ref_path(1,2) set here? Aka in Catch-state.
            // For now, they are just what they were, which is reasonable.
          }

          // Step reference model.
          stepRefModel(m_p_ref_path, m_time_diff);

          // Error variables
          Matrix e_p_path = m_p_ref_path - p_n_path;
          Matrix e_v_path = m_v_ref_path - v_n_path;

          if (m_args.refmodel_use)
          {
            e_p_path = m_refmodel.getPos() - p_n_path;
          }

          // Integral effect of path error.
          m_p_int_value = m_p_int_value + e_p_path * m_time_diff;

          // Constrain integral value.
          if (m_p_int_value.norm_2() > m_args.max_integral)
            m_p_int_value = m_args.max_integral * m_p_int_value / m_p_int_value.norm_2();

          Matrix p = Matrix(3,1,0.0);
          Matrix i = Matrix(3,1,0.0);
          Matrix d = Matrix(3,1,0.0);

          // Separate handling when using reference model.
          if (m_args.refmodel_use)
          {
            // Apply vel feedforward with p-coontroller on position error.
            v_path = m_refmodel.getVel() + m_control.Kp * e_p_path;

            // Set acc feed forward.
            accOut = m_refmodel.getAcc();
            validAccReturn = true;

            // If catching, we are doing separate along-path thing.
            if (m_curr_state == IMC::NetRecoveryState::NR_START
                || m_curr_state == IMC::NetRecoveryState::NR_CATCH)
            {
              v_path(0) = u_d_along_path;
              accOut(0) = 0;
            }

          }

          else if (   m_curr_state == IMC::NetRecoveryState::NR_STANDBY
              || m_curr_state == IMC::NetRecoveryState::NR_APPROACH
              || m_curr_state == IMC::NetRecoveryState::NR_END)
          {
            p = m_control.Kp*e_p_path;
            d = m_control.Kd*e_v_path;
            i = m_control.Ki*m_p_int_value;

            v_path = p + i + d;

            //limit velocity
            if (v_path.norm_2() > m_args.max_norm_v)
            {
              v_path = sqrt(pow(m_args.max_norm_v, 2)) * v_path / v_path.norm_2();
            }
          }
          else if (m_curr_state == IMC::NetRecoveryState::NR_START
                || m_curr_state == IMC::NetRecoveryState::NR_CATCH)
          {
            e_p_path(0) = 0;

            Matrix v_temp = Matrix(3,1,0.0);
            Matrix v_path_yz = Matrix(2, 1, 0.0);
            p = m_control.Kp*e_p_path;
            d = m_control.Kd*e_v_path;
            i = m_control.Ki*m_p_int_value;
            p(0) = 0;
            d(0) = 0;
            i(0) = 0;
            v_temp = p + i + d;
            v_path_yz(0) = v_temp(1);
            v_path_yz(1) = v_temp(2);

            //limit velocity
            // Maximum velocity is limited by also reserving for the desired forward velocity.
            double maximumYZSpeed = sqrt(pow(m_args.max_norm_v, 2) - pow(m_u_ref, 2));
            if (v_path_yz.norm_2() > maximumYZSpeed)
            {
              v_path_yz = maximumYZSpeed * v_path_yz / v_path_yz.norm_2();
            }

            // Set forward velocity based on the velocity ramp profile and yz-controller.
            v_path(0) = u_d_along_path;
            v_path(1) = v_path_yz(0);
            v_path(2) = v_path_yz(1);
          }

          static double startPrint = 0;
          if (Clock::get() - startPrint > 0.3)
          {
            spew("Kp: [%f,%f,%f]", m_args.Kp(0), m_args.Kp(1), m_args.Kp(2));
            spew("m_time_diff: %llu", m_time_diff);
            spew("m_p_int_value: [%f,%f,%f]", m_p_int_value(0), m_p_int_value(1), m_p_int_value(2));
            spew("p_ref_path: [%f,%f,%f]", m_p_ref_path(0), m_p_ref_path(1), m_p_ref_path(2));
            spew("v_ref_path: [%f,%f,%f]", m_v_ref_path(0), m_v_ref_path(1), m_v_ref_path(2));
            spew("p_a_path: [%f,%f,%f]", p_a_path(0), p_a_path(1), p_a_path(2));
            spew("p_n_path: [%f,%f,%f]", p_n_path(0), p_n_path(1), p_n_path(2));
            //Matrix delta = p_a_path-p_n_path;
            //inf("norm delta %f-> des speed %f\n", delta.norm_2(), u_d_along_path);
            spew("  v_path: [%f,%f,%f]\n\n", v_path(0), v_path(1), v_path(2));

            startPrint = Clock::get();
          }

          //Dispatch control parcel logs
          IMC::ControlParcel parcel_pid_along   = m_parcels[PC_PID_ALONG_SURGE];
          IMC::ControlParcel parcel_pid_cross_y = m_parcels[PC_PID_CROSS_Y];
          IMC::ControlParcel parcel_pid_cross_z = m_parcels[PC_PID_CROSS_Z];

          parcel_pid_along.p   = p(0);
          parcel_pid_cross_y.p = p(1);
          parcel_pid_cross_z.p = p(2);

          parcel_pid_along.d   = d(0);
          parcel_pid_cross_y.d = d(1);
          parcel_pid_cross_z.d = d(2);

          parcel_pid_along.i   = i(0);
          parcel_pid_cross_y.i = i(1);
          parcel_pid_cross_z.i = i(2);

          dispatch(parcel_pid_along);
          dispatch(parcel_pid_cross_y);
          dispatch(parcel_pid_cross_z);

          IMC::ControlParcel errors_along_x = m_parcels[PC_ERROR_ALONG_SURGE];
          IMC::ControlParcel errors_cross_y = m_parcels[PC_ERROR_CROSS_Y];
          IMC::ControlParcel errors_cross_z = m_parcels[PC_ERROR_CROSS_Z];

          errors_along_x.p = e_p_path(0);
          errors_cross_y.p = e_p_path(1);
          errors_cross_z.p = e_p_path(2);

          errors_along_x.d = e_v_path(0);
          errors_cross_y.d = e_v_path(1);
          errors_cross_z.d = e_v_path(2);

          errors_along_x.i = m_p_int_value(0);
          errors_cross_y.i = m_p_int_value(1);
          errors_cross_z.i = m_p_int_value(2);

          dispatch(errors_along_x);
          dispatch(errors_cross_y);
          dispatch(errors_cross_z);

          return v_path;
        }

        //! Get desired local
        Matrix
        getDesiredLocalVelocity(Matrix v_p, double course, double pitch)
        {
          return Rzyx(0.0, pitch, course) * v_p;
        }

        void
        sendCentroidLinearState(Matrix vel,Matrix acc, DesiredEntites type)
        {
          //please note that these value are given in the centroid body frame

          m_desired_linear[type].vx = vel(0);
          m_desired_linear[type].vy = vel(1);
          m_desired_linear[type].vz = vel(2);

          m_desired_linear[type].ax = acc(0);
          m_desired_linear[type].ay = acc(1);
          m_desired_linear[type].az = acc(2);

          if (m_args.disable_Z)
          {
            m_desired_linear[type].flags = IMC::DesiredLinearState::FL_VX | IMC::DesiredLinearState::FL_VY
                                         | IMC::DesiredLinearState::FL_AX | IMC::DesiredLinearState::FL_AY;
          }
          else
          {
            m_desired_linear[type].flags = IMC::DesiredLinearState::FL_VX | IMC::DesiredLinearState::FL_VY | IMC::DesiredLinearState::FL_VZ
                                         | IMC::DesiredLinearState::FL_AX | IMC::DesiredLinearState::FL_AY | IMC::DesiredLinearState::FL_AZ;
          }
          dispatch(m_desired_linear[type]);
        }

        virtual void
        reset(void)
        {
          inf("Reset coordinator");
          m_curr_state = IMC::NetRecoveryState::NR_INIT;
          m_initializedCoord = false;
          m_ref_valid = false;
          m_coordinatorEnabled = false;
          m_time_end = Clock::get();
          m_time_diff = 0.0;
          m_p_int_value = Matrix(3, 1, 0.0);
          //reset ramp
          m_ud = 0;
          getPathVelocity(0, m_u_ref, m_ad, true);
          //initCoordinator();
          initRefModel();
        }

        // Resets states, done on entering standby.
        void
        resetStates(void)
        {
          initRefModel();
          m_time_end = Clock::get();
          m_time_diff = 0.0;
        }

        virtual void
        onAutopilotActivation(void)
        {
          inf("AutopilotActivating");
          if (!m_args.use_controller)
          {
            debug("Path activated, but not active: Requesting deactivation");
            requestDeactivation();
            return;
          }
        }

        virtual void
        onAutopilotDeactivation(void)
        {
          debug("Deactivation");
          //requestDeactivation();
        }

        //! Main loop.
        void
        task(void)
        {
          if (!m_args.use_controller || !isActive() || !m_coordinatorEnabled)
            //spew("isActive: %d",isActive());
            return;

          double now = Clock::get();
          m_time_diff = now - m_time_end;
          m_time_end = now;
          //spew("Frequency: %1.1f, %d", 1000.0/m_time_diff,m_coordinatorEnabled);

          //dispatch control if ready

          if (m_curr_state != IMC::NetRecoveryState::NR_STOP
              && m_curr_state != IMC::NetRecoveryState::NR_INIT)
          {
            Matrix p_a_path;
            Matrix v_a_path;

            Matrix p_n_path;
            Matrix v_n_path;

            if (m_curr_state == IMC::NetRecoveryState::NR_STANDBY)
            {
              p_a_path = Matrix(3, 1, 0.0);
              v_a_path = Matrix(3, 1, 0.0);
            }
            else
            {
              if (m_args.use_mean_window_aircraft)
              {
                p_a_path = m_p_path_mean[FIXEDWING];
                v_a_path = m_v_path_mean[FIXEDWING];
              }
              else
              {
                p_a_path = m_p_path[FIXEDWING];
                v_a_path = m_v_path[FIXEDWING];
              }
            }

            p_n_path = getNetPosition(m_p_path);
            v_n_path = getNetVelocity(m_v_path);

            // Acceleration ff
            Matrix a_n_path = Matrix(3, 1, 0.0);
            bool accIsSet = false;


            // Compute velocity setpoints for the net
            // using desired along-track speed, and the current position and velocity of the centroid (net) and aircraft
            Matrix v_path_d = getDesiredPathVelocity(m_ud, p_a_path, v_a_path,
                                                     p_n_path, v_n_path,
                                                     accIsSet, a_n_path);


            if (!accIsSet)
            {
              // For now, assume zero acceleration.
              m_a_des_path(0) = 0;
              m_a_des_path(1) = 0;
              m_a_des_path(2) = 0;
            }
            else
            {
              m_a_des_path(0) = a_n_path(0);
              m_a_des_path(1) = a_n_path(1);
              m_a_des_path(2) = a_n_path(2);
            }
            // Transform the LinearState values to centroid frame (from virtual runway path frame)
            Matrix v_d = RCentroidPath()*v_path_d;
            Matrix a_d = RCentroidPath()*m_a_des_path;

            sendCentroidLinearState(v_d,a_d,D_DESIRED);
          }

        }


        Matrix
        RCentroidPath() const
        {
          return transpose(RNedCentroid())*RNedPath();
        }

        Matrix
        RNedCentroid() const
        {
          return Rz(m_centroid_heading);
        }

        Matrix
        RNedPath() const
        {
          return Rzyx(0,m_runway.theta,m_runway.alpha);
        }

        //! @return  Rotation yaw matrix.
        Matrix
        Rz(double psi) const
        {
          double R_en_elements[] =
            { cos(psi), -sin(psi), 0, sin(psi), cos(psi), 0, 0, 0, 1 };
          return Matrix(R_en_elements, 3, 3);
        }

        //! @return  Rotation matrix.
        Matrix
        Rzyx(double phi, double theta, double psi) const
        {
          double R_en_elements[] =
            { cos(psi) * cos(theta), (-sin(psi) * cos(phi))
                + (cos(psi) * sin(theta) * sin(phi)), (sin(psi) * sin(phi))
                + (cos(psi) * cos(phi) * sin(theta)), sin(psi) * cos(theta), (cos(
                psi) * cos(phi)) + (sin(phi) * sin(theta) * sin(psi)), (-cos(psi)
                * sin(phi)) + (sin(theta) * sin(psi) * cos(phi)), -sin(theta),
                cos(theta) * sin(phi), cos(theta) * cos(phi) };
          return Matrix(R_en_elements, 3, 3);
        }

      };
    }
  }
}

DUNE_TASK
