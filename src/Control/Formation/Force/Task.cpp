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

// USER HEaders
#include <USER/DUNE.hpp>

namespace Control
{
  namespace Formation
  {
    namespace Force
    {
      using DUNE_NAMESPACES;
      //! Controllable loops.
      static const uint32_t c_controllable = IMC::CL_SPEED;
      //! Required loops.
      static const uint32_t c_required = IMC::CL_FORCE;

      //! Vector for System Mapping.
      typedef std::vector<uint32_t> Systems;

      //! UTC seconds for 2015-01-01 to check for clock synch
      const int UTC_SECS_2015 = 1420070400;

      struct GainScheduler
      {
        //! Enable link gain scheduler
        bool enable;

        //! Link gain when in-formation
        double gain_close;

        //! Link gain when large link error
        double gain_far;

        //! Sigmoid constant
        double sigmoid_const;

        //! Desired link error switching distance
        double switch_distance;

        //! Steepness factor of slope between far and close link gain
        double slope;
      };

      struct Arguments
      {
        //! Use Formation Controller
        bool use_controller;

        //! Vehicle list
        std::vector<std::string> formation_systems;

        //! Desired formation
        Matrix desired_formation;

        //! Incidence matrix
        Matrix incidence_matrix;

        //! Link gains
        Matrix link_gains;

        //! Disable formation velocity
        bool disable_formation_velocity;

        //! Disable mission velocity
        bool disable_mission_velocity;

        //! (optional) Constant mission velocity
        Matrix const_mission_velocity;

        //! Disable collision velocity
        bool disable_collision_velocity;

        //! Collision avoidance radius
        double collision_radius;

        //! Minimum difference used in collision avoidance
        double collision_saturation;

        //! Collision avoidance gain
        double collision_gain;

        //! Maximum speed
        double max_speed;

        //! Time constant for low-pass smoothing of meta-data
        double meta_smoothing_T;

        //! Threshold for delay on positioning updates
        double delay_threshold;

        //! Hold current formation
        bool hold_current_formation;

        //! Threshold for sending aborts
        double abort_resend_threshold;

        //! Frequency of pos.data prints
        float print_frequency;

        //! Frequency of pos.data prints
        bool print_EstimatedLocalState_warning;

        //! Whether or not to control altitude
        bool use_altitude;

        //! Disable heave flag,this will utilize new rate controller on some targets
        bool disable_heave;

        //! Disable force output flag,this will disable dispatching of the desired force
        bool disable_force_output;

        //!velocity Controller parameters
        Matrix Kp;
        Matrix Ki;
        Matrix Kd;

        double max_norm_F;

        //low pass filter on desired heading
        bool lowpass_heading;
        //! Time constant for low-pass heading smoothing
        double heading_smoothing_T;

        bool link_filter_enable;
        double link_filter_beta;

        std::string centroid_els_entity_label;

        std::vector<std::string> desired_heading_entity_labels;
        std::vector<std::string> desired_linear_entity_labels;

        //! Vehicle mass
        double mass;

        //! True to divide output force by mass, effectively turning it to an acceleration output.
        bool enable_mass_division;

        //! True to enable wind feed-forward
        bool enable_wind_ff;

        //! Wind drag coefficient
        double wind_drag_coefficient;

        //! Enable bias estimation on control output
        bool enable_bias_compensation;

        //! Max wind speed estiate, used for integral anti windup
        double max_wind_speed_estimate;

        //! Enable integration of u-signal as well.
        bool enable_formation_integration;

        //! Enable sync of bias
        bool enable_bias_sync;

        //! bias sync gain
        double bias_sync_gain;
      };

      static const std::string c_parcel_names[] = { DTR_RT("Force PID-X"), DTR_RT("Force PID-Y"), DTR_RT("Force PID-Z"),
                                                    DTR_RT("Mission-X"), DTR_RT("Mission-Y"), DTR_RT("Mission-Z"),
                                                    DTR_RT("Collision-X"), DTR_RT("Collision-Y"), DTR_RT("Collision-Z"),
                                                    DTR_RT("Formation-X"), DTR_RT("Formation-Y"), DTR_RT("Formation-Z"),
                                                    DTR_RT("Wind_ff")
                                                  };

      enum Parcel {
        PC_PID_X      = 0,
        PC_PID_Y      = 1,
        PC_PID_Z      = 2,
        PC_MISSION_X  = 3,
        PC_MISSION_Y  = 4,
        PC_MISSION_Z  = 5,
        PC_COLLISION_X = 6,
        PC_COLLISION_Y = 7,
        PC_COLLISION_Z = 8,
        PC_FORMATION_X = 9,
        PC_FORMATION_Y = 10,
        PC_FORMATION_Z = 11,
        PC_WIND_FF     = 12,
        PC_NUM         = 13
      };

      static const int NUM_PARCELS = 13;

      struct Task : public DUNE::Control::PeriodicUAVAutopilot
      {
        //! Task arguments
        Arguments m_args;

        //! Vehicle IDs
        Systems m_uav_ID;

        //! Vehicle formation number
        unsigned int m_i;

        //! Incidence matrix;
        Matrix m_D;

        //! Number of agents and links
        unsigned int m_N, m_L;

        //! Link gains
        Matrix m_delta;

        //! Gain scheduling parameters
        GainScheduler m_gain_scheduler;

        //! Desired difference variables
        Matrix m_z_d;

        //! Difference variables
        Matrix m_z;

        //! Difference variables filtered
        Matrix m_z_filtered;        

        //! Desired formation positions
        Matrix m_x_c;

        //! Desired formation positions
        Matrix m_x_c_default;

        //! Vehicle positions
        Matrix m_x;

        //! Vehicles Stream Estimates
        Matrix m_esvs;

        //! Vehicle NED velocities
        Matrix m_v_ned;

        //! Vehicle NED accelerations
        Matrix m_a_ned;

        //! Desired mission velocity in CENTROID
        Matrix m_v_mission_centroid;

        //! Desired mission acceleration in CENTROID
        Matrix m_a_mission_centroid;

        //! Last time position was updated for each vehicle
        Matrix m_last_pos_update;

        //! Last time heading was updated for each vehicle
        Matrix m_last_heading_update;

        //! Rate [Hz] and delay [ms] of position update for each vehicle. Both zero for vehicles we have yet to receive positions from.
        Matrix m_pos_update_rate, m_pos_update_delay;

        //! Desired force on this agent
        IMC::DesiredControl m_desired_force;

        //! Last received ELS from this agent
        IMC::EstimatedLocalState m_local_state;

        //! Last received desired heading from master agent
        IMC::DesiredHeading m_desired_heading;

        //! Last received desired linear setpoint from master agent
        IMC::DesiredLinearState m_linear_setpoint;

        //! Current coordinator state
        IMC::CoordinatorState m_coord_state;

        double m_curr_desired_heading;
        //current centroid heading
        double m_curr_heading;

        Matrix m_v_int_value;

        uint64_t m_time_end;
        uint64_t m_time_diff;

        bool m_configured;

        //! Container for logging errors
        IMC::RelativeState m_error_log;

        //! Wind bias estimator
        Matrix m_bias_estimate;

        //! True if local state is updated from EstimatedLocalState
        bool m_self_local_updated;

        //! Parcel array
        IMC::ControlParcel m_parcels[NUM_PARCELS];


        //! Constructor.
        //! @param[in] name task name.
        //! @param[in] ctx context.
        Task(const std::string& name, Tasks::Context& ctx):
          PeriodicUAVAutopilot(name, ctx, c_controllable, c_required),       
          m_i(0), 
          m_N(0), 
          m_L(0),
          m_v_ned(3,1,0.0),
          m_a_ned(3,1,0.0),
          m_v_mission_centroid(3,1,0.0),
          m_a_mission_centroid(3,1,0.0),
          m_curr_desired_heading(0.0),
          m_curr_heading(0.0),
          m_v_int_value(3, 1, 0.0),
          m_time_end(0.0), 
          m_time_diff(0.0),
          m_configured(false),
          m_bias_estimate(Matrix(3,1, 0.0)),
          m_self_local_updated(false)
        {
          param("Formation Controller", m_args.use_controller)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
           .defaultValue("false").description("Enable Formation Controller.");

          param("Constant Mission Velocity", m_args.const_mission_velocity)
          .defaultValue("0.0, 0.0, 0.0")
          .units(Units::MeterPerSecond)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Constant mission velocity.");

          param("Collision Avoidance Radius", m_args.collision_radius)
          .defaultValue("5.0")
          .units(Units::Meter)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Radius for collision avoidance potential field.");

          param("Collision Avoidance Saturation", m_args.collision_saturation)
          .defaultValue("3.0")
          .units(Units::Meter)
          .description("Maximum difference used in collision avoidance.");

          param("Collision Avoidance Gain", m_args.collision_gain)
          .defaultValue("5.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Gain for collision avoidance potential field.");

          param("Maximum Speed", m_args.max_speed)
          .defaultValue("5.0")
          .units(Units::MeterPerSecond)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          //.scope(Tasks::Parameter::SCOPE_MANEUVER)
          .description("Maximum speed, i.e. controller saturation.");

          param("Meta Smoothing Time Constant", m_args.meta_smoothing_T)
          .defaultValue("2.0")
          .units(Units::Second)
          .description("Time constant used in low-pass smoothing of meta-data");

          param("Delay Threshold", m_args.delay_threshold)
          .defaultValue("100")
          .units(Units::Millisecond)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Threshold for issuing warnings on delay in position updates.");

          param("Resend Abort Threshold", m_args.abort_resend_threshold)
          .defaultValue("200")
          .units(Units::Millisecond)
          .description("Time allowed to pass before resending an abort.");

          param("Print Frequency", m_args.print_frequency)
          .defaultValue("0.0")
          .units(Units::Second)
          .description("Frequency of pos.data prints. Zero => Print on every update.");

          param("Print EstimatedLocalState warning", m_args.print_EstimatedLocalState_warning)
          .defaultValue("false")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Choose whether warning for old EstimatedLocalState should be received.");

          param("Use altitude", m_args.use_altitude)
          .defaultValue("false")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Choose whether to control altitude or not");

          param("Disable Heave flag", m_args.disable_heave)
          .defaultValue("false")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Choose whether to disable heave flag. In turn, this will utilize new rate controller on some targets");

          param("Disable Force flag", m_args.disable_force_output)
          .defaultValue("false")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Choose whether to disable force output flag");

          param("Kp Velocity Control", m_args.Kp)
          .defaultValue("1.0,1.0,1.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Position Controller tuning parameter Kp");

          param("Ki Velocity Control", m_args.Ki)
          .defaultValue("0.0,0.0,0.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Velocity Controller tuning parameter Ki");

          param("Kd Velocity Control", m_args.Kd)
          .defaultValue("0.0,0.0,0.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Velocity Controller tuning parameter Kd");

          param("Maximum Normalized Force", m_args.max_norm_F)
          .defaultValue("5.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Maximum Normalized Force of the Vehicle");

          param("Enable Desired Heading Lowpass", m_args.lowpass_heading)
          .defaultValue("False")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .description("Enable low-pass smoothing of heading input");

          param("Heading Smoothing Time Constant", m_args.heading_smoothing_T)
          .defaultValue("2.0")
          .units(Units::Second)
          .description("Time constant used in low-pass smoothing of heading input");

          param("EstimatedLocalState Entity Label", m_args.centroid_els_entity_label)
          .defaultValue("Formation Centroid")
          .description("Entity label for the centroid EstimatedLocalState");

          param("Desired Heading Entity Labels", m_args.desired_heading_entity_labels)
          .defaultValue("Desired Heading")
          .description("Entity labels for the DesiredHeading message");

          param("Desired Linear Entity Labels", m_args.desired_linear_entity_labels)
          .defaultValue("Desired Linear")
          .description("Entity labels for the DesiredLinearState message");

          param("Enable Link Filter", m_args.link_filter_enable)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .defaultValue("False")
          .description("Increase to make the response slower. ");

          param("Link Filter Beta", m_args.link_filter_beta)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .minimumValue("0.0")
          .defaultValue("0.8")
          .maximumValue("1.0")
          .description("Increase to make the response slower. ");

          param("Vehicle Mass", m_args.mass)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .minimumValue("0.0")
          .defaultValue("2.0")
          .maximumValue("20")
          .description("Mass of the current vehicle. ");

          param("CtrlMisc - Enable Output Division By Mass", m_args.enable_mass_division)
          .defaultValue("true")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Enable or disable division by vehicle mass in final output");

          param("CtrlMisc - Enable Wind Feed Forward", m_args.enable_wind_ff)
          .defaultValue("true")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_PLAN)
          .description("Enable to feed-forward wind effects");

          param("Model - Wind Drag Coefficient", m_args.wind_drag_coefficient)
          .defaultValue("0.05")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Coefficient to use in wind ff");

          param("Enable Bias Compensation", m_args.enable_bias_compensation)
          .defaultValue("true")
          .visibility(Tasks::Parameter::VISIBILITY_USER);

          param("Max wind speed estimate", m_args.max_wind_speed_estimate)
          .defaultValue("10.0")
          .description("Max wind speed estimate, used for integral windup. ");

          param("Enable formation integration", m_args.enable_formation_integration)
          .defaultValue("false")
          .visibility(Tasks::Parameter::VISIBILITY_USER);

          param("Enable bias sync", m_args.enable_bias_sync)
          .defaultValue("false")
          .visibility(Tasks::Parameter::VISIBILITY_USER);

          param("Bias sync gain", m_args.bias_sync_gain)
          .defaultValue("1")
          .visibility(Tasks::Parameter::VISIBILITY_USER);

          // Bind incoming IMC messages
          bind<IMC::DesiredLinearState>(this);
          bind<IMC::EstimatedLocalState>(this);
          bind<IMC::DesiredHeading>(this);
          bind<IMC::FormCoord>(this);
          bind<IMC::CoordConfig>(this);
          bind<IMC::EstimatedStreamVelocity>(this);
        }

        //! Update internal state with new parameter values.
        void
        onUpdateParameters(void)
        {
          debug("Starting update of parameters.");

          if (paramChanged(m_args.const_mission_velocity))
          {
            inf("New constant mission velocity.");

            // Check dimension
            if (m_args.const_mission_velocity.rows() != 3)
              throw DUNE::Exception("Invalid mission velocity vector!");

            // Set constant mission velocity
            m_v_mission_centroid = m_args.const_mission_velocity;
            debug("Mission Velocity: [%1.1f, %1.1f, %1.1f]", m_v_mission_centroid(0),
                m_v_mission_centroid(1), m_v_mission_centroid(2));
          }


          if (paramChanged(m_args.Kp))
            m_args.Kp = checkGainMatrix(m_args.Kp);
          if (paramChanged(m_args.Kd))
            m_args.Kd = checkGainMatrix(m_args.Kd);
          if (paramChanged(m_args.Ki))
            m_args.Ki = checkGainMatrix(m_args.Ki);

        }

        Matrix
        checkGainMatrix(Matrix K)
        {
          Matrix out = K;

          if (K.size() == 0)
          {
            err("Invalid K parameters. Setting to 0");
            out = Matrix(3, 1, 0);
            return out;
          }

          // Check length, if not 3 just use the first one.
          if (K.size() != 3)
          {
            // Use first element
            out = Matrix(3,1, K(0));

            // Special case for two element, use the second as z-gain.
            if (K.size() == 2)
              out(2) = K(1);
          }

          return out;
        }

        //! Reserve entity identifiers.
        void
        onEntityReservation(void)
        {
          for (unsigned i = 0; i < NUM_PARCELS; ++i)
            m_parcels[i].setSourceEntity(reserveEntity(c_parcel_names[i] + " Parcel"));
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
          PeriodicUAVAutopilot::onResourceInitialization();
          IMC::EstimatedState es;
          m_local_state.state.set(es);
        }

        //! Release resources.
        void
        onResourceRelease(void)
        {
        }

        void
        onAutopilotActivation(void)
        {
        }

        void
        onAutopilotDeactivation(void)
        {
          Matrix zero_vel(3, 1, 0.0);
          sendDesiredForce(zero_vel);
        }

        virtual void
        reset(void)
        {
          m_time_end = Clock::getMsec();
          m_time_diff = 0.0;
          m_bias_estimate = Matrix(3,1, 0.0);
        }

        void
        updateFormation()
        {
          static double last_print;

          //spew("New desired formation.");
          double now = Clock::getSinceEpoch();
          

          //low pass filtered heading
          if (m_args.lowpass_heading)
          {
            double diff = now - m_last_heading_update(m_i); //m_i is me
            m_curr_desired_heading = lowPassSmoothing(m_curr_desired_heading, m_desired_heading.value, diff, m_args.heading_smoothing_T);
          }
          m_x_c = Rzyx(0, 0, m_curr_desired_heading) * m_x_c_default;

          calcDiffVariable(&m_z_d, m_D, m_x_c);

          if (!m_args.print_frequency || !last_print
              || (now - last_print) > 1.0 / m_args.print_frequency)
          {
            debug("Default desired formation:");
            printMatrix(m_x_c_default);

            debug("Current desired formation:");
            printMatrix(m_x_c);
            debug("Current desired heading: %f [rad]", m_curr_desired_heading);
            debug("Current Centroid Heading: %f [rad]", m_curr_heading);
            debug("Current Mission Velocity: [%1.1f, %1.1f, %1.1f]", m_v_mission_centroid(0),
                m_v_mission_centroid(1), m_v_mission_centroid(2));

            debug("Current positions:");
            printMatrix(m_x);
            debug("Current difference variables:");
            printMatrix(m_z_d);
            debug("Current difference variables:");
            debug("\n\n");
            last_print = now;
          }
          //
        }

        void
        updateFormationParameters()
        {
          debug("New desired difference variables.");
          if (m_N > 1)
          {
            // Resize and reset difference variables matrices to fit number of links
            m_z.resizeAndFill(3, m_L, 0);
            m_z_filtered.resizeAndFill(3, m_L, 0);
            m_z_d.resizeAndFill(3, m_L, 0);
            // Update desired difference variables matrix
            calcDiffVariable(&m_z_d, m_D, m_x_c);
            printMatrix(m_z_d);
          }

          // Initialize matrices
          m_last_pos_update = Matrix(1, m_N, Clock::get());
          m_last_heading_update = Matrix(1, m_N, Clock::get());
          m_pos_update_rate = Matrix(1, m_N, 0);
          m_pos_update_delay = Matrix(1, m_N, 0);
        }

        bool
        configParseParticipants(const IMC::CoordConfig* config)
        {
          debug("New desired formation.");

          const IMC::MessageList<IMC::VehicleFormationParticipant>* part = &config->participants;
          IMC::MessageList<IMC::VehicleFormationParticipant>::const_iterator p_itr;
          // Extract vehicle IDs
          m_uav_ID.clear();

          if (static_cast<unsigned int>(part->size()) == 0)
          {
            m_uav_ID.push_back(this->getSystemId());
            m_N = 1;
            m_i = 0;
          }
          else
          {
            m_N = static_cast<unsigned int>(part->size());
            bool found_self = false;
            unsigned int uav = 0;

            // Resize desired formation matrix to fit number of vehicles
            m_x_c_default.resizeAndFill(3, m_N, 0);
            for (p_itr = part->begin(); p_itr != part->end(); p_itr++)
            {
              //all participants
              m_uav_ID.push_back((*p_itr)->vid);

              if ((*p_itr)->vid == this->getSystemId())
              {
                m_i = uav; // Set my formation id
                found_self = true;
              }
              m_x_c_default(0,uav) = (*p_itr)->off_x;
              m_x_c_default(1,uav) = (*p_itr)->off_y;
              m_x_c_default(2,uav) = (*p_itr)->off_z;
              debug("UAV %u: [%1.1f, %1.1f, %1.1f]", uav,
                    m_x_c_default(0, uav), m_x_c_default(1, uav),
                    m_x_c_default(2, uav));
              uav += 1;
            }
            if (!found_self)
            {
              war("Vehicle is no longer a part of the formation");
              return found_self;
            }
            // Resize and reset position and velocity matrices to fit number of vehicles
            m_x.resizeAndFill(3, m_N, 0);
            m_esvs.resizeAndFill(3, m_N, 0.0);
            m_v_ned.resizeAndFill(3, 1, 0);
            m_a_ned.resizeAndFill(3, 1, 0);

            m_x_c = m_x_c_default;
          }
          return true;
        }

        void
        configParseIncidence(const IMC::CoordConfig* config)
        {
          debug("New incidence matrix.");
          const IMC::MessageList<IMC::CoordIncidenceAgent>* incidence_agents = &config->incidence;
          IMC::MessageList<IMC::CoordIncidenceAgent>::const_iterator i_itr;

          // Update number of links
          unsigned int noLinks = 0;
          unsigned int link = 0;
          unsigned int uav = 0;
          debug("m_L=%d,m_N=%d",m_L,m_N);
          if (static_cast<unsigned int>(incidence_agents->size()) != m_N)
            err("Number of agents in incidence matrix (%d) does not match the number of agents in the desired formation (%d)!",
                static_cast<unsigned int>(incidence_agents->size()),m_N);

          for (i_itr = incidence_agents->begin(); i_itr != incidence_agents->end(); i_itr++)
          {
            //all participants

            //parse through links
            const IMC::MessageList<IMC::CoordIncidenceLink>* incidence_links = &(*i_itr)->links;
            IMC::MessageList<IMC::CoordIncidenceLink>::const_iterator j_itr;

            if (i_itr == incidence_agents->begin())
            {
              noLinks = static_cast<unsigned int>(incidence_links->size());
              m_D.resize(m_N, noLinks);
              debug("noLinks=%d,m_N=%d",noLinks,m_N);
            }

            if (static_cast<unsigned int>(incidence_links->size()) != noLinks)
              err("Number of links to the agent (%d) does not match the number of links in the previous agents (%d)!",
                  static_cast<unsigned int>(incidence_links->size()), noLinks);

            link = 0;
            for (j_itr = incidence_links->begin(); j_itr != incidence_links->end(); j_itr++)
            {
              debug("uav=%d,link=%d",uav,link);
              m_D(uav, link) = (*j_itr)->orientation;
              link += 1;
            }
            uav += 1;
          }
          m_L = noLinks;
          debug("m_L=%d,m_N=%d",m_L,m_N);
          printMatrix(m_D);
        }
        void
        configParseLinkGains(const IMC::CoordConfig* config)
        {
          debug("New link gains.");
          const IMC::MessageList<IMC::CoordLinkGain>* link_gain = &config->link_gains;
          IMC::MessageList<IMC::CoordLinkGain>::const_iterator i_itr;

          Matrix linkGains = Matrix(link_gain->size(),1);
          unsigned int link = 0;
          for (i_itr = link_gain->begin(); i_itr != link_gain->end(); i_itr++)
          {
            linkGains(link) = (*i_itr)->value;
            link += 1;
          }
          if (linkGains.size() == 1)
          {
            // Scalar gain, set all to this
            m_delta = Matrix(m_L, 1, linkGains(0));
          }
          else if ((unsigned int)linkGains.size() != m_L)
            err("No of link gains (%d) doesn't match number of links (%d)!",linkGains.size(),m_L);
          else
          {
            // Update gains
            m_delta = linkGains;
          }
          printMatrix(m_delta);
        }
        void
        configParseLinkGainScheduling(const IMC::CoordConfig* config)
        {
          debug("New link gain scheduling parameters.");
          const IMC::MessageList<IMC::CoordLinkGainScheduler>* link_gain_scheduler = &config->link_gains_scheduling;
          IMC::MessageList<IMC::CoordLinkGainScheduler>::const_iterator i_itr;

          if (link_gain_scheduler->size() > 1)
          {
            war("Only one scheduler for all links supported, using the first CoordLinkGainScheduler message");
          }
          i_itr = link_gain_scheduler->begin();

          m_gain_scheduler.enable = (*i_itr)->enable_scheduler;
          m_gain_scheduler.gain_close = (*i_itr)->gain_close;
          m_gain_scheduler.gain_far = (*i_itr)->gain_far;
          m_gain_scheduler.switch_distance = (*i_itr)->switch_distance;
          m_gain_scheduler.slope = (*i_itr)->slope;

          //! calculate sigmoid-function constant (to ensure correct end-points of gain-scheduler)
          double temp_c = std::exp(-m_gain_scheduler.slope*m_gain_scheduler.switch_distance);
          m_gain_scheduler.sigmoid_const = m_gain_scheduler.gain_close*(temp_c + 1) - temp_c*m_gain_scheduler.gain_far;
        }
        void
        consume(const IMC::CoordConfig* config)
        {
          double now = Clock::getSinceEpoch();
          static double last_print;
          if (!m_args.print_frequency || !last_print
              || (now - last_print) > 1.0 / m_args.print_frequency)
          {
            spew("Got CoordConfig from '%s'", resolveSystemId(config->getSource()));
            last_print = now;
          }

          if (config->update || !m_configured)
          {            
            //CoordConfig contains new formation data
            // Parse participants
            bool found_self = configParseParticipants(config);
            if (!found_self)
            {
              m_configured=false;
              debug("Not configured for formation");
              return;
            }
            debug("m_N=%d",m_N);
            if (m_N > 1)
            {
              // Parse incidence matrix
              configParseIncidence(config);
              // Parse link gains
              configParseLinkGains(config);
              // Parse gain scheduling params
              configParseLinkGainScheduling(config);
            }
            updateFormationParameters();
            if (!m_configured)
            {
              m_configured = true;
              debug("Configured with IMC::CoordConfig");
            }
          }
          else
          {
            //CoordConfig only contains new flags
            m_args.disable_collision_velocity = config->disable_collision_vel;
            m_args.disable_formation_velocity = config->disable_formation_vel;
            m_args.disable_mission_velocity   = config->disable_mission_vel;
            m_args.hold_current_formation     = config->formation;
          }
          debug("CoordConfig handled, m_configured=%d",m_configured);
        }

        //! Consume Formation Position
        void
        consume(const IMC::EstimatedLocalState* msg)
        {
          if (!m_configured)
          {
            spew("Not configured!");
            return;
          }
          double now = Clock::getSinceEpoch();
          static double last_print;
          if (!m_args.print_frequency || !last_print
              || (now - last_print) > 1.0 / m_args.print_frequency)
          {
            spew("Got EstimatedLocalState from '%s'", resolveSystemId(msg->getSource()));
            last_print = now;
          }

          if (msg->getSource() == this->getSystemId())
          {
            if (resolveEntity(msg->getSourceEntity()).c_str() == m_args.centroid_els_entity_label)
            {
              //centroid message, extract heading and return
              m_curr_heading = msg->state->psi;
              return;
            }
            // Update local state. Flag used to check if to run controller or not.
            m_self_local_updated = true;
            m_local_state = *msg;
            // Update NED velocity (only really needed from local vehicle)
            m_v_ned(0) = msg->state->vx;
            m_v_ned(1) = msg->state->vy;
            m_v_ned(2) = msg->state->vz;
            // Update NED acceleration (only really needed from local vehicle)
            m_a_ned(0) = msg->acc->x;
            m_a_ned(1) = msg->acc->y;
            m_a_ned(2) = msg->acc->z;
          }

          for (unsigned int uav = 0; uav < m_N; uav++)
          {
            if (m_uav_ID[uav] == msg->getSource())
            {
              // Get current time and time of transmission
              double stamp = msg->ots;

              // Calculate update frequency
              double diff = now - m_last_pos_update(uav);
              if (diff > 0)
              {
                double rate = 1 / diff;
                // Unless first data: Apply smoothing
                if (!m_pos_update_rate(uav))
                  m_pos_update_rate(uav) = rate;
                else
                  m_pos_update_rate(uav) = lowPassSmoothing(
                      m_pos_update_rate(uav), rate, diff,
                      m_args.meta_smoothing_T);
              }
              else
              {
                if (m_args.print_EstimatedLocalState_warning)
                  war("Received old EstimatedLocalState! Diff = %f seconds", diff);
              }

              // Do some logging
              if (uav == 1)
                m_error_log.virt_err_x = diff;
              if (uav == 2)
                m_error_log.virt_err_y = diff;

              // Calculate update delay
              double delay_ms = (now - stamp) * 1E3;
              // Unless first data: Apply smoothing
              if (!m_pos_update_delay(uav))
                m_pos_update_delay(uav) = delay_ms;
              else
                m_pos_update_delay(uav) = lowPassSmoothing(
                    m_pos_update_delay(uav), delay_ms, diff,
                    m_args.meta_smoothing_T);

              // If update from other vehicle, delay will only be calculated correctly if clock is synched.
              // Hence, warning is only given if synched or update is local.
              bool missing_utc_synch = now < UTC_SECS_2015
                  || stamp < UTC_SECS_2015;
              if ((!missing_utc_synch || uav == m_i)
                  && delay_ms > m_args.delay_threshold)
              {
                war("Positioning delay for vehicle '%s' above threshold: %f",
                    resolveSystemId(msg->getSource()), delay_ms);
              }

              // Update position
              //spew("Update positions, uav=%d,m_x.size()=%d",uav,m_x.size());
              m_x(0, uav) = msg->state->x;
              m_x(1, uav) = msg->state->y;
              m_x(2, uav) = msg->state->z;

              //spew("m_last_pos_update.size()=%d",m_last_pos_update.size());
              m_last_pos_update(uav) = now;

              if (!m_args.print_frequency || !last_print
                  || (now - last_print) > 1.0 / m_args.print_frequency)
              {
                // Print stuff for debugging
                trace("Update frequency [Hz]:");
                printMatrix(m_pos_update_rate, DEBUG_LEVEL_TRACE);
                trace("Update delay [ms]:");
                printMatrix(m_pos_update_delay, DEBUG_LEVEL_TRACE);
                trace("Positions [m]:");
                printMatrix(m_x, DEBUG_LEVEL_TRACE);

                last_print = now;
              }
              break;
            }
          }
        }

        //! Consume Others stream velocities
        void
        consume(const IMC::EstimatedStreamVelocity* msg)
        {
          if (!m_configured)
          {
            spew("Not configured!");
            return;
          }
          double now = Clock::get();
          static double last_print;
          if (!m_args.print_frequency || !last_print
              || (now - last_print) > 1.0 / m_args.print_frequency)
          {
            spew("Got EstimatedStreamVelocity from '%s'", resolveSystemId(msg->getSource()));
            last_print = now;
          }



          for (unsigned int uav = 0; uav < m_N; uav++)
          {
            if (m_uav_ID[uav] == msg->getSource())
            {
              // Todo: Add some delay checking.

              // Update estimates
              m_esvs(0, uav) = msg->x;
              m_esvs(1, uav) = msg->y;
              m_esvs(2, uav) = msg->z;

              break;
            }
          }
        }

        void
        consume(const IMC::FormCoord* msg)
        {
          if (!m_configured)
          {
            spew("Not configured!");
            return;
          }
          double now = Clock::getSinceEpoch();
          static double last_print;
          if (!m_args.print_frequency || !last_print
              || (now - last_print) > 1.0 / m_args.print_frequency)
          {
            spew("Got FormCoord from system '%s' and entity '%s'.",
                         resolveSystemId(msg->getSource()),
                         resolveEntity(msg->getSourceEntity()).c_str());
            last_print = now;
          }


          // Check if request to start formation
          if (msg->type == IMC::FormCoord::FCT_REQUEST
              && msg->op == IMC::FormCoord::FCOP_START)
          {
            if (m_args.hold_current_formation)
            {
              inf("Using current vehicle positions as desired formation.");
              // Set desired formation to current positions
              m_x_c = m_x;
              m_x_c_default = transpose(RNedCentroid())*m_x;
              // Update desired difference variables matrix
              calcDiffVariable(&m_z_d, m_D, m_x_c);
              printMatrix(m_z_d);
            }
          }
        }

        bool
        isDesiredLinear(uint8_t msgSourceEntity)
        {
          std::string msgEntity = resolveEntity(msgSourceEntity).c_str();
          for (std::vector<std::string>::iterator it = m_args.desired_linear_entity_labels.begin(); it != m_args.desired_linear_entity_labels.end(); ++it)
            if (*it == msgEntity)
              return true;
          return false;
        }

        bool
        isDesiredHeading(uint8_t msgSourceEntity)
        {
          std::string msgEntity = resolveEntity(msgSourceEntity).c_str();
          for (std::vector<std::string>::iterator it = m_args.desired_heading_entity_labels.begin(); it != m_args.desired_heading_entity_labels.end(); ++it)
            if (*it == msgEntity)
              return true;
          return false;
        }

        void
        consume(const IMC::DesiredLinearState* msg)
        {
          double now = Clock::getSinceEpoch();
          static double last_print;
          bool new_mission_linearstate;

          //Desired linear state should only come from master only
          //if receiving local message, it should have the correct entity (desired vs reference)
          new_mission_linearstate = true;
          if (   msg->getSource() == this->getSystemId()
              && !isDesiredLinear(msg->getSourceEntity())      )
            new_mission_linearstate = false;

          if (!m_args.print_frequency || !last_print
              || (now - last_print) > 1.0 / m_args.print_frequency)
          {
            trace("Got DesiredLinearState from '%s'", resolveSystemId(msg->getSource()));
            trace("DesiredSurge [%f]",msg->vx);
            trace("DesiredHeave [%f]",msg->vz);
            trace("new_mission_linearstate=%d",new_mission_linearstate);
            last_print = now;
          }
          if (!new_mission_linearstate)
            return;
          //should contain the desired centroid velocity and acceleration

          m_v_mission_centroid(0) = 0;
          if ((msg->flags & IMC::DesiredLinearState::FL_VX) != 0)
            m_v_mission_centroid(0) = msg->vx;
          m_v_mission_centroid(1) = 0;
          if ((msg->flags & IMC::DesiredLinearState::FL_VY) != 0)
            m_v_mission_centroid(1) = msg->vy;
          m_v_mission_centroid(2) = 0;
          if ((msg->flags & IMC::DesiredLinearState::FL_VZ) != 0)
            m_v_mission_centroid(2) = msg->vz;

          m_a_mission_centroid(0) = 0;
          if ((msg->flags & IMC::DesiredLinearState::FL_AX) != 0)
            m_a_mission_centroid(0) = msg->ax;
          m_a_mission_centroid(1) = 0;
          if ((msg->flags & IMC::DesiredLinearState::FL_AY) != 0)
            m_a_mission_centroid(1) = msg->ay;
          m_a_mission_centroid(2) = 0;
          if ((msg->flags & IMC::DesiredLinearState::FL_AZ) != 0)
            m_a_mission_centroid(2) = msg->az;

          if (!m_args.print_frequency || !last_print
              || (now - last_print) > 1.0 / m_args.print_frequency)
          {
            trace("m_v_mission_centroid=[%f,%f,%f]",m_v_mission_centroid(0),m_v_mission_centroid(1),m_v_mission_centroid(2));
            trace("m_a_mission_centroid=[%f,%f,%f]",m_a_mission_centroid(0),m_a_mission_centroid(1),m_a_mission_centroid(2));
          }
        }

        void
        consume(const IMC::DesiredHeading* msg)
        {
          double now = Clock::getSinceEpoch();
          static double last_print;
          if (!m_args.print_frequency || !last_print
              || (now - last_print) > 1.0 / m_args.print_frequency)
          {
            spew("Got DesiredHeading from '%s'", resolveSystemId(msg->getSource()));
            spew("DesiredHeading [%f]",msg->value);
            last_print = now;
          }
          //Desired heading should only come from master only
          //if receiving local message, it should have the correct entity (desired vs reference)
          if (   msg->getSource() == this->getSystemId()
              && !isDesiredHeading(msg->getSourceEntity())      )
            return;
          /*
          if (abs(msg->value - m_desired_heading.value) > 0.0)
          {
            return;
          }*/
          m_desired_heading = *msg;
          m_curr_desired_heading = m_desired_heading.value;


          bool id_found = false;
          for (unsigned int uav = 0; uav < m_N; uav++)
          {
            if (m_uav_ID[uav] == msg->getSource())
            {
              id_found = true;
              m_last_heading_update(uav) = now;
            }
          }
          if (!id_found)
            war("Received DesiredHeading from unknown vehicle '%s'",
                resolveSystemId(msg->getSource()));
        }

        //! Print matrix (for debuging)
        void
        printMatrix(Matrix m, DUNE::Tasks::DebugLevel dbg = DEBUG_LEVEL_DEBUG)
        {
          if (getDebugLevel() >= dbg)
          {
            printf("[DEBUG Matrix]\n");
            for (int i = 0; i < m.rows(); i++)
            {
              for (int j = 0; j < m.columns(); j++)
              {
                printf("%f ", m.element(i, j));
              }
              printf("\n");
            }
          }
        }

        //! Saturate
        Matrix
        saturate(Matrix vect, double sat)
        {
          if (vect.norm_2() > sat)
            vect *= sat / vect.norm_2();
          return vect;
        }

        Matrix
        lowPassSmoothing(Matrix output, Matrix input, double dt, double RC)
        {
          if (output.isEmpty())
            return input;

          // Check valid inputs
          assert(input.size() == output.size());
          assert(dt > 0);
          assert(RC > 0);

          // Calculate alpha
          double alpha = dt / (RC + dt);

          // Apply smoothing to all data
          for (int i = 0; i < input.size(); i++)
          {
            output(i) += alpha * (input(i) - output(i));
          }
          return output;
        }

        double
        lowPassSmoothing(double output, double input, double dt, double RC)
        {
          // Check valid inputs
          assert(dt > 0);
          assert(RC > 0);

          // Calculate alpha
          double alpha = dt / (RC + dt);

          // Apply smoothing
          return output += alpha * (input - output);
        }

        //! Calculate difference variables
        //! @param[in] z reference to matrix with difference variables
        //! @param[in] D incidence matrix
        //! @param[in] x matrix with coordinated variables
        void
        calcDiffVariable(Matrix* z, Matrix D, Matrix x)
        {
          unsigned int N = D.rows();
          unsigned int L = D.columns();

          for (unsigned int link = 0; link < L; link++)
          {
            Matrix z_k(3, 1, 0);
            for (unsigned int uav = 0; uav < N; uav++)
            {
              z_k += D(uav, link) * x.column(uav);
            }
            z->put(0, link, z_k);
          }
        }

        //! Check if any reason to abort formation (e.g. pos.data timeout/missing)
        void
        checkFormation(void)
        {
          static double last_abort_sent;

          // Check that age of pos.data from all vehicles is within threshold
          double now = Clock::getSinceEpoch();
          for (unsigned int uav = 0; uav < m_N; uav++)
          {
            double pos_update_age = (now - m_last_pos_update(uav)) * 1E3;
            if (pos_update_age > m_args.delay_threshold)
            {
              war("Age of pos.data for '%s' above threshold (%1.1f): %1.1f ms",
                  resolveSystemId(m_uav_ID[uav]), m_args.delay_threshold,
                  pos_update_age);

              // Issue abort if more than set time since last abort (to avoid spam)
              if ((now - last_abort_sent) * 1E3 > m_args.abort_resend_threshold)
              {
                IMC::Abort abort_msg;
                abort_msg.setDestination(getSystemId());
                dispatch(abort_msg);
                debug("Abort sent.");
                last_abort_sent = now;
              }
            }
          }
        }

        //! Calculate stram sync bias
        Matrix
        streamSyncBias(void)
        {
          Matrix u_bias(3, 1, 0);

          Matrix bias_z = Matrix();
          bias_z.resizeAndFill(3, m_L, 0);

          // Calculate z_tilde
          calcDiffVariable(&bias_z, m_D, m_esvs);

          Matrix z_tilde = bias_z;

          for (unsigned int link = 0; link < m_L; link++)
          {
            u_bias -= m_D(m_i, link) * z_tilde.column(link);
          }

          return u_bias;
        }

        //! Calculate formation velocity
        Matrix
        formationVelocity(void)
        {
          Matrix u_form(3, 1, 0);
          if (m_args.disable_formation_velocity)
            return u_form;

          // Calculate z_tilde
          calcDiffVariable(&m_z, m_D, m_x);

          Matrix z_tilde = m_z - m_z_d;
          if (m_args.link_filter_enable)
          {
            double beta = m_args.link_filter_beta;
            m_z_filtered = m_z*beta + (1-beta)*m_z_filtered;
            z_tilde = m_z_filtered - m_z_d;
          } 

          m_coord_state.link_distance       = m_z.norm_2();
          m_coord_state.link_distance_error = z_tilde.norm_2();
          m_coord_state.link_vel = 0; //currently not available
          dispatch(m_coord_state);
          // Calculate formation velocity component
          static double last_print;

          Matrix link_errors_agent = Matrix(3,1, 0.0);

          if (m_gain_scheduler.enable)
          {
            double gain = sigmoidGain(z_tilde);

            double now = Clock::getSinceEpoch();
            if (!m_args.print_frequency || !last_print
                || (now - last_print) > 1.0 / m_args.print_frequency)
            {
              trace("Link gain = %f",gain);
              trace("Z_tilde_n = %f",z_tilde.norm_2());
              last_print = now;
            }

            for (unsigned int link = 0; link < m_L; link++)
            {
              u_form -= m_D(m_i, link) * gain * z_tilde.column(link);
              // FIeld hack: Take the gain to one third
              u_form(2) *= 0.3;
              link_errors_agent += m_D(m_i, link) * z_tilde.column(link);
            }
          }
          else
          {
            for (unsigned int link = 0; link < m_L; link++)
            {
              u_form -= m_D(m_i, link) * m_delta(link) * z_tilde.column(link);

              link_errors_agent += m_D(m_i, link) * z_tilde.column(link);
            }
          }
          //spew("u_form: [%1.1f, %1.1f, %1.1f]", u_form(0), u_form(1), u_form(2));
          // Store the logs

          m_error_log.err_x = link_errors_agent(0);
          m_error_log.err_y = link_errors_agent(1);
          m_error_log.err_z = link_errors_agent(2);
          m_error_log.err   = link_errors_agent.norm_2();

          // Log mission velocities
          logAndDispatchParcel(PC_FORMATION_X, u_form);

          return u_form;
        }

        //! Calculate collision avoidance velocity
        double
        sigmoidGain(Matrix z_tilde)
        {
          double z_t_norm = z_tilde.norm_2();
          double temp = double(1) + std::exp(-m_gain_scheduler.slope*(z_t_norm - m_gain_scheduler.switch_distance));
          return m_gain_scheduler.sigmoid_const + (m_gain_scheduler.gain_far-m_gain_scheduler.sigmoid_const)/temp;
        }

        //! Calculate collision avoidance velocity
        Matrix
        collAvoidVelocity(void)
        {
          Matrix u_coll(3, 1, 0);
          if (m_args.disable_collision_velocity)
            return u_coll;

          static double u_coll_max = 0;
          static double d_ij_min = std::numeric_limits<double>::infinity();
          static Matrix d_ij_prev(1, m_N, m_args.collision_radius);

          for (unsigned int uav = 0; uav < m_N; uav++)
          {
            // Only calc for other vehicles that we have received positions for
            if (uav != m_i && m_pos_update_rate(uav) > 0)
            {
              // Get vector and distance to neighbour
              Matrix x_ij = m_x.column(uav) - m_x.column(m_i);
              double d_ij = x_ij.norm_2();
              // Check if distance is zero (to avoid div by zero)
              if (d_ij == 0)
              {
                war("Distance to '%s' is zero!", resolveSystemId(m_uav_ID[uav]));
                continue;
              }

              // Check if inside potential field
              if (d_ij < m_args.collision_radius)
              {
                // Warning if still closing
                if (d_ij < d_ij_prev(uav))
                {
                  war("Proximity warning with '%s': %1.2f m and closing!",
                      resolveSystemId(m_uav_ID[uav]), d_ij);
                }
                d_ij_prev(uav) = d_ij;

                // Add repelling velocity
                u_coll -= std::min(m_args.collision_saturation,
                                   m_args.collision_radius - d_ij) * x_ij / d_ij;
              }
              // Save minimum distance
              d_ij_min = std::min(d_ij_min, d_ij);
            }
          }
          // Multiply total repelling velocity with gain
          u_coll *= m_args.collision_gain;
          // Save (static) maximum repelling speed
          u_coll_max = std::max(u_coll_max, u_coll.norm_2());

          /*
          spew("u_coll: [%1.1f, %1.1f, %1.1f]", u_coll(0), u_coll(1), u_coll(2));
          spew("Max u_coll: %1.1f", u_coll_max);
          spew("Min dist: %1.1f", d_ij_min);
          */


          // Log mission velocities
          logAndDispatchParcel(PC_COLLISION_X, u_coll);

          return u_coll;
        }

        //! Calculate mission velocity (in CENTROID body)
        Matrix
        missionVelocity(void)
        {
          Matrix u_mission(3, 1, 0);
          if (m_args.disable_mission_velocity)
            return u_mission;

          // Use constant mission velocity if set non-zero
          if (m_args.const_mission_velocity.norm_2() > 0)
            u_mission = m_args.const_mission_velocity;
          else
            u_mission = m_v_mission_centroid;

          if (!m_args.use_altitude)
            u_mission(2) = 0;


          // Log mission velocities
          Matrix tmp = RNedCentroid()*u_mission;
          logAndDispatchParcel(PC_MISSION_X, tmp);


          return u_mission;
        }

        void
        logAndDispatchParcel(uint8_t parcelIndexStart, Matrix& data, bool doDispatch = true)
        {
          if (parcelIndexStart + data.size() > NUM_PARCELS)
          {
            war("Invalid parcel index + data size given");
            return;
          }

          for (int i = 0; i < data.size(); ++i)
          {
            m_parcels[parcelIndexStart + i].p = data(i);
            if (doDispatch)
            {
              dispatch(m_parcels[parcelIndexStart + i]);
            }
          }


        }

        //! Control velocity in AGENT frame, return desired force in NED
        //! u in AGENT body
        //! v_des in NED
        //! dv_des in NED
        Matrix
        velocityControl(Matrix u, Matrix v_des, Matrix dv_des)
        {
          //m_v and m_a in AGENT body

          //F_i in AGENT i body
          Matrix F_i = Matrix(3, 1, 0.0);
          Matrix Rni = RNedAgent();

          Matrix v_error_ned  = v_des - m_v_ned;
          Matrix dv_error_ned = dv_des - m_a_ned;

          // F_i is transformed to NED before dispatch.
          F_i(0) = m_args.Kp(0) * v_error_ned(0);
          F_i(1) = m_args.Kp(1) * v_error_ned(1);
          F_i(2) = m_args.Kp(2) * v_error_ned(2);

          // Add damping on acceleration. (Basically acceleration feed-forward. Will act as a mass-increaser, might make more robust to wind)
          F_i(0) += m_args.Kd(0) * dv_error_ned(0);
          F_i(1) += m_args.Kd(1) * dv_error_ned(1);
          F_i(2) += m_args.Kd(2) * dv_error_ned(2);

          // Do integration of the mission velocity error
          int formation_int_enable = m_args.enable_formation_integration ? 1.0 : 0.0;

          Matrix u_bias_est = 0.1*u;
          double max_contr = 0.5;
          if (u_bias_est.norm_2() > max_contr)
            u_bias_est = max_contr * u_bias_est / u_bias_est.norm_2();

          Matrix u_sync = Matrix(3,1, 0.0);
          if (m_args.enable_bias_sync)
          {
            u_sync = m_args.bias_sync_gain*streamSyncBias();
          }

          m_bias_estimate(0) += ((double)m_time_diff/1.0E3) * m_args.Ki(0) * ((v_error_ned(0) + formation_int_enable * u_bias_est(0)) + u_sync(0));
          m_bias_estimate(1) += ((double)m_time_diff/1.0E3) * m_args.Ki(1) * ((v_error_ned(1) + formation_int_enable * u_bias_est(1)) + u_sync(1));
          m_bias_estimate(2) += ((double)m_time_diff/1.0E3) * m_args.Ki(2) * ((v_error_ned(2) + formation_int_enable * u_bias_est(2)) + u_sync(2));


          // Integral anti-windup
          // Relationship is b = d * wind
          if (m_bias_estimate.norm_2() / m_args.wind_drag_coefficient > m_args.max_wind_speed_estimate)
            m_bias_estimate = (m_args.max_wind_speed_estimate * m_args.wind_drag_coefficient) * m_bias_estimate / m_bias_estimate.norm_2();

          // Add acceleration feed-forward and coordination input u
          F_i += m_args.mass * dv_des + u;

          // Add optional bias compensation
          if (m_args.enable_bias_compensation)
            F_i += m_bias_estimate;

          // Add optional wind feed forward
          if (m_args.enable_wind_ff)
            F_i += m_args.wind_drag_coefficient * m_v_ned;

          // Do some logging.
          m_error_log.rf_err_vx = v_error_ned(0);
          m_error_log.rf_err_vy = v_error_ned(1);
          m_error_log.rf_err_vz = v_error_ned(2);

          m_error_log.ss_x = m_bias_estimate(0);
          m_error_log.ss_y = m_bias_estimate(1);
          m_error_log.ss_z = m_bias_estimate(2);


          dispatch(m_error_log);

          IMC::EstimatedStreamVelocity esv;
          esv.x = m_bias_estimate(0)/m_args.wind_drag_coefficient;
          esv.y = m_bias_estimate(1)/m_args.wind_drag_coefficient;
          esv.z = m_bias_estimate(2)/m_args.wind_drag_coefficient;

          dispatch(esv, DF_LOOP_BACK);


          return F_i;
        }

        //! Dispatch desired force
        void
        sendDesiredForce(Matrix F_des)
        {

          // Option to turn force into desired acceleration.
          if (m_args.enable_mass_division)
            F_des = F_des / m_args.mass;


          if (F_des.norm_2() > m_args.max_norm_F)
          {
            F_des = sqrt(pow(m_args.max_norm_F, 2)) * F_des / F_des.norm_2();
          }

          m_desired_force.x = F_des(0);
          m_desired_force.y = F_des(1);
          m_desired_force.z = F_des(2);


          // Enable desired force in all directions
          m_desired_force.flags =   IMC::DesiredControl::FL_X
                                  | IMC::DesiredControl::FL_Y
                                  | IMC::DesiredControl::FL_Z;


          // Optional disables.
          if (m_args.disable_force_output)
            m_desired_force.flags = 0x00;
          else if (m_args.disable_heave)
            m_desired_force.flags =   IMC::DesiredControl::FL_X
                                    | IMC::DesiredControl::FL_Y;


          dispatch(m_desired_force);

        }

        //! Main loop.
        void
        task(void)
        {
          if (!m_configured)
          {
            spew("Not configured!");
            setEntityState(IMC::EntityState::ESTA_ERROR, Status::CODE_MISSING_DATA);
          }
          if (!m_args.use_controller || !isActive())
            setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_IDLE);

          if (!m_args.use_controller || !isActive() || !m_configured)
            return;

          if (!m_self_local_updated)
            return;

          // Reset the flag
          m_self_local_updated = false;



          if (m_N > 1)
          {
            updateFormation();
          }
          // Check if we should abort
          checkFormation();

          // Calculate internal feedback, tau in NED
          // Mission vel in NED.


          // Get mission information in NED frame.
          Matrix v_mission_ned  = RNedCentroid()*missionVelocity();
          Matrix dv_mission_ned = RNedCentroid()*m_a_mission_centroid;

          // Saturate
          v_mission_ned = saturate(v_mission_ned, m_args.max_speed);

          // update formation based on current desired heading
          // NB: only master should change according to his desired heading?
          Matrix u = Matrix(3,1,0.0);
          if (m_N > 1)
          {
            // Calculate external feedback, u (desired velocity in NED)
            u = formationVelocity() + collAvoidVelocity();
            // Set heave to zero if not controlling altitude
            if (!m_args.use_altitude)
              u(2) = 0;
          }




          // calculate velocity control force, tau
          m_time_diff = Clock::getMsec() - m_time_end;
          m_time_end = Clock::getMsec();

          if (m_time_diff > 1*1E3)
            err("Overflow in task execution!");

          // Tau is in NED-frame.
          Matrix tau = velocityControl(u, v_mission_ned, dv_mission_ned);
          //if (m_args.disable_force_output)
          //  return;
          sendDesiredForce(tau);


        }

        //! R^(agent)_(centroid)
        Matrix
        RAgentCentroid() const
        {
          return transpose(RNedAgent())*RNedCentroid();
        }

        //! R^(n)_(centroid)
        Matrix
        RNedCentroid() const
        {
          return Rz(m_curr_heading);
        }

        //! R^(n)_(agent)
        Matrix
        RNedAgent() const
        {
          return Rzyx(m_local_state.state->phi,m_local_state.state->theta,m_local_state.state->psi);
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
