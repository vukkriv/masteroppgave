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

// Local headers.
#include "PeriodicUAVAutopilot.hpp"

namespace Control
{
  namespace FormationControl
  {
    using DUNE_NAMESPACES;

    //! Controllable loops.
    static const uint32_t c_controllable = IMC::CL_PATH;
    //! Required loops.
    static const uint32_t c_required = IMC::CL_SPEED;

    //! Vector for System Mapping.
    typedef std::vector<uint32_t> Systems;

    //! UTC seconds for 2015-01-01 to check for clock synch
    const int UTC_SECS_2015 = 1420070400;

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
    };

    struct Task: public PeriodicUAVAutopilot
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

      //! Desired difference variables
      Matrix m_z_d;

      //! Difference variables
      Matrix m_z;

      //! Desired formation positions
      Matrix m_x_c;

      //! Vehicle positions
      Matrix m_x;

      //! Vehicle velocities
      Matrix m_v;

      //! Mission velocity
      Matrix m_v_mission;

      //! Last time position was updated for each vehicle
      Matrix m_last_pos_update;

      //! Rate [Hz] and delay [ms] of position update for each vehicle. Both zero for vehicles we have yet to receive positions from.
      Matrix m_pos_update_rate, m_pos_update_delay;

      //! Desired velocity
      //IMC::DesiredVelocity m_desired_velocity;
      IMC::TranslationalSetpoint m_desired_velocity;



      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        PeriodicUAVAutopilot(name, ctx, c_controllable, c_required),
        //Periodic(name, ctx),
        m_i(0),
        m_N(0),
        m_L(0)
      {
        param("Formation Controller", m_args.use_controller)
        .visibility(Tasks::Parameter::VISIBILITY_USER)
        .scope(Tasks::Parameter::SCOPE_MANEUVER)
        .defaultValue("false")
        .description("Enable Formation Controller.");

        param("Vehicle List", m_args.formation_systems)
        .defaultValue("")
        .visibility(Tasks::Parameter::VISIBILITY_USER)
        .description("System name list of the formation vehicles.");

        param("Desired Formation", m_args.desired_formation)
        .defaultValue("0.0, 0.0, 0.0")
        .units(Units::Meter)
        .visibility(Tasks::Parameter::VISIBILITY_USER)
        .description("Desired formation positions matrix.");

        param("Incidence Matrix", m_args.incidence_matrix)
        .defaultValue("0")
        .visibility(Tasks::Parameter::VISIBILITY_USER)
        .description("Incidence matrix.");

        param("Link Gains", m_args.link_gains)
        .defaultValue("1.0")
        .visibility(Tasks::Parameter::VISIBILITY_USER)
        .description("Gains assigned to formation links.");

        param("Disable Formation Velocity", m_args.disable_formation_velocity)
        .defaultValue("false")
        .visibility(Tasks::Parameter::VISIBILITY_USER)
        .description("Disable formation velocity.");

        param("Disable Mission Velocity", m_args.disable_mission_velocity)
        .defaultValue("false")
        .visibility(Tasks::Parameter::VISIBILITY_USER)
        .description("Disable mission velocity.");

        param("Constant Mission Velocity", m_args.const_mission_velocity)
        .defaultValue("0.0, 0.0, 0.0")
        .units(Units::MeterPerSecond)
        .visibility(Tasks::Parameter::VISIBILITY_USER)
        .description("Constant mission velocity.");

        param("Disable Collision Velocity", m_args.disable_collision_velocity)
        .defaultValue("false")
        .visibility(Tasks::Parameter::VISIBILITY_USER)
        .description("Disable collision velocity.");

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

        param("Hold Current Formation", m_args.hold_current_formation)
        .defaultValue("true")
        .visibility(Tasks::Parameter::VISIBILITY_USER)
        .description("Use current position of vehicles as desired formation");

        param("Resend Abort Threshold", m_args.abort_resend_threshold)
        .defaultValue("200")
        .units(Units::Millisecond)
        .description("Time allowed to pass before resending an abort.");

        param("Print Frequency", m_args.print_frequency)
        .defaultValue("0.0")
        .units(Units::Second)
        .description("Frequency of pos.data prints. Zero => Print on every update.");

        // Bind incoming IMC messages
        bind<IMC::FormPos>(this);
        bind<IMC::FormCoord>(this);
        bind<IMC::DesiredVelocity>(this);
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        debug("Starting update of parametes.");

        bool calc_desired_diff_var = false;


        if (paramChanged(m_args.formation_systems))
        {
          inf("New Formation vehicles' list.");

          // Extract vehicle IDs
          m_uav_ID.clear();
          if (m_args.formation_systems.empty())
          {
            war("Formation vehicle list is empty!");
            m_uav_ID.push_back(this->getSystemId());
            m_N = 1;
            m_i = 0;
          }
          else
          {
            m_N = m_args.formation_systems.size();
            bool found_self = false;
            for (unsigned int uav = 0; uav < m_N; uav++)
            {
              debug("UAV %u: %s", uav, m_args.formation_systems[uav].c_str());
              m_uav_ID.push_back(this->resolveSystemName(m_args.formation_systems[uav]));
              if (m_uav_ID[uav] == this->getSystemId())
              {
                m_i = uav; // Set my formation id
                found_self = true;
              }

            }
            if (!found_self)
              throw DUNE::Exception("Vehicle not found in formation vehicle list!");
          }
          // Resize and reset position and velocity matrices to fit number of vehicles
          m_x.resizeAndFill(3,m_N,0);
          m_v.resizeAndFill(3,m_N,0);
        }


        if (paramChanged(m_args.desired_formation))
        {
          inf("New desired formation.");

          // Check dimensions
          if (m_args.desired_formation.size() == 0)
            throw DUNE::Exception("Desired formation positons matrix is empty!");
          if (m_args.desired_formation.size()%3 != 0)
            throw DUNE::Exception("Unvalid number of coordinates in desired formation positions matrix!");
          if ((unsigned int)m_args.desired_formation.size()/3 != m_N)
            throw DUNE::Exception("Incorrect number of vehicles in desired formation positions matrix!");

          // Resize desired formation matrix to fit number of vehicles
          m_x_c.resizeAndFill(3, m_N, 0);
          // Update desired formation matrix
          for (unsigned int uav_id = 0; uav_id < m_N; uav_id++)
          {
            for (unsigned int coord = 0; coord < 3; coord++)
              m_x_c(coord, uav_id) = m_args.desired_formation(coord + uav_id*3);
            debug("UAV %u: [%1.1f, %1.1f, %1.1f]", uav_id,
                m_x_c(0, uav_id), m_x_c(1, uav_id), m_x_c(2, uav_id));
          }
          // Desired difference variables will have to be calculated
          calc_desired_diff_var = true;
        }


        if (paramChanged(m_args.incidence_matrix))
        {
          inf("New incidence matrix.");

          // Check dimensions
          if (m_args.incidence_matrix.size() == 0)
            throw DUNE::Exception("Incidence matrix is empty!");
          if (m_args.incidence_matrix.rows()%m_N != 0)
            throw DUNE::Exception("Incidence matrix doesn't match number of vehicles!");

          // Update number of links
          m_L = m_args.incidence_matrix.rows()/m_N;
          // Resize incidence matrix
          m_D.resize(m_N,m_L);
          // Update incidence matrix
          for (unsigned int link = 0; link < m_L; link++)
          {
            for (unsigned int uav = 0; uav < m_N; uav++)
              m_D(uav, link) = m_args.incidence_matrix(uav + link*m_N);
          }
          // Desired difference variables will have to be calculated
          calc_desired_diff_var = true;

          printMatrix(m_D);
        }


        if (paramChanged(m_args.link_gains))
        {
          inf("New link gains.");

          if (m_args.link_gains.size() == 1)
          {
            // Scalar gain, set all to this
            m_delta = Matrix(m_L, 1, m_args.link_gains(0));
          }
          else if ((unsigned int)m_args.link_gains.size() != m_L)
            throw DUNE::Exception("Link gains doesn't match number of links!");
          else
          {
            // Update gains
            m_delta = m_args.link_gains;
          }

          printMatrix(m_delta);
        }


        if (calc_desired_diff_var)
        {
          inf("New desired difference variables.");

          // Resize and reset difference variables matrices to fit number of links
          m_z.resizeAndFill(3, m_L, 0);
          m_z_d.resizeAndFill(3, m_L, 0);
          // Update desired difference variables matrix
          calcDiffVariable(&m_z_d,m_D,m_x_c);

          printMatrix(m_z_d);
        }

        if (paramChanged(m_args.const_mission_velocity))
        {
          inf("New constant mission velocity.");

          // Check dimension
          if (m_args.const_mission_velocity.rows() != 3)
            throw DUNE::Exception("Unvalid mission velocity vector!");

          // Set constant mission velocity
          m_v_mission = m_args.const_mission_velocity;
          debug("Mission Velocity: [%1.1f, %1.1f, %1.1f]",
              m_v_mission(0), m_v_mission(1), m_v_mission(2));
        }
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
        PeriodicUAVAutopilot::onResourceInitialization();

        // Initialize matrices
        m_last_pos_update = Matrix(1, m_N, Clock::get());
        m_pos_update_rate = Matrix(1, m_N, 0);
        m_pos_update_delay = Matrix(1, m_N, 0);
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
      }

      void
      onActivation(void)
      {
      }

      void
      onDeactivation(void)
      {
        Matrix zero_vel(3,1,0);
        sendDesiredVelocity(zero_vel);
      }

      //! Consume Formation Position
      void
      consume(const IMC::FormPos* msg)
      {
        spew("Got FormPos from '%s'", resolveSystemId(msg->getSource()));

        static double last_print;

        bool id_found = false;
        for (unsigned int uav = 0; uav < m_N; uav++)
        {
          if (m_uav_ID[uav] == msg->getSource())
          {
            id_found = true;

            // Get current time and time of transmission
            double stamp = msg->ots;
            double now = Clock::getSinceEpoch();

            // Calculate update frequency
            double diff = now - m_last_pos_update(uav);
            if (diff > 0)
            {
              double rate = 1/diff;
              // Unless first data: Apply smoothing
              if (!m_pos_update_rate(uav))
                m_pos_update_rate(uav) = rate;
              else
                m_pos_update_rate(uav) = lowPassSmoothing(m_pos_update_rate(uav), rate, diff, m_args.meta_smoothing_T);
            }
            else
            {
              war("Received old FormPos! Diff = %f seconds",
                  diff);
            }

            // Calculate update delay
            double delay_ms = (now - stamp)*1E3;
            // Unless first data: Apply smoothing
            if (!m_pos_update_delay(uav))
              m_pos_update_delay(uav) = delay_ms;
            else
              m_pos_update_delay(uav) = lowPassSmoothing(m_pos_update_delay(uav), delay_ms, diff, m_args.meta_smoothing_T);

            // If update from other vehicle, delay will only be calculated correctly if clock is synched.
            // Hence, warning is only given if synched or update is local.
            bool missing_utc_synch = now < UTC_SECS_2015 || stamp < UTC_SECS_2015;
            if ((!missing_utc_synch || uav == m_i) && delay_ms > m_args.delay_threshold)
            {
              war("Positioning delay for vehicle '%s' above threshold: %f",
                  resolveSystemId(msg->getSource()), delay_ms);
            }

            // Update position
            m_x(0,uav) = msg->x;
            m_x(1,uav) = msg->y;
            m_x(2,uav) = msg->z;

            // Update velocity (only really needed from local vehicle)
            m_v(0,uav) = msg->vx;
            m_v(1,uav) = msg->vy;
            m_v(2,uav) = msg->vz;

            m_last_pos_update(uav) = now;


            if (!m_args.print_frequency || !last_print || (now - last_print) > 1.0/m_args.print_frequency)
            {
              // Print stuff for debugging
              trace("Update frequency [Hz]:");
              printMatrix(m_pos_update_rate,DEBUG_LEVEL_TRACE);
              trace("Update delay [ms]:");
              printMatrix(m_pos_update_delay,DEBUG_LEVEL_TRACE);
              trace("Positions [m]:");
              printMatrix(m_x,DEBUG_LEVEL_TRACE);
              trace("Velocities [m/s]:");
              printMatrix(m_v,DEBUG_LEVEL_TRACE);

              last_print = now;
            }

            break;
          }
        }
        if (!id_found)
          war("Received FormPos from unknown vehicle '%s'", resolveSystemId(msg->getSource()));
      }

      void
      consume(const IMC::FormCoord* msg)
      {
        trace("Got FormCoord from system '%s' and entity '%s'.",
              resolveSystemId(msg->getSource()),
              resolveEntity(msg->getSourceEntity()).c_str());

        // Check if request to start formation
        if (msg->type == IMC::FormCoord::FCT_REQUEST && msg->op == IMC::FormCoord::FCOP_START)
        {
          if (m_args.hold_current_formation)
          {
            inf("Using current vehicle positions as desired formation.");
            // Set desired formation to current positions
            m_x_c = m_x;
            // Update desired difference variables matrix
            calcDiffVariable(&m_z_d,m_D,m_x_c);
            printMatrix(m_z_d);
          }
        }
      }

      //! Consume Desired Velocity (mission velocity from FormationGuidance)
      void
      consume(const IMC::DesiredVelocity* msg)
      {
        m_v_mission(0) = msg->u;
        m_v_mission(1) = msg->v;
        m_v_mission(2) = msg->w;

        spew("Got Mission Velocity: [%1.1f, %1.1f, %1.1f]",
            m_v_mission(0), m_v_mission(1), m_v_mission(2));
      }


      //! Print matrix (for debuging)
      void
      printMatrix(Matrix m, DUNE::Tasks::DebugLevel dbg = DEBUG_LEVEL_DEBUG){
        if (getDebugLevel() >= dbg)
        {
          printf("[DEBUG Matrix]\n");
          for(int i = 0; i<m.rows(); i++ ){
            for(int j = 0; j<m.columns();j++){
              printf("%f ", m.element(i,j));
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
          vect *= sat/vect.norm_2();
        return vect;
      }

      Matrix
      lowPassSmoothing(Matrix output, Matrix input, double dt, double RC)
      {
        if (output.isZeroSized())
          return input;

        // Check valid inputs
        assert(input.size() == output.size());
        assert(dt > 0);
        assert(RC > 0);

        // Calculate alpha
        double alpha = dt/(RC + dt);

        // Apply smoothing to all data
        for (int i = 0; i < input.size(); i++)
        {
          output(i) += alpha*(input(i) - output(i));
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
        double alpha = dt/(RC + dt);

        // Apply smoothing
        return output += alpha*(input - output);
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
          Matrix z_k(3,1,0);
          for (unsigned int uav = 0; uav < N; uav++)
          {
            z_k += D(uav,link)*x.column(uav);
          }
          z->put(0,link,z_k);
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
          double pos_update_age = (now - m_last_pos_update(uav))*1E3;
          if (pos_update_age > m_args.delay_threshold)
          {
            war("Age of pos.data for '%s' above threshold (%1.1f): %1.1f ms",
                resolveSystemId(m_uav_ID[uav]), m_args.delay_threshold, pos_update_age);

            // Issue abort if more than set time since last abort (to avoid spam)
            if ((now - last_abort_sent)*1E3 > m_args.abort_resend_threshold)
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

      //! Calculate formation velocity
      Matrix
      formationVelocity(void)
      {
        Matrix u_form(3,1,0);
        if (m_args.disable_formation_velocity)
          return u_form;

        // Calculate z_tilde
        calcDiffVariable(&m_z, m_D, m_x);
        Matrix z_tilde = m_z - m_z_d;

        // Calculate formation velocity component
        for (unsigned int link = 0; link < m_L; link++)
        {
          u_form -= m_D(m_i,link)*m_delta(link)*z_tilde.column(link);
        }

        spew("u_form: [%1.1f, %1.1f, %1.1f]",
            u_form(0), u_form(1), u_form(2));
        return u_form;
      }

      //! Calculate collision avoidance velocity
      Matrix
      collAvoidVelocity(void)
      {
        Matrix u_coll(3,1,0);
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
              war("Distance to '%s' is zero!",
                  resolveSystemId(m_uav_ID[uav]));
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
              u_coll -= std::min(m_args.collision_saturation, m_args.collision_radius - d_ij)*x_ij/d_ij;
            }
            // Save minimum distance
            d_ij_min = std::min(d_ij_min, d_ij);

          }
        }
        // Multiply total repelling velocity with gain
        u_coll *= m_args.collision_gain;
        // Save maximum repelling speed
        u_coll_max = std::max(u_coll_max, u_coll.norm_2());

        spew("u_coll: [%1.1f, %1.1f, %1.1f]",
            u_coll(0), u_coll(1), u_coll(2));
        spew("Max u_coll: %1.1f", u_coll_max);
        spew("Min dist: %1.1f", d_ij_min);
        return u_coll;
      }

      //! Calculate mission velocity
      Matrix
      missionVelocity(void)
      {
        Matrix u_mission(3,1,0);
        if (m_args.disable_mission_velocity)
          return u_mission;

        // Use constant mission velocity if set non-zero
        if (m_args.const_mission_velocity.norm_2() > 0)
          u_mission = m_args.const_mission_velocity;
        else
          u_mission = m_v_mission;

        spew("u_mission: [%1.1f, %1.1f, %1.1f]",
            u_mission(0), u_mission(1), u_mission(2));
        return u_mission;
      }

      //! Dispatch desired velocity
      void
      sendDesiredVelocity(Matrix velocity)
      {
        m_desired_velocity.u = velocity(0);
        //m_desired_velocity.flags |= IMC::DesiredVelocity::FL_SURGE;
        m_desired_velocity.flags |= IMC::TranslationalSetpoint::FL_SURGE;

        m_desired_velocity.v = velocity(1);
        m_desired_velocity.flags |= IMC::TranslationalSetpoint::FL_SWAY;

        m_desired_velocity.w = velocity(2);
        m_desired_velocity.flags |= IMC::TranslationalSetpoint::FL_HEAVE;

        dispatch(m_desired_velocity);
        spew("v_d: [%1.1f, %1.1f, %1.1f]",
            m_desired_velocity.u, m_desired_velocity.v, m_desired_velocity.w);
      }


      //! Main loop.
      void
      task(void)
      {
        if(!m_args.use_controller || !isActive())
          return;

        // Check if we should abort
        checkFormation();

        // Calculate external feedback, u
        Matrix u = formationVelocity() + collAvoidVelocity();

        // Calculate internal feedback, tau
        Matrix tau = u + missionVelocity();

        // Saturate and dispatch control output
        tau = saturate(tau,m_args.max_speed);
        sendDesiredVelocity(tau);
      }
    };
  }
}

DUNE_TASK
