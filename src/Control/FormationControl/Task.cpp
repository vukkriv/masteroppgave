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

namespace Control
{
  namespace FormationControl
  {
    using DUNE_NAMESPACES;

    //! Vector for System Mapping.
    typedef std::vector<uint32_t> Systems;

    struct Arguments
    {
      //! Use controller
      bool use_controller;

      //! Vehicle list
      std::vector<std::string> formation_systems;

      //! Desired formation
      Matrix desired_formation;

      //! Incidence matrix
      Matrix incidence_matrix;

      //! Link gains
      Matrix link_gains;

      //! (optional) Constant mission velocity
      Matrix const_mission_velocity;

      //! Collision avoidance radius
      double collision_radius;

      //! Collision avoidance gain
      double collision_gain;

      //! Maximum speed
      double max_speed;

      //! Memory factor when calculating average update rates
      double mem_factor;

      //! Threshold for delay on positioning updates
      double delay_threshold;
    };

    struct Task: public DUNE::Tasks::Periodic
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
      unsigned int m_N;
      unsigned int m_L;

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

      //! Rate [Hz] and delay [ms] of position update for each vehicle
      Matrix m_pos_update_rate, m_pos_update_delay;

      //! Memory factor when calculating average update rates
      double m_mf;

      //! Desired velocity
      IMC::DesiredVelocity m_desired_velocity;


      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Periodic(name, ctx),
        m_i(0),
        m_N(0),
        m_L(0),
        m_mf(0)
      {
        paramActive(Tasks::Parameter::SCOPE_MANEUVER,
                    Tasks::Parameter::VISIBILITY_USER);

        param("Formation Controller", m_args.use_controller)
        .visibility(Tasks::Parameter::VISIBILITY_USER)
        .scope(Tasks::Parameter::SCOPE_MANEUVER)
        .defaultValue("false")
        .description("Enable formation controller.");

        param("Vehicle List", m_args.formation_systems)
        .defaultValue("")
        .visibility(Tasks::Parameter::VISIBILITY_USER)
        .scope(Tasks::Parameter::SCOPE_MANEUVER)
        .description("System name list of the formation vehicles.");

        param("Desired Formation", m_args.desired_formation)
        .defaultValue("0.0, 0.0, 0.0")
        .units(Units::Meter)
        .visibility(Tasks::Parameter::VISIBILITY_USER)
        .scope(Tasks::Parameter::SCOPE_MANEUVER)
        .description("Desired formation positions matrix.");

        param("Incidence Matrix", m_args.incidence_matrix)
        .defaultValue("0")
        .visibility(Tasks::Parameter::VISIBILITY_USER)
        .scope(Tasks::Parameter::SCOPE_MANEUVER)
        .description("Incidence matrix.");

        param("Link Gains", m_args.link_gains)
        .defaultValue("1.0")
        .visibility(Tasks::Parameter::VISIBILITY_USER)
        .scope(Tasks::Parameter::SCOPE_MANEUVER)
        .description("Gains assigned to formation links.");

        param("Constant Mission Velocity", m_args.const_mission_velocity)
        .defaultValue("0.0, 0.0, 0.0")
        .units(Units::MeterPerSecond)
        .visibility(Tasks::Parameter::VISIBILITY_USER)
        .scope(Tasks::Parameter::SCOPE_MANEUVER)
        .description("Constant mission velocity.");

        param("Collision Avoidance Radius", m_args.collision_radius)
        .defaultValue("5.0")
        .units(Units::Meter)
        .visibility(Tasks::Parameter::VISIBILITY_USER)
        .scope(Tasks::Parameter::SCOPE_MANEUVER)
        .description("Radius for collision avoidance potential field.");

        param("Collision Avoidance Gain", m_args.collision_gain)
        .defaultValue("0.0")
        .visibility(Tasks::Parameter::VISIBILITY_USER)
        .scope(Tasks::Parameter::SCOPE_MANEUVER)
        .description("Gain for collision avoidance potential field.");

        param("Maximum Speed", m_args.max_speed)
        .defaultValue("5.0")
        .units(Units::MeterPerSecond)
        .visibility(Tasks::Parameter::VISIBILITY_USER)
        .scope(Tasks::Parameter::SCOPE_MANEUVER)
        .description("Maximum speed, i.e. controller saturation.");

        param("Memory Factor", m_args.mem_factor)
        .defaultValue("0.95")
        .description("Memory factor when calculating update rates.");

        param("Delay Threshold", m_args.delay_threshold)
        .defaultValue("100")
        .units(Units::Millisecond)
        .description("Threshold for issuing warnings on delay in position updates.");



        // Bind incoming IMC messages
        bind<IMC::FormPos>(this);
        //bind<IMC::DesiredVelocity>(this);
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        debug("Starting update of parametes.");


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


        // TODO: Only update m_z and m_z_d when neccessary
        inf("New desired difference variables.");

        // Resize and reset difference variables matrices to fit number of links
        m_z.resizeAndFill(3, m_L, 0);
        m_z_d.resizeAndFill(3, m_L, 0);
        // Update desired difference variables matrix
        calcDiffVariable(&m_z_d,m_D,m_x_c);

        printMatrix(m_z_d);


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


        if (paramChanged(m_args.mem_factor))
        {
          inf("New memory factor.");

          if (m_args.mem_factor >= 1 || m_args.mem_factor < 0)
          {
            war("Invalid memory factor %1.1f; using default value 0.95.",
                m_args.mem_factor);
            m_mf = 0.95;
          }
          else
            m_mf = m_args.mem_factor;
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
        IMC::ControlLoops cloops;
        cloops.enable = IMC::ControlLoops::CL_ENABLE;
        cloops.mask = IMC::CL_SPEED;
        dispatch(cloops);
        inf("Sent ControlLoops enable SPEED");
      }

      void
      onDeactivation(void)
      {
        Matrix zero_vel(3,1,0.0);
        sendDesiredVelocity(zero_vel);

        IMC::ControlLoops cloops;
        cloops.enable = IMC::ControlLoops::CL_DISABLE;
        cloops.mask = IMC::CL_SPEED;
        dispatch(cloops);
        inf("Sent ControlLoops disable SPEED");
      }

      //! Consume Formation Position
      void
      consume(const IMC::FormPos* msg)
      {
        spew("Got Formation Position");

        double stamp = msg->ots;
        bool id_found = false;
        for (unsigned int uav = 0; uav < m_N; uav++)
        {
          if (m_uav_ID[uav] == msg->getSource())
          {
            id_found = true;

            // Calculate update frequency and delay
            double delay_ms = (Clock::getSinceEpoch() - stamp)*1E3;
            double diff = stamp - m_last_pos_update(uav);
            if (diff > 0)
            {
              if (!m_pos_update_rate(uav))
                m_pos_update_rate(uav) = 1/diff;
              else
                m_pos_update_rate(uav) = m_mf*m_pos_update_rate(uav) + (1-m_mf)/diff;
            }
            else
            {
              war("Received old FormPos! Diff = %f s , Delay = %f ms",
                  diff, delay_ms);
            }
            m_last_pos_update(uav) = stamp;

            if (delay_ms > m_args.delay_threshold)
            {
              war("Positioning delay for vehicle '%s' above threshold: %f",
                  resolveSystemId(msg->getSource()), delay_ms);
            }

            if (!m_pos_update_delay(uav))
              m_pos_update_delay(uav) = delay_ms;
            else
              m_pos_update_delay(uav) = m_mf*m_pos_update_delay(uav) + (1-m_mf)*delay_ms;

            spew("Update frequency [Hz]:");
            printMatrix(m_pos_update_rate,DEBUG_LEVEL_SPEW);
            spew("Update delay [ms]:");
            printMatrix(m_pos_update_delay,DEBUG_LEVEL_SPEW);

            // Update position
            m_x(0,uav) = msg->x;
            m_x(1,uav) = msg->y;
            m_x(2,uav) = msg->z;
            spew("Updated position of vehicle '%s'",
                resolveSystemId(msg->getSource()));
            printMatrix(m_x,DEBUG_LEVEL_SPEW);

            // Update velocity (only really needed from local vehicle)
            m_v(0,uav) = msg->vx;
            m_v(1,uav) = msg->vy;
            m_v(2,uav) = msg->vz;
            /*spew("Updated velocity of vehicle '%s'",
                resolveSystemId(msg->getSource()));
            printMatrix(m_v,DEBUG_LEVEL_SPEW);*/

            break;
          }
        }
        if (!id_found)
          war("Received FormPos from unknown vehicle '%s'", resolveSystemId(msg->getSource()));
      }

      //! Consume Desired Velocity (mission velocity from FormationGuidance)
      void
      consume(const IMC::DesiredVelocity* msg)
      {
        if (msg->getSourceEntity() == resolveEntity("Formation Guidance"))
        {
          m_v_mission(0) = msg->u;
          m_v_mission(1) = msg->v;
          m_v_mission(2) = msg->w;

          spew("Got Mission Velocity: [%1.1f, %1.1f, %1.1f]",
              m_v_mission(0), m_v_mission(1), m_v_mission(2));
        }
      }

      void
      consume(const IMC::ControlLoops* msg)
      {
        IMC::ControlLoops cloops;
        cloops.enable = IMC::ControlLoops::CL_ENABLE;
        cloops.mask = IMC::CL_SPEED;
        cloops.scope_ref = msg->scope_ref;
        dispatch(cloops);
        debug("Sent ControlLoops");
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

      //! Calculate formation velocity
      Matrix
      formationVelocity(void)
      {
        // Calculate z_tilde
        calcDiffVariable(&m_z, m_D, m_x);
        Matrix z_tilde = m_z - m_z_d;

        // Calculate formation velocity component
        Matrix u_form(3,1,0);
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
        static double u_coll_max = 0;
        static double d_ij_min = std::numeric_limits<double>::infinity();

        for (unsigned int uav = 0; uav < m_N; uav++)
        {
          if (uav != m_i)
          {
            // Get vector and distance to neighbour
            Matrix x_ij = m_x.column(uav) - m_x.column(m_i);
            double d_ij = x_ij.norm_2();
            // Check if inside potential field
            if (d_ij < m_args.collision_radius)
            {
              // Add repelling velocity
              u_coll -= (m_args.collision_radius - d_ij)*x_ij/d_ij;
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

      //! Dispatch desired velocity
      void
      sendDesiredVelocity(Matrix velocity)
      {
        m_desired_velocity.u = velocity(0);
        m_desired_velocity.flags |= IMC::DesiredVelocity::FL_SURGE;

        m_desired_velocity.v = velocity(1);
        m_desired_velocity.flags |= IMC::DesiredVelocity::FL_SWAY;

        m_desired_velocity.w = velocity(2);
        m_desired_velocity.flags |= IMC::DesiredVelocity::FL_HEAVE;

        dispatch(m_desired_velocity);
        spew("v_d: [%1.1f, %1.1f, %1.1f]",
            m_desired_velocity.u, m_desired_velocity.v, m_desired_velocity.w);
      }

      //! Main loop.
      void
      task(void)
      {
        if(!isActive())
          return;

        // Calculate external feedback, u
        Matrix u = formationVelocity() + collAvoidVelocity();

        // Calculate internal feedback, tau
        Matrix tau = u + m_v_mission;
        tau = saturate(tau,m_args.max_speed);
        sendDesiredVelocity(tau);
      }
    };
  }
}

DUNE_TASK
