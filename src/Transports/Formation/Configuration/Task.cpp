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

namespace Transports
{
  namespace Formation
  {
    namespace Configuration
    {
      using DUNE_NAMESPACES;

      //! Vector for System Mapping.
      typedef std::vector<uint32_t> Systems;

      struct GainScheduler
      {
        //! Enable link gain scheduler
        bool enable;

        //! Link gain when in-formation
        double gain_close;

        //! Link gain when large link error
        double gain_far;

        //! Desired link error switching distance
        double switch_distance;

        //! Desired percent of in-formation gain.
        double distance_percent_gain_close;

        //! Desired error link distance at <distance_percent_gain_close> in-formation gain.
        double percent_gain_close;
      };

      struct Arguments
      {
        //! Send Formation Centroid
        bool use_task;

        //! Vehicle list
        std::vector<std::string> formation_systems;

        //! Desired formation
        Matrix desired_formation;

        //! Incidence matrix
        Matrix incidence_matrix;

        //! Link gains
        Matrix link_gains;

        //! Gain scheduling
        GainScheduler gain_scheduler;

        //! Disable formation velocity
        bool disable_formation_velocity;

        //! Disable mission velocity
        bool disable_mission_velocity;

        //! Disable collision velocity
        bool disable_collision_velocity;

        //! Hold current formation
        bool hold_current_formation;

        //! Reference latitide
        double ref_lat;
        //! Reference longitude
        double ref_lon;
        //! Reference height (above elipsoid)
        double ref_hae;

        bool use_static_ref;

      };

      struct Task: public Tasks::Periodic
      {
        Arguments m_args;

        //! Localization origin (WGS-84)
        fp64_t m_ref_lat, m_ref_lon;
        fp32_t m_ref_hae;
        bool m_ref_valid;

        //! Vehicle IDs
        Systems m_uav_ID;

        //! Vehicle formation number
        unsigned int m_i;

        //! Desired formation positions
        Matrix m_x_c_default;

        //! Incidence matrix;
        Matrix m_D;

        //! Number of agents and links
        unsigned int m_N, m_L;

        IMC::CoordConfig m_config;
        IMC::VehicleFormationParticipant m_agent;
        IMC::CoordIncidenceAgent m_incidence_agent;
        IMC::CoordIncidenceLink m_incidence_link;
        IMC::CoordLinkGain m_link_gain;
        IMC::CoordLinkGainScheduler m_link_gain_scheduler;

        //! Constructor.
        //! @param[in] name task name.
        //! @param[in] ctx context.
        Task(const std::string& name, Tasks::Context& ctx):
          Tasks::Periodic(name, ctx),
          m_i(0),
          m_N(0),
          m_L(0)
        {
          param("Formation Configuration", m_args.use_task)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          //.scope(Tasks::Parameter::SCOPE_MANEUVER)
          .defaultValue("true")
          .description("Enable Formation Configuration.");

          param("Vehicle List", m_args.formation_systems)
          .defaultValue("")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_IDLE)
          .description("System name list of the formation vehicles.");

          param("Desired Formation", m_args.desired_formation)
          .defaultValue("0.0, 0.0, 0.0")
          .units(Units::Meter)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_IDLE)
          .description("Desired formation positions matrix, where the order of the position vectors corresponds to the order of the vehicle list.");

          param("Incidence Matrix", m_args.incidence_matrix)
          .defaultValue("0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_IDLE)
          .description("Incidence matrix.");

          param("Link Gains", m_args.link_gains)
          .defaultValue("1.0")
          .scope(Tasks::Parameter::SCOPE_IDLE)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Gains assigned to formation links.");

          param("Gain Scheduling -- Enable", m_args.gain_scheduler.enable)
          .defaultValue("False")
          .scope(Tasks::Parameter::SCOPE_IDLE)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Enable link gains scheduling.");

          param("Gain Scheduling -- Gain in-formation", m_args.gain_scheduler.gain_close)
          .defaultValue("False")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_IDLE)
          .description("Enable link gains scheduling.");

          param("Gain Scheduling -- Gain far", m_args.gain_scheduler.gain_far)
          .defaultValue("False")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_IDLE)
          .description("Enable link gains scheduling.");

          param("Gain Scheduling -- Switch distance", m_args.gain_scheduler.switch_distance)
          .defaultValue("False")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_IDLE)
          .description("Enable link gains scheduling.");

          param("Gain Scheduling -- Percent of gain", m_args.gain_scheduler.percent_gain_close)
          .defaultValue("90.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_IDLE)
          .description("Desired error link distance at percent of in-formation gain.");

          param("Gain Scheduling -- Distance at % gain", m_args.gain_scheduler.distance_percent_gain_close)
          .defaultValue("1.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_IDLE)
          .description("Desired error link distance at percent of in-formation gain.");

          param("Disable Formation Velocity", m_args.disable_formation_velocity)
          .defaultValue("false")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Disable formation velocity.");

          param("Disable Mission Velocity", m_args.disable_mission_velocity)
          .defaultValue("false")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Disable mission velocity.");

          param("Disable Collision Velocity", m_args.disable_collision_velocity)
          .defaultValue("false")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Disable collision velocity.");

          param("Hold Current Formation", m_args.hold_current_formation)
          .defaultValue("false")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Use current formation as desired formation.");

          param("Latitude", m_args.ref_lat)
          .defaultValue("-999.0")
          .units(Units::Degree)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Reference Latitude");

          param("Longitude", m_args.ref_lon)
          .defaultValue("0.0")
          .units(Units::Degree)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Reference Longitude");

          param("Height", m_args.ref_hae)
          .defaultValue("0.0")
          .units(Units::Meter)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Reference Height (above ellipsoid)");

          param("Use Static Reference", m_args.use_static_ref)
          .defaultValue("true")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Static reference LLH is set from config file");

          setEntityState(IMC::EntityState::ESTA_BOOT, Status::CODE_ACTIVATING);
        }

        //! Update internal state with new parameter values.
        void
        onUpdateParameters(void)
        {
          if (paramChanged(m_args.formation_systems) ||
              paramChanged(m_args.desired_formation) ||
              paramChanged(m_args.incidence_matrix) ||
              paramChanged(m_args.link_gains) ||
              paramChanged(m_args.gain_scheduler.enable) ||
              paramChanged(m_args.gain_scheduler.gain_close) ||
              paramChanged(m_args.gain_scheduler.gain_far) ||
              paramChanged(m_args.gain_scheduler.distance_percent_gain_close) ||
              paramChanged(m_args.gain_scheduler.percent_gain_close) ||
              paramChanged(m_args.gain_scheduler.switch_distance)
              )
          {
            debug("m_config.update = true");
            m_config.update = true;

            setEntityState(IMC::EntityState::ESTA_BOOT, Status::CODE_SYNCING);


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
              debug("m_N = %d",m_N);
              bool found_self = false;
              for (unsigned int uav = 0; uav < m_N; uav++)
              {
                debug("UAV %u: %s", uav, m_args.formation_systems[uav].c_str());
                m_uav_ID.push_back(
                    this->resolveSystemName(m_args.formation_systems[uav]));
                if (m_uav_ID[uav] == this->getSystemId())
                {
                  m_i = uav; // Set my formation id
                  found_self = true;
                }
              }
              if (!found_self)
                throw DUNE::Exception(
                    "Vehicle not found in formation vehicle list!");
            }

            if (m_N > 1)
            {
              inf("New desired formation.");
              debug("m_N = %d",m_N);
              // Check dimensions
              if (m_args.desired_formation.size() == 0)
                throw DUNE::Exception(
                    "Desired formation positons matrix is empty!");
              if (m_args.desired_formation.size() % 3 != 0)
                throw DUNE::Exception(
                    "Invalid number of coordinates in desired formation positions matrix!");
              if ((unsigned int)m_args.desired_formation.size() / 3 != m_N)
                throw DUNE::Exception(
                    "Incorrect number of vehicles in desired formation positions matrix!");
              // Resize desired formation matrix to fit number of vehicles
              m_x_c_default.resizeAndFill(3, m_N, 0);
              // Update desired formation matrix
              for (unsigned int uav_id = 0; uav_id < m_N; uav_id++)
              {
                for (unsigned int coord = 0; coord < 3; coord++)
                  m_x_c_default(coord, uav_id) = m_args.desired_formation(
                      coord + uav_id * 3);
                debug("UAV %u: [%1.1f, %1.1f, %1.1f]", uav_id,
                      m_x_c_default(0, uav_id), m_x_c_default(1, uav_id),
                      m_x_c_default(2, uav_id));
              }
            }

            if (m_N > 1)
            {
              inf("New incidence matrix.");

              // Check dimensions
              if (m_args.incidence_matrix.size() == 0)
               throw DUNE::Exception("Incidence matrix is empty!");
              if (m_args.incidence_matrix.rows() % m_N != 0)
               throw DUNE::Exception(
                   "Incidence matrix doesn't match number of vehicles!");

              // Update number of links
              m_L = m_args.incidence_matrix.rows() / m_N;
              // Resize incidence matrix
              m_D.resize(m_N, m_L);
              // Update incidence matrix
              for (unsigned int link = 0; link < m_L; link++)
              {
               for (unsigned int uav = 0; uav < m_N; uav++)
                 m_D(uav, link) = m_args.incidence_matrix(uav + link * m_N);
              }
              printMatrix(m_D);
            }
            if (paramChanged(m_args.link_gains))
            {
              inf("New link gains.");
            }
            //now add data (and dispatch message?)

            m_config.participants.clear();
            m_config.incidence.clear();
            //! participants and incidence matrix
            debug("m_L=%d,m_N=%d",m_L,m_N);
            for (unsigned int uav = 0; uav < m_N; uav++)
            {
              if (m_N > 1)
              {
                m_agent.off_x = m_x_c_default(0,uav);
                m_agent.off_y = m_x_c_default(1,uav);
                m_agent.off_z = m_x_c_default(2,uav);
              }
              else
              {
                m_agent.off_x = 0;
                m_agent.off_y = 0;
                m_agent.off_z = 0;
              }
              m_agent.vid = m_uav_ID[uav];
              m_config.participants.push_back(m_agent);
              if (m_N > 1)
              {
                for (unsigned int link = 0; link < m_L; link++)
                {
                  m_incidence_agent.links.clear();
                  m_incidence_link.orientation = m_args.incidence_matrix(uav + link * m_N);
                  m_incidence_agent.links.push_back(m_incidence_link);
                  debug("[%d,%d]:m_incidence_link.orientation=%d",uav,link,m_incidence_link.orientation);
                }
                debug("m_incidence_agent[%d]",uav);
                m_config.incidence.push_back(m_incidence_agent);
              }
              else
              {
                m_incidence_agent.links.clear();
                m_config.incidence.push_back(m_incidence_agent);
              }
            }
            //! link gains
            m_config.link_gains.clear();
            if (m_N > 1)
            {
              for (unsigned int link = 0; link < m_L; link++)
              {
                if (m_args.link_gains.size() == 1)
                {
                  // Scalar gain, set all to this
                  m_link_gain.value = m_args.link_gains(0);
                }
                else if ((unsigned int)m_args.link_gains.size() != m_L)
                  throw DUNE::Exception("Link gains doesn't match number of links!");
                else
                {
                  // Update gains
                  m_link_gain.value = m_args.link_gains(link);
                }
                m_config.link_gains.push_back(m_link_gain);
              }
            }
            //! link gain scheduling
            m_config.link_gains_scheduling.clear();
            m_link_gain_scheduler.clear();
            m_link_gain_scheduler.enable_scheduler = m_args.gain_scheduler.enable;
            if (m_args.gain_scheduler.enable)
            {
              m_link_gain_scheduler.gain_close = m_args.gain_scheduler.gain_close;
              m_link_gain_scheduler.gain_far = m_args.gain_scheduler.gain_far;
              m_link_gain_scheduler.switch_distance = m_args.gain_scheduler.switch_distance;

              double temp = std::log((double)(100/m_args.gain_scheduler.percent_gain_close) -1);
              double slope = -(double(1.0)/std::abs(m_args.gain_scheduler.switch_distance - m_args.gain_scheduler.distance_percent_gain_close))*temp;
              m_link_gain_scheduler.slope = slope;
            }
            else
            {
              m_link_gain_scheduler.gain_close = 0;
              m_link_gain_scheduler.gain_far = 0;
              m_link_gain_scheduler.switch_distance = 0;
              m_link_gain_scheduler.slope = 0;
            }
            m_config.link_gains_scheduling.push_back(m_link_gain_scheduler);

            dispatch(m_config);
            debug("CoordConfig dispatched from update");
            setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);

          }

          if (paramChanged(m_args.ref_lat)
              || paramChanged(m_args.ref_lon)
              || paramChanged(m_args.ref_hae)
              || paramChanged(m_args.use_static_ref))
          {
           m_config.update = false;
           m_config.use_fallback = m_args.use_static_ref;
           setEntityState(IMC::EntityState::ESTA_BOOT, Status::CODE_SYNCING);

           if (m_args.ref_lat == -999) // Reference not set; return
             return;

           inf("New reference position.");

           // Check validity
           if (std::abs(m_args.ref_lat) > 90)
             throw DUNE::Exception("Nonvalid reference latitude!");
           if (std::abs(m_args.ref_lon) > 180)
             throw DUNE::Exception("Nonvalid reference longitude!");

           m_ref_lat = Angles::radians(m_args.ref_lat);
           m_ref_lon = Angles::radians(m_args.ref_lon);
           m_ref_hae = m_args.ref_hae;
           m_ref_valid = true;
           inf("Ref. LLH set from ini: [Lat = %f, Lon = %f, Height = %.1f]",
               Angles::degrees(m_ref_lat), Angles::degrees(m_ref_lon), m_ref_hae);

           //now add data and dispatch message
           m_config.lat   = m_ref_lat;
           m_config.lon   = m_ref_lon;
           m_config.height= m_ref_hae;

           dispatch(m_config);
           debug("CoordConfig dispatched from new llh");
           setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
          }          

          if (paramChanged(m_args.disable_collision_velocity) ||
              paramChanged(m_args.disable_formation_velocity) ||
              paramChanged(m_args.disable_mission_velocity)   ||
              paramChanged(m_args.hold_current_formation)
             )
          {
            debug("m_config.update = false");
            m_config.update = false;
            setEntityState(IMC::EntityState::ESTA_BOOT, Status::CODE_SYNCING);

            m_config.disable_collision_vel = m_args.disable_collision_velocity;
            m_config.disable_formation_vel = m_args.disable_formation_velocity;
            m_config.disable_mission_vel   = m_args.disable_mission_velocity;
            m_config.formation  = m_args.hold_current_formation;

            dispatch(m_config);
            debug("CoordConfig dispatched from new flags");
            setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
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
        }

        //! Release resources.
        void
        onResourceRelease(void)
        {
        }

        void
        task(void)
        {
          if (m_args.use_task)
          {
            m_config.update = false;
            dispatch(m_config);
            debug("CoordConfig dispatched from task");
          }
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
      };
    }
  }
}

DUNE_TASK
