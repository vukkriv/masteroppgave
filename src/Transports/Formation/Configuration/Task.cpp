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
        IMC::CoordIncidenceLinks m_incidence_links;


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
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .defaultValue("false")
          .description("Enable Formation Configuration.");

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

          param("Disable Collision Velocity", m_args.disable_collision_velocity)
          .defaultValue("false")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Disable collision velocity.");

          param("Latitude", m_args.ref_lat)
          .defaultValue("-999.0")
          .units(Units::Degree)
          .description("Reference Latitude");

          param("Longitude", m_args.ref_lon)
          .defaultValue("0.0")
          .units(Units::Degree)
          .description("Reference Longitude");

          param("Height", m_args.ref_hae)
          .defaultValue("0.0")
          .units(Units::Meter)
          .description("Reference Height (above ellipsoid)");

          param("Use Static Reference", m_args.use_static_ref)
          .defaultValue("true")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Static reference LLH is set from config file");
        }

        //! Update internal state with new parameter values.
        void
        onUpdateParameters(void)
        {
          if (paramChanged(m_args.disable_collision_velocity) ||
              paramChanged(m_args.disable_formation_velocity) ||
              paramChanged(m_args.disable_mission_velocity)   ||
              paramChanged(m_args.hold_current_formation)
             )
          {
            debug("m_config.update = false");
            m_config.update = false;
            m_config.disable_collision_vel = m_args.disable_collision_velocity;
            m_config.disable_formation_vel = m_args.disable_formation_velocity;
            m_config.disable_mission_vel   = m_args.disable_mission_velocity;
            m_config.formation  = m_args.hold_current_formation;
            dispatch(m_config);
            debug("CoordConfig dispatched");
          }

          if (paramChanged(m_args.formation_systems) ||
              paramChanged(m_args.desired_formation) ||
              paramChanged(m_args.incidence_matrix) ||
              paramChanged(m_args.link_gains)
              )
          {
            debug("m_config.update = true");
            m_config.update = true;

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

            if (paramChanged(m_args.desired_formation))
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

            if (paramChanged(m_args.incidence_matrix))
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
            }
            if (paramChanged(m_args.link_gains))
            {
              inf("New link gains.");
              if (m_args.link_gains.size() == 1)
              {
                // Scalar gain, set all to this
                //m_delta = Matrix(m_L, 1, m_args.link_gains(0));
              }
              else if ((unsigned int)m_args.link_gains.size() != m_L)
                throw DUNE::Exception("Link gains doesn't match number of links!");
            }
            //now add data (and dispatch message?)

            m_config.incidence.clear();
            m_config.participants.clear();
            for (unsigned int uav = 0; uav < m_N; uav++)
            {
              m_agent.off_x = m_x_c_default(0,uav);
              m_agent.off_y = m_x_c_default(1,uav);
              m_agent.off_z = m_x_c_default(2,uav);
              m_agent.vid = m_uav_ID[uav];
              m_config.participants.push_back(m_agent);

              //! incidence row, comma separated list
              //m_incidence_links.vehicles =
              m_config.incidence.push_back(m_incidence_links);
            }
            //! link gains, comma separated list

            IMC::Parser a;
          }

          if (paramChanged(m_args.ref_lat) || paramChanged(m_args.ref_lon) || paramChanged(m_args.ref_hae))
          {
           m_config.update = false;
           m_config.use_fallback = m_args.use_static_ref;

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
          }
          dispatch(m_config);
          debug("CoordConfig dispatched");
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
          //if (!isActive())
          //  return;
        }
      };
    }
  }
}

DUNE_TASK
