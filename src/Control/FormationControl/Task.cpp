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

    };

    struct Task: public DUNE::Tasks::Periodic
    {
      //! Task arguments
      Arguments m_args;

      //! Vehicle IDs
      Systems m_uav_ID;

      //! Vehicle formation number
      int m_i;

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

      //! Desired velocity
      IMC::DesiredVelocity m_desired_velocity;


      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Periodic(name, ctx),
        m_i(0),
        m_N(0),
        m_L(0)
      {

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

        // Bind incoming IMC messages
        bind<IMC::FormPos>(this);

      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        spew("Starting update of parametes.");

        spew("onUpdateParameters - 1");
        if (this->paramChanged(m_args.formation_systems))
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
            m_i = -1;
            for (unsigned int uav = 0; uav < m_N; uav++)
            {
              debug("UAV %u: %s", uav, m_args.formation_systems[uav].c_str());
              m_uav_ID.push_back(this->resolveSystemName(m_args.formation_systems[uav]));
              if (m_uav_ID[uav] == this->getSystemId())
                m_i = uav; // Set my formation id
            }
            if (m_i < 0)
            {
              war("Vehicle not found in formation vehicle list!");
              throw DUNE::Exception("Vehicle not found in formation vehicle list!");
            }
          }
        }

        spew("onUpdateParameters - 2");
        if (paramChanged(m_args.desired_formation))
        {
          inf("New desired formation.");

          if (m_args.desired_formation.size() == 0)
            throw DUNE::Exception("Desired formation positons matrix is empty!");
          if (m_args.desired_formation.size()%3 != 0)
            throw DUNE::Exception("Unvalid number of coordinates in desired formation positions matrix!");
          if (m_args.desired_formation.size()/3 != m_N)
            throw DUNE::Exception("Incorrect number of vehicles in desired formation positions matrix!");

          m_x_c.resize(3, m_N);
          for (unsigned int uav_id = 0; uav_id < m_N; uav_id++)
          {
            for (unsigned int coord = 0; coord < 3; coord++)
              m_x_c(coord, uav_id) = m_args.desired_formation(coord + uav_id*3);
            debug("UAV %u: [x=%1.1f, y=%1.1f, z=%1.1f]", uav_id,
                m_x_c(0, uav_id), m_x_c(1, uav_id), m_x_c(2, uav_id));
          }

          // TODO: Calculate z_d based on x_c
        }

        spew("onUpdateParameters - 3");
        if (paramChanged(m_args.incidence_matrix))
        {
          inf("New incidence matrix.");

          if (m_args.incidence_matrix.size() == 0)
            throw DUNE::Exception("Incidence matrix is empty!");
          if (m_args.incidence_matrix.rows()%m_N != 0)
            throw DUNE::Exception("Incidence matrix doesn't match number of vehicles!");

          // Update number of links
          m_L = m_args.incidence_matrix.rows()/m_N;
          // Resize incidence matrix
          m_D.resize(m_L,m_N);
          // Update incidence matrix
          for (unsigned int link = 0; link < m_L; link++)
          {
            for (unsigned int uav = 0; uav < 3; uav++)
              m_D(uav, link) = m_args.incidence_matrix(uav + link*m_L);
          }

          //printMatrix(m_D);
        }

        spew("onUpdateParameters - 4");
        if (paramChanged(m_args.link_gains))
        {
          inf("New link gains.");

          if (m_args.link_gains.size() == 1)
          {
            // Scalar gain, set all to this
            m_delta = Matrix(m_L, 1, m_args.link_gains(0));
          }
          else if (m_args.link_gains.size() != m_L)
            throw DUNE::Exception("Link gains doesn't match number of links!");
          else
          {
            m_delta = m_args.link_gains;
          }

          //printMatrix(m_delta);
        }

        spew("onUpdateParameters - 5");
        // Resize position and velocity matrices to fit number of vehicles
        m_x.resize(3,m_N);
        m_v.resize(3,m_N);
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

      //! Print matrix (for debuging)
      void
      printMatrix(Matrix m){
        printf("[HERE]\n");
        for(int i = 0; i<m.rows(); i++ ){
          for(int j = 0; j<m.columns();j++){
            printf("%f ", m.element(i,j));
          }
          printf("\n");
        }
      }

      //! Consume Formation Position
      void
      consume(const IMC::FormPos* msg)
      {
        spew("Got Formation Position");

        bool id_found = false;
        for (unsigned int uav = 0; uav < m_N; uav++)
        {
          if (m_uav_ID[uav] == msg->getSource())
          {
            id_found = true;
            // Update position
            m_x(0,uav) = msg->x;
            m_x(1,uav) = msg->y;
            m_x(2,uav) = msg->z;
            // Update velocity (only really needed from local vehicle)
            m_v(0,uav) = msg->vx;
            m_v(1,uav) = msg->vy;
            m_v(2,uav) = msg->vz;
            spew("Updated position of vehicle '%s'", resolveSystemId(msg->getSource()));
            printMatrix(m_x);
            break;
          }
        }
        if (!id_found)
          war("Received FormPos from unknown vehicle '%s'", resolveSystemId(msg->getSource()));
      }

      //! Main loop.
      void
      task(void)
      {
      }
    };
  }
}

DUNE_TASK
