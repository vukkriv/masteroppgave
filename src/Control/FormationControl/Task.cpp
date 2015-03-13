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
      std::vector<double> link_gains;

    };

    struct Task: public DUNE::Tasks::Periodic
    {
      //! Task arguments
      Arguments m_args;

      //! Vehicle IDs
      Systems m_IDs;

      //! Vehicle formation number
      int m_i;

      //! Incidence matrix;
      Matrix m_D;

      //! Number of agents and links
      int m_N;
      int m_L;

      //! Desired difference variables
      Matrix m_z_d;

      //! Difference variables
      Matrix m_z;

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

      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        spew("Starting update of parametes.");

        if (this->paramChanged(m_args.formation_systems))
        {
          inf("New Formation vehicles' list.");

          // Extract vehicle IDs
          m_IDs.clear();
          if (m_args.formation_systems.empty())
          {
            war("Formation vehicle list is empty!");
            m_IDs.push_back(this->getSystemId());
            m_N = 1;
            m_i = 0;
          }
          else
          {
            m_N = m_args.formation_systems.size();
            m_i = -1;
            for (unsigned int i = 0; i < m_N; i++)
            {
              debug("UAV %u: %s", i, m_args.formation_systems[i].c_str());
              m_IDs.push_back(this->resolveSystemName(m_args.formation_systems[i]));
              if (m_IDs[i] == this->getSystemId())
                m_i = i;
            }
            if (m_i < 0)
              throw RestartNeeded("Vehicle not found in formation vehicle list!", 10);
          }
        }

        // TODO: Check dimensions of incidence matrix to vehicle list
        // Update incidence matrix
        m_D = m_args.incidence_matrix;

        // Extract number of links
        m_L = m_D.rows();

        // TODO: Update z_d (kron?)
        // TODO: compare getSystemName() with id-matrix to decide ID

        // TODO: Extract link gains; multiply if scalar
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

      //! Main loop.
      void
      task(void)
      {
      }
    };
  }
}

DUNE_TASK
