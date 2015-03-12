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

    struct Arguments
    {
      //! Use controller
      bool use_controller;

      //! Formation leader
      bool is_leader;

      //! Formation identity
      int formation_id;

      //! Desired formation
      Matrix desired_formation;

      // Incidence matrix
      Matrix incidence_matrix;
    };

    struct Task: public DUNE::Tasks::Periodic
    {
      //! Task arguments
      Arguments m_args;

      //! Incidence matrix;
      Matrix m_D;

      //! Number of agents and links
      int m_N;
      int m_L;

      //! Desired difference variables
      Matrix m_z_d;

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
        m_N(0),
        m_L(0)
      {
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        //! Update incidence matrix
        m_D = m_args.desired_formation;
        m_N = m_D.columns();
        m_L = m_D.rows();

        //TODO: Update z_d (kron?)
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
