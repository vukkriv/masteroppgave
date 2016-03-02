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
// Author: Kristian Klausen                                                 *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

// Local headers
#include "CopterPendulumModel.hpp"

namespace Simulators
{
  namespace CopterPendulum5Dof

  {
    using DUNE_NAMESPACES;

    struct Arguments
    {
      //! Model parameters
      double mass_copter;
      double mass_load;
      double length;
    };

    struct Task: public DUNE::Tasks::Periodic
    {
      //! Task arguments
      Arguments m_args;

      //! Model
      CopterPendulumModel m_model;

      //! Current state
      Matrix m_eta;
      Matrix m_nu;
      //! Previous setp time
      double m_time_prev_step;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Periodic(name, ctx),
        m_eta(5,1, 0.0),
        m_nu (5,1, 0.0)
      {

        param("Model - Mass Copter", m_args.mass_copter)
        .defaultValue("2.5");

        param("Model - Mass Load", m_args.mass_load)
        .defaultValue("0.5");

        param("Model - Length", m_args.length)
        .defaultValue("3");


        m_time_prev_step = Time::Clock::get() - 1/getFrequency();


      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        m_model.setMassCopter(m_args.mass_copter);
        m_model.setMassLoad(m_args.mass_load);
        m_model.setLength(m_args.length);

      }

      void
      step(Matrix tau)
      {
        if(tau.size() < 5)
        {
          err("Invalid tau sent. Aborting simulation step. ");
          return;
        }
        double now = Time::Clock::get();
        double dt  = now - m_time_prev_step;

        if (dt < 0.0 || dt > 2.0)
        {
          err("delta t to small or to large. Aborting simulation step. ");
          return;
        }

        Matrix M = m_model.getMassMatrix(m_eta);
        Matrix C = m_model.getCoreolisMatrix(m_eta, m_nu);
        Matrix G = m_model.getGravityMatrix(m_eta);

        Matrix next_eta = m_eta;
        Matrix next_nu  = m_nu;

        try{
          next_eta = m_eta + dt * m_nu;
          next_nu  = m_nu  + dt * inverse(M) * (tau - C*m_nu - G);
        }
        catch(Matrix::Error& error)
        {
          err("Error doing step. Aborting simulation step: %s", error.what());
          return;
        }

        // Store new values
        m_eta = next_eta;
        m_nu  = next_nu;


        m_time_prev_step = now;
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
