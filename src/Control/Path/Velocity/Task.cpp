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
// https://www.lsts.pt/dune/licence.                                        *
//***************************************************************************
// Author: Kristian Klausen                                                 *
//***************************************************************************

// ISO C++ 98 headers.
#include <cmath>

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Control
{
  namespace Path
  {
    namespace Velocity
    {
      using DUNE_NAMESPACES;

      struct Arguments
      {
        bool use_controller;
        double max_speed;
        double refmodel_omega_n;
        double refmodel_xi;
        double Kp;
      };

      struct Task: public DUNE::Control::PathController
      {
        IMC::DesiredVelocity m_velocity;
        //! Task arguments.
        Arguments m_args;
        //! Reference model state
        Matrix m_refmodel_x;
        //! Reference model trans.matrix
        Matrix m_refmodel_A;
        //! Reference model input matrix
        Matrix m_refmodel_B;


        Task(const std::string& name, Tasks::Context& ctx):
          DUNE::Control::PathController(name, ctx)
        {

          param("Velocity Controller", m_args.use_controller)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .defaultValue("false")
          .description("Enable Velocoty Controller");

          param("Max Speed", m_args.max_speed)
            .defaultValue("5.0")
            .units(Units::MeterPerSecond)
            .description("Max speed of the vehicle");

          param("Reference Model - Natural Frequency",m_args.refmodel_omega_n)
          .units(Units::RadianPerSecond)
          .defaultValue("0.1")
          .description("Natural frequency for the speed reference model");

          param("Reference Model - Relative Damping", m_args.refmodel_xi)
          .units(Units::None)
          .defaultValue("0.9")
          .description("Relative Damping Factorof the speed reference model");

          param("Velocity Controller - Kp", m_args.Kp)
          .units(Units::None)
          .defaultValue("0.1")
          .description("P-Gain of the velocity controller");






          // Initialize refmodel
          initRefmodel();

        }

        void
        onUpdateParameters(void)
        {
          PathController::onUpdateParameters();
        }

        void
        onEntityReservation(void)
        {
          PathController::onEntityReservation();
        }

        void
        initRefmodel(void)
        {
          // Convencience matrix
          double ones[] = {1.0, 1.0, 1.0};

          Matrix eye = Matrix(ones, 3);

          // Restart refmodel
          m_refmodel_x = Matrix(6, 1, 0.0);

          // Set model
          Matrix A_12 = eye;
          Matrix A_11 = Matrix(3,3, 0.0);

          Matrix A_21 = -pow(m_args.refmodel_omega_n, 2)*eye;
          Matrix A_22 = -2*m_args.refmodel_omega_n * m_args.refmodel_xi * eye;

          Matrix A_1 = A_11.horzCat(A_12);
          Matrix A_2 = A_21.horzCat(A_22);

          m_refmodel_A = A_1.vertCat(A_2);

          m_refmodel_B = Matrix(3,3, 0.0).vertCat(eye) * pow(m_args.refmodel_omega_n,2);

        }

        void
        onPathActivation(void)
        {
          if (!m_args.use_controller)
            return;
          // Activate velocity controller.
          enableControlLoops(IMC::CL_SPEED);

          // Activate height controller
          enableControlLoops(IMC::CL_ALTITUDE);
          inf("Vel-control activated.");

          // Restart ref model
          initRefmodel();
        }

        void
        step(const IMC::EstimatedState& state, const TrackingState& ts)
        {

          if (!m_args.use_controller)
            return;


          /*
           *
           * Refmodel: Deactivated for now.
           *
          // NB: Refmodel position is relative to start coordinates
          Matrix x_d = Matrix(3,1, 0.0);
          x_d(0) = ts.end.x - ts.start.x;
          x_d(1) = ts.end.y - ts.start.y;
          x_d(2) = state.height - ts.end.z - ts.start.z;

          // Update reference
          m_refmodel_x += ts.delta * (m_refmodel_A * m_refmodel_x + m_refmodel_B * x_d);

          // Saturate velocity
          Matrix vel = m_refmodel_x.get(3,5,0,0);


          if( vel.norm_2() > m_args.max_speed )
          {
            vel = m_args.max_speed * vel / vel.norm_2();

            m_refmodel_x.put(3,0,vel);
          }

          debug("Vel norm: %f", m_refmodel_x.get(3,5,0,0).norm_2());




          // Move at a rate towards the target

          // Head straight to target

          vel(0) = m_refmodel_x(3) - m_args.Kp * (state.x - m_refmodel_x(0));
          vel(1) = m_refmodel_x(4) - m_args.Kp * (state.y - m_refmodel_x(1));
          vel(2) = m_refmodel_x(5) - m_args.Kp * (state.z - m_refmodel_x(2));

          */

          Matrix vel = Matrix(3,1, 0.0);

          vel(0) = - m_args.Kp * (state.x - ts.end.x);
          vel(1) = - m_args.Kp * (state.y - ts.end.y);
          vel(2) = - m_args.Kp * (state.z - ts.end.z);

          if( vel.norm_2() > m_args.max_speed )
          {
            vel = m_args.max_speed * vel / vel.norm_2();
          }

          m_velocity.u = vel(0);
          m_velocity.v = vel(1);
          m_velocity.w = vel(2);

          // Todo: Add seperate altitude controller.
          m_velocity.w = 0;

          m_velocity.flags = IMC::DesiredVelocity::FL_SURGE | IMC::DesiredVelocity::FL_SWAY | IMC::DesiredVelocity::FL_HEAVE;

          dispatch(m_velocity);
          debug("Sent vel data.");
        }
      };
    }
  }
}

DUNE_TASK
