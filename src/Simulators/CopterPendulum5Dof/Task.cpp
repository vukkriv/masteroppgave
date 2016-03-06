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
      //! Start point parameters
      double lat;
      double lon;
      double hgt;

      //! Model parameters
      double mass_copter;
      double mass_load;
      double length;
      double air_drag_coeff;
      bool   enable_load;

      //! Disturbance
      double wind[3];
    };

    struct Task: public DUNE::Tasks::Periodic
    {
      //! Task arguments
      Arguments m_args;
      //! Model
      CopterPendulumModel m_model;
      //! Current state
      Matrix m_eta;
      //! Current velocity (inertial!)
      Matrix m_nu;
      //! Previous step time
      double m_time_prev_step;
      //! Last input received
      Matrix m_tau;
      //! Current simulated state
      IMC::SimulatedState m_sstate;
      //! Current euler angles
      IMC::EulerAngles m_eangles;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Periodic(name, ctx),
        m_eta(5,1, 0.0),
        m_nu (5,1, 0.0),
        m_tau(5,1, 0.0)
      {

        param("Model - Mass Copter", m_args.mass_copter)
        .defaultValue("2.5");

        param("Model - Mass Load", m_args.mass_load)
        .defaultValue("0.5");

        param("Model - Length", m_args.length)
        .defaultValue("3");

        param("Model - Lat", m_args.lat)
        .defaultValue("63.628814")
        .units(Units::Degree);

        param("Model - Lon", m_args.lon)
        .defaultValue("9.727400")
        .units(Units::Degree);

        param("Model - Height", m_args.hgt)
        .defaultValue("20")
        .units(Units::Meter);

        param("Model - Air Drag Coefficient", m_args.air_drag_coeff)
        .defaultValue("0.15");

        param("Model - Enable Load", m_args.enable_load)
        .defaultValue("false");

        param("Wind - X", m_args.wind[0])
        .defaultValue("3.0")
        .units(Units::MeterPerSecond);

        param("Wind - Y", m_args.wind[1])
        .defaultValue("3.0")
        .units(Units::MeterPerSecond);

        param("Wind - Z", m_args.wind[2])
        .defaultValue("0.0")
        .units(Units::MeterPerSecond);


        m_time_prev_step = Time::Clock::get() - 1.0;

        bind<IMC::DesiredControl>(this);



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
      onResourceInitialization(void)
      {
        // Set first tau  to hold it steady
        if( m_args.enable_load)
          m_tau(2) = - Math::c_gravity * (m_args.mass_copter + m_args.mass_load);
        else
          m_tau(2) = - Math::c_gravity * (m_args.mass_copter);

        // Reset
        m_eta = Matrix(5, 1, 0.0);
        m_nu  = Matrix(5, 1, 0.0);


        m_time_prev_step = Time::Clock::get() - 1.0;
      }

      void
      consume(const IMC::DesiredControl* msg)
      {
        if ((msg->flags & IMC::DesiredControl::FL_X) && !isnan(msg->x))
          m_tau(0) = msg->x*m_args.mass_copter;

        if ((msg->flags & IMC::DesiredControl::FL_Y) && !isnan(msg->y))
          m_tau(1) = msg->y*m_args.mass_copter;

        if ((msg->flags & IMC::DesiredControl::FL_Z) && !isnan(msg->z))
        {
          if( m_args.enable_load )
            m_tau(2) = msg->z*m_args.mass_copter - Math::c_gravity * (m_args.mass_copter + m_args.mass_load);
          else
            m_tau(2) = msg->z*m_args.mass_copter - Math::c_gravity * (m_args.mass_copter);
        }


        spew("Received and updated tau. ");
      }

      void
      stepWithoutPendulum(Matrix tau, double dt)
      {
        // Simply a second order model.
        double g[3] = {0.0, 0.0, -m_args.mass_copter * Math::c_gravity};
        Matrix G = Matrix(g, 3, 1);

        double d[3] = {m_args.air_drag_coeff, m_args.air_drag_coeff, m_args.air_drag_coeff};
        Matrix D = Matrix(d, 3);

        Matrix eta = m_eta.get(0,2,0,0);
        Matrix nu = m_nu.get(0,2,0,0);

        Matrix wind = Matrix(m_args.wind, 3, 1);

        Matrix next_eta = eta + dt * nu;
        Matrix next_nu = nu + dt * (1.0/m_args.mass_copter) * (tau.get(0,2,0,0) - G - D*(nu - wind));

        m_eta = next_eta.vertCat(Matrix(2,1, 0.0));
        m_nu  = next_nu.vertCat(Matrix(2,1, 0.0));
      }

      void
      stepWithPendulum(Matrix tau, double dt)
      {
        Matrix M = m_model.getMassMatrix(m_eta);
        Matrix C = m_model.getCoreolisMatrix(m_eta, m_nu);
        Matrix G = m_model.getGravityMatrix(m_eta);
        double d[5] = {m_args.air_drag_coeff, m_args.air_drag_coeff, m_args.air_drag_coeff, 0.01, 0.01};
        Matrix D = Matrix(d, 5);

        Matrix next_eta = m_eta;
        Matrix next_nu  = m_nu;

        Matrix wind_13 = Matrix(m_args.wind, 3, 1);
        Matrix wind_45 = Matrix(2,1, 0.0);
        Matrix wind    = wind_13.vertCat(wind_45);

        try{
          next_eta = m_eta + dt * m_nu;
          next_nu  = m_nu  + dt * inverse(M) * (tau - C*m_nu - G - D*(m_nu - wind));
          if (next_nu.norm_2() > 50)
          {
            err("Insanely large nu, aborting. Nu, Theta, Inverted M:");
            printMatrix(next_eta, DEBUG_LEVEL_NONE);
            printMatrix(next_nu, DEBUG_LEVEL_NONE);
            printMatrix(inverse(M), DEBUG_LEVEL_NONE);


            throw RestartNeeded("Large NU! ", 1);


          }
        }
        catch(Matrix::Error& error)
        {
          err("Error doing step. Aborting simulation step: %s", error.what());
          return;
        }

        // Store new values
        m_eta = next_eta;
        m_nu  = next_nu;
      }

      void
      step(Matrix tau)
      {
        if(tau.size() < 5)
        {
          err("Invalid tau sent. Aborting simulation step. ");
          return;
        }

        double maxNorm = 500.0;
        if(tau.norm_2() > maxNorm)
        {
          tau = maxNorm * tau / tau.norm_2();
          war("Constraining tau..");
        }
        double now = Time::Clock::get();
        double dt  = now - m_time_prev_step;

        if (dt < 0.0 || dt > 2.0)
        {
          err("delta t to small or to large. Aborting simulation step.: %.4f", dt);
          //err("Now: %f, Prev: %f", now, m_time_prev_step);
          //err("Calc: %f", 1.0/getFrequency());

          if (dt > 3.0)
            throw RestartNeeded("DT more than 3 seconds.  ", 1);

          return;
        }

        if( m_args.enable_load )
          stepWithPendulum(tau, dt);
        else
          stepWithoutPendulum(tau, dt);



        m_time_prev_step = now;
        spew("Did a step with dt=%.3f", dt);
      }


      void
      fill(void)
      {
        m_sstate.lat = Angles::radians(m_args.lat);
        m_sstate.lon = Angles::radians(m_args.lon);
        m_sstate.height = m_args.hgt;

        m_sstate.x   = m_eta(0);
        m_sstate.y   = m_eta(1);
        m_sstate.z   = m_eta(2);


        BodyFixedFrame::toBodyFrame(0.0, 0.0, 0.0,
                                    m_nu(0), m_nu(1), m_nu(2),
                                    &m_sstate.u, &m_sstate.v, &m_sstate.w);

        // All angles are zero.

        m_eangles.time = m_time_prev_step;
        m_eangles.phi  = m_eta(3);
        m_eangles.theta = m_eta(4);

        // Stream speed
        m_sstate.svx = m_args.wind[0];
        m_sstate.svy = m_args.wind[1];
        m_sstate.svz = m_args.wind[2];
      }



      //! Main loop.
      void
      task(void)
      {
        // Do the step
        step(m_tau);

        // Fill and dispatch
        fill();

        dispatch(m_sstate);
        dispatch(m_eangles);



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
    };
  }
}

DUNE_TASK
