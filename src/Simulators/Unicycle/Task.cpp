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
// Author: Daniel Røyland                                                  *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>
#include "math.h"

namespace Simulators
{
  namespace Unicycle
  {
    using DUNE_NAMESPACES;

    struct Arguments
    {
      //! Initial position of vehicle in llh in deg:
      double lat0;
      double lon0;
      double height0;

      //! Initial displacement of vehicle in meters north and meters east:
      double x0;
      double y0;

      //! Desired velocity:
      double V_d;

      //! Test mode (constant heading)
      bool testMode;
      double testHeading;

    };

    struct Task: public DUNE::Tasks::Periodic
    {
      //! Task arguments:
      Arguments m_args;

      IMC::DesiredHeading m_psi_d;

      //! Displacement from last position:
      Matrix m_displacement;

      //! Base station position (lat-lon):
      long double m_base_lat;
      long double m_base_lon;

      // Previous and current time:
      double m_time_prev;
      double m_dt;

      //! Estimated state of vehicle:
      IMC::EstimatedState m_estate;

      Task(const std::string& name, Tasks::Context& ctx):
        Periodic(name, ctx),
        m_displacement(2,1,0.0),
        m_base_lat(1.11054269),
        m_base_lon(0.169757722),
        m_time_prev(Clock::get()),
        m_dt(0.0)

      {
        param("Lat0", m_args.lat0)
        .defaultValue("0.0")
        .description("");

        param("Lon0", m_args.lon0)
        .defaultValue("0.0")
        .description("");

        param("Height0", m_args.height0)
        .defaultValue("100")
        .description("");

        param("X0", m_args.x0)
        .defaultValue("1200.0")
        .description("Initial displacement of vehicle in meters north");

        param("Y0", m_args.y0)
        .defaultValue("1000.0")
        .description("Initial displacement of vehicle in meters east");

        param("DesiredVelocity", m_args.V_d)
        .defaultValue("10")
        .description("Desired Velocity in m/s");

        param("TestMode", m_args.testMode)
        .defaultValue("true")
        .description("Test Mode");

        param("TestHeading", m_args.testHeading)
        .defaultValue("0.0")
        .description("Test Heading in degrees");

        //! Bind incoming IMC message:
        bind<IMC::DesiredHeading>(this);

        //! Initiate current position matrix:
        initPos();
      }


      void
      initPos(void)
      {

        m_estate.lat = m_base_lat;
        m_estate.lon = m_base_lon;

        // Os:
        //m_current_lat = 1.091081534;
        //m_current_lon = 0.19501912872;

        // Other side of fjord:
        //m_current_lat = 1.1110698474;
        //m_current_lon = 0.17263353573;

        // Gloshaugen:
        //m_current_lat = 63.418545 * (Math::c_pi/180.0); //1.1110698474;
        //m_current_lon = 10.402840 * (Math::c_pi/180.0); //0.17263353573;

      }


      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
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

        m_displacement(0) = m_args.x0;
        m_displacement(1) = m_args.y0;

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
      consume(const IMC::DesiredHeading *msg)
      {
        m_psi_d = *msg;
      }

      void
      task(void)
      {

        //! Compute timestep dt:
        double dt = Clock().get() - m_time_prev;
        m_time_prev = Clock::get();

        if (dt > 2*(1/this->getFrequency()) )
        {
          debug("Warning: Missed time. Should be %f, was %f", 1/this->getFrequency(), dt);
        }
        //inf("Task freq.: %f", this->getFrequency());

        //! Choose between heading from ES controller or test heading:
        double psi_d;
        if (m_args.testMode == true)
        {
          //! Test heading:
          psi_d = m_args.testHeading * (Math::c_pi/180.0);
        }
        else
        {
          //! ES heading:
          psi_d = m_psi_d.value;
        }

        //! Calculate displacement:
        //! Based on simple unicycle equations, m_displacement = x_{k+1} - x_k
        //! integrated with Forward Euler.
        Matrix R = Matrix(2,1,0.0);
        R(0) = cos(psi_d);
        R(1) = sin(psi_d);
        m_displacement = m_displacement + m_args.V_d*dt * R;

        //! Store next_pos in m_estate and dispatch estate to IMC message bus:
        m_estate.x = m_displacement(0);
        m_estate.y = m_displacement(1);
        m_estate.psi = psi_d;
        dispatch(m_estate);


      }
    };
  }
}

DUNE_TASK
