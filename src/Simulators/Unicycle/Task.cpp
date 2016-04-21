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
      //! Initial position of vehicle in llh in rad:
      double lat0;
      double lon0;
      double height0;

      //! Desired velocity:
      double dvelocity;

      //! Sample time;
      double T;

      //! Test mode (constant heading)
      bool testmode;

    };

    struct Task: public DUNE::Tasks::Task
    {
      //! Task arguments:
      Arguments m_args;

      IMC::DesiredHeading m_psi_receive;

      //! Displacement from last position:
      Matrix m_displacement;

      //! Current vehicle position (lat-lon):
      long double m_current_lat;
      long double m_current_lon;

      // Previous and current time:
      double m_time_prev;
      double m_dt;

      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_displacement(2,1,0.0),
        //m_current_lat(1.1110653618992075),
        //m_current_lon(0.17123088442770734)
        m_current_lat(0),
        m_current_lon(0),
        m_time_prev(0),
        m_dt(0)


      {
        param("Lat0", m_args.lat0)
        .defaultValue("0")//1.11054170")
        .description("");

        param("Lon0", m_args.lon0)
        .defaultValue("0")//0.16976546")
        .description("");

        param("Height0", m_args.height0)
        .defaultValue("50")
        .description("");

        param("DesiredVelocity", m_args.dvelocity)
        .defaultValue("0")
        .description("");

        param("SampleTime", m_args.T)
        .defaultValue("0.1")
        .description("Sample time");

        param("testMode", m_args.testmode)
        .defaultValue("true")
        .description("Test Mode");

        //! Bind incoming IMC message:
        bind<IMC::DesiredHeading>(this);

        //! Initiate current position matrix:
        initPos();
      }


      void
      initPos(void)
      {
        //inf("latlon set");

        //m_current_pos(0) = m_args.lat0;
        //m_current_pos(1) = m_args.lon0;

        // Os:
        //m_current_lat = 1.091081534; //m_args.lat0;
        //m_current_lon = 0.19501912872; //m_args.lon0;

        // Other side of fjord:
        //m_current_lat = 1.1110698474; //m_args.lat0;
        //m_current_lon = 0.17263353573; //m_args.lon0;

        // Gloshaugen:
        m_current_lat = 63.418545 * (Math::c_pi/180.0); //1.1110698474;
        m_current_lon = 10.402840 * (Math::c_pi/180.0); //0.17263353573;

        // Init time:
        m_time_prev = Clock().get();
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
        m_psi_receive = *msg;

        //! Calculate next position when new desired heading occurs:
        calculateNextPos(m_psi_receive);
        inf("Print this");
      }

      void
      calculateNextPos(IMC::DesiredHeading psi_arg)
      {
        double psi_d;
        if (m_args.testmode == true)
        {
          //! Test heading:
          psi_d = 0;
        }else{
          psi_d = psi_arg.value;
        }

        //! Get time:
        double time_current = Clock().get();
        m_dt = time_current - m_time_prev;
        m_time_prev = time_current;


        //! Calculate next position based on current position, heading and speed:
        Matrix R = Matrix(2,1,0.0);
        R(0) = cos(psi_d);
        R(1) = sin(psi_d);

        //! Only use initial latlon:
        //Matrix next_pos = m_current_pos + m_args.dvelocity*m_args.T * R;

        //! Calculate displacement:
        m_displacement = m_args.dvelocity*m_args.T * R;
        //m_displacement = 2*1*R;

        //! Update current position:
        //m_displacement = next_pos;

        //! NED to LLH transformation:
        WGS84::displace(m_displacement(0),m_displacement(1), &m_current_lat, &m_current_lon);

        //! Print current latlon position:
        //inf("Current lat: %Lf", m_current_lat);
        //inf("Current lon: %Lf", m_current_lon);

        //! Store next_pos in estate and dispatch estate to IMC message bus:
        IMC::EstimatedState estate;
        estate.lat = m_current_lat;
        estate.lon = m_current_lon;
        estate.psi = psi_d;
        dispatch(estate);

      }

      //! Main loop.
      void
      onMain(void)
      {
        //! Initiate current position matrix:
        //initPos();

        while (!stopping())
        {
          //! Temporary running this in main instead of in consume() due to no desired heading received
          //calculateNextPos();

          waitForMessages(1.0);
        }
      }
    };
  }
}

DUNE_TASK
