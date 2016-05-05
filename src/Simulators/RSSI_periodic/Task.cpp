//***************************************************************************
// Copyright 2007-2016 Universidade do Porto - Faculdade de Engenharia      *
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

namespace Simulators
{
  namespace RSSI_periodic
  {
    using DUNE_NAMESPACES;

    struct Arguments
    {
      bool sim_flag;
      double noise_amp;
      bool drop_meas;
    };

    struct Task: public DUNE::Tasks::Periodic
    {
      //! Task arguments:
      Arguments m_args;


      double m_lat_base;
      double m_lon_base;
      double m_rssi;

      IMC::GpsFix gpsdata;
      IMC::EstimatedState estate;
      IMC::RSSI m_sim_rssi;

      Task(const std::string& name, Tasks::Context& ctx):
        Periodic(name, ctx),
        m_lat_base(1.11054170),
        m_lon_base(0.16976546),
        m_rssi(0)
      {
        param("SimFlag", m_args.sim_flag)
        .defaultValue("true")
        .description("True if unicycle simulator is used");

        param("NoiseAmp", m_args.noise_amp)
        .defaultValue("0.1")
        .description("Size of white noise on RSSI");

        param("UseDropMeas", m_args.drop_meas)
        .defaultValue("true")
        .description("True if drop of RSSI measurements should be used");


        bind<IMC::GpsFix>(this);
        bind<IMC::EstimatedState>(this);
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
      consume(const IMC::GpsFix* gps)
      {
          if(resolveEntity(gps->getSourceEntity()) == "Autopilot"){
            gpsdata = *gps;
          }
      }


      void
      consume(const IMC::EstimatedState* msg)
      {
          estate = *msg;

      }

      //! Generate random number used to simulate white noise.
      double
      randNumber(void)
      {
       srand(time(NULL));
       double r = rand() % 10 - 5;
       return r;
      }

      void
      task(void)
      {
        if (m_args.sim_flag)
        {
          double dist_x = estate.x;
          double dist_y = estate.y;

          // Distance from base station to uav (in rad):
          double dist = sqrt(pow(dist_x,2) + pow(dist_y,2));

          // RSSI test formula:
          m_rssi = 100*exp(-dist*0.001);

        }
        else
        {
          double lat_uav = gpsdata.lat;
          double lon_uav = gpsdata.lon;

          double dlat = m_lat_base - lat_uav;
          double dlon = m_lon_base - lon_uav;

          // Distance from base station to uav (in rad):
          double dist = sqrt(pow(dlat,2) + pow(dlon,2));

          // RSSI test formula:
          m_rssi = 100*exp(-dist*2000);
        }

        //! White noise:
        double rand = randNumber();

        //! If dropped measurements should be simulated.
        if (m_args.drop_meas)
        {
          //! Simulate dropped measurements:
          if (rand == 1) // Should happend 10% of the time.
          {
            m_sim_rssi.value = 0;
          }
          else
          {
            m_sim_rssi.value = m_rssi + m_args.noise_amp*rand;
          }
        }
        else
        {
          m_sim_rssi.value = m_rssi + m_args.noise_amp*rand;
        }


        //! Dispatch to IMC bus.
        dispatch(m_sim_rssi);


      }
    };
  }
}

DUNE_TASK
