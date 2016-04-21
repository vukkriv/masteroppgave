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
  namespace RSSI
  {
    using DUNE_NAMESPACES;

    struct Task: public DUNE::Tasks::Task
    {
      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.


      //double lat_uav;
      //double lon_uav;
      double m_lat_base;
      double m_lon_base;
      double m_rssi;
      //double dist;
      IMC::GpsFix gpsdata;
      //IMC::EstimatedState estate;
      IMC::RSSI m_sim_rssi;

      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        //lat_uav(0.0),
        //lon_uav(0.0),
        m_lat_base(1.11054170),
        m_lon_base(0.16976546),
        m_rssi(0)
      //dist(0.0)
      {
        bind<IMC::GpsFix>(this);

        //bind<IMC::EstimatedState>(this);
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
      consume(const IMC::GpsFix* gps){
        gpsdata = *gps;
        //inf("gps lat: %f: ", gps->lat);

        calcSimRSSI_gps(gpsdata);
      }

      /*
        void
        consume(const IMC::EstimatedState* msg)
        {
          estate = *msg;

          calcSimRSSI(estate);

        }

       */

      void
      calcSimRSSI_gps(IMC::GpsFix gpsarg){
        double lat_uav = gpsarg.lat;
        double lon_uav = gpsarg.lon;

        double dlat = m_lat_base - lat_uav;
        double dlon = m_lon_base - lon_uav;

        // Distance from base station to uav (in rad):
        double dist = sqrt(pow(dlat,2) + pow(dlon,2));

        // RSSI test formula:
        double rssi = 100*exp(-dist*100);

        // use RSSI message from IMC
        IMC::RSSI sim_rssi;
        sim_rssi.value = rssi;

        dispatch(sim_rssi);

      }

      /*
        void
        calcSimRSSI(IMC::EstimatedState statearg){
          double lat_uav = statearg.lat;
          double lon_uav = statearg.lon;

          double dlat = m_lat_base - lat_uav;
          double dlon = m_lon_base - lon_uav;

          // Distance from base station to uav (in rad):
          double dist = sqrt(pow(dlat,2) + pow(dlon,2));

          // RSSI test formula:
          m_rssi = 100*exp(-dist*100);

          // use RSSI message from IMC
          //IMC::RSSI sim_rssi;
          m_sim_rssi.value = m_rssi;

          dispatch(m_sim_rssi);

          //inf("RSSI value is: %f", m_sim_rssi.value);

        }
       */


      //! Main loop.
      void
      onMain(void)
      {

        while (!stopping())
        {

          waitForMessages(1.0);
        }
      }
    };
  }
}


DUNE_TASK
