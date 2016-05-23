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
// Author: Kjetil Hope Sørbø                                              *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

// USER headers
#include <USER/DUNE.hpp>

namespace Navigation
{
  namespace General
  {
    namespace RtkNavigation
    {
      using DUNE_NAMESPACES;

      struct Task: public DUNE::Tasks::Task
      {
        //! Accumulated EstimatedState message
        IMC::EstimatedState m_estate;
        //! Accumulated GpsFix message
        IMC::GpsFix m_gps;
        //! Last received RTK Fix message
        IMC::GpsFixRtk m_rtk;

        //! Constructor.
        //! @param[in] name task name.
        //! @param[in] ctx context.
        Task(const std::string& name, Tasks::Context& ctx):
          DUNE::Tasks::Task(name, ctx)
        {
          bind<IMC::GpsFixRtk>(this);
        }

        void
        consume(const IMC::GpsFixRtk* rtkfix)
        {
          // Only care about fixes from ourselves
          if (rtkfix->getSource() != getSystemId())
            return;

          // Only care if position is valid
          if (!(rtkfix->validity & IMC::GpsFixRtk::RFV_VALID_POS))
            return;

          // Only care if Base pos is valid
          if (!(rtkfix->validity & IMC::GpsFixRtk::RFV_VALID_BASE))
          {
            spew("Ignored RtkFix message: Invalid base. ");
            return;
          }

          m_rtk = *rtkfix;

          fillEstimatedState();
          fillGpsFix();
          dispatch(m_estate);
          dispatch(m_gps);
        }

        void
        fillGpsFix()
        {
          double lat = m_estate.lat;
          double lon = m_estate.lon;
          double height = m_estate.height;
          Coordinates::WGS84_Accurate::displace(m_estate.x,m_estate.y,m_estate.z,&lat,&lon,&height);
          m_gps.lat = lat;
          m_gps.lon = lon;
          m_gps.height = height;
          m_gps.validity |= IMC::GpsFix::GFV_VALID_POS;
        }
        void
        fillEstimatedState()
        {
          m_estate.x = m_rtk.n;
          m_estate.y = m_rtk.e;
          m_estate.z = m_rtk.d;

          m_estate.lat = m_rtk.base_lat;
          m_estate.lon = m_rtk.base_lon;
          m_estate.height = m_rtk.base_height;

          if (m_rtk.validity == IMC::GpsFixRtk::RFV_VALID_VEL)
          {
            m_estate.vx = m_rtk.v_n;
            m_estate.vy = m_rtk.v_e;
            m_estate.vz = m_rtk.v_d;
          }
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
}

DUNE_TASK
