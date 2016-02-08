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
// Author: Kristian Klausen                                                 *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Simulators
{
  namespace GPSRTK
  {
    using DUNE_NAMESPACES;

    struct Arguments {
      bool enable;
      std::string fix_level;
    };

    struct Task: public DUNE::Tasks::Periodic
    {
      //! Task arguments
      Arguments m_args;
      //! Last received data
      IMC::ExternalNavData m_navdata;
      //! Generated RtkFix Message
      IMC::GpsFixRtk m_rtk;
      //! Fix level to send
      IMC::GpsFixRtk::TypeEnum m_fix_type;



      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Periodic(name, ctx),
        m_fix_type(IMC::GpsFixRtk::RTK_NONE)
      {

        param("Enable", m_args.enable)
        .defaultValue("true");

        param("Fix Level", m_args.fix_level)
        .values("Fix,Float,None")
        .defaultValue("Fix");


        m_navdata.clear();
        m_rtk.clear();


        bind<IMC::ExternalNavData>(this);
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        if (m_args.fix_level == "Fix")
          m_fix_type = IMC::GpsFixRtk::RTK_FIXED;
        else if(m_args.fix_level == "Float")
          m_fix_type = IMC::GpsFixRtk::RTK_FLOAT;
        else
          m_fix_type = IMC::GpsFixRtk::RTK_NONE;
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
      consume(const IMC::ExternalNavData* navdata)
      {
        m_navdata = *navdata;
      }

      void
      fillRtkMessage()
      {

        if (m_navdata.state.isNull())
        {
          trace("Attempted to fill from an empty EstimatedState message. Ignoring. ");
          return;
        }

        m_rtk.n = m_navdata.state->x;
        m_rtk.e = m_navdata.state->y;
        m_rtk.d = m_navdata.state->z;
        m_rtk.validity |= IMC::GpsFixRtk::RFV_VALID_POS;

        m_rtk.v_n = m_navdata.state->vx;
        m_rtk.v_e = m_navdata.state->vy;
        m_rtk.v_d = m_navdata.state->vz;
        m_rtk.validity |= IMC::GpsFixRtk::RFV_VALID_VEL;

        m_rtk.base_lat = m_navdata.state->lat;
        m_rtk.base_lon = m_navdata.state->lon;
        m_rtk.base_height = m_navdata.state->height;
        m_rtk.validity |= IMC::GpsFixRtk::RFV_VALID_BASE;

        m_rtk.type = m_fix_type;
      }



      void
      task(void)
      {
        if (m_args.enable)
        {
          fillRtkMessage();
          dispatch(m_rtk);
        }
      }
    };
  }
}

DUNE_TASK
