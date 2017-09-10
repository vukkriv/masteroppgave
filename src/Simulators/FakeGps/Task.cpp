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
// Author: Kristoffer Gryte                                                 *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Simulators
{
  namespace FakeGps
  {
    using DUNE_NAMESPACES;

    struct Arguments
    {
      bool valid_pos;
      double lon_start;
      double lat_start;
      double height_start;
    };

    struct Task: public DUNE::Tasks::Task
    {
      //! GPS Fix message.
      IMC::GpsFix m_fix;
      //! RTK Fix message.
      IMC::GpsFixRtk m_rtkfix;
      Arguments m_args;
      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx)
      {
        param("Valid Position", m_args.valid_pos)
		.defaultValue("True")
    .visibility(Parameter::VISIBILITY_USER)
		.description("Chose whether the position is valid or not ");

        param("Start point -- latitude", m_args.lat_start)
        .units(Units::Degree)
        .minimumValue("-90")
        .defaultValue("63.629246")
        .maximumValue("90");

        param("Start point -- longitude", m_args.lon_start)
        .units(Units::Degree)
        .minimumValue("-180")
        .defaultValue("9.726731")
        .maximumValue("180");

        param("Start point -- height", m_args.height_start)
        .units(Units::Meter)
        .defaultValue("90");
        //CMAC  -35.363261 149.165230 584.353;
        

        bind<IMC::GpsFixRtk>(this);

        m_fix.validity |= IMC::GpsFix::GFV_VALID_POS;
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
        setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
      }

      void
      consume(const IMC::GpsFixRtk* msg)
      {
        debug("Consuming RTK-Fix");

        // Defining origin.
        m_rtkfix.base_lat = msg->base_lat;
        m_rtkfix.base_lon = msg->base_lon;
        m_rtkfix.base_height = msg->base_height;

        spew("Base locked at lat %f lon %f", m_rtkfix.base_lat, m_rtkfix.base_lon);
      }

      //! Main loop.
      void
      onMain(void)
      {
        m_fix.lat = Angles::radians(m_args.lat_start);
        m_fix.lon = Angles::radians(m_args.lon_start);
        m_fix.height = m_args.height_start;

        while (!stopping())
        {
          if (m_args.valid_pos)
          {
            m_fix.validity |= IMC::GpsFix::GFV_VALID_POS;
          }
          else
          {
            m_fix.validity &= ~IMC::GpsFix::GFV_VALID_POS;
          }


          waitForMessages(1.0);
          m_fix.lat += 0.000001;
          dispatch(m_fix);
          spew("Lat is now %f", m_fix.lat);
          Delay::wait(1.0);
        }
      }
    };
  }
}

DUNE_TASK
