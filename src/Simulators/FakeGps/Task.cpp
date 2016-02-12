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
		.description("Chose whether the position is valid or not ");

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
        //CMAC
        m_fix.lat = Angles::radians(-35.363261);
        m_fix.lon = Angles::radians(149.165230);
        m_fix.height = 584.353;
        if (m_args.valid_pos)
        {
        	m_fix.validity |= IMC::GpsFix::GFV_VALID_POS;
        }
        else
        {
        	m_fix.validity &= ~IMC::GpsFix::GFV_VALID_POS;
      	}
        while (!stopping())
        {
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
