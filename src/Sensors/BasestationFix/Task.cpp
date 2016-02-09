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

namespace Sensors
{
  namespace BasestationFix
  {
    using DUNE_NAMESPACES;

    struct Arguments
    {
      bool base_is_fixed;
      uint8_t comm_timeout;

    };
    struct Task: public DUNE::Tasks::Task
    {
      //! GPS Fix message.
      IMC::GpsFix m_fix;
      //! RTK Fix message.
      IMC::GpsFixRtk m_rtkfix;
      //! Task arguments.
      Arguments m_args;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx)
      {
        param("Communication Timeout1", m_args.comm_timeout)
        .defaultValue("5")
        .units(Units::Second)
        .description("Timeout for base and local communication.");
        
        param("BaseIsFixed", m_args.base_is_fixed)
        .defaultValue("False")
        .description("When set to true by operator, ");
        
        clearMessages();

        bind<IMC::GpsFix>(this);
        //bind<IMC::GpsFixRtk>(this);
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        debug("Updating parameters");
        if (paramChanged(m_args.base_is_fixed))
        {
          if (m_args.base_is_fixed == true)
          {
            //Base pos has been locked by the operator
            //Set the gps pos of the base
            m_rtkfix.base_lat = m_fix.lat;
            m_rtkfix.base_lon = m_fix.lon;
            m_rtkfix.base_height = m_fix.height;
            m_rtkfix.validity = 1;
            
            //Send base pos to all rovers
            dispatch(m_rtkfix);
          }
        }
      }

      void
      consume(const IMC::GpsFix* msg)
      {
        debug("Consuming GPS-Fix");

        // Defining origin.
        m_fix.lat = msg->lat;
        m_fix.lon = msg->lon;
        m_fix.height = msg->height;

        spew("Im at lat %f lon %f", m_fix.lat, m_fix.lon);
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
      clearMessages(void)
      {
        m_rtkfix.clear();
        m_fix.clear();
      }

      //! Main loop.
      void
      onMain(void)
      {
        while (!stopping())
        {
          Delay::wait(10.0);
          if (m_args.base_is_fixed)
          {
            dispatch(m_rtkfix);
          }
          //waitForMessages(1.0);
          spew("Im alive! Base_is_fixed %d, timeout %d", m_args.base_is_fixed, m_args.comm_timeout);
        }
      }
    };
  }
}

DUNE_TASK
