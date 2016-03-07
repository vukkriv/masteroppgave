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
      std::string elabel_gps;
      double hae_offset;
    };
    struct Task: public DUNE::Tasks::Periodic
    {
      //! GPS Fix message.
      IMC::GpsFix m_fix;
      //! RTK Fix message.
      IMC::GpsFixRtk m_rtkfix;
      //! Task arguments.
      Arguments m_args;
      //! GPS entity eid.
      int m_gps_eid;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Periodic(name, ctx)
      {
        param("BaseIsFixed", m_args.base_is_fixed)
        .defaultValue("False")
        .visibility(Parameter::VISIBILITY_USER)
        .description("When set to true by operator, ");
        
        param("Entity Label - GPS", m_args.elabel_gps)
        .description("Entity label of 'GpsFix' and 'GroundVelocity' messages");

        param("Height Offset", m_args.hae_offset)
        .minimumValue("-100.0")
        .defaultValue("0.0")
        .maximumValue("100.0")
        .description("Offset in height to apply. ");


        clearMessages();

        bind<IMC::GpsFix>(this);
        //bind<IMC::GpsFixRtk>(this);
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        debug("Updating parameters");


        if ((paramChanged(m_args.base_is_fixed)) && (m_args.base_is_fixed == true))
        {
          //Base pos has been locked by the operator
          if (m_fix.validity & IMC::GpsFix::GFV_VALID_POS)
          {
            //GPS pos is valid
            //Set the gps pos of the base
            m_rtkfix.base_lat = m_fix.lat;
            m_rtkfix.base_lon = m_fix.lon;
            m_rtkfix.base_height = m_fix.height + m_args.hae_offset;
            m_rtkfix.validity = IMC::GpsFixRtk::RFV_VALID_BASE;
            
            //Send base pos to all rovers
            debug("Dispatching base position");
            dispatch(m_rtkfix);
          }
          else
          {
            //GPS pos is not valid
            debug("Unfixing base due to invalid GPS pos");
            //m_args.base_is_fixed = false;

            IMC::EntityParameter fix_base;
            fix_base.name = "BaseIsFixed";
            fix_base.value = "False";

            MessageList<IMC::EntityParameter> msgList;
            msgList.push_back(fix_base);

            IMC::SetEntityParameters toSet;
            toSet.name = getEntityLabel();
            toSet.params = msgList;

            dispatch(toSet, DF_LOOP_BACK);


          }
        }

      }

      void
      consume(const IMC::GpsFix* msg)
      {
        if (msg->getSourceEntity() != m_gps_eid)
          return;

        debug("Consuming GPS-Fix");

        // Defining origin.
        m_fix = *msg;

        // Dispatch to navigation task. Set own id.
        m_fix.setSourceEntity(getEntityId());
        m_fix.setTimeStamp();

        if (m_args.base_is_fixed)
        {
          // override fields
          m_fix.lat = m_rtkfix.base_lat;
          m_fix.lon = m_rtkfix.base_lon;
          m_fix.height = m_rtkfix.base_height;
        }

        dispatch(m_fix);

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
        try {
          m_gps_eid = resolveEntity(m_args.elabel_gps);
        }
        catch (...)
        {
          m_gps_eid = 0;
          war("Unable to resolve entity. ");
        }
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
      clearMessages(void)
      {
        m_rtkfix.clear();
        m_fix.clear();
      }

      //! Main loop.
      void
      task(void)
      {
          // Handle IMC messages from bus
          ///consumeMessages();
      //    waitForMessages(1.0);

          //Delay::wait(10.0);
          if (m_args.base_is_fixed)
          {
            debug("Dispatching base position");
            dispatch(m_rtkfix);
          }
          //waitForMessages(1.0);
          spew("Im alive! Base_is_fixed %d", m_args.base_is_fixed);
      }
    };
  }
}

DUNE_TASK
