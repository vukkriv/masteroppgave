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

// This task is responsible for routing data from various sources to
// dispatch EstimatedState. Later versions could also include filtering.


// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Navigation
{
  namespace UAV
  {
    namespace Navigation
    {
      using DUNE_NAMESPACES;

      struct Arguments {
        bool use_rtk;
        float rtk_tout;
        float rtk_tout_lowerlevel;
        float rtk_min_fix_time;
        std::string rtk_fix_level_activate;
        std::string rtk_fix_level_deactivate;

      };

      struct Task: public DUNE::Tasks::Task
      {
        //! Task arguments
        Arguments m_args;
        //! Accumulated EstimatedState message
        IMC::EstimatedState m_estate;
        //! Last received External nav message
        IMC::ExternalNavData m_extnav;
        //! Last received RTK Fix message
        IMC::GpsFixRtk m_rtk;
        //! Input watchdog.
        Time::Counter<float> m_rtk_wdog_keep_available;
        //! Input watchdog for lower fix level
        Time::Counter<float> m_rtk_wdog_deactivation;
        //! Timer for minimum time needed at activation fix level
        Time::Counter<float> m_rtk_wdog_activation;
        //! Fix level to activate
        IMC::GpsFixRtk::TypeEnum m_rtk_fix_level_activate;
        //! Fix level to deactivate
        IMC::GpsFixRtk::TypeEnum m_rtk_fix_level_deactivate;
        //! Currently used nav sources
        IMC::NavSources m_navsources;
        //! Is RTK available
        bool m_rtk_available;

        //! Constructor.
        //! @param[in] name task name.
        //! @param[in] ctx context.
        Task(const std::string& name, Tasks::Context& ctx):
          DUNE::Tasks::Task(name, ctx),
          m_rtk_fix_level_activate(IMC::GpsFixRtk::RTK_FIXED),
          m_rtk_fix_level_deactivate(IMC::GpsFixRtk::RTK_FIXED),
          m_rtk_available(false)
        {

          param("Use RTK If Available", m_args.use_rtk)
          .defaultValue("false")
          .visibility(Parameter::VISIBILITY_USER);

          param("RTK - Timeout", m_args.rtk_tout)
          .defaultValue("1.0");

          param("RTK - Timeout Lower Level", m_args.rtk_tout_lowerlevel)
          .defaultValue("10.0");

          param("RTK - Time needed on fix level to activate", m_args.rtk_min_fix_time)
          .defaultValue("2.0");

          param("RTK - Minimum Fix Level to Activate", m_args.rtk_fix_level_activate)
          .values("Fix,Float")
          .defaultValue("Fix");

          param("RTK - Minimum Fix Type to Deactivate", m_args.rtk_fix_level_deactivate)
          .values("Fix,Float")
          .defaultValue("Float");

          // Default, we use full external state
          m_navsources.mask = (NS_EXTERNAL_FULLSTATE | NS_EXTERNAL_AHRS | NS_EXTERNAL_POSREF);



          bind<IMC::GpsFixRtk>(this);
          bind<IMC::ExternalNavData>(this);
        }

        //! Update internal state with new parameter values.
        void
        onUpdateParameters(void)
        {
          if (m_args.rtk_fix_level_activate == "Float")
            m_rtk_fix_level_activate = IMC::GpsFixRtk::RTK_FLOAT;
          else
            m_rtk_fix_level_activate = IMC::GpsFixRtk::RTK_FIXED;

          if (m_args.rtk_fix_level_deactivate == "Float")
            m_rtk_fix_level_deactivate = IMC::GpsFixRtk::RTK_FLOAT;
          else
            m_rtk_fix_level_deactivate = IMC::GpsFixRtk::RTK_FIXED;
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
          m_rtk_wdog_keep_available.setTop(m_args.rtk_tout);
          m_rtk_wdog_deactivation.setTop(m_args.rtk_tout_lowerlevel);
          m_rtk_wdog_activation.setTop(m_args.rtk_min_fix_time);
        }

        //! Release resources.
        void
        onResourceRelease(void)
        {
        }

        bool usingRtk()
        {
          return (m_navsources.mask & IMC::NS_GNSS_RTK);
        }

        void
        consume(const IMC::ExternalNavData* navdata)
        {
          m_extnav = *navdata;

          // Just check if using rtk or not
          if (!usingRtk())
          {

            m_estate = *m_extnav.state.get();

            sendStateAndSource();
          }
        }



        void
        updateRtkTimers(void)
        {

          if (m_rtk.type >= m_rtk_fix_level_deactivate)
            m_rtk_wdog_keep_available.reset();


          if (m_rtk_available)
          {

            if (m_rtk.type >= m_rtk_fix_level_activate)
              m_rtk_wdog_deactivation.reset();
          }
          else
          {
            // RTK is currenty not available.
            if (m_rtk.type < m_rtk_fix_level_activate)
            {
              // Not high enough fix level, resetting timer.
              m_rtk_wdog_activation.reset();
            }
          }
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

          updateRtkTimers();



          // Just check if using rtk or not
          if (usingRtk())
          {

            // First fill most field.
            m_estate = *m_extnav.state.get();

            // Overwrite with RTK relevant fields.
            m_estate.lat = m_rtk.base_lat;
            m_estate.lon = m_rtk.base_lon;
            m_estate.height = m_rtk.base_height;

            m_estate.x = m_rtk.n;
            m_estate.y = m_rtk.e;
            m_estate.z = m_rtk.d;

            if (m_rtk.validity & IMC::GpsFixRtk::RFV_VALID_VEL)
            {
              m_estate.vx = m_rtk.v_n;
              m_estate.vy = m_rtk.v_e;
              m_estate.vz = m_rtk.v_d;
            }

            sendStateAndSource();

            spew("Sending state using RTK. ");
          }
          else
          {
            spew("Got RTKFix message, but not using it.  ");
          }
        }

        void
        sendStateAndSource(void)
        {
          dispatch(m_estate);
          dispatch(m_navsources);
        }


        void
        checkRtkTimersUpdateAvailable(void)
        {
          bool was_rtk_available = m_rtk_available;

          if ( was_rtk_available && m_rtk_wdog_keep_available.overflow())
          {
            m_rtk_available = false;
            debug("GPS RTK Unavailable: Timeout. ");
          }

          if ( was_rtk_available && m_rtk_wdog_deactivation.overflow())
          {
            m_rtk_available = false;
            debug("GPS RTK Unavailable: To long time in lower fix type. ");
          }

          if (!was_rtk_available && m_rtk_wdog_activation.overflow() && !m_rtk_wdog_keep_available.overflow())
          {
            m_rtk_available = true;
            inf("GPS RTK Available. ");
          }
        }

        void
        enableRtk(void)
        {
          m_navsources.mask |= NS_GNSS_RTK;
          m_navsources.mask &= ~(NS_EXTERNAL_FULLSTATE | NS_EXTERNAL_POSREF);
        }

        void
        disableRtk(void)
        {
          m_navsources.mask &= ~NS_GNSS_RTK;
          m_navsources.mask |= (NS_EXTERNAL_FULLSTATE | NS_EXTERNAL_POSREF);
        }

        //! Main loop.
        void
        onMain(void)
        {
          while (!stopping())
          {
            waitForMessages(1.0);

            checkRtkTimersUpdateAvailable();



            // Check if there is a change in rtk usage
            bool didChangeUsage = false;

            // If not available, and are using, disable. (don't care m_args.use_rtk)
            if (!m_rtk_available && usingRtk())
            {
              disableRtk();
              didChangeUsage = true;
              inf("Disable RTK. No longer available.");
            }

            // If should use, is available and not currently using
            else if (m_args.use_rtk && m_rtk_available && !usingRtk())
            {
              enableRtk();
              didChangeUsage = true;
              inf("Enabled use of RTK.");
            }

            // If not supposed to use, and are currently using, disable. availability don't care.
            else if (!m_args.use_rtk && usingRtk())
            {
              disableRtk();
              didChangeUsage = true;
              inf("Disable RTK. Disabled by parameter. ");
            }

            // Passthrough scenarios: use           &&  available &&  using
            //                        (use || !use) && !available && !using
            //
            // Three states => 8 total states. All covered.

            // Check if we have a switch to report
            if (didChangeUsage)
            {

              dispatch(m_navsources);

              // Reset all timers
              m_rtk_wdog_activation.reset();
              m_rtk_wdog_deactivation.reset();
              m_rtk_wdog_keep_available.reset();
            }


          }
        }
      };
    }
  }
}

DUNE_TASK
