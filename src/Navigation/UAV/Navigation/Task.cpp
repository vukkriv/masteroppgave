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
      typedef enum NavState
      {
        Init=0,
        UseExternal=1,
        RtkReady=2,
        UseRtk=3,
        UseShortLossComp=4
      } NavState;

      typedef enum CallSource
      {
        ExternalNav=0,
        Rtk=1,
        Main=2
      } CallSource;

      class MovingAverage
      {
      public:
        MovingAverage():
            curColumn(0),
            data(Matrix(1,1,0.0))
          {
          // Intentionally empty
          }
        void
        setDataMatrix(int nSamples,int nRows)
        {
          if (nSamples <=0)
          {
            nSamples = 1;
          }
          if (nRows <=0)
          {
            nRows = 3;
          }

          data.resize(nRows,nSamples);
        }
        int
        getRow()
        {
          return data.rows();
        }
        int
        getCol()
        {
          return data.columns();
        }
        int
        getCurCol()
        {
          return curColumn;
        }

        Matrix getAverage()
        {
          // calc average
          Matrix sum = Matrix(data.rows(), 1, 0.0);
          for (int j = 0; j<data.columns(); ++j)
          {
            sum(0,0) += data(0,j);
            sum(1,0) += data(1,j);
            sum(2,0) += data(2,j);
          }
          return sum/data.columns();
        }
        void newSample(Matrix sample)
        {
          // Insert new data at curColumn,
          data(0,curColumn) = sample(0,0);
          data(1,curColumn) = sample(1,0);
          data(2,curColumn) = sample(2,0);

          // Update curColun
          curColumn = (++curColumn) % data.columns();
        };

      private:
        int curColumn;
        Matrix data;

      };



      using DUNE_NAMESPACES;

      struct Arguments {
        bool use_rtk;
        float rtk_tout;
        float rtk_tout_lowerlevel;
        float rtk_min_fix_time;
        std::string rtk_fix_level_activate;
        std::string rtk_fix_level_deactivate;
        double base_alt;
        double antenna_height;
        bool shortRtkLoss_enable;
        float shortRtkLoss__activate;
        float shortRtkLoss__deactivate;
        int shortRtkLoss_n_samples;

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
        Time::Counter<float> m_rtk_wdog_comm_timeout;
        //! Input watchdog for lower fix level
        Time::Counter<float> m_rtk_wdog_deactivation;
        //! Timer for minimum time needed at activation fix level
        Time::Counter<float> m_rtk_wdog_activation;
        //! Timer for deactivation of short rtk loss compensator
        Time::Counter<float> m_shortRtkLoss_wdog_deactivation;
        //! Timer for activation of short rtk loss compensator
        Time::Counter<float> m_shortRtkLoss_wdog_activation;
        //! Fix level to activate
        IMC::GpsFixRtk::TypeEnum m_rtk_fix_level_activate;
        //! Fix level to deactivate
        IMC::GpsFixRtk::TypeEnum m_rtk_fix_level_deactivate;
        //! Currently used nav sources
        IMC::NavSources m_navsources;
        //! Is RTK available
        bool m_rtk_available;
        //! Position offset in use
        bool m_shortRtkLoss_in_use;
        //! Average between N rtk messages
        MovingAverage m_rtk_average;
        //! Current State
        NavState m_current_state;


        //! Constructor.
        //! @param[in] name task name.
        //! @param[in] ctx context.
        Task(const std::string& name, Tasks::Context& ctx):
          DUNE::Tasks::Task(name, ctx),
          m_rtk_fix_level_activate(IMC::GpsFixRtk::RTK_FIXED),
          m_rtk_fix_level_deactivate(IMC::GpsFixRtk::RTK_FIXED),
          m_rtk_available(false),
          m_shortRtkLoss_in_use(false),
          m_current_state(Init)
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
          .visibility(Parameter::VISIBILITY_USER)
          .defaultValue("Fix");

          param("RTK - Minimum Fix Type to Deactivate", m_args.rtk_fix_level_deactivate)
          .values("Fix,Float")
          .defaultValue("Float");

          param("Base Altitude", m_args.base_alt)
          .minimumValue("0.0")
          .defaultValue("1.6")
          .units(Units::Meter)
          .visibility(Parameter::VISIBILITY_USER)
          .description("Altitude of the base antenna for RTK. Is used to calculate the altitude of the vehicle. ");

          param("Antenna Height", m_args.antenna_height)
          .defaultValue("0.0")
          .minimumValue("-1.0")
          .maximumValue("1.0")
          .units(Units::Meter)
          .visibility(Parameter::VISIBILITY_USER)
          .description("If > 0, apply correction from attitude");

          param("Short RtkFix Loss Compensator - Enable",m_args.shortRtkLoss_enable)
          .defaultValue("false")
          .visibility(Parameter::VISIBILITY_USER)
          .description("Use a position offset to minimize the position difference between the external and internal nav data");

          param("Short RtkFix Loss Compensator - Time activation",m_args.shortRtkLoss__activate)
                    .defaultValue("0.2")
                    .minimumValue("0.0");

          param("Short RtkFix Loss Compensator - Time deactivation",m_args.shortRtkLoss__deactivate)
          .defaultValue("2.0")
          .minimumValue("0.0");

          param("Short RtkFix Loss Compensator - N samples to average",m_args.shortRtkLoss_n_samples)
          .defaultValue("10")
          .minimumValue("1");


          // param:
          /*"Short RtkFix Loss Compensator - Enable"
          "Short RtkFix Loss Compensator - Time deactivation"
          "Short RtkFix Loss Compensator - N samples to average"

          m_args.shortRtkLoss_enable*/

          // Default, we use full external state
          m_navsources.mask = (NS_EXTERNAL_FULLSTATE | NS_EXTERNAL_AHRS | NS_EXTERNAL_POSREF);

          m_extnav.state.set(IMC::EstimatedState());

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
          m_rtk_wdog_comm_timeout.setTop(m_args.rtk_tout);
          m_rtk_wdog_deactivation.setTop(m_args.rtk_tout_lowerlevel);
          m_rtk_wdog_activation.setTop(m_args.rtk_min_fix_time);
          m_shortRtkLoss_wdog_deactivation.setTop(m_args.shortRtkLoss__deactivate);
          m_shortRtkLoss_wdog_activation.setTop(m_args.shortRtkLoss__activate);
          m_rtk_average.setDataMatrix(m_args.shortRtkLoss_n_samples,3);
        }

        //! Release resources.
        void
        onResourceRelease(void)
        {
        }

        /*bool usingRtk()
        {
          return (m_navsources.mask & IMC::NS_GNSS_RTK);
        }*/

        void
        consume(const IMC::ExternalNavData* navdata)
        {
          m_extnav = *navdata;
          updateState(ExternalNav);
        }

        void
        addOffset()
        {
          Matrix average = m_rtk_average.getAverage();
          m_estate.x = m_estate.x + average(0,0);
          m_estate.y = m_estate.y + average(1,0);
          m_estate.z = m_estate.z + average(2,0);
        }

        /*void
        updateShortRtkLossTimer(void)
        {
          if (m_args.shortRtkLoss_enable)
            m_shortRtkLoss_wdog_activation.reset();
        }*/

        /*void
        updateRtkTimers(void)
        {

          if (m_rtk_wdog_comm_timeout.overflow())
            m_rtk_wdog_activation.reset();

          if (m_rtk.type >= m_rtk_fix_level_deactivate)
            m_rtk_wdog_comm_timeout.reset();


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
        }*/

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

          if( getDebugLevel() == DEBUG_LEVEL_SPEW)
            rtkfix->toText(std::cerr);

          updateState(Rtk);
        }

        void
        addAntennaOffset()
        {
          // Apply antenna offset
          if( m_args.antenna_height > 0.03 || m_args.antenna_height < -0.03 )
          {
            float ox, oy, oz;
            BodyFixedFrame::toInertialFrame(m_estate.phi, m_estate.theta, m_estate.psi,
                                            0.0, 0.0, -m_args.antenna_height,
                                            &ox, &oy, &oz);

            m_estate.x -= ox;
            m_estate.y -= oy;
            m_estate.z -= oz;

            spew("Added antenna offset (ned): %.3f, %.3f, %.3f", ox, oy, oz);
          }
        }

        void
        useRtk()
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

          addAntennaOffset();

          // Calculate altitude
          m_estate.alt = m_args.base_alt - m_rtk.d;

          if (m_rtk.validity & IMC::GpsFixRtk::RFV_VALID_VEL)
          {
            m_estate.vx = m_rtk.v_n;
            m_estate.vy = m_rtk.v_e;
            m_estate.vz = m_rtk.v_d;
          }
        }

        void
        updateDifference()
        {
          double lat = m_extnav.state.get()->lat;
          double lon = m_extnav.state.get()->lon;
          double height = m_extnav.state.get()->height;
          double n;
          double e;
          double d;
          Coordinates::WGS84::displace(m_extnav.state.get()->x,m_extnav.state.get()->y,m_extnav.state.get()->z,&lat,&lon,&height);
          Coordinates::WGS84::displacement(m_rtk.base_lat,m_rtk.base_lon,m_rtk.base_height,lat,lon,height,&n,&e,&d);
          Matrix newSample = Matrix(3,1,0.0);
          // Calculating the difference
          newSample(0,0) = m_rtk.n - n;
          newSample(1,0) = m_rtk.e - e;
          newSample(2,0) = m_rtk.d - d;
          // Apply antenna offset
          if( m_args.antenna_height > 0.03 || m_args.antenna_height < -0.03 )
          {
            float ox, oy, oz;
            BodyFixedFrame::toInertialFrame(m_estate.phi, m_estate.theta, m_estate.psi,
                                            0.0, 0.0, -m_args.antenna_height,
                                            &ox, &oy, &oz);

            newSample(0,0) -= ox;
            newSample(1,0) -= oy;
            newSample(2,0) -= oz;

            spew("Added antenna offset (ned): %.3f, %.3f, %.3f", ox, oy, oz);
          }
          m_rtk_average.newSample(newSample);
          // For debuging purpose
          Matrix average = m_rtk_average.getAverage();
          inf("Difference: n = %f e = %f d = %f",average(0,0),average(1,0),average(2,0));
        }

        void
        sendStateAndSource(void)
        {
          dispatch(m_estate);
          dispatch(m_navsources);
        }

        /*void
        checkShortRtkLossTimers()
        {
          bool was_in_use = m_shortRtkLoss_in_use;
          if (was_in_use && m_shortRtkLoss_wdog_deactivation.overflow())
          {
            m_shortRtkLoss_in_use = false;
            inf("Usage of short rtk fix loss compensator timeout.");
          }
          if (!was_in_use && m_shortRtkLoss_wdog_activation.overflow())
          {
            m_shortRtkLoss_in_use = true;
            inf("Activating short rtk fix loss compensator timeout.");
          }
          if(was_in_use && !m_shortRtkLoss_wdog_activation.overflow() && ! m_shortRtkLoss_wdog_deactivation.overflow())
          {
            m_shortRtkLoss_in_use = false;
            inf("Usage of short rtk fix loss compensator disables due to new rtk message.");
          }
        }
        void*/
        /*checkRtkTimersUpdateAvailable(void)
        {
          bool was_rtk_available = m_rtk_available;

          if ( was_rtk_available && m_rtk_wdog_comm_timeout.overflow())
          {
            m_rtk_available = false;
            debug("GPS RTK Unavailable: Timeout. ");
          }

          if ( was_rtk_available && m_rtk_wdog_deactivation.overflow())
          {
            m_rtk_available = false;
            debug("GPS RTK Unavailable: To long time in lower fix type. ");
          }

          if (!was_rtk_available && m_rtk_wdog_activation.overflow() && !m_rtk_wdog_comm_timeout.overflow())
          {
            m_rtk_available = true;
            inf("GPS RTK Available. ");
          }
        }*/

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

        //! Update the state machine
        void
        updateState(CallSource callSource)
        {
          NavState currState = m_current_state;
          switch (currState)
          {
            case Init:
              if (callSource==ExternalNav)
              {
                setState(UseExternal);
              }
              break;
            case UseExternal:
              switch (callSource)
              {
                case ExternalNav:
                  m_estate = *m_extnav.state.get();
                  sendStateAndSource();
                  break;
                case Rtk:
                  if (m_rtk.type<m_rtk_fix_level_activate)
                  {
                    // Reset timer
                    m_rtk_wdog_activation.reset();
                  }
                  m_rtk_wdog_comm_timeout.reset();
                  break;
                case Main:
                  if (m_rtk_wdog_activation.overflow() && !m_rtk_wdog_comm_timeout.overflow())
                  {
                    setState(RtkReady);
                  }
                  break;
              }
              // something
              break;
            case RtkReady:
              switch (callSource)
              {
                case ExternalNav:
                  m_estate = *m_extnav.state.get();
                  sendStateAndSource();
                  break;
                case Rtk:
                  if (m_rtk.type==IMC::GpsFixRtk::RTK_NONE)
                  {
                    setState(UseExternal);
                  }
                  else if (m_rtk.type>=m_rtk_fix_level_deactivate)
                  {
                    m_rtk_wdog_comm_timeout.reset();
                    updateDifference();
                  }
                  break;
                case Main:
                  if (m_rtk_wdog_comm_timeout.overflow())
                  {
                    setState(UseExternal);
                  }
                  else if (m_args.use_rtk)
                  {
                    setState(UseRtk);
                  }
                  break;
              }
              break;
            case UseRtk:
              switch (callSource)
              {
                case ExternalNav:
                  break;
                case Rtk:
                  if (m_rtk.type < m_rtk_fix_level_deactivate)
                  {
                    setState(UseShortLossComp);
                  }
                  else
                  {
                    m_shortRtkLoss_wdog_activation.reset();
                    updateDifference();
                    useRtk();
                    sendStateAndSource();
                  }
                  break;
                case Main:
                  if (m_shortRtkLoss_wdog_activation.overflow())
                  {
                    setState(UseShortLossComp);
                  }
                  else if (!m_args.use_rtk)
                  {
                    setState(RtkReady);
                  }
                  break;
              }
              break;
            case UseShortLossComp:
              switch (callSource)
              {
                case ExternalNav:
                  m_estate = *m_extnav.state.get();
                  addOffset();
                  sendStateAndSource();
                  break;
                case Rtk:
                  if (m_rtk.type>=m_rtk_fix_level_deactivate)
                  {
                    setState(UseRtk);
                  }
                  break;
                case Main:
                  if (m_shortRtkLoss_wdog_deactivation.overflow())
                  {
                    setState(UseExternal);
                  }
                  break;
              }
              break;
          }

        }

        //! Transition into next state
        void
        setState(NavState nextState)
        {
          NavState currState = m_current_state;
          inf("Current state %d",currState);
          inf("Next state %d",nextState);
          switch (currState)
          {
            case Init:
              // Next state UseExternal
              m_rtk_wdog_activation.reset();
              m_rtk_wdog_comm_timeout.reset();
              m_estate = *m_extnav.state.get();
              sendStateAndSource();
              break;
            case UseExternal:
              // Next state RtkReady
              m_rtk_wdog_comm_timeout.reset();
              m_navsources.available_mask |= NS_GNSS_RTK;
              dispatch(m_navsources);
              break;
            case RtkReady:
              if(nextState==UseRtk)
              {
                enableRtk();
                useRtk();
                sendStateAndSource();
                m_shortRtkLoss_wdog_activation.reset();
              }
              else
              {
                // Next state UseExternal
                m_rtk_wdog_activation.reset();
                m_rtk_wdog_comm_timeout.reset();
                m_navsources.available_mask &= ~NS_GNSS_RTK;
                dispatch(m_navsources);
              }

              break;
            case UseRtk:
              if (nextState == UseShortLossComp)
              {
                m_estate = *m_extnav.state.get();
                addOffset();
                sendStateAndSource();
                m_shortRtkLoss_wdog_deactivation.reset();
              }
              else if(nextState == RtkReady)
              {
                disableRtk();
                m_estate = *m_extnav.state.get();
                sendStateAndSource();
                m_rtk_wdog_comm_timeout.reset();
              }
              else
              {
                // Next state UseExternal, however not in use
                disableRtk();
                m_navsources.available_mask &= ~NS_GNSS_RTK;
                m_estate = *m_extnav.state.get();
                sendStateAndSource();
              }
              break;
            case UseShortLossComp:
              if (nextState == UseRtk)
              {
                useRtk();
                sendStateAndSource();
                m_shortRtkLoss_wdog_activation.reset();
              }
              else
              {
                // Next state UseExternal
                disableRtk();
                m_rtk_wdog_activation.reset();
                m_rtk_wdog_comm_timeout.reset();
                m_navsources.available_mask &= ~NS_GNSS_RTK;
                m_estate = *m_extnav.state.get();
                sendStateAndSource();
              }

          }
          m_current_state = nextState;
        }
        //! Main loop.
        void
        onMain(void)
        {
          while (!stopping())
          {
            waitForMessages(1.0);

            updateState(Main);
            /*bool was_rtk_available = m_rtk_available;

            checkRtkTimersUpdateAvailable();

            // Check if there is a change in rtk availability
            bool didChangeAvailability = false;

            if (was_rtk_available != m_rtk_available && m_rtk_available)
            {
              m_navsources.available_mask |= NS_GNSS_RTK;
              didChangeAvailability = true;
            }
            else if (was_rtk_available != m_rtk_available && ~m_rtk_available)
            {
              m_navsources.available_mask &= ~NS_GNSS_RTK;
              didChangeAvailability = true;
            }

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

            if(m_args.shortRtkLoss_enable && usingRtk())
            {
              checkShortRtkLossTimers();
              if (!m_shortRtkLoss_in_use)
              {
                m_shortRtkLoss_wdog_deactivation.reset();
              }
            }
            else
            {
              m_shortRtkLoss_in_use = false;
              m_shortRtkLoss_wdog_activation.reset();
              m_shortRtkLoss_wdog_deactivation.reset();
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
              m_rtk_wdog_comm_timeout.reset();
            }
            else if (didChangeAvailability && ~didChangeUsage)
            {
              dispatch(m_navsources);
            }
            */
          }
        }
      };
    }
  }
}

DUNE_TASK
