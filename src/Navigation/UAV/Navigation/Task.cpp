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
        UsingExternal=1,
        RtkReady=2,
        UsingRtk=3,
        UsingShortLossComp=4
      } NavState;

      static std::string getNavStateString(NavState state) {
        switch(state)
        {
          case Init:
            return "Init";
          case UsingExternal:
            return "UsingExernal";
          case RtkReady:
            return "RtkReady";
          case UsingRtk:
            return "UsingRtk";
          case UsingShortLossComp:
            return "UsingShortLossComp";
          default:
            return "Invalid State. ";
        }
      }

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

          // Update curColumn
          ++curColumn;
          curColumn = (curColumn) % data.columns();
        };

      private:
        int curColumn;
        Matrix data;
      };

      using DUNE_NAMESPACES;

      struct Arguments {
        bool use_rtk;
        float rtk_timeout_connection;
        float rtk_timeout_lower_level;
        float rtk_min_fix_time;
        std::string rtk_fix_level_activate;
        std::string rtk_fix_level_deactivate;
        double base_alt;
        double antenna_height;
        bool shortRtkLoss_enable;
        float shortRtkLoss_activate_timeout;
        float shortRtkLoss_max_time;
        int shortRtkLoss_n_samples;
        float shortRtkLoss_message_frequency_timer;

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
        //! State timer
        Time::Counter<float> m_state_time;
        //! Short loss fix message timer
        Time::Counter<float> m_shortRtkLoss_wdog_message_frequency_timer;
        //! Fix level to activate
        IMC::GpsFixRtk::TypeEnum m_rtk_fix_level_activate;
        //! Fix level to deactivate
        IMC::GpsFixRtk::TypeEnum m_rtk_fix_level_deactivate;
        //! Currently used nav sources
        IMC::NavSources m_navsources;
        //! Moving average class that contain N samples of the average difference between rtk and external
        MovingAverage m_compensator;
        //! Current State
        NavState m_current_state;


        //! Constructor.
        //! @param[in] name task name.
        //! @param[in] ctx context.
        Task(const std::string& name, Tasks::Context& ctx):
          DUNE::Tasks::Task(name, ctx),
          m_rtk_fix_level_activate(IMC::GpsFixRtk::RTK_FIXED),
          m_rtk_fix_level_deactivate(IMC::GpsFixRtk::RTK_FIXED),
          m_current_state(Init)
        {

          param("Use RTK If Available", m_args.use_rtk)
          .defaultValue("false")
          .visibility(Parameter::VISIBILITY_USER);

          param("RTK - Timeout", m_args.rtk_timeout_connection)
          .defaultValue("1.0");

          param("RTK - Timeout Lower Level", m_args.rtk_timeout_lower_level)
          .defaultValue("1.0");

          param("RTK - Time needed on fix level to activate", m_args.rtk_min_fix_time)
          .defaultValue("2.0");

          param("RTK - Minimum Fix Level to Activate", m_args.rtk_fix_level_activate)
          .values("Fix,Float")
          .visibility(Parameter::VISIBILITY_USER)
          .defaultValue("Fix");

          param("RTK - Minimum Fix Type to Deactivate", m_args.rtk_fix_level_deactivate)
          .values("Fix,Float")
          .visibility(Parameter::VISIBILITY_USER)
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

          param("Short RtkFix Loss Compensator - Time activation",m_args.shortRtkLoss_activate_timeout)
          .defaultValue("0.2")
          .minimumValue("0.0");

          param("Short RtkFix Loss Compensator - Time deactivation",m_args.shortRtkLoss_max_time)
          .defaultValue("2.0")
          .minimumValue("0.0");

          param("Short RtkFix Loss Compensator - N samples to average",m_args.shortRtkLoss_n_samples)
          .defaultValue("10")
          .minimumValue("1");

          param("Short RtkFix Loss Compensator - Message frequency",m_args.shortRtkLoss_message_frequency_timer)
          .defaultValue("0.1")
          .minimumValue("0.0")
          .description("The message output frequency when in short rtk loss compensator state")
          .visibility(Parameter::VISIBILITY_USER);

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
          m_rtk_wdog_comm_timeout.setTop(m_args.rtk_timeout_connection);
          m_compensator.setDataMatrix(m_args.shortRtkLoss_n_samples,3);
        }

        //! Release resources.
        void
        onResourceRelease(void)
        {
        }

        void
        consume(const IMC::ExternalNavData* navdata)
        {
          m_extnav = *navdata;
          updateState(ExternalNav);
        }

        void
        addOffset()
        {
          Matrix compensator = m_compensator.getAverage();
          m_estate.x = m_estate.x + compensator(0,0);
          m_estate.y = m_estate.y + compensator(1,0);
          m_estate.z = m_estate.z + compensator(2,0);
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
          m_rtk_wdog_comm_timeout.reset();
          updateCompensator();

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
        updateCompensator()
        {
          if (m_rtk.type<m_rtk_fix_level_deactivate)
            return;

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
          m_compensator.newSample(newSample);
          // For debuging purpose
          Matrix average = m_compensator.getAverage();
          spew("Difference: n = %f e = %f d = %f",average(0,0),average(1,0),average(2,0));
        }

        void
        sendStateAndSource(void)
        {
          dispatch(m_estate);
          dispatch(m_navsources);
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
                setState(UsingExternal);
              }
              break;
            case UsingExternal:
              switch (callSource)
              {
                case ExternalNav:
                  m_estate = *m_extnav.state.get();
                  sendStateAndSource();
                  break;
                case Rtk:
                  if (m_rtk.type < m_rtk_fix_level_activate)
                  {
                    m_state_time.reset();
                  }
                  if (m_state_time.overflow() && !m_rtk_wdog_comm_timeout.overflow())
                  {
                    setState(RtkReady);
                  }
                  break;
                case Main:
                  if (m_rtk_wdog_comm_timeout.overflow())
                  {
                    //! Prevent the timer to build up when there are no connection
                    m_state_time.reset();
                  }
                  break;
              }
              break;
            case RtkReady:
              switch (callSource)
              {
                case ExternalNav:
                  m_estate = *m_extnav.state.get();
                  sendStateAndSource();
                  break;
                case Rtk:
                  if (m_rtk.type>=m_rtk_fix_level_deactivate)
                  {
                    m_state_time.reset();
                  }
                  if (m_state_time.overflow() || m_rtk.type==IMC::GpsFixRtk::RTK_NONE)
                  {
                    setState(UsingExternal);
                  }
                  else if (m_args.use_rtk)
                  {
                    setState(UsingRtk);
                  }
                  break;
                case Main:
                  if (m_rtk_wdog_comm_timeout.overflow())
                  {
                    setState(UsingExternal);
                  }
                  break;
              }
              break;
            case UsingRtk:
              switch (callSource)
              {
                case ExternalNav:
                  break;
                case Rtk:
                  if (m_rtk.type < m_rtk_fix_level_deactivate)
                  {
                    if (m_args.shortRtkLoss_enable)
                    {
                      setState(UsingShortLossComp);
                    }
                    else
                    {
                      setState(UsingExternal);
                    }
                  }
                  else
                  {
                    m_state_time.reset();
                    useRtk();
                    sendStateAndSource();
                  }
                  break;
                case Main:
                  if (m_state_time.overflow())
                  {
                    if (m_args.shortRtkLoss_enable)
                    {
                      setState(UsingShortLossComp);
                    }
                    else
                    {
                      setState(UsingExternal);
                    }
                  }
                  else if (!m_args.use_rtk)
                  {
                    setState(RtkReady);
                  }
                  break;
              }
              break;
            case UsingShortLossComp:
              switch (callSource)
              {
                case ExternalNav:
                  break;
                case Rtk:
                  if (m_rtk.type>=m_rtk_fix_level_deactivate)
                  {
                    setState(UsingRtk);
                  }
                  break;
                case Main:
                  if (m_state_time.overflow())
                  {
                    setState(UsingExternal);
                  }
                  if (m_shortRtkLoss_wdog_message_frequency_timer.overflow())
                  {
                    m_shortRtkLoss_wdog_message_frequency_timer.reset();
                    m_estate = *m_extnav.state.get();
                    addOffset();
                    sendStateAndSource();
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

          inf("Switching from %s to %s. ",
              getNavStateString(currState).c_str(), getNavStateString(nextState).c_str());
          switch (nextState)
          {
            case Init:
              // Never happens
              break;
            case UsingExternal:
              disableRtk();
              m_navsources.available_mask &= ~NS_GNSS_RTK;
              m_state_time.setTop(m_args.rtk_min_fix_time);
              m_estate = *m_extnav.state.get();
              sendStateAndSource();
              break;
            case RtkReady:
              disableRtk();
              m_state_time.setTop(m_args.rtk_timeout_lower_level);
              m_navsources.available_mask |= NS_GNSS_RTK;
              dispatch(m_navsources);
              break;
            case UsingRtk:
              enableRtk();
              useRtk();
              sendStateAndSource();
              if (m_args.shortRtkLoss_enable)
              {
                m_state_time.setTop(m_args.shortRtkLoss_activate_timeout);
              }
              else
              {
                m_state_time.setTop(m_args.rtk_timeout_lower_level);
              }

              break;
            case UsingShortLossComp:
              m_estate = *m_extnav.state.get();
              addOffset();
              sendStateAndSource();
              m_shortRtkLoss_wdog_message_frequency_timer.setTop(m_args.shortRtkLoss_message_frequency_timer);
              m_state_time.setTop(m_args.shortRtkLoss_max_time);
              break;
          }
          m_current_state = nextState;
        }
        //! Main loop.
        void
        onMain(void)
        {
          while (!stopping())
          {
            waitForMessages(0.1);

            updateState(Main);
          }
        }
      };
    }
  }
}

DUNE_TASK
