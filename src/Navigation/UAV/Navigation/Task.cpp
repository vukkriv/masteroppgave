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

// USER headers
#include <USER/DUNE.hpp>

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
            data(Matrix(1,1,0.0)),
            filledData(false)
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
            nRows = 1;
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
          if (filledData)
          {
            for (int j = 0; j<data.columns(); ++j)
            {
              for (int i = 0;i<data.rows();i++)
              {
                sum(i,0) += data(i,j);
              }
            }
            return sum/data.columns();
          }
          else
          {
            for (int j = 0; j<curColumn+1; ++j)
            {
              for (int i = 0;i<data.rows();i++)
                {
                  sum(i,0) += data(i,j);
                }
            }
            return sum/(curColumn+1);
          }

        }
        void newSample(Matrix sample)
        {
          // Insert new data at curColumn,
          for (int i=0;i<data.rows();i++)
          {
            data(i,curColumn) = sample(i,0);
          }
          // Update curColumn
          ++curColumn;
          if (curColumn%data.columns()==0 && !filledData)
            filledData = true;
          curColumn = (curColumn) % data.columns();
        };

      private:
        int curColumn;
        Matrix data;
        bool filledData;
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
        int shortRtkLoss_activate_frequency_multiplier;
        int shortRtkLoss_max_time_frequency_multiplier;
        int shortRtkLoss_n_samples;
        int rtkMessageFrequency_n_samples;

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
        //! Class that contain the average frequency of rtk messages
        MovingAverage m_rtkFrequency;
        //! Previous rtk timestamp
        double m_prevRtkTimestamp;
        //! Short rtk loss compensator frequency reference
        double m_shortRtkLossFrequencyRef;
        //! Current State
        NavState m_current_state;


        //! Constructor.
        //! @param[in] name task name.
        //! @param[in] ctx context.
        Task(const std::string& name, Tasks::Context& ctx):
          DUNE::Tasks::Task(name, ctx),
          m_rtk_fix_level_activate(IMC::GpsFixRtk::RTK_FIXED),
          m_rtk_fix_level_deactivate(IMC::GpsFixRtk::RTK_FIXED),
          m_prevRtkTimestamp(0.0),
          m_shortRtkLossFrequencyRef(0.0),
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

          param("RTK - N samples to average frequency", m_args.rtkMessageFrequency_n_samples)
          .minimumValue("0")
          .defaultValue("10");

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

          param("Short RtkFix Loss Compensator - Activation rtk frequency multiplier",m_args.shortRtkLoss_activate_frequency_multiplier)
          .defaultValue("2")
          .minimumValue("1");

          param("Short RtkFix Loss Compensator - Deactivation rtk frequency multiplier",m_args.shortRtkLoss_max_time_frequency_multiplier)
          .defaultValue("20")
          .minimumValue("1");

          param("Short RtkFix Loss Compensator - N samples to average",m_args.shortRtkLoss_n_samples)
          .defaultValue("10")
          .minimumValue("1");

          // Default, we use full external state
          m_navsources.mask = (NS_EXTERNAL_FULLSTATE | NS_EXTERNAL_AHRS | NS_EXTERNAL_POSREF);
          m_navsources.available_mask = (NS_EXTERNAL_FULLSTATE | NS_EXTERNAL_AHRS | NS_EXTERNAL_POSREF);


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
          m_rtkFrequency.setDataMatrix(m_args.rtkMessageFrequency_n_samples,1);
          inf("Colum rtk: %d row %d",m_rtkFrequency.getCol(),m_rtkFrequency.getRow());
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
        addShortLossCompensator()
        {
          Matrix compensator = m_compensator.getAverage();
          m_estate.x = m_estate.x + compensator(0,0);
          m_estate.y = m_estate.y + compensator(1,0);
          m_estate.z = m_estate.z + compensator(2,0);
          debug("Compensator: x= %f, y= %f, z= %f",compensator(0,0),compensator(1,0),compensator(2,0));
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

          //! Add new frequency sample
          Matrix timeDiff = Matrix(1,1,0.0);
          timeDiff(0,0) = m_rtk.getTimeStamp()-m_prevRtkTimestamp;

          if (timeDiff(0,0)!=0 && m_prevRtkTimestamp!=0)
          {
            m_rtkFrequency.newSample(timeDiff);
          }
          m_prevRtkTimestamp = m_rtk.getTimeStamp();


          Matrix rtkfrequency = m_rtkFrequency.getAverage();
          debug("The frequency of rtk is %f", rtkfrequency(0,0));

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
        fillStateRtk()
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

          double lat = m_rtk.base_lat;
          double lon = m_rtk.base_lon;
          double height = m_rtk.base_height;
          double n,e,d;

          // Fill llh coordinates of current RTK pos.
          Coordinates::WGS84_Accurate::displace(m_rtk.n,m_rtk.e,m_rtk.d,
                                      &lat,&lon,&height);

          // Find the ned position of the rtk solution in the external nav frame
          Coordinates::WGS84::displacement(m_extnav.state.get()->lat,m_extnav.state.get()->lon,m_extnav.state.get()->height,
                                          lat,lon,height,
                                          &n,&e,&d);
          // Fill sample matrix with the difference between rtk and external state
          Matrix newSample = Matrix(3,1,0.0);
          // Calculating the difference

          newSample(0,0) = n - m_extnav.state.get()->x;
          newSample(1,0) = e - m_extnav.state.get()->y;
          newSample(2,0) = d - m_extnav.state.get()->z;
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
          // Add new sample to the short loss compensator
          m_compensator.newSample(newSample);
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

        void
        setNavSourceAvailable(bool available)
        {
          if (available)
          {
            m_navsources.available_mask |= NS_GNSS_RTK;
          }
          else
          {
            m_navsources.available_mask &= ~NS_GNSS_RTK;
          }
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
                    fillStateRtk();
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
                  else if (m_shortRtkLoss_wdog_message_frequency_timer.overflow())
                  {
                    m_shortRtkLoss_wdog_message_frequency_timer.reset();
                    m_estate = *m_extnav.state.get();
                    addShortLossCompensator();
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
              setNavSourceAvailable(false);
              m_state_time.setTop(m_args.rtk_min_fix_time);
              m_estate = *m_extnav.state.get();
              sendStateAndSource();
              break;
            case RtkReady:
              disableRtk();
              m_state_time.setTop(m_args.rtk_timeout_lower_level);
              setNavSourceAvailable(true);
              dispatch(m_navsources);
              break;
            case UsingRtk:
              enableRtk();
              fillStateRtk();
              sendStateAndSource();
              if (m_args.shortRtkLoss_enable)
              {
                m_shortRtkLossFrequencyRef = m_rtkFrequency.getAverage()(0,0);
                debug("Short loss frequency ref %f",m_shortRtkLossFrequencyRef);
                m_state_time.setTop(m_args.shortRtkLoss_activate_frequency_multiplier*m_shortRtkLossFrequencyRef);
              }
              else
              {
                m_state_time.setTop(m_args.rtk_timeout_lower_level);
              }

              break;
            case UsingShortLossComp:
              m_estate = *m_extnav.state.get();
              addShortLossCompensator();
              sendStateAndSource();
              m_shortRtkLoss_wdog_message_frequency_timer.setTop(m_shortRtkLossFrequencyRef);
              m_state_time.setTop(m_args.shortRtkLoss_max_time_frequency_multiplier*m_shortRtkLossFrequencyRef);
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
