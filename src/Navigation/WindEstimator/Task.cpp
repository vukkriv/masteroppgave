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
// Author: Siri Holthe Mathisen                                             *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>
#include <mavlink/ardupilotmega/mavlink.h>
#include <time.h>

using namespace std;

namespace Navigation
{
  namespace WindEstimator
  {
    using DUNE_NAMESPACES;

    struct Arguments
    {
      string auto_pilot_type;
      int sample_window_size;
      double trustedlim;
      double q_multi;
    };

    struct Task: public DUNE::Tasks::Task
    {
      //! Type definition for Arduino packet handler.
      typedef void (Task::* PktHandler)(const mavlink_message_t* msg);
      typedef std::map<int, PktHandler> PktHandlerMap;
      PktHandlerMap m_mlh;

      Math::Matrix m_C, m_P, m_Q, m_R, m_w, m_G, m_R_bn, m_vn, m_vb, m_vr, m_d, m_S, m_K;
      double m_e, m_alpha, m_beta,  m_air_speed, m_dt, timer;
      bool hasEstate, measurementsIstrusted;
      int m_n_samples;

      IMC::EstimatedState m_estate;
      IMC::EstimatedStreamVelocity m_wind_estimated;
      IMC::EstimatedStreamVelocity m_wind_at_the_moment;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Arguments m_args;
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx)
      {
        param("Auto Pilot", m_args.auto_pilot_type)
        .defaultValue("ardupilot")
        .description("Tuning parameters are set depending on the autopilot");

        param("Sample Window Size", m_args.sample_window_size)
        .defaultValue("1000")
        .description("Sample window size");

        param("Trusted Limit", m_args.trustedlim)
        .defaultValue("0")
        .description("Trusted observability gramian limit");

        param("Q Size", m_args.q_multi)
        .defaultValue("1")
        .description("Q size multiplier");

        bind<EstimatedState>(this);
        bind<IndicatedSpeed>(this);
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        hasEstate = false;
        measurementsIstrusted = false;

        double adjp = 1, adjq = m_args.q_multi, adjr = 1;

        double p[] = {0.01*adjp, 0.01*adjp, 0.001*adjp, 1e-4*adjp};
        m_P = Matrix(p,4);
        double r[] = {1*adjr};
        m_R = Matrix(r, 1, 1);
        double q[] = {1e-3*adjq, 1e-3*adjq,  1e-4*adjq,  1e-8*adjq};
        m_Q = Matrix(q,4);
        double w[] = {0,0,0,1};
        m_w = Matrix(w,4,1);
        double g[] = {0,0,0,0};
        m_G = Matrix(g, 4);
        double d[] = {1, 0, 0};
        m_d = Matrix(d,1,3);

        m_e = 0;
        m_alpha = 0;
        m_beta = 0;
        m_dt = 0;
        m_air_speed = 0;

        timer = Clock().get();

        setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
      }

      void
      handleIas(void)
      {
        m_dt = Clock().get() - timer; //First might be large
        timer = Clock().get();

        if(m_n_samples<m_args.sample_window_size){
          m_n_samples++;
        }


        if(hasEstate){
          double r_bn[] = {m_estate.phi, m_estate.theta, m_estate.psi};
          double vb[] = {m_estate.u, m_estate.v, m_estate.w};
          double vn[] ={m_estate.vx, m_estate.vy, m_estate.vz};
          // Body to NED rotation matrix
          m_R_bn =  Matrix(r_bn,3, 1).toQuaternion().toDCM();
          m_vb =    Matrix(vb,  3, 1);
          m_vn =    Matrix(vn,  3, 1);

          double tas[] = {m_air_speed};
//          m_C = horseCat(m_d * transpose(m_R_bn), Matrix(tas,1,1));
          m_C = m_d * transpose(m_R_bn);
          m_C = m_C.horzCat(Matrix(tas,1,1));

          m_P = m_P + m_Q * pow(m_dt,2);
          m_S = m_C * m_P * transpose(m_C) + m_R;
          m_K = m_P * transpose(m_C) * inverse(m_S);
          m_P = (Matrix(4) - (m_K * m_C)) * m_P * transpose(Matrix(4) - m_K * m_C) + m_K * m_R * transpose(m_K);
          m_P = (m_P * transpose(m_P)) / 2;

          m_e = (m_d * m_vb - (m_C * m_w)).element(0,0);

          m_w = m_w + (m_K * m_e);

          m_vr = m_vb - m_R_bn * m_w.get(0, 2, 0, 0);

          //Find normalized observability gramian
          m_G = m_G + m_dt * transpose(m_C) * m_C / m_n_samples;

          //double temp[] = {m_air_speed,0,0};

//          // Measured wind velocity
//          m_wind_at_the_moment.x = (m_vn - (transpose(m_R_bn) * Matrix(temp,3,1))).element(0,0);
//          m_wind_at_the_moment.y = (m_vn - (transpose(m_R_bn) * Matrix(temp,3,1))).element(1,0);
//          m_wind_at_the_moment.z = (m_vn - (transpose(m_R_bn) * Matrix(temp,3,1))).element(2,0);

          m_wind_estimated.x = +m_w.element(0,0);
          m_wind_estimated.y = +m_w.element(1,0);
//          m_wind_estimated.y = -m_wind_estimated.y;
          m_wind_estimated.z = +m_w.element(2,0);
          if(isNaN(m_wind_estimated.x) || isNaN(m_wind_estimated.y) || isNaN(m_wind_estimated.z))
            inf("Problems with wind estimator: Output: %f %f %f", m_wind_estimated.x, m_wind_estimated.y, m_wind_estimated.z);
//          dispatch(m_wind_at_the_moment);
          dispatch(m_wind_estimated);

          if((m_G.detr() > m_args.trustedlim) and !measurementsIstrusted){
            measurementsIstrusted = true;
            war("Measurement trusted with a gramian of %f", m_G.detr());
          }
          else if(!(m_G.detr() > m_args.trustedlim) and measurementsIstrusted){
            measurementsIstrusted = false;
            war("Measurement not trusted with a gramian of %f", m_G.detr());
          }

          //Find AoA and SSA
          m_alpha = atan(m_vr.element(2,0) / m_vr.element(0,0)) * 180 / Math::c_pi;
          m_beta = asin(m_vr.element(1,0) / m_vr.norm_2()) * 180 / Math::c_pi;
        }
      }

      //! Reserve entity identifiers.
      void
      onEntityReservation(void)
      {
        m_wind_at_the_moment.setSourceEntity(reserveEntity("Current wind"));
      }

      //! Resolve entity names.
      void
      onEntityResolution(void)
      {
      }

      void
      consume(const IMC::EstimatedState* msg)
      {
        m_estate = *msg;
        hasEstate = true;
      }

      void
      consume(const IMC::IndicatedSpeed* msg)
      {
        m_air_speed = msg->value;
        handleIas();
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

DUNE_TASK
