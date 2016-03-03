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
// Author: RecepCetin                                                       *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>


namespace Simulators
{
  namespace SimulatedToEstimated
  {
    using DUNE_NAMESPACES;

    struct Arguments
    {
      double frequency;
    };

    struct Task: public DUNE::Tasks::Task
    {
      //! Task Arguments
      Arguments m_args;
      //! Last dispatched estimated state
      IMC::EstimatedState m_estate;
      //! Time of last dispatch
      double m_time_last_dispatch;



      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx)
      {

        param("Dispatch Frequency", m_args.frequency)
        .defaultValue("25");


        bind<IMC::SimulatedState>(this);
        m_time_last_dispatch = Clock::get();
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
      void
      consume(const IMC::SimulatedState* simState)
      {
        //debug("Consumed SimulatedState \n");
        double dt = Clock::get() - m_time_last_dispatch;

        if (dt < 1/m_args.frequency)
          return;

        m_estate.lat = simState->lat;
        m_estate.lon = simState->lon;
        m_estate.height = simState->height;
        m_estate.alt    = - simState->z;

        m_estate.x = simState->x;
        m_estate.y = simState->y;
        m_estate.z = simState->z;

        m_estate.u = simState->u;
        m_estate.v = simState->v;
        m_estate.w = simState->w;

        BodyFixedFrame::toInertialFrame(m_estate.phi, m_estate.theta, m_estate.psi,
                                       m_estate.u, m_estate.v, m_estate.w,
                                       &m_estate.vx, &m_estate.vy, &m_estate.vz);

        m_estate.theta = simState->theta;
        m_estate.phi = simState->phi;
        m_estate.psi = simState->psi;

        m_estate.p  = simState->p;
        m_estate.q  = simState->q;
        m_estate.r  = simState->r;


        Matrix groundSpeed = Matrix(3,1, 0.0);
        groundSpeed(0) = m_estate.vx;
        groundSpeed(1) = m_estate.vy;
        groundSpeed(2) = m_estate.vz;

        Matrix windSpeed = Matrix(3,1, 0.0);
        windSpeed(0) = simState->svx;
        windSpeed(1) = simState->svy;
        windSpeed(2) = simState->svz;

        Matrix airSpeed = groundSpeed - windSpeed;

        IMC::IndicatedSpeed ias;
        IMC::TrueSpeed gs;

        ias.value = airSpeed.norm_2();
        gs.value  = groundSpeed.norm_2();


        dispatch(m_estate);
        dispatch(ias);
        dispatch(gs);

        debug("Dispatched Estimated state.");

      }


    };
  }
}


DUNE_TASK
