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
// Author: Daniel Røyland                                                  *
//***************************************************************************
// Extremum seeking optimizer. 
// Inputs: RSSI and heading
// Output: z_hat

// DUNE headers.
#include <DUNE/DUNE.hpp>
#include "math.h"

namespace Control
{
  namespace UAV
  {
    namespace RSSIExtremum
    {
      using DUNE_NAMESPACES;

      #define PI 3.141592653589793

      struct Arguments
      {
        // Convergence factor:
        double kappa;
        // Convergence speed:
        double eta;
        //! inv. prop. to raduis of circling motion:
        double omega;
        //! Sampling time:
        double T;
      };

      struct Task: public DUNE::Tasks::Task
      {
        //! Task arguments
        Arguments m_args;


        //! Variables:
        Matrix m_D;
        Matrix m_R;
        Matrix m_zhat;
        Matrix m_est_gradient;
        double m_psi_k;
        Matrix m_psi_next;
        double m_psi_wrap;

        Matrix m_one;
        Matrix m_Dzhat;
        Matrix m_Rt;
        double m_norm_dzhat;

        IMC::NavigationData zhat_receive;




        //! Constructor.
        //! @param[in] name task name.
        //! @param[in] ctx context.
        Task(const std::string& name, Tasks::Context& ctx):
          DUNE::Tasks::Task(name, ctx),
          m_D(2,3,0.0),
          m_R(2,1,0.0),
          m_zhat(3,1,0.0),
          m_est_gradient(2,1,0.0),
          m_psi_k(0.0),
          m_psi_next(1,1,0.0),
          m_psi_wrap(0),
          m_one(1,1,1),
          m_Dzhat(2,1,0.0),
          m_Rt(1,2,0.0),
          m_norm_dzhat(0.0)
        {
          //! Parameters:
          param("Kappa", m_args.kappa)
          .defaultValue("1.0")
          .description("Convergence factor");

          param("Eta", m_args.eta)
          .defaultValue("0.5")
          .description("Convergence speed");

          param("Omega", m_args.omega)
          .defaultValue("0.1")
          .description("Proportional to inv. radius of circles");

          param("T", m_args.T)
          .defaultValue("0.1")
          .description("Sampling time");


          //! Bind incoming IMC messages:
          //bind<IMC::Pressure>(this);
          bind<IMC::NavigationData>(this);

          //! Initiate matrices:
          initMatrices();

        }

        void
        initMatrices(void)
        {
          m_D(0,1) = 1;
          m_D(1,2) = 1;
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
        /*
        void
        consume(const IMC::Pressure* msg)
        {
          //inf("Pressure is %f", msg->value);
        }
         */
        void
        consume(const IMC::NavigationData* zhat_send){
          zhat_receive = *zhat_send;

          calculateNextHeading(zhat_receive);

        }

        void
        calculateNextHeading(IMC::NavigationData zhat_arg)
        {

          //inf("Running calcnextheading from consume zhat");
          // Read from IMC message:
          m_zhat(0) = zhat_arg.custom_x;
          m_zhat(1) = zhat_arg.custom_y;
          m_zhat(2) = zhat_arg.custom_z;


          //inf("zhat is: ");
          //std::cout << zhat;

          // Rotation matrix:
          m_R(0) = cos(m_psi_k + 0.5*m_args.omega*m_args.T);
          m_R(1) = sin(m_psi_k + 0.5*m_args.omega*m_args.T);

          m_Rt = transpose(m_R);

          // Help matrices:
          Matrix m_psi_m = Matrix(1,1,m_psi_k);
          m_Dzhat = m_D*m_zhat;
          m_norm_dzhat = m_Dzhat.norm_2();

          // Calculate next optimal heading:
          m_psi_next = m_psi_m + m_args.omega*m_args.T*(m_one-m_Rt*(m_Dzhat)*((m_args.kappa*m_args.eta)/(m_args.eta + m_args.kappa*m_norm_dzhat)));

          //! Wrap heading to the interval (-pi,pi]:
          m_psi_wrap = fmod(m_psi_next(0) - PI, 2*PI) - PI;

          // Dispatch desired heading:
          IMC::DesiredHeading psi_send;
          psi_send.value = m_psi_wrap;
          dispatch(psi_send);

          // Update psi_k:
          m_psi_k = m_psi_next(0);

          //return m_psi_next(0);

          /*
          //! (Scaled) estimated gradient:
          est_gradient(0) = zhat(1);
          est_gradient(1) = zhat(2);

           */
        }



        //! Main loop.
        void
        onMain(void)
        {


          //IMC::DesiredHeading psi_send;
          //psi_send.value = 0;

          while (!stopping())
          {
            //calculateNextHeading(zhat_receive);
            //dispatch(psi_send);
            //inf("Psi is: %f", psi_send.value);

            //Delay::wait(1.0);    // Wait doing nothing.

            waitForMessages(1.0);
          }
        }
      };
    }
  }
}

DUNE_TASK
