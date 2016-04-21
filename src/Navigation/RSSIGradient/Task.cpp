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
// Extremum seeking observer. 
// Input: z_hat
// Output: heading 


// Math library
//#include <cmath>
#include "math.h"

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Navigation
{
  namespace RSSIGradient
  {
    using DUNE_NAMESPACES;

    struct Arguments
    {
      //! Forgetting factor:
      double lambda;
      //! inv. prop. to raduis of circling motion:
      double omega;
      //! Sampling time:
      double T;
    };

    struct Task: public DUNE::Tasks::Task
    {
      //! Task arguments:
      Arguments m_args;

      //! Variables:
      Matrix m_C;
      Matrix m_I;
      Matrix m_L;
      Matrix m_zhat;
      Matrix m_zhatm;
      Matrix m_P;
      Matrix m_Pm;
      Matrix m_A;
      Matrix m_R;

      //double m_psi;

      IMC::DesiredHeading dpsi_receive;
      IMC::RSSI rssi_receive;
      IMC::EstimatedState states_receive;

      //IMC::NavigationData zhat_send;

      //! Constructor.

      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_C(1,3,0.0),
        m_I(3,3,0.0),
        m_L(3,1,0.0),
        m_zhat(3,1,0.0),
        m_zhatm(3,1,0.0),
        m_P(3,3,0.0),
        m_Pm(3,3,0.0),
        //m_psi(0.0)
        m_A(3,3,0.0),
        m_R(2,1,0.0)
      {

        param("Lambda", m_args.lambda)
        .defaultValue("0.999")
        .description("Forgetting factor");

        param("Omega", m_args.omega)
        .defaultValue("0.1")
        .description("Proportional to inv. radius of circles");

        param("T", m_args.T)
        .defaultValue("0.1")
        .description("Sampling time");


        //! Bind incoming IMC messages:
        bind<IMC::DesiredHeading>(this);
        bind<IMC::EstimatedState>(this);
        bind<IMC::RSSI>(this);

        //! Initiate matrices:
        initMatrices();

      }


      void initMatrices(void){
        m_C(0) = 1;
        m_C(1) = 0;
        m_C(2) = 0;
        double ones[] = {1.0, 1.0, 1.0};
        m_I = Matrix(ones, 3);
        m_P = Matrix(ones,3);
        m_Pm = Matrix(ones,3);
        m_A = Matrix(ones, 3);
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
        inf("Starting: %s", resolveEntity(getEntityId()).c_str());
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
      consume(const IMC::EstimatedState* states)
      {
        states_receive = *states;
        //psi = states->psi;
        //inf("heading is: %f", states->psi);
      }

      void
      consume(const IMC::DesiredHeading* msg)
      {
        dpsi_receive = *msg;
      }


      void
      consume(const IMC::RSSI* rssi)
      {
        rssi_receive = *rssi;
        //inf("rssi is: %f", rssi->value);

        // When a new rssi measurement occurs, estimate and dispatch a new m_zhat:
        //estimateStatevector(rssi_receive,states_receive);
        estimateStatevector(rssi_receive, states_receive);
      }


      void
      estimateStatevector(IMC::RSSI rssi_arg, IMC::EstimatedState states_arg) //IMC::DesiredHeading psi_arg)//)
      {
        //inf("Running observer");

        // Inputs to the observer:
        double y_k = rssi_arg.value;
        double psi_est = states_arg.psi; // feedback of actual heading from estimatedstate
        //double psi_est = psi_arg.value; // feedback of desired heading from optimizer

        //inf("Pm is: ");
        //std::cout << Pm;

        // Update estimate:
        Matrix temp = Matrix(1,1,0.0);

        temp(0) = (1/(1-m_args.lambda)) + (1/m_args.lambda)*m_Pm(0,0);
        m_L = (1/m_args.lambda)*m_Pm*transpose(m_C)*inverse(temp);
        m_zhat = m_zhatm + m_L*(y_k - m_zhatm(0,0));
        //std::cout << zhat;

        IMC::NavigationData zhat_send;
        zhat_send.custom_x = m_zhat(0);
        zhat_send.custom_y = m_zhat(1);
        zhat_send.custom_z = m_zhat(2);
        dispatch(zhat_send);

        Matrix temp2 = m_I-(m_L*m_C);

        m_P = (1/m_args.lambda)*(temp2)*m_Pm*transpose(temp2) + ((1/(1-m_args.lambda))*m_L*transpose(m_L));
        //std::cout << m_P;

        m_R(0,0) = cos(psi_est);
        m_R(1,0) = sin(psi_est);

        // Observer prediction step:
        Matrix temp3 = m_args.omega*m_args.T*transpose(m_R);
        m_A(0,1) = temp3(0);
        m_A(0,2) = temp3(1);
        m_zhatm = m_A*m_zhat;
        m_Pm = m_A*m_P*transpose(m_A);

        //return m_zhat;
        // Initialize zhat and Pm?
        //return(zhat);

      }

      //! Main loop.
      void
      onMain(void)
      {


        //Matrix zhat_temp = Matrix(3,1,0.0);
        //zhat_temp(3,1,0.0);

        //IMC::Pressure msg;   // use temperature message from IMC
        //msg.value = 500;     // Initialize the temperature value.
        //inf("Inside main");
        //inf("Dummy: %f", dummy);

        /*
        IMC::NavigationData zhat_send;
        zhat_send.custom_x = 0;
        zhat_send.custom_y = 0;
        zhat_send.custom_z = 0;
         */

        while (!stopping())
        {
          //inf("Print this to console");
          //inf("First element of C: %f", C(0));
          //inf("I matrix element 22: %f", I(2,2));


          //estimateStatevector(rssi_receive, states_receive);
          /*
            zhat_send.custom_x = zhat_temp(0);
            zhat_send.custom_y = zhat_temp(1);
            zhat_send.custom_z = zhat_temp(2);
            dispatch(zhat_send);
           */

          /*

            //msg.value += 1;      // increment the value just to see the output
            //dispatch(msg);       // Dispatch the value to the message bus

           */
          //Delay::wait(1.0);    // Wait doing nothing.

          waitForMessages(1.0);
        }
      }
    };
  }
}

DUNE_TASK
