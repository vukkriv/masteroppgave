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
// Author: Kristoffer Gryte                                                 *
//***************************************************************************

// DUNE headers.
#include <DUNE/Config.hpp>
#include <USER/Math/StateSpace.hpp>
#include <USER/Smoothing/ReferenceModel.hpp>


namespace DUNE
{
  namespace Smoothing
  {
    ReferenceModel::ReferenceModel(Math::Matrix numerator, Math::Matrix denominator):
      m_state(Math::Matrix(denominator.columns()-1,1,0.0))
    {

      /* if(numerator.columns() > 1) */
      /*   throw std::out_of_range("ReferenceModel only supports numerator of order zero"); */
      m_statespace = Math::StateSpace(numerator,denominator);
      m_order = denominator.columns()-1;
    }

    ReferenceModel::ReferenceModel(double damp, double omega0, int order)
    {
      setDampFreq(damp,omega0,order,true);
    }

    void
    ReferenceModel::setDampFreq(double damp, double omega0, int order, bool reset_state)
    {
      Math::StateSpace oldSS = m_statespace;
      // save old output 
      Math::Matrix y_old = Math::Matrix(order,1,0.0);
      switch (order) {
        case 1:
        {
          double num[] = {omega0};
          double den[] = {1,omega0};
          m_statespace = Math::StateSpace(Math::Matrix(num,1,1),Math::Matrix(den,1,2));
          break;
        }
        case 2:
        {
          double num[] = {omega0*omega0};
          double den[] = {1,2*damp*omega0,omega0*omega0};
          m_statespace = Math::StateSpace(Math::Matrix(num,1,1),Math::Matrix(den,1,3));
          break;
        }
        case 3:
        {
          /* double num[] = {(2*damp + 1)*omega0*omega0,omega0*omega0*omega0}; */
          double num[] = {omega0*omega0*omega0};
          double den[] = {1,(2*damp + 1)*omega0,(2*damp + 1)*omega0*omega0,omega0*omega0*omega0};
          m_statespace = Math::StateSpace(Math::Matrix(num,1,2),Math::Matrix(den,1,4));
          break;
        }
        default:
          throw std::out_of_range("Reference model only supports 1st, 2nd and 3rd order filters");
      }
      // only reset state if forced, or if dimensions change
      if(reset_state || m_state.isEmpty())
      {
        m_state = Math::Matrix(order,1,0.0);
        return;
      }
      /* else if (order > m_order) */
      /* { */
      /*   // increase the size of the internal state vector */
      /*   // by adding a zero first */
      /*   Math::Matrix old_state = m_state; */
      /*   m_state = Math::Matrix(order,1,0.0); */
      /*   for (int i = 0; i < m_order; ++i) */
      /*     m_state(i+1) = old_state(i); */
      /*   m_state(0) = 0.0; */
      /* } */
      /* else if (order < m_order) */
      /* { */
      /*   Math::Matrix old_state = m_state; */
      /*   m_state = Math::Matrix(order,1,0.0); */
      /*   // decrease the size of the internal state vector */
      /*   // by removing the first element */
      /*   for (int i = 0; i < order; ++i) */
      /*     m_state(i) = old_state(i+1); */
      /* } */
      Math::Matrix tmp_new = m_statespace.getC();
      Math::Matrix tmp_old = oldSS.getC();
      Math::Matrix oldState = m_state;
      // initialize the state so that the output and its derivatives remain unchanged
      for (int i = 0; i < order; ++i) 
      {
        // make the ith derivative of the output of the new and old state space systems equal
        // y'(n) = C1*A1^n*x1 = C2*A2^n*x2
        // only the order-i th element of C has a value
        m_state(i) = tmp_old(order-1-i)/tmp_new(order-1-i)*oldState(i);
        tmp_old = tmp_old*oldSS.getA();
        tmp_new = tmp_new*m_statespace.getA();
        
      }
      /* y_old = m_statespace.getC()*m_state; */
      m_order = order;
    }

    void
    ReferenceModel::init(Math::Matrix y0)
    {
      for (int i = 0; i < m_order; ++i) {
        m_state(i,0) = y0(m_order-1-i)/m_statespace.getC().element(m_order-1);
      }
    }

    void
    ReferenceModel::init(double* y0)
    {
      // can not access/initialize more than elements in the internal state
      /* if(n > m_order) */
        /* n = m_order; */

      // initialize the state where no initial condition is given to zero
      /* for (int i = 0; i < (m_order - n - 1); ++i) */ 
      /*   m_state(i,0) = 0.0; */
      // initialize the state corresponding to the given initial outputs.
      /* for (int i = (m_order - n); i <= m_order; ++i) */ 
      /*   m_state(i,0) = y0[(n - 1) - (m_order -n)]/m_statespace.getC().element(m_order-1); */



      for (int i = 0; i < m_order; ++i) {
        m_state(i,0) = y0[m_order-1-i]/m_statespace.getC().element(m_order-1);
      }

      /* std::cout << m_state.size() << std::endl; */
      /* for (int i = 1; i <= n; ++i) */ 
      /* { */
      /*   std::cout << i + m_order - n << std::endl; */
      /*   m_state(i + (m_order - n),0) = y0[n-i]/m_statespace.getC().element(m_order-1); */
      /* } */
      /* m_num_derivatives = n-1; */
    }

    double
    ReferenceModel::step(double input, double timestep, double* derivatives)
    {

      //do simple Euler integration for now
      Math::Matrix state_dot = m_statespace.getA()*m_state + m_statespace.getB()*input;
      m_state += timestep*state_dot;
      Math::Matrix tmp = m_statespace.getC();
      /* for (int i = 0; i < m_num_derivatives+1; ++i) { */
      for (int i = 0; i < m_order; ++i) {
        //nth derivative of y = C*A^n*x
        derivatives[i] = (tmp*m_state).element(0,0);
        tmp = tmp*m_statespace.getA();
      }
      tmp = m_statespace.getC()*m_state;
      return tmp.element(0,0); //only has one element
    }
  }
}
