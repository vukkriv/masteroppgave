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

#include <USER/Smoothing/Differentiability.hpp>

namespace DUNE
{
  namespace Smoothing
  {
  Differentiability::Differentiability(double duration, int order):
      m_duration(duration)
    {
      // make sure order is acceptable, by checking that the largest call to Math::factorial is acceptable
      if (order < 0)
        throw std::out_of_range("Negative orders are not defined");

      try
      {
        Math::factorial(2*order + 1);
      }
      catch(std::out_of_range& e)
      {
        throw std::out_of_range("Order too large");
      }
      m_order = order;

      // pre-compute the polynom coefficients
      p = std::vector<double>(2*m_order+1,0.0);
      for(int i = (m_order); i < (2*m_order+1); ++i)
      {
        p[i] = calculateCoeff(i);
      }
    }

    void
    Differentiability::init(double start_val, double end_val, double duration)
    {
      m_duration = duration;
      m_timer.reset();
      m_timer.setTop(m_duration);
      m_start_val = start_val;
      m_end_val = end_val;
    }

    //! param[out] filtered_vals array of size m_order, that contains the smoothed reference and its derivatives
    double
    Differentiability::update(double* filtered_vals)
    {
      if(m_timer.overflow())
      {
        for(int i=0; i < m_order+2; ++i)
        {
          filtered_vals[i] = 0.0;
        }
        filtered_vals[0] = m_end_val;
        return m_end_val;
      }
      else
        return filter(m_start_val, m_end_val, m_timer.getElapsed(), filtered_vals);
    }

    // for testing 
    double
    Differentiability::filter(double start_val, double end_val, double elapsed_time, double* filtered_vals)
    {
      // loop over all the derivatives
      for(int k = 0; k < m_order+2; ++k)
      {
        filtered_vals[k] = (end_val - start_val)*pt(elapsed_time,k);
      }
      return filtered_vals[0] += start_val;
    }

    double
    Differentiability::calculateCoeff(int i)
    {
      return std::pow((-1),(i-m_order))*Math::factorial(2*m_order+1)/((i+1)*Math::factorial(m_order)*Math::factorial(i-m_order)*Math::factorial(2*m_order-i));
    }

    double
    Differentiability::pt(double time, int derivative)
    {
      double val = 0;
      for(int i = (m_order); i <= (2*m_order); ++i)
      {
        val += Math::factorial(i+1)/Math::factorial(i+1-derivative)*p[i]*std::pow(time/m_duration,i+1-derivative);
      }
      return val;
    }
  }
}
