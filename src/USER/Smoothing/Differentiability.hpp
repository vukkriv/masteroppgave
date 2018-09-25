//***************************************************************************
// Copyright 2007-2018 Norwegian University of Science and Technology (NTNU)*
// Centre for Autonomous Marine Operations and Systems (AMOS)               *
// Department of Engineering Cybernetics (ITK)                              *
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

#ifndef USER_DIFFERENTIABILITY_HPP_INCLUDED_
#define USER_DIFFERENTIABILITY_HPP_INCLUDED_

// ISO C++ 98 headers.
#include <vector>

// DUNE headers.
#include <DUNE/Config.hpp>
#include <DUNE/Time.hpp>
#include <DUNE/Math/General.hpp>

namespace DUNE
{
  namespace Smoothing
  {
    // Export DLL Symbol.
    class DUNE_DLL_SYM Smoothing;



    //! Class for smoothing a reference jump, by providing
    //! 1) the initial value (smoothing from)
    //! 2) the final value (smoothing to)
    //! 3) the duration of the smoothing
    //! 4) how smooth the reference should be; C1, C2, C3 ....
    //! use the init function to initiate a smoothing. 
    //! Subsequent calls to update will then provide a smoothed reference
    class Differentiability
    {
    public:
      Differentiability(double duration, int order);

      void
      init(double start_val, double end_val, double duration);

      //! param[out] filtered_vals array of size m_order, that contains the smoothed reference and its derivatives
      double
      update(double* filtered_vals);

      // for testing 
      double
      filter(double start_val, double end_val, double elapsed_time, double* filtered_vals);

      // for testing
      double
      getCoeff(int i){ return p[i]; }

      double
      getOrder(void){ return m_order; }

    private:
      double
      calculateCoeff(int i);

      double
      pt(double time, int derivative);

      double m_duration;
      Time::Counter<double> m_timer;
      double m_start_val;
      double m_end_val;
      int m_order;
      // vector of polynomials
      std::vector<double> p;
    };
  }
}

#endif
