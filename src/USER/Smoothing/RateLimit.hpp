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

#ifndef USER_RATELIM_HPP_INCLUDED_
#define USER_RATELIM_HPP_INCLUDED_

// ISO C++ 98 headers.
#include <vector>

// DUNE headers.
#include <DUNE/Config.hpp>
#include <DUNE/Time.hpp>
#include <DUNE/Math/General.hpp>

namespace DUNE
{
  //! Functionality for limiting the rate of change in a signal, typically to ensure 
  //! that actuator outputs are smooth
  namespace Smoothing
  {
    // Export DLL Symbol.
    class DUNE_DLL_SYM Smoothing;

    double
    rateLimit(double val, double prev_val, double min_rate, double max_rate, double ts)
    {
      return Math::trimValue(val, prev_val + min_rate*ts, prev_val + max_rate*ts);
    }

    class RateLimiter
    {
    public:
      RateLimiter(double duration, int order);

      void
      init(double val, double min_rate, double max_rate, double ts)
      {
        m_prev_val = val;
        m_low_rate = min_rate;
        m_high_rate = max_rate;
        m_ts = ts;
      }

      //! param[in] val the value to be rate limited
      double
      limit(double val)
      {
        return limit(val, m_prev_val, m_ts);
      }
      
      //! param[in] val the value to be rate limited
      //! param[in] last_val the previous output
      //! param[in] ts time since last call
      double
      limit(double val, double last_val, double ts)
      {
        m_prev_val = Math::trimValue(val, last_val + m_low_rate*ts, last_val + m_high_rate*ts);
        return m_prev_val;
      }

    private:
      double m_prev_val;
      double m_low_rate;
      double m_high_rate;
      double m_ts;
    };
  }
}

#endif
