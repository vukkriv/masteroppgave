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

#ifndef USER_REFMOD_HPP_INCLUDED_
#define USER_REFMOD_HPP_INCLUDED_

// DUNE headers.
#include <DUNE/Config.hpp>
#include <USER/Math/StateSpace.hpp>

namespace DUNE
{
  namespace Smoothing
  {
    // Export DLL Symbol.
    class DUNE_DLL_SYM Smoothing;


    class ReferenceModel
    {
    public:
      //! @param[in] numerator numerator of a transfer function polynomial describing the Reference model filter
      //! @param[in] denominator denominator of a transfer function polynomial describing the Reference model filter
      ReferenceModel(Math::Matrix numerator, Math::Matrix denominator);

      //! @param[in] damp damping ratio
      //! @param[in] omega0 undamped angular frequency
      //! @param[in] reset_state true if the internal state should be initialized
      ReferenceModel(double damp, double omega0, int order);

      //! @param[in] damp damping ratio
      //! @param[in] omega0 undamped angular frequency
      void
      setDampFreq(double damp, double omega0, int order, bool reset_state);

      //! @param[in] y0 initial value for the output and n-1 of its derivatives
      //! y0 = [y(0) y'(0) y''(0) ... y'(n-1)(0)]
      // NB y0 is reversed compared to the internal state vector
      //! @param[in] n length of y0. This controls how many derivatives will be 
      //! calculated in the step
      void
      init(double* y0);

      void
      init(Math::Matrix y0);
      
      //! @param[in] input the unfiltered reference
      //! @param[in] timestep timestep for the numerical integration
      //! @param[out] derivatives the ith component of derivatives is the
      //! ith derivative of the filtered reference
      //! @return the filtered value, i.e. derivatives[0]
      double
      step(double input, double timestep, double* derivatives);

    private:
      // state space representation of the reference model
      Math::StateSpace m_statespace;
      Math::Matrix m_state;
      // order of the system
      int m_order;
      // how many derivatives we should return: 0..m_order
      int m_num_derivatives;
    };
  }
}

#endif
