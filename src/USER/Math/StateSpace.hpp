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

#ifndef DUNE_MATH_STATESPACE_HPP_INCLUDED_
#define DUNE_MATH_STATESPACE_HPP_INCLUDED_

// DUNE headers.
#include <DUNE/Config.hpp>
#include <DUNE/Math/Matrix.hpp>

namespace DUNE
{
  namespace Math
  {
    // Export DLL Symbol.
    class DUNE_DLL_SYM StateSpace;

    class StateSpace
    {
    public:
      //! Constructor.
      //! Constructs a scalar state space model A=B=C=D=0
      StateSpace(void);

      //! Constructor.
      //! Constructs a controllable cannonical state space model from a transfer function
      // @param[in] numerator matrix where each row corresponds to one output and the indeces in each column correspond to the coefficients in the numerator polynomial, in decending powers
      // @param[in] denominator row vector where the indeces correspond to the coefficients in the denominator polynomial, in decending powers
      StateSpace(Math::Matrix numerator, Math::Matrix denominator);

      // Computes the controllable cannonical form state space representation of a transfer function
      // Heavily based on tf2ss in scipy.signals
      // @param[in] numerator matrix where each row corresponds to one output and the indeces in each column correspond to the coefficients in the numerator polynomial, in decending powers
      // @param[in] denominator row vector where the indeces correspond to the coefficients in the denominator polynomial, in decending powers
      void
      computeStateSpace(Matrix numerator, Matrix denominator);

      //! Compute non-square identity matrices
      //! @param[in] rows number of rows in the returned matrix
      //! @param[in] cols number of colums in the returned matrix
      Math::Matrix
      eye(int rows, int cols);

      Math::Matrix
      getA(void){ return A;}

      Math::Matrix
      getB(void){ return B;}

      Math::Matrix
      getC(void){ return C;}

      Math::Matrix
      getD(void){ return D;}

    private:
      // system matrices
      Math::Matrix A,B,C,D;
    };
  }
}
#endif
