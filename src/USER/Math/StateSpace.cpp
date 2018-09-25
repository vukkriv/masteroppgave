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

// DUNE headers.
#include <DUNE/Config.hpp>

#include <USER/Math/StateSpace.hpp>

namespace DUNE
{
  namespace Math
  {
    StateSpace::StateSpace(void):
      A(Matrix(1,1,0.0)),
      B(Matrix(1,1,0.0)),
      C(Matrix(1,1,0.0)),
      D(Matrix(1,1,0.0))
    {
    }

    // @param[in] numerator matrix where each row corresponds to one output and the indeces in each column correspond to the coefficients in the numerator polynomial, in decending powers
    // @param[in] denominator row vector where the indeces correspond to the coefficients in the denominator polynomial, in decending powers
    StateSpace::StateSpace(Math::Matrix numerator, Math::Matrix denominator):
      A(Matrix(1,1,0.0)),
      B(Matrix(1,1,0.0)),
      C(Matrix(1,1,0.0)),
      D(Matrix(1,1,0.0))
    {
      computeStateSpace(numerator,denominator);
    }

    // Computes the controllable cannonical form state space representation of a transfer function
    // Heavily based on tf2ss in scipy.signals
    // @param[in] numerator matrix where each row corresponds to one output and the indeces in each column correspond to the coefficients in the numerator polynomial, in decending powers
    // @param[in] denominator row vector where the indeces correspond to the coefficients in the denominator polynomial, in decending powers
    void
    StateSpace::computeStateSpace(Math::Matrix num, Math::Matrix den)
    {
      // numerator/denominator number of rows
      int r_num = num.rows();
      // numerator/denominator number of columns
      int c_num = num.columns();
      int c_den = den.columns();

      if (c_num > c_den)
      {
        throw std::out_of_range("Improper transfer function. `num` is longer than `den`.");
      }

      if ((c_num == 0) || (c_den == 0))  // Null system
        A = Math::Matrix(1,1,0.0); B = Math::Matrix(1,1,0.0); C = Math::Matrix(1,1,0.0); D = Math::Matrix(1,1,0.0);

      // pad numerator to have same number of columns has denominator and normalize
      if (c_den - c_num > 0)
      {
        num = Matrix(r_num, c_den - c_num,0.0).horzCat(num)/den.element(0,0);
        c_num = num.columns();
      }
      else
        num /= den.element(0,0);

      if (!num.isEmpty())
      {
        // get first col
        if(r_num == 1)
          D = num.get(0,r_num-1,0,0);
      }
      else
      {
          // We don't assign it an empty array because this system
          // is not 'null'. It just doesn't have a non-zero D
          // matrix. Thus, it should have a non-zero shape so that
          // it can be operated on by functions like 'ss2tf'
          D = 0;
      }

      if (c_den == 1)
      {
          D.resizeAndKeep(r_num,c_num);//reshape(num.shape)
          A = Matrix(1,1,0.0);
          B = Matrix(1,D.columns(),0.0);
          C = Matrix(D.rows(),1,0.0);
          return;
      }

      Matrix frow = -den.get(0,0,1,c_den-1);
      A = frow.vertCat(eye(c_den - 2, c_den - 1));
      B = eye(c_den - 1, 1);
      C = num.get(0,r_num-1,1,c_num-1) - num.get(0,r_num-1, 0,0)*den.get(0,0,1,c_den-1);
      D.resizeAndKeep(C.rows(), B.columns());
      return;
    }

    Matrix
    StateSpace::eye(int rows, int cols)
    {
      // create an identity matrix, and pick the submatrix 0..rows x 0..cols
      // to allow for non-square identity matrices
      Matrix I = Matrix(std::max(rows,cols));
      return I.get(0,rows-1,0,cols-1);
    }
  }
}
