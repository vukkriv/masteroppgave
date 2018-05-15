//***************************************************************************
// Copyright 2007-2017 Universidade do Porto - Faculdade de Engenharia      *
// Laboratório de Sistemas e Tecnologia Subaquática (LSTS)                  *
//***************************************************************************
// This file is part of DUNE: Unified Navigation Environment.               *
//                                                                          *
// Commercial Licence Usage                                                 *
// Licencees holding valid commercial DUNE licences may use this file in    *
// accordance with the commercial licence agreement provided with the       *
// Software or, alternatively, in accordance with the terms contained in a  *
// written agreement between you and Faculdade de Engenharia da             *
// Universidade do Porto. For licensing terms, conditions, and further      *
// information contact lsts@fe.up.pt.                                       *
//                                                                          *
// Modified European Union Public Licence - EUPL v.1.1 Usage                *
// Alternatively, this file may be used under the terms of the Modified     *
// EUPL, Version 1.1 only (the "Licence"), appearing in the file LICENCE.md *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// https://github.com/LSTS/dune/blob/master/LICENCE.md and                  *
// http://ec.europa.eu/idabc/eupl.html.                                     *
//***************************************************************************
// Author: Kristoffer Gryte                                                 *
//***************************************************************************

// ISO C++ 98 headers.
#include <iostream>

// DUNE headers.
#include <DUNE/DUNE.hpp>
#include <USER/Math/StateSpace.hpp>

using DUNE_NAMESPACES;

// Local headers.
#include "Test.hpp"



int
main(void)
{
  Test test("StateSpace from TF");
  {
    // H(s) = (s^2 + 2s+3)/(s^2 + 2s + 3)
    double num[] = {1,2,3};
    double den[] = {1,2,3};
    double tmpA[] = {-2, -3, 1, 0};
    Matrix A = Matrix(tmpA,2,2);
    double tmpB[] = {1, 0};
    Matrix B = Matrix(tmpB,2,1);
    double tmpC[] = {0,0};
    Matrix C = Matrix(tmpC,1,2);
    double tmpD[] = {1};
    Matrix D = Matrix(tmpD,1,1);
    StateSpace m_ss(Matrix(num,1,3),Matrix(den,1,3));

    test.boolean("Order 2 num / Order 2 den: A", A == m_ss.getA());
    test.boolean("Order 2 num / Order 2 den: B", B == m_ss.getB());
    test.boolean("Order 2 num / Order 2 den: C", C == m_ss.getC());
    test.boolean("Order 2 num / Order 2 den: D", D == m_ss.getD());
  }
  {
    // H(s) = (2s+3)/(s^2 + 2s + 3)
    double num[] = {2,3};
    double den[] = {1,2,3};
    double tmpA[] = {-2, -3, 1, 0};
    Matrix A = Matrix(tmpA,2,2);
    double tmpB[] = {1, 0};
    Matrix B = Matrix(tmpB,2,1);
    double tmpC[] = {2,3};
    Matrix C = Matrix(tmpC,1,2);
    double tmpD[] = {0};
    Matrix D = Matrix(tmpD,1,1);
    StateSpace m_ss(Matrix(num,1,2),Matrix(den,1,3));

    test.boolean("Order 1 num / Order 2 den: A", A == m_ss.getA());
    test.boolean("Order 1 num / Order 2 den: B", B == m_ss.getB());
    test.boolean("Order 1 num / Order 2 den: C", C == m_ss.getC());
    test.boolean("Order 1 num / Order 2 den: D", D == m_ss.getD());
  }
  {
    // H(s) = 2
    double num[] = {2};
    double den[] = {1};
    double tmpA[] = {0};
    Matrix A = Matrix(tmpA,1,1);
    double tmpB[] = {0};
    Matrix B = Matrix(tmpB,1,1);
    double tmpC[] = {0};
    Matrix C = Matrix(tmpC,1,1);
    double tmpD[] = {2};
    Matrix D = Matrix(tmpD,1,1);
    StateSpace m_ss(Matrix(num,1,1),Matrix(den,1,1));

    test.boolean("Scalar case: A", A == m_ss.getA());
    test.boolean("Scalar case: B", B == m_ss.getB());
    test.boolean("Scalar case: C", C == m_ss.getC());
    test.boolean("Scalar case: D", D == m_ss.getD());
  } 
  {
    // H(s) = [2]      [3]
    //        [4] s  + [5]
    //        ____________
    //        s^2 + 2s + 3
    double num[] = {2,3,4,5};
    double den[] = {1,2,3};
    double tmpA[] = {-2, -3, 1, 0};
    Matrix A = Matrix(tmpA,2,2);
    double tmpB[] = {1, 0};
    Matrix B = Matrix(tmpB,2,1);
    double tmpC[] = {2,3,4,5};
    Matrix C = Matrix(tmpC,2,2);
    double tmpD[] = {0,0};
    Matrix D = Matrix(tmpD,2,1);
    StateSpace m_ss(Matrix(num,2,2),Matrix(den,1,3));

    test.boolean("Twodimensional numerator: A", A == m_ss.getA());
    test.boolean("Twodimensional numerator: B", B == m_ss.getB());
    test.boolean("Twodimensional numerator: C", C == m_ss.getC());
    test.boolean("Twodimensional numerator: D", D == m_ss.getD());
  }
  {
    // H(s) = 0;
    double num[] = {0};
    double den[] = {1,2,3};
    double tmpA[] = {-2, -3, 1, 0};
    Matrix A = Matrix(tmpA,2,2);
    double tmpB[] = {1, 0};
    Matrix B = Matrix(tmpB,2,1);
    double tmpC[] = {0,0};
    Matrix C = Matrix(tmpC,1,2);
    double tmpD[] = {0};
    Matrix D = Matrix(tmpD,1,1);
    StateSpace m_ss(Matrix(num,1,1),Matrix(den,1,3));

    test.boolean("Zero numerator: A", A == m_ss.getA());
    test.boolean("Zero numerator: B", B == m_ss.getB());
    test.boolean("Zero numerator: C", C == m_ss.getC());
    test.boolean("Zero numerator: D", D == m_ss.getD());
  }
  {
    // improper transfer function
    double num[] = {6,5,4,3};
    double den[] = {1,2,3};
    bool passed = false;
    try
    {
      StateSpace m_ss(Matrix(num,1,4),Matrix(den,1,3));
      passed = false;
    }
    catch(std::out_of_range& e)
    {
      passed = true;
    }
    test.boolean("Throw exception on improper TF", passed);
  }

  return 0;
}
