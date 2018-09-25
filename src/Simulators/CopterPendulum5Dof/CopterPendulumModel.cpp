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
// https://www.lsts.pt/dune/licence.                                        *
//***************************************************************************
// Author: Kristian Klausen                                                 *
//***************************************************************************

// Local headers.
#include "CopterPendulumModel.hpp"

namespace Simulators
{
  namespace CopterPendulum5Dof
  {


    Matrix
    CopterPendulumModel::getMassMatrix(Matrix& eta)
    {
      Matrix mass = Matrix(5,5, 0.0);

      f_5dof_MassMatrix_singularity_avoidance(eta, m_length, m_mass_copter, m_mass_load, c_epsilon, mass);

      return mass;
    }



    Matrix
    CopterPendulumModel::getCoreolisMatrix(Matrix& eta, Matrix& nu)
    {
      Matrix C = Matrix(5,5, 0.0);
      f_5dof_CoreolisMatrix_singularity_avoidance(eta, nu, m_length, m_mass_copter, m_mass_load, c_epsilon, C);
      return C;
    }

    Matrix
    CopterPendulumModel::getGravityMatrix(Matrix& eta)
    {
      Matrix G = Matrix(5,1, 0.0);
      f_5dof_Gravity(eta, m_length, m_mass_copter, m_mass_load, Math::c_gravity, G);
      return G;
    }


    void CopterPendulumModel::f_5dof_CoreolisMatrix_singularity_avoidance(const Matrix& in1, const
      Matrix& in2, double L, double, double m_L, double epsilon, Matrix& CoreolisMatrix)
    {
      double t2;
      double t3;
      double t4;
      double t5;
      double t6;
      double t8;
      double x[25];
      t2 = std::sin(in1(4));
      t3 = std::cos(in1(3));
      t4 = std::cos(in1(4));
      t5 = std::sin(in1(3));
      t6 = L * L;
      t8 = std::sin(in1(4) * 2.0);
      x[0] = 0.0;
      x[1] = 0.0;
      x[2] = 0.0;
      x[3] = 0.0;
      x[4] = 0.0;
      x[5] = 0.0;
      x[6] = 0.0;
      x[7] = 0.0;
      x[8] = 0.0;
      x[9] = 0.0;
      x[10] = 0.0;
      x[11] = 0.0;
      x[12] = 0.0;
      x[13] = 0.0;
      x[14] = 0.0;
      x[15] = 0.0;
      x[16] = L * m_L * (in2(3) * t4 * t5 + in2(4) * t2 * t3);
      x[17] = -L * m_L * (in2(3) * t3 * t4 - in2(4) * t2 * t5);
      x[18] = in2(4) * t8 * (epsilon - m_L * t6) * 0.5;
      x[19] = in2(3) * m_L * t6 * t8 * 0.5;
      x[20] = -L * in2(4) * m_L * t2;
      x[21] = L * m_L * (in2(3) * t2 * t3 + in2(4) * t4 * t5);
      x[22] = L * m_L * (in2(3) * t2 * t5 - in2(4) * t3 * t4);
      x[23] = in2(3) * m_L * t6 * t8 * -0.5;
      x[24] = 0.0;
      CoreolisMatrix.fill(5,5, x);
    }



    void CopterPendulumModel::f_5dof_Gravity(const Matrix& in1, double L, double m_c, double m_L,
                        double g, Matrix& GravityMatrix)
    {
      double Gravity[5];
      Gravity[0] = 0.0;
      Gravity[1] = 0.0;
      Gravity[2] = -g * (m_L + m_c);
      Gravity[3] = L * g * m_L * std::cos(in1(4)) * std::sin(in1(3));
      Gravity[4] = L * g * m_L * std::cos(in1(3)) * std::sin(in1(4));

      GravityMatrix.fill(5,1, Gravity);
    }

    void CopterPendulumModel::f_5dof_MassMatrix_singularity_avoidance(const Matrix& in1, double L,
      double m_c, double m_L, double epsilon, Matrix& MassMatrix)
    {
      double t2;
      double t3;
      double t4;
      double t5;
      double t6;
      double t7;
      double t8;
      double t9;
      double x[25];
      t2 = m_L + m_c;
      t3 = std::cos(in1(4));
      t4 = std::sin(in1(3));
      t5 = std::cos(in1(3));
      t6 = std::sin(in1(4));
      t7 = L * m_L * t3;
      t8 = L * m_L * t4 * t6;
      t9 = L * L;
      x[0] = t2;
      x[1] = 0.0;
      x[2] = 0.0;
      x[3] = 0.0;
      x[4] = t7;
      x[5] = 0.0;
      x[6] = t2;
      x[7] = 0.0;
      x[8] = -L * m_L * t3 * t5;
      x[9] = t8;
      x[10] = 0.0;
      x[11] = 0.0;
      x[12] = t2;
      x[13] = -L * m_L * t3 * t4;
      x[14] = -L * m_L * t5 * t6;
      x[15] = 0.0;
      x[16] = -L * m_L * t3 * t5;
      x[17] = -L * m_L * t3 * t4;
      x[18] = epsilon * (t6 * t6) + m_L * (t3 * t3) * t9;
      x[19] = 0.0;
      x[20] = t7;
      x[21] = t8;
      x[22] = -L * m_L * t5 * t6;
      x[23] = 0.0;
      x[24] = m_L * t9;
      MassMatrix.fill(5,5, x);
    }



    void CopterPendulumModel::f_5dof_Rload(double phi_L, double theta_L, double Rnb[9])
    {
      double t2;
      double t3;
      double t4;
      double t5;
      double x[9];
      int k;
      t2 = std::sin(theta_L);
      t3 = std::cos(theta_L);
      t4 = std::sin(phi_L);
      t5 = std::cos(phi_L);
      x[0] = t3;
      x[1] = t2 * t4;
      x[2] = -t2 * t5;
      x[3] = 0.0;
      x[4] = t5;
      x[5] = t4;
      x[6] = t2;
      x[7] = -t3 * t4;
      x[8] = t3 * t5;
      for (k = 0; k < 9; k++) {
        Rnb[k] = x[k];
      }
    }


  }
}

