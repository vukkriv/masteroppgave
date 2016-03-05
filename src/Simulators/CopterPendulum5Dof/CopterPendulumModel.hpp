//***************************************************************************
// Copyright 2007-2014 Universidade do Porto - Faculdade de Engenharia      *
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
// https://www.lsts.pt/dune/licence.                                        *
//***************************************************************************
// Author: Kristian Klausen                                                 *
//***************************************************************************

#ifndef SIMULATORS_COPTERPENDULUM_COPTERPENDULUMMODEL_HPP_INCLUDED_
#define SIMULATORS_COPTERPENDULUM_COPTERPENDULUMMODEL_HPP_INCLUDED_

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Simulators
{
  namespace CopterPendulum5Dof
  {
    // Needed?
    using DUNE_NAMESPACES;

    // Export DLL Symbol.
    class DUNE_DLL_SYM CopterPendulumModel;

    static const double c_epsilon = 0.01;

    class CopterPendulumModel {
    public:

      //CopterPendulum(): CopterPendulum(2.5, 0.5, 3){};

      CopterPendulumModel(double m_c = 3, double m_L = 1, double L = 3):
        m_mass_copter(m_c),
        m_mass_load(m_L),
        m_length(L),
        m_Mass(5,5, 0.0),
        m_Coreolis(5,5, 0.0),
        m_Gravity(5,1, 0.0),
        m_Tau(5,1, 0.0)
      {
        // Intentionally empty.
      };

      void
      setMassCopter(double m_c) { m_mass_copter = (m_c > 0) ? m_c : m_mass_copter;};

      void
      setMassLoad(double m_l)   { m_mass_load   = (m_l > 0) ? m_l : m_mass_load;  };

      void
      setLength(double L)       { m_length      = (L   > 0) ? L   : m_length;     };


      Matrix
      getMassMatrix(Matrix& eta);

      Matrix
      getCoreolisMatrix(Matrix& eta, Matrix& nu);

      Matrix
      getGravityMatrix(Matrix& eta);


    private:
      double m_mass_copter;
      double m_mass_load;
      double m_length;



      Matrix m_Mass;
      Matrix m_Coreolis;
      Matrix m_Gravity;

      Matrix m_Tau;






      void
      f_5dof_CoreolisMatrix_singularity_avoidance(const Matrix& eta,
          const Matrix& nu, double L, double m_c, double m_L, double epsilon, Matrix& CoreolisMatrix);

      void
      f_5dof_Gravity(const Matrix& eta, double L, double m_c, double m_L,
        double g, Matrix& Gravity);

      void
      f_5dof_MassMatrix_singularity_avoidance(const Matrix& eta, double
        L, double m_c, double m_L, double epsilon, Matrix& MassMatrix);


      void
      f_5dof_Rload(double phi_L, double theta_L, double Rnb[9]);

    };

  }
}

#endif /* SIMULATORS_COPTERPENDULUM_COPTERPENDULUMMODEL_HPP_INCLUDED_ */
