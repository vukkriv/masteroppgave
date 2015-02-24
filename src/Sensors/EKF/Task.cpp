//***************************************************************************
// Copyright 2007-2015 Universidade do Porto - Faculdade de Engenharia      *
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
// Author: RecepCetin                                                       *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>


namespace Sensors
{
namespace EKF
{
  using DUNE_NAMESPACES;

  struct Task: public DUNE::Tasks::Task
  {

	Math::Matrix m_F, m_H, m_Q, m_R, m_P, m_X, m_S, m_K;
	double m_Pinit, m_Rinit,m_Qinit;
	bool newDistMeasurement;

	IMC::BeaconDistance m_beaconDist;

	//! Constructor.
	//! @param[in] name task name.
	//! @param[in] ctx context.
	Task(const std::string& name, Tasks::Context& ctx):
	  DUNE::Tasks::Task(name, ctx)
	{
		bind<BeaconDistance>(this);
	}


	//! Update internal state with new parameter values.
	void
	onUpdateParameters(void)
	{
		newDistMeasurement = false;

		m_Pinit = 1.0;
		m_Qinit = 0.1;
		m_Rinit = 0.550;


		double x[] = {4,3,1};
		m_X = Matrix(x,3,1);

		double p[] = {m_Pinit, m_Pinit, m_Pinit};
		m_P = Matrix(p,3);
		double q[] = {m_Qinit,m_Qinit,m_Qinit};
		m_Q = Matrix(q,3);
		double r[] = {m_Rinit,m_Rinit,m_Rinit,m_Rinit,m_Rinit,m_Rinit};
		m_R = Matrix(r,6);

	}

	//! Reserve entity identifiers.
	void
	onEntityReservation(void)
	{
	}

	//! Resolve entity names.
	void
	onEntityResolution(void)
	{
	}

	//! Acquire resources.
	void
	onResourceAcquisition(void)
	{
	}

	//! Initialize resources.
	void
	onResourceInitialization(void)
	{
	}

	//! Release resources.
	void
	onResourceRelease(void)
	{
	}

	//! Main loop.
	void
	onMain(void)
	{
	  while (!stopping())
	  {
		waitForMessages(1.0);
		printMatrix(m_X);
	  }
	}

	void
	consume(const IMC::BeaconDistance* bcn)
	{
		m_beaconDist = *bcn;
		newDistMeasurement = true;
		handleEKF();
	}

	void
	handleEKF(){

	}

	void
	printMatrix(Matrix m){
		printf("[TEST PRINTOUT]\n");
		for(int i = 0; i<m.rows(); i++ ){
		  for(int j = 0; j<m.columns();j++){
			printf("%f ", m.element(i,j));
		  }
		  printf("\n");
		}
	}


  };
}
}


DUNE_TASK
