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

// ISO c++ 98 headers
#include <iomanip>
#include <cmath>

// DUNE headers.
#include <DUNE/DUNE.hpp>


namespace Sensors
{
namespace EKF
{
  using DUNE_NAMESPACES;

  struct Arguments
  {
	  // Beacon treshold
	  float bcn_treshold;
	  // State Covariance
	  float state_cov;
	  // Measurement noise covariance
	  float bcn_mnoise;
	  // Process noise covariance
	  float bcn_pnoise;
	  // timeout
	  float bcn_timeout;
	  // node number
  };

  struct Task: public DUNE::Tasks::Task
  {
	  // Last north reference
		double m_last_n;
		// Last east reference
		double m_last_e;
		// Last down reference
		double m_last_d;

		bool m_measurements_active;

		IMC::BeaconDistance* m_bcnmeas;

		// navigation data - costum
		IMC::NavigationData* m_origin;

		// bcn pos estimate
		IMC::LblEstimate* m_estimate;

		// time whitout measurements
		Time::Counter<double> m_time_whitout_meas;

		// KalmanFilter matrices
		KalmanFilter m_kal;

		// Task arguments
		Arguments m_args;


	//! Constructor.
	//! @param[in] name task name.
	//! @param[in] ctx context.
	Task(const std::string& name, Tasks::Context& ctx):
	  DUNE::Tasks::Task(name, ctx),
	  m_origin(NULL)
	{
		param("State Covariance Initial State", m_args.state_cov)
	    .defaultValue("1.0")
	    .minimumValue("1.0")
	    .description("Kalman Filter State Covariance initial values");


		param("Beacon Threshold", m_args.bcn_treshold)
	   .defaultValue("3.14")
	   .minimumValue("2.0")
	   .description("Beacon Threshold value for the pos");

		param("Beacon Measure Noise Covariance", m_args.bcn_mnoise)
	   .defaultValue("50.0")
	   .minimumValue("10")
	   .description("Kalman Filter bcn Measurement Noise Covariance value");


		param("LBL Process Noise Covariance", m_args.bcn_pnoise)
	  .defaultValue("1e-1")
	  .minimumValue("0.0")
	  .description("Kalman Filter pos Process Noise Covariance value");

		param("GPS timeout", m_args.bcn_timeout)
	   .units(Units::Second)
	   .defaultValue("3.0")
	   .minimumValue("1.0")
	   .description("No Meas readings timeout");

		m_estimate = NULL;
		m_last_n = 4.0;
		m_last_e = 2.0;
		m_last_d = 3.0;
		m_measurements_active = false;

		bind<IMC::BeaconDistance>(this);
		bind<IMC::NavigationData>(this);
		bin<IMC::LblEstimate>(this);

	}


	//! Update internal state with new parameter values.
	void
	onUpdateParameters(void)
	{
		if (paramChanged(m_args.bcn_timeout))
			m_time_whitout_meas.setTop(m_args.bcn_timeout);
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
		setEntityState(IMC::EntityState::ESTA_BOOT, Status::CODE_WAIT_GPS_FIX);
	}

	//! Release resources.
	void
	onResourceRelease(void)
	{
		Memory::clear(m_origin);
		Memory::clear(m_estimate);
	}

	void
	consume(const IMC::BeaconDistance* msg)
	{
		float ni,ei,di;
		m_time_whitout_meas.reset();
		m_measurements_active = true;

		short sender = msg->sender;

		if (sender == 1) { ni = 0; ei = 0; di = 0;}
		else if (sender == 2) { ni = 4; ei = 0; di = 0;}
		else if (sender == 3) { ni = 8; ei = 0; di = 0;}
		else if (sender == 4) { ni = 8; ei = 6.87; di = 0;}
		else if (sender == 5) { ni = 4; ei = 6.87; di = 0;}
		else if (sender == 6) { ni = 0; ei = 6.87; di = 0;}
		else {return;}


		double dx = m_estimate->x - ni;
		double dy = m_estimate->y - ei;
		double dz = m_estimate->var_x - di;
		double exp_range = std::sqrt(dx * dx + dy * dy + dz*dz);

		m_kal.setObservation(sender,1, dx / exp_range);
		m_kal.setObservation(sender,2, dy / exp_range);
		m_kal.setObservation(sender,3, dz / exp_range);

		// REJECTION FILTER SOMEWHERE HERE

		m_kal.resetOutputs();


	}

	//! Main loop.
	void
	onMain(void)
	{
	  while (!stopping())
	  {
		waitForMessages(1.0);

	  }
	}


  };
}
}


DUNE_TASK
