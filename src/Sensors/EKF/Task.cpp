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


// DUNE headers.
#include <DUNE/DUNE.hpp>



namespace Sensors
{
namespace EKF
{
  using DUNE_NAMESPACES;

  struct Arguments
     {
        double q; // process noise
        double r; // measurement noise
        double p; // inital covar
        double n_init; // North
        double e_init; // East
        double d_init; // Down
        std::vector<double> B1;
        std::vector<double> B2;
        std::vector<double> B3;
        std::vector<double> B4;
        std::vector<double> B5;
        std::vector<double> B6;
     };

  struct Task: public DUNE::Tasks::Task
  {
    Math::Matrix m_R, m_Q,m_X,m_P,m_F,m_B,dX,gXi,m_H,m_S,m_K,m_meas;
    double N,E,D, meas[6],R_init[6],R_excite[6];
    Arguments m_args;
    bool init_done;



	//! Constructor.
	//! @param[in] name task name.
	//! @param[in] ctx context.
	Task(const std::string& name, Tasks::Context& ctx):
	  DUNE::Tasks::Task(name, ctx)
	{
		param("Process noise", m_args.q)
	    .defaultValue("1.0")
	    .description("Process noise");

		param("Measurement noise", m_args.r)
		      .defaultValue("1.0")
		      .description("Measurement noise");

    param("Initial covariance", m_args.p)
          .defaultValue("1.0")
          .description("Initial covariance");

		param("init-North", m_args.n_init)
		          .defaultValue("1.0")
		          .description("Pos North");

		param("init-East", m_args.e_init)
		          .defaultValue("1.0")
		          .description("Pos East");

		param("init-Down", m_args.d_init)
		          .defaultValue("1.0")
		          .description("Pos Down");

    param("Anchors-1-pos", m_args.B1)
                  .defaultValue("-1.977, 2.0115, 2.776")
                  .description("Anchors-1-pos [NED]");

    param("Anchors-2-pos", m_args.B2)
                      .defaultValue("-0.017, 1.917, 2.76")
                      .description("Anchors-2-pos [NED]");

    param("Anchors-3-pos", m_args.B3)
                      .defaultValue("3.1896, 1.9081, 2.7455")
                      .description("Anchors-3-pos [NED]");

    param("Anchors-4-pos", m_args.B4)
                      .defaultValue("3.162, -3.4489, 2.759")
                      .description("Anchors-4-pos [NED]");

    param("Anchors-5-pos", m_args.B5)
                      .defaultValue("0.464, -3.459, 2.754")
                      .description("Anchors-5-pos [NED]");

    param("Anchors-6-pos", m_args.B6)
                      .defaultValue("-2.268, -3.4065, 2.7799")
                      .description("Anchors-6-pos [NED]");

		init_done =  false;
		bind<IMC::BeaconDistance>(this);
	}

	//! Update internal state with new parameter values.
	void
	onUpdateParameters(void)
	{

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

	void
	consume(const IMC::BeaconDistance* msg)
	{
	  // REMEMBER TO CONVERT DISTANCE TO METERS!!!!!!!!!!!!!!!

	  if (init_done == true)
	  {

	    predictKalman();
	    updateKalman(msg);

	  }
	}

	void
	initKalman(double r, double q,double p, double n_init, double e_init, double d_init,std::vector<double> B1,std::vector<double> B2,std::vector<double> B3
	    ,std::vector<double> B4,std::vector<double> B5,std::vector<double> B6)
	{
	  int excite = 10;
	  for (int i=0;i<6;i++){
	    R_init[i] = r;
	    R_excite[i] = r*excite;
	  }
	  m_R = Matrix(R_init,6);
	  double Q_init[] = {q,q,q};
	  m_Q = Matrix(Q_init,3);
	  double P_init[] = {p,p,p};
	  m_P = Matrix(P_init,3);
	  double X_init[] = {n_init,e_init,d_init};
	  m_X = Matrix(X_init,3,1);
	  double process[] = {1,1,1};
	  m_F = Matrix(process,3);

	  //double B[18] = {B1,B2,B3,B4,B5,B6};



	  double B[18];
	  for (int i = 0; i <= 2; i++)
	  {
	     B[i] = B1[i];
	     B[i+3] = B2[i];
	     B[i+6] = B3[i];
	     B[i+9] = B4[i];
	     B[i+12] = B5[i];
	     B[i+15] = B6[i];
	  }

	  m_B = Matrix(B,6,3);
	  dX = Matrix();
	  dX.resize(6,3);
	  m_H = Matrix();
	  m_H.resize(6,3);



	}
	void
	predictKalman()
	{
	  m_P = m_F * m_P * transpose(m_F) + m_Q;
	}

	void
	updateKalman(const IMC::BeaconDistance* msg)
	{


	    meas[msg->sender-1] = msg->dist / 100.0  ;
	    m_meas = Matrix(meas,6,1);
	    // MEASUREMENT UPDATE
	    Math::Matrix k;
	    double gXi_val[6], K_temp[3];
	    int a = 0;
	    double H[6*3];
      for (int i = 0; i < 6; i++)
      {


        k = Matrix(1,1,m_X.element(0) - m_B.element(i,0));
        dX.put(i, 0, k);
        k = Matrix(1,1,m_X.element(1) - m_B.element(i,1));
        dX.put(i, 1, k);
        k = Matrix(1,1,m_X.element(2) - m_B.element(i,2));
        dX.put(i, 2, k);

        gXi_val[i] = std::pow( dX.element(i,0),2) + std::pow( dX.element(i,1),2) + std::pow(dX.element(i,2),2);

        // Jacobian
        for (int j = 0; j<=2; j++)
        {
          H[a] = dX.element(i,j) / gXi_val[i];
          a++;
        }

    }
      gXi = Matrix(gXi_val,6,1);
      m_H = Matrix(H,6,3);


      // THE INNOVATION COVARIANCE
      m_S = m_H * m_P * transpose(m_H) + m_R;

      // The Kalman Gain
      m_K = m_P * transpose(m_H) * inverse(m_S);

      // Set K to zero for all except this measurement
      for (int i=0; i<=2; i++)
        K_temp[i] = m_K.element(i,msg->sender-1);


      m_K.fill(0.0);

      k = Matrix(1,1,K_temp[0]);
      m_K.put(0,msg->sender-1,k);
      k = Matrix(1,1,K_temp[1]);
      m_K.put(1,msg->sender-1,k);
      k = Matrix(1,1,K_temp[2]);
      m_K.put(2,msg->sender-1,k);


      m_X = m_X + m_K * (m_meas - gXi);
      m_P = m_P - (m_P * transpose(m_H) * inverse(m_S) * m_H) * m_P;

      m_R = Matrix(R_init,6);
      if ( (m_meas - gXi).norm_2() > 20000.0 )
      {
        m_R = Matrix(R_excite,6);
      }

      if (m_X.element(0) != m_X.element(0) || m_X.element(1) != m_X.element(1) || m_X.element(2) != m_X.element(2))
      {
         initKalman(m_args.r,m_args.q,m_args.p,m_args.n_init,m_args.e_init,m_args.d_init,m_args.B1,m_args.B2,m_args.B3,m_args.B4,m_args.B5,m_args.B6);
      }

      IMC::RtkFix pos;
      pos.n = m_P.element(0);
      pos.e = m_P.element(1);
      pos.d = m_P.element(2);
      dispatch(pos);



	}

	//! Main loop.
	void
	onMain(void)
	{
	  initKalman(m_args.r,m_args.q,m_args.p,m_args.n_init,m_args.e_init,m_args.d_init,m_args.B1,m_args.B2,m_args.B3,m_args.B4,m_args.B5,m_args.B6);
	  init_done = true;

	  while (!stopping())
	  {
		waitForMessages(1.0);

	  }
	}

void
printMatrix(Matrix m){
  printf("[HERE]\n");
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
