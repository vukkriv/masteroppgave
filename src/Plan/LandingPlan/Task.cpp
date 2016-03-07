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
// Author: Kjetilhs                                                         *
//***************************************************************************

// This task generates a dubins path that can be used for landing
// DUNE headers.
#include <DUNE/DUNE.hpp>

#include <vector>
#include <cmath>
#define PI 3.1415926535897932384626433832795

namespace Plan
{
  namespace LandingPlan
  {
    using DUNE_NAMESPACES;

    struct Arguments
    {

    };

    struct Task: public DUNE::Tasks::Task
    {
      //! Task arguments
      Arguments m_arg;
      //! Accumulated EstimatedState message
      IMC::EstimatedState m_estate;
      //! Start turning circle
      double m_Rs;
      //! Finish turning circle
      double m_Rf;
      //! Angle of descent
      double m_gamma_d;
      //! Angle of attack
      double m_gamma_a;
      //! Calculated path
      std::vector<double [3]> m_path;
      //! Start pose
      std::vector<double [4]> m_Xs;
      //! Finish pose
      std:: vector<double [4]> m_Xf;
      //! Number of points in arc
      int m_N;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx)
      {
        bind<IMC::EstimatedState>(this);
        bind<IMC::PlanGeneration>(this);
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
      }

      //! Update estimatedState
      void
      consume(const IMC::EstimatedState *msg)
      {

      }
      //! Receive nett pose and landing spesifications
      void
      consume(const IMC::PlanGeneration *msg)
      {

      }

      //! Construct Dubins Path between two waypoints with given heading
      bool dubinsPath(const double Xs[3],const double Xf[3], std::vector<double[3]> &Path,bool &EndTurn,double &OCF[2])
      {
        //! Define turning directions
        bool RightS;
        bool RightF;
        //! Declare parameters
        //! Start circle center
        double Xcs;
        double Ycs;
        double OCS[2];
        //! Finish circle center
        double Xcf;
        double Ycf;
        //! Radius of secound end turning circle
        double Rsec;

        //! Define start turning circle center (Ocs)

        if (std::atan2(Xs[1]-Xf[1],Xs[0]-Xf[0])<0)
        {
          RightS = false;
          Xcs = Xs[0]-m_Rs*std::cos(Xs[2]-PI/2);
          Ycs = Xs[1]-m_Rs*std::sin(Xs[2]-PI/2);
        }
        else
        {
          RightS = true;
          Xcs = Xs[0]-m_Rs*std::cos(Xs[2]+PI/2);
          Ycs = Xs[1]-m_Rs*std::sin(Xs[2]+PI/2);
        }
        OCS[0] = Xcs;
        OCS[1] = Ycs;

        //! Define end turning circle center (Ofs)

        if (std::atan2(Xs[1]-Xf[1],Xs[0]-Xf[0])<0)
        {
          RightF = false;
          Xcf = Xf[0]-m_Rf*std::cos(Xf[2]-PI/2);
          Ycf = Xf[1]-m_Rf*std::sin(Xf[2]-PI/2);
        }
        else
        {
          RightF = true;
          Xcf = Xf[0]-m_Rf*std::cos(Xf[2]+PI/2);
          Ycf = Xf[1]-m_Rf*std::sin(Xf[2]+PI/2);
        }
        OCF[0] = Xcf;
        OCF[1] = Ycf;
        //! Calculate radius of second end turning circle
        Rsec = std::abs(m_Rf-m_Rs);

        //! Calculate the line between Ocs and Ofs
        double cbx = Xcs;
        double cax = Xcf - cbx;
        double cby = Ycs;
        double cay = Ycf - cby;

        //! Calculate the length of c
        double dc = std::sqrt(std::pow(cax,2)+std::pow(cay,2));
        //! Check that Dubins path exists
        if (Rsec>dc)
        {
          war("Dubins Path does not exist from start position to end position");
          return false;
        }
        //! Calculate distance from start pose to end pose
        double dXsXf = sqrt(std::pow(Xs[0]-Xf[0],2)+std::pow(Xs[1]-Xf[1],2));

        //! Check if the start pose and end pose is to close
        if (dXsXf<2*m_Rf)
        {
          war("The start pose and end pose are to close.");
          return false;
        }

        //! Calculate alpha
        double alpha = std::asin((m_Rf-m_Rs)/dc);

        //! Calculate beta
        double beta = std::atan2(Ycf-Ycs,Xcf-Xcs);

        //! Define tangent points
        //! First
        double thetaS = turn(RightS,alpha,beta);
        //! Second
        double thetaF = turn(RightF,alpha,beta);
        //! Exit tangent point for first circle
        double Pchi[2];
        Pchi[0] = Xcs+m_Rs*cos(thetaS);
        Pchi[1] = Ycs+m_Rs*sin(thetaS);
        //! Entry tangent point
        double PN[2];
        PN[0] = Xcf+m_Rf*cos(thetaF);
        PN[1] = Ycf+m_Rf*sin(thetaF);
        //! Define turning arc
        std::vector<double [2]> arc;
        //! Declare angle array
        double theta[m_N];
        //! First arc
        double theta0 = std::atan2(Xs[1]-Ycs,Xs[0]-Xcs);
        double theta1 = std::atan2(Pchi[1]-Ycs,Pchi[0]-Xcs);
        if (RightS)
        {
          if (Angles::normalizeRadian(theta1-PI)>=theta0 || (theta0>theta1 && sign(theta1)==sign(theta0)))
          {
            calculateTurningArcAngle(-std::abs(Angles::normalizeRadian(theta1-theta0)),theta);
          }
          else
          {
            calculateTurningArcAngle(-(2*PI-std::abs(Angles::normalizeRadian(theta1-theta0))),theta);
          }
        }
        else
        {
          if (Angles::normalizeRadian(theta1-PI)<=theta0 && sign(theta1-PI)==sign(theta0) || (theta0<theta1 && (sign(theta0)==sign(theta1) || theta0==0)))
          {
            calculateTurningArcAngle(std::abs(Angles::normalizeRadian(theta1-theta0)),theta);
          }
          else
          {
            calculateTurningArcAngle(2*PI-std::abs(Angles::normalizeRadian(theta1-theta0)),theta);
          }
        }
        ConstructArc(theta,theta0,m_Rs,OCS,arc);
        AddToPath(arc,Path);
        //! Second arc
        theta0 = std::atan2(PN[1]-Ycf,PN[0]-Xcf);
        theta1 = std::atan2(Xf[1]-Ycf,Xf[0]-Xcf);
        if (RightF)
        {
          if((Angles::normalizeRadian(theta1-PI)<=theta0) || (theta0>theta1 && (sign(theta1)==sign(theta0))))
          {
            calculateTurningArcAngle(-std::abs(Angles::normalizeRadian(theta1-theta0)),theta);
          }
          else
          {
            calculateTurningArcAngle(-(2*PI-std::abs(Angles::normalizeRadian(theta1-theta0))),theta);
          }
        }
        else
        {
          if ((Angles::normalizeRadian(theta1-PI)<=theta0 && sign(theta1-PI)==sign(theta0)) || (theta0<theta1 && sign(theta0)==sign(theta1)))
          {
            calculateTurningArcAngle(std::abs(Angles::normalizeRadian(theta1-theta0)),theta);
            }
            else
            {
              calculateTurningArcAngle(2*PI-std::abs(Angles::normalizeRadian(theta1-theta0)),theta);
            }
        }
        ConstructArc(theta,theta0,m_Rf,OCF,arc);
        AddToPath(arc,Path);
        return true;

      }
      //! Return turn direction
      double
      turn(const bool Right,const double alpha,const double beta)
      {
        if (Right)
        {
          return alpha+beta+PI/2;
        }
        else
        {
          return beta-alpha+(3*PI)/2;
        }
      }
      //! Return N angle from 0 theta
      void
      calculateTurningArcAngle(const double theta_limit,double &theta[m_N])
      {
        double step = theta_limit/(m_N-1);
        for (int i=0;i<m_N;i++)
        {
          theta[i]=i*theta_limit;
        }
      }
      //! Return the sign of a number. 0 is considered positive
      int
      sign(double x)
      {
        if (x<0)
          return -1;
        else
          return 1;
      }
      //! Constructs an arc from theta[0] to theta[m_N] with R radius
      void
      ConstructArc(const double theta[m_N],const double theta0,const double R,const double center[2],std::vector<double [2]> &arc)
      {
        double tempP[2];
        for (int i=0;i<m_N;i++)
        {
          tempP[0] = center[0] + R*std::cos(theta0+theta[i]);
          tempP[1] = center[1] + R*std::sin(theta0+theta[i]);
          arc.push_back(tempP);
        }

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
        }
      }
    };
  }
}

DUNE_TASK
