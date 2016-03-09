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
      //! Wait at loiter
      bool waitLoiter;

    };

    struct LandingPathArguments
    {
      //! Length of point behind the net
      double a0;
      //! Length of final approach
      double a1;
      //! Length of glideslope
      double a2;
      //! Length of approach
      double a3;
      //! Length of waypoint behind the nett
      double b1;
      //! Angle of attack
      double gamma_a;
      //! Angle of descent
      double gamma_d;
      //! Net position
      Matrix Net;
      //! Net orientation
      double netHeading;
      //! Landing waypoints
      Matrix WP;
      //! Auxiliary waypoint
      Matrix WPa;
      //! Net lat
      double net_lat;
      //! Net lon
      double net_lon;
      //! Net height
      double net_height;
      //! Net WGS84 height
      double net_WGS84_height;
      //! Speed WP4
      double speed_WP4;
      //! Speed WP3
      double speed_WP3;
      //! Speed WP3
      double speed_WP2;
      //! Speed WP2
      double speed_WP1;
      //! Speed WP1

    };

    struct Task: public DUNE::Tasks::Task
    {
      //! Task arguments
      Arguments m_arg;
      //! Landing path arguments
      LandingPathArguments m_landArg;
      //! Accumulated EstimatedState message
      IMC::EstimatedState m_estate;
      //! Start turning circle
      double m_Rs;
      //! Finish turning circle
      double m_Rf;
      //! Number of points in arc
      int m_N;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_Rs(0.0),
        m_Rf(0.0),
        m_N(0.0)
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
        m_estate = *msg;
      }
      //! Receive nett pose and landing spesifications
      void
      consume(const IMC::PlanGeneration *msg)
      {
        if (msg->op != IMC::PlanGeneration::OP_REQUEST)
        {
          return;
        }
        if (!extractPlan(msg))
        {
          return;
        }
        if (msg->plan_id=="land")
        {
          if(!generateLandingPath())
          {
            war("Unable to generate a landing path");
          }
        }
        else
        {
          //! Other Paths
        }

      }

      //! Extract information from a PlanGeneration message
      bool
      extractPlan(const IMC::PlanGeneration *msg)
      {
        //! Check if landing path container should be filled. It's assumed that input data is in NED
        if (msg->plan_id=="land")
        {
          TupleList tList(msg->params,"=",";",true);
          m_landArg.net_lat = Angles::radians(tList.get("land_lat",1.1105));
          m_landArg.net_lon = Angles::radians(tList.get("land_lon",0.1698));
          m_landArg.net_WGS84_height = tList.get("net_WGS84_height",0.0);
          m_landArg.netHeading = Angles::radians(tList.get("land_heading",1.1105));
          m_landArg.net_height = tList.get("net_height",3.0);
          m_landArg.gamma_a = Angles::radians(tList.get("attack_angle",0.0524));
          m_landArg.gamma_d = Angles::radians(tList.get("descend_angle", 0.0524));
          m_landArg.a0 = tList.get("behind_net",10.0);
          m_landArg.a1 = tList.get("final_approach",10.0);
          m_landArg.a2 = tList.get("glideslope",300.0);
          m_landArg.a3 = tList.get("approach",100.0);
          m_landArg.speed_WP4 = tList.get("speed_wp4",12.0);
          m_landArg.speed_WP3 = tList.get("speed_wp3",12.0);
          m_landArg.speed_WP2 = tList.get("speed_wp2",12.0);
          m_landArg.speed_WP1 = tList.get("speed_wp1",12.0);
          m_Rs = tList.get("min_turn_radius", 20.0);
          m_Rf = tList.get("loiter_radius",40.0);

          //! Fill WP matrix
          m_landArg.WP = Matrix(3,4,0.0);
          m_landArg.WP(0,0) = -m_landArg.a0;
          m_landArg.WP(2,0) = m_landArg.net_height+m_landArg.a0*std::tan(m_landArg.gamma_a);

          m_landArg.WP(0,1) = m_landArg.a1;
          m_landArg.WP(2,1) = m_landArg.net_height-m_landArg.a1*std::tan(m_landArg.gamma_a);

          m_landArg.WP(0,2) = m_landArg.WP(0,1)+m_landArg.a2;
          m_landArg.WP(2,2) = m_landArg.WP(2,1)-m_landArg.a2*std::tan(m_landArg.gamma_a);

          m_landArg.WP(0,3) = m_landArg.WP(0,2)+m_landArg.a3;
          m_landArg.WP(2,3) = m_landArg.WP(2,2);

          m_landArg.WPa = Matrix(3,1,0.0);
          m_landArg.WPa(0,0) = m_landArg.WP(0,1) + (m_landArg.a2)/2;
          m_landArg.WPa(1,0) = m_Rf;
          m_landArg.WPa(2,0) = m_landArg.WP(2,1)-m_landArg.a2*std::tan(m_landArg.gamma_a);

          //! Rotate all WP into NED
          m_landArg.WP.column(1) = Rzyx(0,0,m_landArg.netHeading)*m_landArg.WP.column(1);
          m_landArg.WP.column(2) = Rzyx(0,0,m_landArg.netHeading)*m_landArg.WP.column(2);
          m_landArg.WP.column(3) = Rzyx(0,0,m_landArg.netHeading)*m_landArg.WP.column(3);
          m_landArg.WP.column(4) = Rzyx(0,0,m_landArg.netHeading)*m_landArg.WP.column(4);

          m_landArg.WPa = Rzyx(0,0,m_landArg.netHeading)*m_landArg.WPa;

          return true;
        }
        return false;
      }
      //! Generates a landing path
      bool
      generateLandingPath()
      {
        //! Initialize the start pose
        Matrix Xs = Matrix(4,1,0.0);
        Xs(0,0) = m_estate.x;
        Xs(1,0) = m_estate.y;
        Xs(2,0) = m_estate.z;
        Xs(3,0) = m_estate.psi;
        //! Direction of end turn
        bool RightF;
        //! Center of end turning circle
        Matrix OCF = Matrix(2,1,0.0);
        //! End pose in dubins paht
        Matrix Xf = Matrix(4,1,0.0);
        Xf = m_landArg.WP.column(4);
        //! Calculated path
        std::vector<Matrix> m_path;
        if (!dubinsPath(Xs,Xf,m_path,RightF,OCF))
        {
          //! Need an extra WP
          Xf = m_landArg.WPa;
          if (!dubinsPath(Xs,Xf,m_path,RightF,OCF))
          {
            war("Could not generate a landing path: Abort");
            return false;
          }
          Xf = m_landArg.WP.column(4);
          if (!dubinsPath(m_landArg.WPa,Xf,m_path,RightF,OCF))
          {
            war("Could not generate a landing path: Abort");
            return false;
          }
        }
        //! Is correct height
        bool correctHeight;
        glideSlope(Xs,m_landArg.WP.column(4),m_landArg.gamma_d,correctHeight,m_path);
        glideSpiral(OCF,RightF,m_landArg.WP(2,3),correctHeight,m_landArg.gamma_d,m_path);

        //! Create plan set request
        IMC::PlanDB plan_db;
        plan_db.type = IMC::PlanDB::DBT_REQUEST;
        plan_db.op = IMC::PlanDB::DBOP_SET;
        plan_db.plan_id = "LandingPath";
        plan_db.request_id = 0;

        //! Create plan specification
        IMC::PlanSpecification plan_spec;
        plan_spec.plan_id = plan_db.plan_id;
        plan_spec.start_man_id = 1;
        plan_spec.description = "Plan activating LandingPath";

        //! Create a list of maneuvers
        IMC::MessageList<IMC::Maneuver> maneuverList;

        //! Create a followPath maneuver
        IMC::FollowPath fPath;
        double cur_lat;
        double cur_lon;
        Coordinates::WGS84::displace(m_estate.x,m_estate.y,&cur_lat,&cur_lon);
        fPath.lat = cur_lat;
        fPath.lon = cur_lon;
        fPath.z_units = IMC::Z_HEIGHT;
        fPath.z = m_estate.height - m_estate.z;
        addPathPoint(m_path,&fPath);
        maneuverList.push_back(fPath);

        //! Add loiter maneuver if the waitLoiter flag is set to true
        if (m_arg.waitLoiter)
        {
          //addLoiter();
        }
        else
        {
          //addNetApproach();
        }



        //! Create plan maneuver
        IMC::PlanManeuver man_spec;
        man_spec.maneuver_id = 1;

        //! Add a maneuver list to a plan
        // addMnaeuverToPlan(maneuverList,&man_spec);

        man_spec.data.set(fPath);

        plan_spec.maneuvers.push_back(man_spec);

        plan_db.arg.set(plan_spec);

        //! Send set plan request
        dispatch(plan_db);

        //! Create and send plan start request
        IMC::PlanControl plan_ctrl;
        plan_ctrl.type = IMC::PlanControl::PC_REQUEST;
        plan_ctrl.op = IMC::PlanControl::PC_START;
        plan_ctrl.plan_id = plan_spec.plan_id;
        plan_ctrl.request_id = 0;
        plan_ctrl.arg.set(plan_spec);
        dispatch(plan_ctrl);

        return true;

      }
      //! Add the landing approach toward the net
      void
      addNetApproach()
      {
        //3
        /*IMC::Goto* gotoWP = new IMC::Goto();
        double w3_lat;
        double w3_lon;
        double w3_h = m_landArg.net_WGS84_height - m_landArg.WP(2,2);
        Coordinates::WGS84::displace(m_landArgman_spec.WP(0,2),m_landArg.WP(1,2),&w3_lat,&w3_lon);
        gotoWP->lat = w3_lat;
        gotoWP->lon = w3_lon;
        gotoWP->z = w3_h;
        gotoWP->z_units = IMC::Z_HEIGHT;
        gotoWP->speed = m_landArg.speed_WP3;
        gotoWP->speed_units = IMC::SUNITS_METERS_PS;
        m_maneuvers.push_back(*gotoWP);
        delete gotoWP;

        //2
        IMC::Goto* gotoWP = new IMC::Goto();
        double w2_lat;
        double w2_lon;
        double w2_h = m_landArg.net_WGS84_height - m_landArg.WP(2,1);
        Coordinates::WGS84::displace(m_landArg.WP(0,1),m_landArg.WP(1,1),&w2_lat,&w2_lon);
        gotoWP->lat = w2_lat;
        gotoWP->lon = w2_lon;
        gotoWP->z = w2_h;
        gotoWP->z_units = IMC::Z_HEIGHT;
        gotoWP->speed = m_landArg.speed_WP2;
        gotoWP->speed_units = IMC::SUNITS_METERS_PS;
        m_maneuvers.push_back(*gotoWP);
        delete gotoWP;

        //1
        IMC::Goto* gotoWP = new IMC::Goto();
        double w1_lat;
        double w1_lon;
        double w1_h = m_landArg.net_WGS84_height - m_landArg.WP(2,0);
        Coordinates::WGS84::displace(m_landArg.WP(0,0),m_landArg.WP(1,0),&w1_lat,&w1_lon);
        gotoWP->lat = w1_lat;
        gotoWP->lon = w1_lon;
        gotoWP->z = w1_h;
        gotoWP->z_units = IMC::Z_HEIGHT;
        gotoWP->speed = m_landArg.speed_WP1;
        gotoWP->speed_units = IMC::SUNITS_METERS_PS;
        m_maneuvers.push_back(*gotoWP);
        delete gotoWP;*/


      }
      //! Add path point to follow path
      void
      addPathPoint(std::vector<Matrix> m_path,IMC::FollowPath* fPath)
      {
        IMC::PathPoint pPoint;
        for (int i=0;i<m_path.size();i++)
        {
          pPoint.x = m_path[i](0,0);
          pPoint.y = m_path[i](1,0);
          pPoint.z = m_path[i](2,0);
          fPath->points.push_back(pPoint);
        }
      }
      //! Construct Dubins Path between two waypoints with given heading
      bool
      dubinsPath(const Matrix Xs,const Matrix Xf, std::vector<Matrix> Path,bool &RightF,Matrix &OCF)
      {
        //! Define start turning direction
        bool RightS;
        //! Declare parameters
        //! Start circle center
        double Xcs;
        double Ycs;
        Matrix OCS = Matrix(2,1,0.0);
        //! Finish circle center
        double Xcf;
        double Ycf;
        //! Radius of second end turning circle
        double Rsec;

        //! Define start turning circle center (Ocs)

        if (std::atan2(Xs(1,0)-Xf(1,0),Xs(0,0)-Xf(0,0))<0)
        {
          RightS = false;
          Xcs = Xs(0,0)-m_Rs*std::cos(Xs(3,0)-PI/2);
          Ycs = Xs(1,0)-m_Rs*std::sin(Xs(3,0)-PI/2);
        }
        else
        {
          RightS = true;
          Xcs = Xs(0,0)-m_Rs*std::cos(Xs(3,0)+PI/2);
          Ycs = Xs(1,0)-m_Rs*std::sin(Xs(3,0)+PI/2);
        }
        OCS(0,0) = Xcs;
        OCS(1,0) = Ycs;

        //! Define end turning circle center (Ofs)

        if (std::atan2(Xs(1,0)-Xf(1,0),Xs(0,0)-Xf(0,0))<0)
        {
          RightF = false;
          Xcf = Xf(0,0)-m_Rf*std::cos(Xf(3,0)-PI/2);
          Ycf = Xf(1,0)-m_Rf*std::sin(Xf(3,0)-PI/2);
        }
        else
        {
          RightF = true;
          Xcf = Xf(0,0)-m_Rf*std::cos(Xf(3,0)+PI/2);
          Ycf = Xf(1,0)-m_Rf*std::sin(Xf(3,0)+PI/2);
        }
        OCF(0,0) = Xcf;
        OCF(1,0) = Ycf;
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
        double dXsXf = sqrt(std::pow(Xs(0,0)-Xf(0,0),2)+std::pow(Xs(1,0)-Xf(1,0),2));

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
        Matrix Pchi = Matrix(2,1,0.0);
        Pchi(0,0) = Xcs+m_Rs*cos(thetaS);
        Pchi(1,0) = Ycs+m_Rs*sin(thetaS);
        //! Entry tangent point
        Matrix PN = Matrix(2,1,0.0);
        PN(0,0) = Xcf+m_Rf*cos(thetaF);
        PN(1,0) = Ycf+m_Rf*sin(thetaF);
        //! Define turning arc
        std::vector<Matrix> arc;
        //! Declare angle array
        Matrix theta =Matrix(1,m_N,0.0);
        //! First arc
        double theta0 = std::atan2(Xs(1,0)-Ycs,Xs(0,0)-Xcs);
        double theta1 = std::atan2(Pchi(1,0)-Ycs,Pchi(0,0)-Xcs);
        if (RightS)
        {
          if (Angles::normalizeRadian(theta1-theta0)<=0)
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
          if (Angles::normalizeRadian(theta1-theta0)>=0)
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
        theta0 = std::atan2(PN(1,0)-Ycf,PN(0,0)-Xcf);
        theta1 = std::atan2(Xf(1,0)-Ycf,Xf(0,0)-Xcf);
        if (RightF)
        {
          if(Angles::normalizeRadian(theta1-theta0)<=0)
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
          if (Angles::normalizeRadian(theta1-theta0)>=0)
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
      calculateTurningArcAngle(const double theta_limit,Matrix &theta)
      {
        double step = theta_limit/(m_N-1);
        for (int i=0;i<m_N;i++)
        {
          theta(0,i)=i*step;
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
      ConstructArc(const Matrix theta,const double theta0,const double R,const Matrix center,std::vector<Matrix> arc)
      {
        Matrix tempP = Matrix(3,1,0.0);
        for (int i=0;i<m_N;i++)
        {

          tempP(0,0) = center(0,0) + R*std::cos(theta0+theta(0,i));
          tempP(1,0) = center(1,0) + R*std::sin(theta0+theta(0,i));
          arc.push_back(tempP);
        }

      }
      //! Add an arc to the path.
      void
      AddToPath(std::vector<Matrix> arc,std::vector<Matrix> &path)
      {
        std::vector<Matrix>::iterator it;
        for (it=arc.begin();it!=arc.end();it++)
        {
          path.push_back(*it);
        }
        //! Empty arc
        arc.erase(arc.begin(),arc.end());

      }
      //! Constructs a glideslope from x0 towards height of loiter point along dubins path
      void
      glideSlope(const Matrix x0,const Matrix WP,double descentAngle,bool &correctHeigth,std::vector<Matrix> &Path)
      {
        //! The path starts at the same height as x0
        Path[1](2,0) = x0(2,0);
        correctHeigth = false;
        double D;
        int interationLength = Path.size()-1;
        for (int i=0;i<interationLength;i++)
        {
          D = sqrt(std::pow(Path[i+1](0,0)-Path[i](0,0),2)+std::pow(Path[i+1](1,0)-Path[i](1,0),2));
          if (std::abs(std::atan2(WP(2,3)-Path[i](2,0),D))<abs(descentAngle))
          {
            correctHeigth = true;
            descentAngle = std::atan2(WP(2,3)-Path[i](2,0),D);
            Path[i+1](2,0) = Path[i](2,0)+D*tan(descentAngle);
          }
          else if (!correctHeigth)
          {
            Path[i+1](2,0) = Path[i](2,0) + D*tan(descentAngle);
          }
          else
          {
            Path[i+1](2,0) = Path[i](2,0);
          }
        }
      }
      //! Create a spiral path towards the desired height dHeight
      void
      glideSpiral(const Matrix OF,const bool RightF,const double dHeight,bool &correctHeigth, double descentAngle,std::vector<Matrix> &Path)
      {
        if(correctHeigth)
        {
          return;
        }
        double theta0 = std::atan2(Path[Path.size()-1](1,0)-OF(1,0),Path[Path.size()-1](0,0)-OF(0,0));
        Matrix WP4 = Path.back();
        Matrix theta = Matrix(1,m_N);
        if (RightF)
        {
          calculateTurningArcAngle(-2*PI,theta);
        }
        else
        {
          calculateTurningArcAngle(2*PI,theta);
        }
        Matrix WPS0 = Path.back();
        double xnn = OF(0,0) + m_Rf*cos(theta0+theta(0,1));
        double ynn = OF(1,0) + m_Rf*sin(theta0+theta(0,1));
        double D = std::sqrt(std::pow(xnn-WPS0(0,0),2)+std::pow(ynn-WPS0(1,0),2));
        double znn = WPS0(2,0)+D*std::tan(descentAngle);
        Matrix WPS1 = Matrix(3,1,0.0);
        WPS1(0,0) = xnn;
        WPS1(1,0) = ynn;
        WPS1(2,0) = znn;
        int n = 3;
        Path.push_back(WPS0);
        Path.push_back(WPS1);
        while(!correctHeigth)
        {
          if (std::abs(std::atan2(dHeight-WPS1(2,0),D))<abs(descentAngle))
          {
            descentAngle = std::atan2(dHeight-WPS1(2,0),D);
            correctHeigth = true;
          }
          WPS0 = WPS1;
          xnn = OF(0,0) + m_Rf*cos(theta0+theta(0,n));
          ynn = OF(1,0) + m_Rf*sin(theta0+theta(0,n));
          D = std::sqrt(std::pow(xnn-WPS0(0,0),2)+std::pow(ynn-WPS0(1,0),2));
          znn = WPS0(2,0)+D*std::tan(descentAngle);
          WPS1(0,0) = xnn;
          WPS1(1,0) = ynn;
          WPS1(2,0) = znn;
          Path.push_back(WPS1);
          n = n+1;
          //! Check if n has reached m_N. Then set to 1, such that the 0 value is only used once
          if (n>m_N)
          {
            n = 1;
          }
        }
        double thetaH0 = std::atan2(WPS1(1,0)-OF(1,0),WPS1(0,0)-OF(0,0));
        double thetaH1 = std::atan2(WP4(1,0)-OF(1,0),WP4(0,0)-OF(0,0));
        std::vector<Matrix> arc;
        if (RightF)
        {
          if (Angles::normalizeRadian(thetaH1-thetaH0)<=0)
          {
            calculateTurningArcAngle(-std::abs(Angles::normalizeRadian(thetaH1-thetaH0)),theta);
          }
          else
          {
            calculateTurningArcAngle(-(2*PI-std::abs(Angles::normalizeRadian(thetaH1-thetaH0))),theta);
          }
        }
        else
        {
          if (Angles::normalizeRadian(thetaH1-thetaH0)>=0)
          {
            calculateTurningArcAngle(std::abs(Angles::normalizeRadian(thetaH1-thetaH0)),theta);
          }
          else
          {
            calculateTurningArcAngle((2*PI-std::abs(Angles::normalizeRadian(thetaH1-thetaH0))),theta);
          }
        }
        ConstructArc(theta,thetaH0,m_Rf,OF,arc);
        AddToPath(arc,Path);
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
      //! @return  Rotation matrix.
      Matrix
      Rzyx(double phi, double theta, double psi) const
      {
        double R_en_elements[] =
          { cos(psi) * cos(theta), (-sin(psi) * cos(phi))
              + (cos(psi) * sin(theta) * sin(phi)), (sin(psi) * sin(phi))
              + (cos(psi) * cos(phi) * sin(theta)), sin(psi) * cos(theta), (cos(
              psi) * cos(phi)) + (sin(phi) * sin(theta) * sin(psi)), (-cos(psi)
              * sin(phi)) + (sin(theta) * sin(psi) * cos(phi)), -sin(theta),
              cos(theta) * sin(phi), cos(theta) * cos(phi) };
        return Matrix(R_en_elements, 3, 3);
      }
    };
  }
}

DUNE_TASK
