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
      //! Arc segment distance
      double arc_segment_distance;
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
      //! The side the auxiliary wp
      bool RightWPa;
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
      //! Speed WP2
      double speed_WP2;
      //! Speed WP1
      double speed_WP1;
      //! Start turning circle radius
      double Rs;
      //! Finish turning circle radius
      double Rf;
      //! Waiting at loiter
      bool wait_at_loiter = false;
      //! Advance options
      //! Automatic generation of start and finish circle
      bool automatic;
      //! Right start turning direction
      bool rightStartTurningDirection;
      //! Right finish turning direction
      bool rightFinishTurningCircle;

    };

    struct LandingPath
    {
      //! Landing waypoints
      //! WP1
      Matrix WP1;
      //! WP1
      Matrix WP2;
      //! WP1
      Matrix WP3;
      //! WP1
      Matrix WP4;
      //! Auxiliary waypoint
      Matrix WPa;
      //! Finish turning circle rotation
      bool clockwise;
      //! Finish turning circle center
      Matrix OCF;
      //! Start position
      Matrix Xs;
      double state_lat;
      double state_lon;
      double state_height;
      //! Finish turning circle offset height
      double OCFz;
    };

    struct DubinsParametersContainer
    {
      //! Rotation direction for the start circle
      bool CounterClockwiseS;
      //! Rotation direction for the final circle
      bool CounterClockwiseF;
      //! X-coordinate for the start circle
      double Xcs;
      //! Y-coordinate for the start circle
      double Ycs;
      //! Matrix containing the center of the start circle
      Matrix OCS;
      //! X-coordinate for the final circle
      double Xcf;
      //! Y-coordinate for the final circle
      double Ycf;
      //! Matrix containing the center of the final circle
      Matrix OCF;
      //! Matrix containing the exit tangent point for arc on the start circle
      Matrix Pchi;
      //! Matrix containing the entry tangent point for the arc on the final circle
      Matrix PN;
      //! Length of the dubins path
      double LengthPath;
    };

    struct Task: public DUNE::Tasks::Task
    {
      //! Task arguments
      Arguments m_args;
      //! Landing path arguments
      LandingPathArguments m_landArg;
      //! Landing path parameters
      LandingPath m_landParameteres;
      //! Vector containing information about all variants of dubins path
      std::vector<DubinsParametersContainer> m_dubinsPaths;
      //! Accumulated EstimatedState message
      IMC::EstimatedState m_estate;
      //! Segments in the start circle
      int m_Ns;
      //! Segments in the finish circle
      int m_Nf;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx)
      {
        param("Wait At Loiter", m_args.waitLoiter)
        .visibility(Tasks::Parameter::VISIBILITY_USER)
        .defaultValue("false")
        .description("Enable Wait At Loiter");

        param("Distance Between Arc Segment",m_args.arc_segment_distance)
        .visibility(Tasks::Parameter::VISIBILITY_USER)
        .defaultValue("20")
        .description("Distance Between Arc Segment");

        //! Initialize the default values
        TupleList tList("","=",";",true);
        readTupleList(tList);
        bind<IMC::EstimatedState>(this);
        bind<IMC::PlanGeneration>(this);
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        m_Ns = std::floor((2*m_landArg.Rs*PI)/m_args.arc_segment_distance);
        m_Nf = std::floor((2*m_landArg.Rf*PI)/m_args.arc_segment_distance);

        //! Check if m_Ns and m_Nf is bellow 4
        if (m_Ns<4)
        {
          m_Ns = 4;
        }
        if (m_Nf<4)
        {
          m_Nf = 4;
        }
        if (!m_args.waitLoiter && m_landArg.wait_at_loiter)
        {

          //! Create plan set request
          IMC::PlanDB plan_db;
          plan_db.type = IMC::PlanDB::DBT_REQUEST;
          plan_db.op = IMC::PlanDB::DBOP_SET;
          plan_db.plan_id = "land";
          plan_db.request_id = 0;

          //! Create plan specification
          IMC::PlanSpecification plan_spec;
          plan_spec.plan_id = plan_db.plan_id;
          plan_spec.start_man_id = 1;
          plan_spec.description = "Plan activating land";
          //! Create a list of maneuvers
          IMC::MessageList<IMC::Maneuver> maneuverList;

          addNetApproach(maneuverList);

          //! Add a maneuver list to a plan
          addManeuverListToPlan(&maneuverList,plan_spec);

          plan_db.arg.set(plan_spec);

          //! Send set plan request
          dispatch(plan_db);

          //! Create and send plan start request
          IMC::PlanControl plan_ctrl;
          plan_ctrl.type = IMC::PlanControl::PC_REQUEST;
          plan_ctrl.op = IMC::PlanControl::PC_LOAD;
          plan_ctrl.plan_id = plan_spec.plan_id;
          plan_ctrl.request_id = 0;
          plan_ctrl.arg.set(plan_spec);
          dispatch(plan_ctrl);
          m_landArg.wait_at_loiter = false;
        }
      }

      //! Update estimatedState
      void
      consume(const IMC::EstimatedState *msg)
      {
        m_estate = *msg;
      }
      //! Receive net pose and landing spesifications
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
          inf("Generate a landing path");
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

          readTupleList(tList);

          m_Ns = std::floor((2*m_landArg.Rs*PI)/m_args.arc_segment_distance);
          m_Nf = std::floor((2*m_landArg.Rf*PI)/m_args.arc_segment_distance);

          //! Check if m_Ns and m_Nf is bellow 4
          if (m_Ns<4)
          {
            m_Ns = 4;
          }
          if (m_Nf<4)
          {
            m_Nf = 4;
          }

          inf("m_Ns = %d m_Nf = %d",m_Ns,m_Nf);

          //! Fill WP matrix
          m_landParameteres.WP1 = Matrix(3,1,0.0);
          m_landParameteres.WP1(0,0) = -m_landArg.a0;
          m_landParameteres.WP1(2,0) = m_landArg.net_height+m_landArg.a0*std::tan(m_landArg.gamma_a);

          m_landParameteres.WP2 = Matrix(3,1,0.0);
          m_landParameteres.WP2(0,0) = m_landArg.a1;
          m_landParameteres.WP2(2,0) = m_landArg.net_height-m_landArg.a1*std::tan(m_landArg.gamma_a);

          m_landParameteres.WP3 = Matrix(3,1,0.0);
          m_landParameteres.WP3(0,0) = m_landParameteres.WP2(0,0)+m_landArg.a2;
          m_landParameteres.WP3(2,0) = m_landParameteres.WP2(2,0)-m_landArg.a2*std::tan(m_landArg.gamma_a);

          m_landParameteres.WP4 = Matrix(3,1,0.0);
          m_landParameteres.WP4(0,0) = m_landParameteres.WP3(0,0)+m_landArg.a3;
          m_landParameteres.WP4(2,0) = m_landParameteres.WP3(2,0);

          m_landParameteres.WPa = Matrix(3,1,0.0);
          m_landParameteres.WPa(0,0) = m_landParameteres.WP1(0,0);
          //! Ensure that the extra wp is valid
          if (m_landArg.RightWPa)
          {
            m_landParameteres.WPa(1,0) = 2*m_landArg.Rf;
          }
          else
          {
            m_landParameteres.WPa(1,0) = -2*m_landArg.Rf;
          }

          m_landParameteres.WPa(2,0) = m_landParameteres.WP2(2,0)-m_landArg.a2*std::tan(m_landArg.gamma_a);

          //! Rotate all WP into NED
          m_landParameteres.WP1 = Rzyx(0,0,m_landArg.netHeading)*m_landParameteres.WP1;
          m_landParameteres.WP2 = Rzyx(0,0,m_landArg.netHeading)*m_landParameteres.WP2;
          m_landParameteres.WP3 = Rzyx(0,0,m_landArg.netHeading)*m_landParameteres.WP3;
          m_landParameteres.WP4 = Rzyx(0,0,m_landArg.netHeading)*m_landParameteres.WP4;
          m_landParameteres.WPa = Rzyx(0,0,m_landArg.netHeading)*m_landParameteres.WPa;

          inf("WP1 x=%f y=%f z=%f",m_landParameteres.WP1(0,0),m_landParameteres.WP1(1,0),m_landParameteres.WP1(2,0));
          inf("WP2 x=%f y=%f z=%f",m_landParameteres.WP2(0,0),m_landParameteres.WP2(1,0),m_landParameteres.WP2(2,0));
          inf("WP3 x=%f y=%f z=%f",m_landParameteres.WP3(0,0),m_landParameteres.WP3(1,0),m_landParameteres.WP3(2,0));
          inf("WP4 x=%f y=%f z=%f",m_landParameteres.WP4(0,0),m_landParameteres.WP4(1,0),m_landParameteres.WP4(2,0));

          return true;
        }
        return false;
      }
      //! Read tuplelist
      void
      readTupleList(TupleList tList)
      {
        m_landArg.rightStartTurningDirection = (tList.get("right_start_turning_direction") == "true") ? true : false;
        m_landArg.rightFinishTurningCircle = (tList.get("right_finish_turning_circle") == "true") ? true : false;
        m_landArg.net_lat = Angles::radians(tList.get("land_lat",63.629409));
        m_landArg.net_lon = Angles::radians(tList.get("land_lon",9.726401));
        m_landArg.net_WGS84_height = tList.get("net_WGS84_height",0.0);
        m_landArg.netHeading = Angles::radians(tList.get("land_heading",60));
        m_landArg.net_height = tList.get("net_height",-3.0);
        m_landArg.gamma_a = Angles::radians(tList.get("attack_angle",3.0));
        m_landArg.gamma_d = Angles::radians(tList.get("descend_angle", 3.0));
        m_landArg.a0 = tList.get("dist_behind",10.0);
        m_landArg.a1 = tList.get("final_approach",10.0);
        m_landArg.a2 = tList.get("glideslope",300.0);
        m_landArg.a3 = tList.get("approach",100.0);
        m_landArg.speed_WP4 = tList.get("speed345",12.0);
        m_landArg.speed_WP3 = tList.get("speed345",12.0);
        m_landArg.speed_WP2 = tList.get("speed12",12.0);
        m_landArg.speed_WP1 = tList.get("speed12",12.0);
        m_landArg.Rs = tList.get("min_turn_radius", 150.0);
        m_landArg.Rf = tList.get("loiter_radius",150.0);
        m_landArg.RightWPa = (tList.get("auxiliary_WPa_side") == "true") ? true:false;
        m_landArg.automatic = (tList.get("automatic") == "true") ? true : false;
        debug("Content from tList:");
        debug("Net Lat %f",m_landArg.net_lat);
        debug("Net lon %f",m_landArg.net_lon);
        debug("Net WGS %f",m_landArg.net_WGS84_height);
        debug("Net Height %f",m_landArg.net_height);
        debug("Net heading %f", m_landArg.netHeading);
        debug("Attack angle %f",m_landArg.gamma_a);
        debug("Descent %f",m_landArg.gamma_d);
        debug("A0 %f",m_landArg.a0);
        debug("A1 %f",m_landArg.a1);
        debug("A2 %f",m_landArg.a2);
        debug("A3 %f",m_landArg.a3);
        debug("Speed 35 %f",m_landArg.speed_WP4);
        debug("Speed 12 %f",m_landArg.speed_WP1);
        debug("Rs %f",m_landArg.Rs);
        debug("Rf %f",m_landArg.Rf);
        debug("Automatic %d",m_landArg.automatic);
        debug("Right start dir %d",m_landArg.rightStartTurningDirection);
        debug("Right finish %d",m_landArg.rightFinishTurningCircle);
        debug("WPA %d",m_landArg.RightWPa);
        inf("Extracted arguments from neptus");
      }
      //! Generates a landing path
      bool
      generateLandingPath()
      {

        std::vector<Matrix> path;
        if (!createPath(path))
          return false;

        //! Create plan set request
        IMC::PlanDB plan_db;
        plan_db.type = IMC::PlanDB::DBT_REQUEST;
        plan_db.op = IMC::PlanDB::DBOP_SET;
        plan_db.plan_id = "land";
        plan_db.request_id = 0;

        //! Create plan specification
        IMC::PlanSpecification plan_spec;
        plan_spec.plan_id = plan_db.plan_id;
        plan_spec.start_man_id = 1;
        plan_spec.description = "Plan activating land";

        //! Create a list of maneuvers
        IMC::MessageList<IMC::Maneuver> maneuverList;

        //! Create a followPath maneuver
        IMC::FollowPath fPath;

        /*double cur_lat = m_landArg.net_lat;
        double cur_lon =  m_landArg.net_lon;
        double cur_height = m_landArg.net_height;
        Coordinates::WGS84::displace(m_landArg.Xs(0,0),m_landArg.Xs(1,0),m_landArg.Xs(2,0),&cur_lat,&cur_lon,&cur_height);*/
        fPath.lat = m_landParameteres.state_lat;
        fPath.lon = m_landParameteres.state_lon;
        fPath.z = m_landParameteres.state_height;

        inf("Current lat: %f current lon: %f current height: %f",fPath.lat,fPath.lon,fPath.z);
        fPath.z_units = IMC::Z_HEIGHT;
        fPath.speed = m_landArg.speed_WP1;
        fPath.speed_units = IMC::SUNITS_METERS_PS;
        addPathPoint(path,&fPath);
        maneuverList.push_back(fPath);

        //! Add loiter maneuver if the waitLoiter flag is set to true
        if (m_args.waitLoiter)
        {
          addLoiter(maneuverList);
          m_landArg.wait_at_loiter = true;
        }
        else
        {
          addNetApproach(maneuverList);
        }

        //! Add a maneuver list to a plan
        addManeuverListToPlan(&maneuverList,plan_spec);

        plan_db.arg.set(plan_spec);

        //! Send set plan request
        dispatch(plan_db);

        //! Create and send plan start request
        IMC::PlanControl plan_ctrl;
        plan_ctrl.type = IMC::PlanControl::PC_REQUEST;
        plan_ctrl.op = IMC::PlanControl::PC_LOAD;
        plan_ctrl.plan_id = plan_spec.plan_id;
        plan_ctrl.request_id = 0;
        plan_ctrl.arg.set(plan_spec);
        dispatch(plan_ctrl);

        return true;

      }
      //! Create path towards the start approach
      bool
      createPath(std::vector<Matrix>& path)
      {
        //! Initialize the start pose
        Matrix Xs = Matrix(4,1,0.0);
        Xs(0,0) = m_estate.x;
        Xs(1,0) = m_estate.y;
        Xs(2,0) = m_estate.z;
        Xs(3,0) = m_estate.psi;

        //! Direction of end turn
        bool CounterClockwiseF;
        //! Center of final turning circle
        Matrix OCF = Matrix(2,1,0.0);
        //! End pose in dubins path
        Matrix Xf = Matrix(4,1,0.0);
        Xf(0,0) = m_landParameteres.WP4(0,0);
        Xf(1,0) = m_landParameteres.WP4(1,0);
        Xf(2,0) = m_landParameteres.WP4(2,0);
        Xf(3,0) = Angles::normalizeRadian(m_landArg.netHeading-PI);

        double wp4_lat = m_landArg.net_lat;
        double wp4_lon = m_landArg.net_lon;
        double wp4_h = m_landArg.net_WGS84_height - m_landParameteres.WP4(2,0);

        double state_lat = m_estate.lat;
        double state_lon = m_estate.lon;
        double state_height = m_estate.height;
        //! Find wp4 lat lon
        Coordinates::WGS84::displace(m_landParameteres.WP4(0,0),m_landParameteres.WP4(1,0),&wp4_lat,&wp4_lon);
        Coordinates::WGS84::displace(m_estate.x,m_estate.y,m_estate.z,&state_lat,&state_lon,&state_height);
        //! Find m_estate coordinates relative to net lat lon
        Coordinates::WGS84::displacement(m_landArg.net_lat,m_landArg.net_lon,m_landArg.net_height,state_lat,state_lon,state_height,&Xs(0,0),&Xs(1,0),&Xs(2,0));
        Coordinates::WGS84::displacement(state_lat,state_lon,state_height,wp4_lat,wp4_lon,wp4_h,&Xf(0,0),&Xf(1,0),&Xf(2,0));
        debug("Xf x=%f y=%f z=%f psi=%f",Xf(0,0),Xf(1,0),Xf(2,0),Xf(3,0));
        debug("Xs x=%f y=%f z=%f psi=%f",Xs(0,0),Xs(1,0),Xs(2,0),Xs(3,0));
        debug("m_estate height: %f net height: %f",m_estate.height,m_landArg.net_WGS84_height);
        m_landParameteres.Xs = Xs;
        m_landParameteres.state_lat = state_lat;
        m_landParameteres.state_lon = state_lon;
        m_landParameteres.state_height = state_height;
        Xs(0,0) = 0.0;
        Xs(1,0) = 0.0;
        Xs(2,0) = 0.0;
        //! Calculated path
        bool createdPath = dubinsPath(Xs,Xf,path,CounterClockwiseF,OCF);

        if (!createdPath)
        {
          err("Could not generate a landing path: Abort");
          return false;
        }
        inf("Dubins path has been created");
        //! Create a lateral path
        lateralPath(Xs,Xf,OCF,CounterClockwiseF,path);
        inf("Created lateral path");
        //! Store parameter that has been used in the construction of the path
        m_landParameteres.OCF = OCF;
        m_landParameteres.clockwise = !CounterClockwiseF;
        m_landParameteres.OCFz = Xf(2,0);
        debug("Reach correct height %f from the height %f",path[path.size()-1](2,0),Xs(2,0));
        debug("WP4 = h-z %f",m_landArg.net_WGS84_height-m_landParameteres.WP4(2,0));
        debug("Lat %f lon %f ref height %f",m_landArg.net_lat,m_landArg.net_lon,m_landArg.net_WGS84_height);
        return true;
      }

      //! Create a lateral path to be added to the Dubins path
      void
      lateralPath(Matrix Xs,Matrix Xf,Matrix OCF,bool CounterClokwiseF,std::vector<Matrix> &path)
      {
        if (Xf(2,0)<Xs(2,0))
        {
          m_landArg.gamma_d = -m_landArg.gamma_d;
        }
        bool reachedCorrectHeight = createGlideSlope(Xs,Xf,path);
        if (!reachedCorrectHeight)
        {
          createSpiral(OCF,CounterClokwiseF,Xf(2,0),path);
        }
      }

      //! Extract maneuvers from a list and add them to plan maneuver
      void
      addManeuverListToPlan(IMC::MessageList<IMC::Maneuver>* maneuverList,IMC::PlanSpecification &plan_spec)
      {
        IMC::MessageList<IMC::Maneuver>::const_iterator it;
        IMC::PlanManeuver last_man;
        unsigned i = 1;
        for (it = maneuverList->begin();it!=maneuverList->end();it++,i++)
        {
          IMC::PlanManeuver man_spec;

          man_spec.data.set(*it);
          man_spec.maneuver_id = i;

          if (it!=maneuverList->begin())
          {
            IMC::PlanTransition trans;
            trans.conditions = "maneuverIsDone";
            trans.dest_man = man_spec.maneuver_id;
            trans.source_man = last_man.maneuver_id;
            plan_spec.transitions.push_back(trans);
          }
          last_man = man_spec;
          plan_spec.maneuvers.push_back(man_spec);
        }

      }
      //! Add a loiter point after dubins
      void
      addLoiter(IMC::MessageList<IMC::Maneuver>& maneuverList)
      {
        IMC::Loiter loiter;
        double loiter_lat = m_landParameteres.state_lat;
        double loiter_lon = m_landParameteres.state_lon;
        double loiter_h = m_landParameteres.state_height;
        Coordinates::WGS84::displace(m_landParameteres.OCF(0,0),m_landParameteres.OCF(1,0),m_landParameteres.OCFz,&loiter_lat,&loiter_lon,&loiter_h);
        loiter.lat = loiter_lat;
        loiter.lon = loiter_lon;
        loiter.z = loiter_h;
        loiter.z_units = IMC::Z_HEIGHT;
        loiter.speed = m_landArg.speed_WP2;
        loiter.speed_units = IMC::SUNITS_METERS_PS;
        loiter.type = IMC::Loiter::LT_CIRCULAR;
        if (m_landParameteres.clockwise)
        {
          loiter.direction = IMC::Loiter::LD_CLOCKW;
        }
        else
        {
          loiter.direction = IMC::Loiter::LD_CCLOCKW;
        }
        loiter.radius = m_landArg.Rf;
        loiter.duration = 0;
        loiter.setSubId(10);
        maneuverList.push_back(loiter);
      }
      //! Add the landing approach toward the net
      void
      addNetApproach(IMC::MessageList<IMC::Maneuver>& maneuverList)
      {
        //3
        addGotoPoint(m_landParameteres.WP3,m_landArg.speed_WP2,maneuverList);

        //2
        addGotoPoint(m_landParameteres.WP2,m_landArg.speed_WP3,maneuverList);

        //1
        addGotoPoint(m_landParameteres.WP1,m_landArg.speed_WP4,maneuverList);

      }
      //!
      void
      addGotoPoint(const Matrix WP,const double speed,IMC::MessageList<IMC::Maneuver>& maneuverList)
      {
        IMC::Goto gotoWP;
        double w1_lat = m_landArg.net_lat;
        double w1_lon = m_landArg.net_lon;
        double w1_h = m_landArg.net_WGS84_height - WP(2,0);
        Coordinates::WGS84::displace(WP(0,0),WP(1,0),&w1_lat,&w1_lon);
        gotoWP.lat = w1_lat;
        gotoWP.lon = w1_lon;
        gotoWP.z = w1_h;
        gotoWP.z_units = IMC::Z_HEIGHT;
        gotoWP.speed = speed;
        gotoWP.speed_units = IMC::SUNITS_METERS_PS;
        maneuverList.push_back(gotoWP);
      }
      //! Add path point to follow path
      void
      addPathPoint(std::vector<Matrix> path,IMC::FollowPath* fPath)
      {
        IMC::PathPoint pPoint;
        for (unsigned i=0;i<path.size();i++)
        {
          pPoint.x = path[i](0,0);
          pPoint.y = path[i](1,0);
          //! Must invert due to dune
          pPoint.z = -1*path[i](2,0);
          debug("x = %f y = %f z = %f",pPoint.x,pPoint.y,pPoint.z);
          fPath->points.push_back(pPoint);
        }
      }
      //! Construct Dubins Path between two waypoints with given heading
      bool
      dubinsPath(const Matrix Xs,const Matrix Xf, std::vector<Matrix>& Path,bool &CounterClockwiseF,Matrix &OCF)
      {
        //! Define start turning direction
        bool CounterClockwiseS;
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
        //! Exit tangent point on start circle
        Matrix Pchi = Matrix(2,1,0.0);
        //! Entry tangent point on finish circle
        Matrix PN = Matrix(2,1,0.0);

        if (m_landArg.automatic)
        {
          inf("Creating a plan automatic");
          DubinsParametersContainer dubinsPath;
          findPath(Xs,Xf,dubinsPath);
          Xcs = dubinsPath.Xcs;
          Ycs = dubinsPath.Ycs;
          OCS = dubinsPath.OCS;
          Xcf = dubinsPath.Xcf;
          Ycf = dubinsPath.Ycf;
          OCF = dubinsPath.OCF;
          Pchi = dubinsPath.Pchi;
          PN = dubinsPath.PN;
          CounterClockwiseS = dubinsPath.CounterClockwiseS;
          CounterClockwiseF = dubinsPath.CounterClockwiseF;
        }
        else
        {
          inf("Creating a user specified path");
          //inf("Start circle %d, Start direction %d Finish circle %d",m_landArg.rigthStartTurningCircle,m_landArg.rightStartTurningDirection,m_landArg.rightFinishTurningCircle);
          //! Define start turning circle center (Ocs)
          if (m_landArg.rightStartTurningDirection)
          {

            Xcs = Xs(0,0)-m_landArg.Rs*std::cos(Xs(3,0)+PI/2);
            Ycs = Xs(1,0)-m_landArg.Rs*std::sin(Xs(3,0)+PI/2);
          }
          else
          {

            Xcs = Xs(0,0)-m_landArg.Rs*std::cos(Xs(3,0)-PI/2);
            Ycs = Xs(1,0)-m_landArg.Rs*std::sin(Xs(3,0)-PI/2);
          }
          CounterClockwiseS = m_landArg.rightStartTurningDirection;
          OCS(0,0) = Xcs;
          OCS(1,0) = Ycs;
          inf("Created start turn circle");
          //! Define end turning circle center (Ofs)

          if (m_landArg.rightFinishTurningCircle)
          {
            Xcf = Xf(0,0)-m_landArg.Rf*std::cos(Xf(3,0)+PI/2);
            Ycf = Xf(1,0)-m_landArg.Rf*std::sin(Xf(3,0)+PI/2);
          }
          else
          {
            Xcf = Xf(0,0)-m_landArg.Rf*std::cos(Xf(3,0)-PI/2);
            Ycf = Xf(1,0)-m_landArg.Rf*std::sin(Xf(3,0)-PI/2);
          }
          CounterClockwiseF = m_landArg.rightFinishTurningCircle;

          OCF(0,0) = Xcf;
          OCF(1,0) = Ycf;
          inf("Created finish turn circle");
          if (!dubinsParameteres(OCS,OCF,m_landArg.Rs,m_landArg.Rf,CounterClockwiseS,CounterClockwiseF,Pchi,PN))
          {
            war("Dubins Path does not exist from start position to end position");
            return false;
          }
        }

        ///////////////////// Construction phase of the function ////////////////////////////////
        //! Define turning arc
        std::vector<Matrix> arc;
        //! Declare angle array
        Matrix thetaTS =Matrix(1,m_Ns,0.0);
        //! First arc
        inf("Starting to construct first arc");
        double theta0 = std::atan2(Xs(1,0)-Ycs,Xs(0,0)-Xcs);
        double theta1 = std::atan2(Pchi(1,0)-Ycs,Pchi(0,0)-Xcs);
        if (CounterClockwiseS)
        {
          if (Angles::normalizeRadian(theta1-theta0)<=0)
          {
            calculateTurningArcAngle(-std::abs(Angles::normalizeRadian(theta1-theta0)),true,thetaTS);
          }
          else
          {
            calculateTurningArcAngle(-(2*PI-std::abs(Angles::normalizeRadian(theta1-theta0))),true,thetaTS);
          }
        }
        else
        {
          if (Angles::normalizeRadian(theta1-theta0)>=0)
          {
            calculateTurningArcAngle(std::abs(Angles::normalizeRadian(theta1-theta0)),true,thetaTS);
          }
          else
          {
            calculateTurningArcAngle(2*PI-std::abs(Angles::normalizeRadian(theta1-theta0)),true,thetaTS);
          }
        }
        ConstructArc(thetaTS,theta0,m_landArg.Rs,OCS,arc);
        AddToPath(arc,Path);
        inf("Constructed first arc");
        //! Second arc
        inf("Starting to construct second arc");
        theta0 = std::atan2(PN(1,0)-Ycf,PN(0,0)-Xcf);
        theta1 = std::atan2(Xf(1,0)-Ycf,Xf(0,0)-Xcf);
        //! Declare angle array
        Matrix thetaTF =Matrix(1,m_Nf,0.0);
        if (CounterClockwiseF)
        {
          if(Angles::normalizeRadian(theta1-theta0)<=0)
          {
            calculateTurningArcAngle(-std::abs(Angles::normalizeRadian(theta1-theta0)),false,thetaTF);
          }
          else
          {
            calculateTurningArcAngle(-(2*PI-std::abs(Angles::normalizeRadian(theta1-theta0))),false,thetaTF);
          }
        }
        else
        {
          if (Angles::normalizeRadian(theta1-theta0)>=0)
          {
            calculateTurningArcAngle(std::abs(Angles::normalizeRadian(theta1-theta0)),false,thetaTF);
          }
          else
          {
            calculateTurningArcAngle(2*PI-std::abs(Angles::normalizeRadian(theta1-theta0)),false,thetaTF);
          }
        }
        ConstructArc(thetaTF,theta0,m_landArg.Rf,OCF,arc);
        AddToPath(arc,Path);
        inf("Constructed second arc");
        inf("Constructed Dubins path");
        return true;

      }
      // ! Return the function parameters for the shortest dubins path
      bool findPath(const Matrix Xs,const Matrix Xf,DubinsParametersContainer &dubinsPath)
      {
        //! Initializing the structs
        DubinsParametersContainer CounterClockwiseCounterClocwise;
        CounterClockwiseCounterClocwise.CounterClockwiseF = true;
        CounterClockwiseCounterClocwise.CounterClockwiseS = true;
        DubinsParametersContainer CounterClockwiseClockwise;
        CounterClockwiseClockwise.CounterClockwiseS = true;
        CounterClockwiseClockwise.CounterClockwiseF = false;
        DubinsParametersContainer ClockwiseCounterClockwise;
        ClockwiseCounterClockwise.CounterClockwiseS = false;
        ClockwiseCounterClockwise.CounterClockwiseF = true;
        DubinsParametersContainer ClockwiseClockwise;
        ClockwiseClockwise.CounterClockwiseS = false;
        ClockwiseClockwise.CounterClockwiseF = false;
        bool filledParameters = fillDubinsParametersContainger(Xs,Xf,CounterClockwiseCounterClocwise);
        if (!filledParameters)
        {
          return false;
        }
        filledParameters = fillDubinsParametersContainger(Xs,Xf,CounterClockwiseClockwise);
        if (!filledParameters)
        {
          return false;
        }
        filledParameters = fillDubinsParametersContainger(Xs,Xf,ClockwiseCounterClockwise);
        if (!filledParameters)
        {
          return false;
        }
        filledParameters = fillDubinsParametersContainger(Xs,Xf,ClockwiseClockwise);
        if (!filledParameters)
        {
          return false;
        }
        //! Clearing the vector to remove old data
        m_dubinsPaths.clear();
        //! Insert new paths
        m_dubinsPaths.push_back(CounterClockwiseCounterClocwise);
        m_dubinsPaths.push_back(CounterClockwiseClockwise);
        m_dubinsPaths.push_back(ClockwiseCounterClockwise);
        m_dubinsPaths.push_back(ClockwiseClockwise);
        Matrix LengthPathV = Matrix(4,1,0.0);
        LengthPathV(0,0) = CounterClockwiseCounterClocwise.LengthPath;
        LengthPathV(1,0) = CounterClockwiseClockwise.LengthPath;
        LengthPathV(2,0) = ClockwiseCounterClockwise.LengthPath;
        LengthPathV(3,0) = ClockwiseClockwise.LengthPath;
        double currShortest = CounterClockwiseCounterClocwise.LengthPath;
        int currIndex = 0;
        //! Find the shortest path
        debug("Path length: %f",currShortest);
        for (int i=1;i<4;i++)
        {
          debug("Path length: %f",LengthPathV(i,0));
          if (currShortest>LengthPathV(i,0))
          {
            currShortest = LengthPathV(i,0);
            currIndex = i;
          }
        }
        debug("The currIndex is %d",currIndex);
        dubinsPath = m_dubinsPaths[currIndex];
        return true;
      }
      //! Fills the struct in the DubinsParametersContainer
      bool fillDubinsParametersContainger(const Matrix Xs,const Matrix Xf,DubinsParametersContainer &dubinsPath)
      {
        dubinsPath.OCS = Matrix(2,1,0.0);
        if (dubinsPath.CounterClockwiseS)
        {
          dubinsPath.Xcs = Xs(0,0)-m_landArg.Rs*cos(Xs(3,0)+PI/2);
          dubinsPath.Ycs = Xs(1,0)-m_landArg.Rs*sin(Xs(3,0)+PI/2);
        }
        else
        {
          dubinsPath.Xcs = Xs(0,0)-m_landArg.Rs*cos(Xs(3,0)-PI/2);
          dubinsPath.Ycs = Xs(1,0)-m_landArg.Rs*sin(Xs(3,0)-PI/2);
        }
        dubinsPath.OCF = Matrix(2,1,0.0);
        if (dubinsPath.CounterClockwiseF)
        {
          dubinsPath.Xcf = Xf(0,0)-m_landArg.Rf*cos(Xf(3,0)+PI/2);
          dubinsPath.Ycf = Xf(1,0)-m_landArg.Rf*sin(Xf(3,0)+PI/2);
        }
        else
        {
          dubinsPath.Xcf = Xf(0,0)-m_landArg.Rf*cos(Xf(3,0)-PI/2);
          dubinsPath.Ycf = Xf(1,0)-m_landArg.Rf*sin(Xf(3,0)-PI/2);
        }

        dubinsPath.OCS(0,0) = dubinsPath.Xcs;
        dubinsPath.OCS(1,0) = dubinsPath.Ycs;
        dubinsPath.OCF(0,0) = dubinsPath.Xcf;
        dubinsPath.OCF(1,0) = dubinsPath.Ycf;
        dubinsPath.Pchi = Matrix(2,1,0.0);
        dubinsPath.PN = Matrix(2,1,0.0);
        if (!dubinsParameteres(dubinsPath.OCS,dubinsPath.OCF,m_landArg.Rs,m_landArg.Rf,dubinsPath.CounterClockwiseS,dubinsPath.CounterClockwiseF,dubinsPath.Pchi,dubinsPath.PN))
        {
          war("Dubins Path does not exist from start position to end position");
          return false;
        }
        double theta0 = std::atan2(Xs(1,0)-dubinsPath.Ycs,Xs(0,0)-dubinsPath.Xcs);
        double theta1 = std::atan2(dubinsPath.Pchi(1,0)-dubinsPath.Ycs,dubinsPath.Pchi(0,0)-dubinsPath.Xcs);
        double sLtheta1 = 0;
        arcAngle(theta0,theta1,dubinsPath.CounterClockwiseS,sLtheta1);
        debug("Arc angle start1 %f",sLtheta1);

        double theta01 = std::atan2(dubinsPath.PN(1,0)-dubinsPath.Ycf,dubinsPath.PN(0,0)-dubinsPath.Xcf);
        double theta11 = std::atan2(Xf(1,0)-dubinsPath.Ycf,Xf(0,0)-dubinsPath.Xcf);
        double fLtheta1 = 0;
        arcAngle(theta01,theta11,dubinsPath.CounterClockwiseF,fLtheta1);
        debug("Arc angle finish1%f",fLtheta1);
        dubinsPath.LengthPath = m_landArg.Rs*sLtheta1 + std::sqrt(std::pow(dubinsPath.Pchi(0,0)-dubinsPath.PN(0,0),2)+std::pow(dubinsPath.Pchi(1,0)-dubinsPath.PN(1,0),2)) + m_landArg.Rf*fLtheta1;
        return true;
      }

      //! Return the parameters in Dubins Path
      bool
      dubinsParameteres(Matrix Ocs,Matrix Ocf,double Rs,double Rf,bool TurnS,bool TurnF,Matrix& Pchi,Matrix& PN)
      {
        double Xcs = Ocs(0,0);
        double Ycs = Ocs(1,0);
        double Xcf = Ocf(0,0);
        double Ycf = Ocf(1,0);

        //! Calculate the line between Ocs and Ofs
        double cbx = Xcs;
        double cax = Xcf - cbx;
        double cby = Ycs;
        double cay = Ycf - cby;

        //! Calculate the length of c
        double dc = std::sqrt(std::pow(cax,2)+std::pow(cay,2));
        //! Check that Dubins path exists
        if (sign(Rf-Rs)*(Rf-Rs)>dc)
        {
          war("Dubins Path does not exist from start position to end position");
          return false;
        }
        //! Calculate alpha
        double alpha = std::asin((m_landArg.Rf-m_landArg.Rs)/dc);

        //! Calculate beta
        double beta = std::atan2(Ycf-Ycs,Xcf-Xcs);

        //! Define tangent points
        //! First
        double thetaS = turn(TurnS,alpha,beta);
        //! Second
        double thetaF = turn(TurnF,alpha,beta);
        //! Exit tangent point for first circle
        //Pchi = Matrix(2,1,0.0);
        Pchi(0,0) = Xcs+m_landArg.Rs*cos(thetaS);
        Pchi(1,0) = Ycs+m_landArg.Rs*sin(thetaS);
        inf("Created exit tangent");
        //! Entry tangent point
        //PN = Matrix(2,1,0.0);
        PN(0,0) = Xcf+m_landArg.Rf*cos(thetaF);
        PN(1,0) = Ycf+m_landArg.Rf*sin(thetaF);
        inf("Created entry tangent");
        return true;
      }

      //! Return the the turning arc given a turning direction
      void
      arcAngle(const double theta0,const double theta1,const bool Right,double &theta)
      {
        if (Right)
          {
            if(Angles::normalizeRadian(theta1-theta0)<=0)
            {
              theta = std::abs(Angles::normalizeRadian(theta1-theta0));
            }
            else
            {
              theta = 2*PI-std::abs(Angles::normalizeRadian(theta1-theta0));
            }
          }
        else
          {
            if (Angles::normalizeRadian(theta1-theta0)>=0)
            {
              theta = std::abs(Angles::normalizeRadian(theta1-theta0));
            }
            else
            {
              theta = 2*PI-std::abs(Angles::normalizeRadian(theta1-theta0));
            }
          }
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
      calculateTurningArcAngle(const double theta_limit,const bool startCircle,Matrix &theta)
      {
        //inf("Test n with floor limitation: %f",std::floor((theta_limit)/(2*PI)*m_args.N));
        //! Find the number of segments that are required to construct the arc
        unsigned N;
        if (startCircle)
        {
          N = std::floor((sign(theta_limit)*theta_limit)/(2*PI)*m_Ns);
        }
        else
        {
          N = std::floor((sign(theta_limit)*theta_limit)/(2*PI)*m_Nf);
        }
        inf("N = %d, Ns = %d Nf = %d",N,m_Ns,m_Nf);
        if (N==0)
          N = 1;
        theta.resize(1,N);
        double step;
        if (N!=1)
        {
          step = theta_limit/(N-1);
        }
        else
        {
          step = theta_limit;
        }
        inf("Step limit %f",theta_limit);
        for (unsigned i=0;i<N;i++)
        {
          theta(0,i)=i*step;
        }
        if (N==1)
        {
          theta(0,0) = step;
        }
        inf("Last theta %f and size theta %d N= %d",theta(0,N-1),theta.columns(),N);
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
      ConstructArc(const Matrix theta,const double theta0,const double R,const Matrix center,std::vector<Matrix>& arc)
      {
        Matrix tempP = Matrix(3,1,0.0);
        for (unsigned i=0;i<theta.columns();i++)
        {

          tempP(0,0) = center(0,0) + R*std::cos(theta0+theta(0,i));
          tempP(1,0) = center(1,0) + R*std::sin(theta0+theta(0,i));
          arc.push_back(tempP);
        }

      }
      //! Add an arc to the path.
      void
      AddToPath(std::vector<Matrix> &arc,std::vector<Matrix> &path)
      {
        for (unsigned i=0;i<arc.size();i++)
        {
          path.push_back(arc[i]);
        }
        //! Empty arc
        arc.erase(arc.begin(),arc.end());
      }
      //! Constructs a glideslope from x0 towards height of loiter point along dubins path
      bool
      createGlideSlope(const Matrix x0,const Matrix WP,std::vector<Matrix> &Path)
      {
        //! The path starts at the same height as x0
        double descentAngle = m_landArg.gamma_d;
        bool correctHeight = false;
        Path[0](2,0) = x0(2,0);
        double D;
        inf("Start height %f desired height %f",x0(2,0),WP(2,0));
        for (unsigned i=0;i<Path.size()-1;i++)
        {
          D = sqrt(std::pow(Path[i+1](0,0)-Path[i](0,0),2)+std::pow(Path[i+1](1,0)-Path[i](1,0),2));
          debug("The distance D = %f",D);
          debug("New angle %f descent angel %f",std::sqrt(std::pow(std::atan2(WP(2,0)-Path[i](2,0),D),2)),std::sqrt(std::pow(descentAngle,2)));
          if (!correctHeight && std::sqrt(std::pow(std::atan2(WP(2,0)-Path[i](2,0),D),2))<std::sqrt(std::pow(descentAngle,2)))
          {
            correctHeight = true;
            inf("Reached correct height");
            descentAngle = std::atan2(WP(2,0)-Path[i](2,0),D);
            Path[i+1](2,0) = Path[i](2,0)+D*tan(descentAngle);
          }
          else if (!correctHeight)
          {
            Path[i+1](2,0) = Path[i](2,0) + D*tan(descentAngle);
          }
          else
          {
            Path[i+1](2,0) = Path[i](2,0);
          }
        }
        inf("Desired height %f path height %f",WP(2,0),Path[Path.size()-1](2,0));
        return correctHeight;
      }
      //! Create a spiral path towards the desired height dHeight
      void
      createSpiral(const Matrix OF,const bool CounterClockwiseF,const double dHeight,std::vector<Matrix> &Path)
      {
        bool correctHeight = false;
        double descentAngle = m_landArg.gamma_d;
        double theta0 = std::atan2(Path[Path.size()-1](1,0)-OF(1,0),Path[Path.size()-1](0,0)-OF(0,0));
        Matrix WP4 = Path.back();
        Matrix theta = Matrix(1,m_Nf);
        if (CounterClockwiseF)
        {
          calculateTurningArcAngle(-2*PI,false,theta);
        }
        else
        {
          calculateTurningArcAngle(2*PI,false,theta);
        }
        Matrix WPS0 = Path.back();
        double xnn = OF(0,0) + m_landArg.Rf*cos(theta0+theta(0,1));
        double ynn = OF(1,0) + m_landArg.Rf*sin(theta0+theta(0,1));
        double D = std::sqrt(std::pow(xnn-WPS0(0,0),2)+std::pow(ynn-WPS0(1,0),2));
        double znn = WPS0(2,0)+D*std::tan(descentAngle);
        Matrix WPS1 = Matrix(3,1,0.0);
        WPS1(0,0) = xnn;
        WPS1(1,0) = ynn;
        WPS1(2,0) = znn;
        inf("Next height %f",m_estate.height-znn);
        inf("Desired height %f",m_landArg.netHeading-dHeight);
        int n = 2;
//        Path.push_back(WPS0);
        Path.push_back(WPS1);
        inf("theta has %d elements",theta.size());
        debug("Glide angle is %f",descentAngle);
        int max_iter = 1000;
        int cur_iter = 0;
        while(!correctHeight && cur_iter < max_iter)
        {
          ++cur_iter;
          if (std::sqrt(std::pow(std::atan2(dHeight-WPS1(2,0),D),2))<std::sqrt(std::pow(descentAngle,2)))
          {
            descentAngle = std::atan2(dHeight-WPS1(2,0),D);
            correctHeight = true;
          }
          WPS0 = WPS1;
          xnn = OF(0,0) + m_landArg.Rf*cos(theta0+theta(0,n));
          ynn = OF(1,0) + m_landArg.Rf*sin(theta0+theta(0,n));
          D = std::sqrt(std::pow(xnn-WPS0(0,0),2)+std::pow(ynn-WPS0(1,0),2));
          znn = WPS0(2,0)+D*std::tan(descentAngle);
          debug("Next height %f",znn);
          WPS1(0,0) = xnn;
          WPS1(1,0) = ynn;
          WPS1(2,0) = znn;
          Path.push_back(WPS1);
          n = n+1;
          //! Check if n has reached m_N. Then set to 1, such that the 0 value is only used once
          if (n>=m_Nf)
          {
            n = 1;
          }
        }
        inf("Exited loop. N iterations: %d", cur_iter);

        double thetaH0 = std::atan2(WPS1(1,0)-OF(1,0),WPS1(0,0)-OF(0,0));
        double thetaH1 = std::atan2(WP4(1,0)-OF(1,0),WP4(0,0)-OF(0,0));
        inf("Start angle %f Finish angle %f",thetaH0,thetaH1);
        std::vector<Matrix> arc;
        if (CounterClockwiseF)
        {
          if (Angles::normalizeRadian(thetaH1-thetaH0)<=0)
          {
            calculateTurningArcAngle(-std::abs(Angles::normalizeRadian(thetaH1-thetaH0)),false,theta);
          }
          else
          {
            calculateTurningArcAngle(-(2*PI-std::abs(Angles::normalizeRadian(thetaH1-thetaH0))),false,theta);
          }
        }
        else
        {
          if (Angles::normalizeRadian(thetaH1-thetaH0)>=0)
          {
            calculateTurningArcAngle(std::abs(Angles::normalizeRadian(thetaH1-thetaH0)),false,theta);
          }
          else
          {
            calculateTurningArcAngle((2*PI-std::abs(Angles::normalizeRadian(thetaH1-thetaH0))),false,theta);
          }
        }
        ConstructArc(theta,thetaH0,m_landArg.Rf,OF,arc);
        for (unsigned i=0;i<arc.size();i++)
        {
          arc[i](2,0) = Path[Path.size()-1](2,0);
        }
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
