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
// The method used to calculate the Dubins path is based on the algorithm   *
// found in "Cooperative path planning of unmanned aerial vehicles". The    *
// method used is the external circle 2-D path.                             *
//***************************************************************************

// This task generates a dubins path that can be used for landing
// DUNE headers.
#include <DUNE/DUNE.hpp>

// USER headers
#include <USER/DUNE.hpp>

#include <vector>
#include <cmath>

#include <aw-dubins-curves/dubins.h>
#include "dubins_callback.hpp"

namespace Plan
{
  namespace LandingPlan
  {
    using DUNE_NAMESPACES;

    struct Arguments
    {
      //! Stationary net landing
      bool dynamically_landing;
      //! Arc segment distance
      double arc_segment_distance;
      std::string overrideTOA;
      std::string lat_ctrl;
      std::string fbwa_lon_ctrl;
      std::string glideslope_ctrl;
    };

    struct LandingPathArguments
    {
      //! Length of point behind the net
      double behind_net;
      //! Length of final approach
      double infront_net;
      //! Length of glideslope
      double length_glideslope;
      //! Length of approach
      double length_approach_glideslope;
      //! Angle of impact towards net
      double gamma_a;
      //! Angle of descent: landing path
      double gamma_d;
      //! Decent angle in approach path
      double approachDecent;
      //! (Optional) Start orientation
      double startHeading;
      //! (Optional) Start lat
      double start_lat;
      //! (Optional) Start lon
      double start_lon;
      //! (Optional) Start height
      double start_height;
      //! Net orientation
      double netHeading;
      //! Net lat
      double net_lat;
      //! Net lon
      double net_lon;
      //! Net height
      double net_height;
      //! Net WGS84 height
      double net_WGS84_height;
      //! Landing speed
      double land_speed;
      //! Approach speed
      double approach_speed;
      //! Start turning circle radius
      double Radius_start;
      //! Finish turning circle radius
      double Radius_end;
      //! Automatic generation of start and finish circle
      bool automatic;
      //! Right start turning direction
      bool rightStartTurningDirection;
      //! Right finish turning direction
      bool rightFinishTurningCircle;
      //! Wait at loiter
      bool wait_at_loiter;
    };

    struct LandingPath
    {
      //! Landing waypoints
      //! WP4: Behind the net
      Matrix WP4;
      //! WP3: In front of the net
      Matrix WP3;
      //! WP2: The start of the glide slope
      Matrix WP2;
      //! WP1: The start of the approach towards the net
      Matrix WP1;
      //! Finish turning circle rotation
      bool clockwise;
      //! Finish turning circle center
      Matrix OCF;
      //! Start position
      Matrix Xs;
      double llhref_lat;
      double llhref_lon;
      double llhref_height;
      //! Finish turning circle offset height
      double OCFz;
    };

    struct Task: public DUNE::Tasks::Task
    {
      //! Task arguments
      Arguments m_args;
      //! Landing path arguments
      LandingPathArguments m_landArg;
      //! Landing path parameters
      LandingPath m_landParameteres;
      //! Accumulated EstimatedState message
      IMC::EstimatedState m_estate;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx)
      {

        param("Distance Between Arc Segment",m_args.arc_segment_distance)
        .visibility(Tasks::Parameter::VISIBILITY_USER)
        .defaultValue("20")
        .description("Distance Between Arc Segment");

        param("Dynamicall Net Landing",m_args.dynamically_landing)
        .visibility(Tasks::Parameter::VISIBILITY_USER)
        .defaultValue("false")
        .description("True = Dynamically net, False = Stationary net");

        param("Override Time-of-Arrival factor", m_args.overrideTOA)
        .defaultValue("1.0")
        .description("Value to override Time-of-Arrival factor for controllers during landing");

        param("Glideslope Height Controller", m_args.glideslope_ctrl)
        .defaultValue("Glideslope Height Controller")
        .description("Glideslope Height Controller that will be used in landing");

        param("FBWA Longitudinal Controller", m_args.fbwa_lon_ctrl)
        .defaultValue("FBWA Longitudinal Controller")
        .description("FBWA Longitudinal Controller that will be used in landing");

        param("Lateral LOS Controller", m_args.lat_ctrl)
        .defaultValue("Lateral LOS Control")
        .description("Lateral LOS Controller that will be used in landing");

        //! Bind IMC messages
        bind<IMC::EstimatedState>(this);
        bind<IMC::LandingPlanGeneration>(this);
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
      //! Receive net pose and landing spesifications
      void
      consume(const IMC::LandingPlanGeneration *msg)
      {
        if (msg->op != IMC::LandingPlanGeneration::OP_REQUEST)
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

      //! Extract information from a LandingPlanGeneration message
      bool
      extractPlan(const IMC::LandingPlanGeneration *msg)
      {
        //! Check if landing path container should be filled. It's assumed that input data is in NED
        if (msg->plan_id=="land")
        {

          m_landArg.start_lat = msg->start_lat;
          m_landArg.start_lon = msg->start_lon;
          m_landArg.startHeading = msg->start_heading;
          m_landArg.start_height = msg->start_height;
          m_landArg.rightStartTurningDirection = msg->startcounterclockwise;
          m_landArg.rightFinishTurningCircle = msg->finishcounterclockwise;
          m_landArg.net_lat = msg->lat;
          m_landArg.net_lon = msg->lon;
          m_landArg.net_WGS84_height = msg->height;
          m_landArg.netHeading = msg->heading;
          m_landArg.net_height = msg->ground;
          m_landArg.gamma_a = msg->finalapproachangle;
          m_landArg.gamma_d = msg->glideslopeangle;
          m_landArg.approachDecent = msg->approachdecentangle;
          m_landArg.behind_net = msg->distancebehind;
          m_landArg.infront_net = msg->finalapproach;
          m_landArg.length_glideslope = msg->glideslope;
          m_landArg.length_approach_glideslope = msg->approach;
          m_landArg.land_speed = msg->landingspeed;
          m_landArg.approach_speed = msg->approachspeed;
          m_landArg.Radius_start = msg->startturningradius;
          m_landArg.Radius_end = msg->finishturningradius;
          m_landArg.automatic = msg->automatic;
          m_landArg.wait_at_loiter = msg->waitloiter;
          debug("Content from tList:");
          debug("Net Lat %f",m_landArg.net_lat);
          debug("Net lon %f",m_landArg.net_lon);
          debug("Net WGS %f",m_landArg.net_WGS84_height);
          debug("Net Height %f",m_landArg.net_height);
          debug("Net heading %f", m_landArg.netHeading);
          debug("Attack angle %f",m_landArg.gamma_a);
          debug("Descent %f",m_landArg.gamma_d);
          debug("Approach decent angle %f", m_landArg.approachDecent);
          debug("Behind net %f",m_landArg.behind_net);
          debug("In front of net %f",m_landArg.infront_net);
          debug("Length glideslope %f",m_landArg.length_glideslope);
          debug("Length approach glideslope %f",m_landArg.length_approach_glideslope);
          debug("Speed 35 %f",m_landArg.land_speed);
          debug("Speed 12 %f",m_landArg.approach_speed);
          debug("Radius_start %f",m_landArg.Radius_start);
          debug("Radius_end %f",m_landArg.Radius_end);
          debug("Automatic %d",m_landArg.automatic);
          debug("Right start dir %d",m_landArg.rightStartTurningDirection);
          debug("Right finish %d",m_landArg.rightFinishTurningCircle);
          debug("Wait at loiter %d",m_landArg.wait_at_loiter);
          inf("Extracted arguments from neptus");

          constructWP();

          return true;
        }
        return false;
      }

      void
      constructWP()
      {
        //! Construct waypoint for the final landing path
        //! WP4 is set behind the net
        m_landParameteres.WP4 = Matrix(3,1,0.0);
        m_landParameteres.WP4(0,0) = -m_landArg.behind_net;
        m_landParameteres.WP4(2,0) = m_landArg.net_height+m_landArg.behind_net*std::tan(m_landArg.gamma_a);

        //! WP3 is set in front of the net, and is the final phase before the net
        m_landParameteres.WP3 = Matrix(3,1,0.0);
        m_landParameteres.WP3(0,0) = m_landArg.infront_net;
        m_landParameteres.WP3(2,0) = m_landArg.net_height-m_landArg.infront_net*std::tan(m_landArg.gamma_a);

        //! WP2 is the start of the glide slope towards the net
        m_landParameteres.WP2 = Matrix(3,1,0.0);
        m_landParameteres.WP2(0,0) = m_landParameteres.WP3(0,0)+m_landArg.length_glideslope;
        m_landParameteres.WP2(2,0) = m_landParameteres.WP3(2,0)-m_landArg.length_glideslope*std::tan(m_landArg.gamma_d);

        //! WP1 is the approach to the glide slope
        m_landParameteres.WP1 = Matrix(3,1,0.0);
        m_landParameteres.WP1(0,0) = m_landParameteres.WP2(0,0)+m_landArg.length_approach_glideslope;
        m_landParameteres.WP1(2,0) = m_landParameteres.WP2(2,0);

        //! Rotate all WP into NED
        m_landParameteres.WP4 = Rzyx(0,0,m_landArg.netHeading)*m_landParameteres.WP4;
        m_landParameteres.WP3 = Rzyx(0,0,m_landArg.netHeading)*m_landParameteres.WP3;
        m_landParameteres.WP2 = Rzyx(0,0,m_landArg.netHeading)*m_landParameteres.WP2;
        m_landParameteres.WP1 = Rzyx(0,0,m_landArg.netHeading)*m_landParameteres.WP1;

        debug("WP4 x=%f y=%f z=%f",m_landParameteres.WP4(0,0),m_landParameteres.WP4(1,0),m_landParameteres.WP4(2,0));
        debug("WP3 x=%f y=%f z=%f",m_landParameteres.WP3(0,0),m_landParameteres.WP3(1,0),m_landParameteres.WP3(2,0));
        debug("WP2 x=%f y=%f z=%f",m_landParameteres.WP2(0,0),m_landParameteres.WP2(1,0),m_landParameteres.WP2(2,0));
        debug("WP1 x=%f y=%f z=%f",m_landParameteres.WP1(0,0),m_landParameteres.WP1(1,0),m_landParameteres.WP1(2,0));

      }

      //! Generates a landing path from the initial position of the plane towards the position of the net
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
        plan_spec.start_man_id = "1";
        plan_spec.description = "Plan activating land";

        //! Create a list of maneuvers
        IMC::MessageList<IMC::Maneuver> maneuverList;

        //! Create a followPath maneuver which is filled with Dubins path
        IMC::FollowPath fPath;

        // Set starting point as NED reference for the path
        fPath.lat = m_landParameteres.llhref_lat;
        fPath.lon = m_landParameteres.llhref_lon;
        fPath.z = m_landParameteres.llhref_height;

        debug("Current lat: %f current lon: %f current height: %f",fPath.lat,fPath.lon,fPath.z);
        fPath.z_units = IMC::Z_HEIGHT;
        fPath.speed = m_landArg.approach_speed;
        fPath.speed_units = IMC::SUNITS_METERS_PS;

        addPathPoint(path,&fPath);
        maneuverList.push_back(fPath);

        //! Add loiter maneuver if the waitLoiter flag is set to true, else finish the landing path
        if (m_landArg.wait_at_loiter)
        {
          addLoiter(maneuverList);
        }
        if (!m_args.dynamically_landing)
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
      //! Create path towards the approach to the glide slope (WP1)
      bool
      createPath(std::vector<Matrix>& path)
      {
        //! Direction of end turn
        bool CounterClockwiseF;
        //! Center of final turning circle
        Matrix OCF = Matrix(2,1,0.0);

        //! End pose in dubins path which is the first waypoint in the net approach
        Matrix Xf = Matrix(4,1,0.0);
        Xf(0,0) = m_landParameteres.WP1(0,0);
        Xf(1,0) = m_landParameteres.WP1(1,0);
        Xf(2,0) = m_landParameteres.WP1(2,0);
        Xf(3,0) = Angles::normalizeRadian(m_landArg.netHeading-Math::c_pi);

        //! Set the reference lat/lon/height for the displacement to be the net position
        double wp1_lat = m_landArg.net_lat;
        double wp1_lon = m_landArg.net_lon;
        double wp1_h = m_landArg.net_WGS84_height - m_landParameteres.WP1(2,0);
        //! Find WP1 lat lon by displacing the WP1 NED position from the net position
        Coordinates::WGS84_Accurate::displace(m_landParameteres.WP1(0,0),m_landParameteres.WP1(1,0),
                                              &wp1_lat,&wp1_lon);



        //! Initialize the start pose
        //! llh
        double startpoint_lat = 0.0;
        double startpoint_lon = 0.0;
        double startpoint_height = 0.0;
        //! NED
        Matrix Xs = Matrix(4,1,0.0);

        //! Set the reference lat/lon/height for the displacement to be the current position
        if ( (m_landArg.start_lon != 0) && (m_landArg.start_lon != 0) && (m_landArg.start_lon != 0) && (m_landArg.start_lon != 0) ) //TODO: Add sanity check on starting pos?
        {
          //! Starting pose has been manually set; 
          //! Set the reference lat/lon/height for the displacement to be the manually selected position
          startpoint_lat = m_landArg.start_lat;
          startpoint_lon = m_landArg.start_lon;
          startpoint_height = m_landArg.start_height;
          //TODO: WHY? They are overwritten by displacement() as long as they are not zero(?)
          Xs(0,0) = 1.0;
          Xs(1,0) = 1.0;
          Xs(2,0) = 1.0;
          Xs(3,0) = m_landArg.startHeading;
        }
        else
        {
          //! Set the reference lat/lon/height for the displacement to be the current position
          startpoint_lat = m_estate.lat;
          startpoint_lon = m_estate.lon;
          startpoint_height = m_estate.height;
          //TODO: WHY? They are overwritten by displacement() as long as they are not zero(?)
          Xs(0,0) = m_estate.x;
          Xs(1,0) = m_estate.y;
          Xs(2,0) = m_estate.z;
          Xs(3,0) = m_estate.psi;
        }
        //! Find WP1 lat lon by displacing the WP1 NED position from the net position
        Coordinates::WGS84_Accurate::displace(m_estate.x,m_estate.y,m_estate.z,
                                              &startpoint_lat,&startpoint_lon,&startpoint_height);

        //! Find NED position of startpoint_lat/lon/height (WGS84), referenced in m_landArg.net_lat/lon/height (WGS84) and place it in Xs
        Coordinates::WGS84::displacement(m_landArg.net_lat,m_landArg.net_lon,m_landArg.net_height,
                                          startpoint_lat,startpoint_lon,startpoint_height,
                                          &Xs(0,0),&Xs(1,0),&Xs(2,0));
        //! Find NED position of startpoint_lat/lon/height (WGS84), referenced in wp1_lat/lon/height (WGS84) and place it in Xf
        Coordinates::WGS84::displacement(startpoint_lat,startpoint_lon,startpoint_height,
                                          wp1_lat,wp1_lon,wp1_h,
                                          &Xf(0,0),&Xf(1,0),&Xf(2,0));

        debug("Xf x=%f y=%f z=%f psi=%f",Xf(0,0),Xf(1,0),Xf(2,0),Xf(3,0));
        debug("Xs x=%f y=%f z=%f psi=%f",Xs(0,0),Xs(1,0),Xs(2,0),Xs(3,0));
        debug("m_estate height: %f net height: %f",m_estate.height,m_landArg.net_WGS84_height);
        debug("State: lat %f lon %f height %f",startpoint_lat,startpoint_lon,startpoint_height);

        // Set the reference lat/lon/height to be the starting point
        // and set the NED start position to zero, accordingly
        m_landParameteres.Xs = Xs; //TODO: why is this set? It is never used...
        m_landParameteres.llhref_lat = startpoint_lat;
        m_landParameteres.llhref_lon = startpoint_lon;
        m_landParameteres.llhref_height = startpoint_height;
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

        //! Create a longitudinal path
        longitudinalPath(Xs,Xf,OCF,CounterClockwiseF,path);
        inf("Created longitudinal path");

        //! Store parameter that has been used in the construction of the path
        m_landParameteres.OCF = OCF;
        m_landParameteres.clockwise = !CounterClockwiseF;
        m_landParameteres.OCFz = Xf(2,0);

        debug("Reach correct height %f from the height %f",path[path.size()-1](2,0),Xs(2,0));
        debug("WP1 = h-z %f",m_landArg.net_WGS84_height-m_landParameteres.WP1(2,0));
        debug("Lat %f lon %f ref height %f",m_landArg.net_lat,m_landArg.net_lon,m_landArg.net_WGS84_height);
        return true;
      }

      //! Create a longitudinal path to be added to the Dubins path
      void
      longitudinalPath(Matrix Xs,Matrix Xf,Matrix OCF,bool CounterClokwiseF,std::vector<Matrix> &path)
      {
        if (Xf(2,0)<Xs(2,0))
        {
          m_landArg.approachDecent = -m_landArg.approachDecent;
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
          man_spec.maneuver_id = unsignedToString(i);

          if (it!=maneuverList->begin())
          {
            IMC::PlanTransition trans;
            trans.conditions = "maneuverIsDone";
            trans.dest_man = man_spec.maneuver_id;
            trans.source_man = last_man.maneuver_id;
            plan_spec.transitions.push_back(trans);
          }
          // Create start actions
          IMC::SetEntityParameters eparam_start;
          IMC::EntityParameter param_t;

          eparam_start.name = m_args.fbwa_lon_ctrl;
          param_t.name = "Override Time Of Arrival Factor";
          param_t.value = m_args.overrideTOA;
          eparam_start.params.push_back(param_t);
          param_t.name = "Use controller";
          param_t.value = "true";
          eparam_start.params.push_back(param_t);

          man_spec.start_actions.push_back(eparam_start);

          //No additional parameters for glideslope
          eparam_start.name = m_args.glideslope_ctrl;

          man_spec.start_actions.push_back(eparam_start);

          //Additional parameters for lateral
          eparam_start.name = m_args.lat_ctrl;
          param_t.name = "Use external Z control";
          param_t.value = "true";
          eparam_start.params.push_back(param_t);
          param_t.name = "Smoothen roll reference";
          param_t.value = "true";
          eparam_start.params.push_back(param_t);

          man_spec.start_actions.push_back(eparam_start);

          eparam_start.clear();
          eparam_start.name = "Autopilot";
          param_t.name = "Ardupilot Tracker";
          param_t.value = "false";
          eparam_start.params.push_back(param_t);

          man_spec.start_actions.push_back(eparam_start);
          
          last_man = man_spec;
          plan_spec.maneuvers.push_back(man_spec);
        }

      }
      //! Add a loiter point after dubins
      void
      addLoiter(IMC::MessageList<IMC::Maneuver>& maneuverList)
      {
        IMC::Loiter loiter;
        double loiter_lat = m_landParameteres.llhref_lat;
        double loiter_lon = m_landParameteres.llhref_lon;
        double loiter_h = m_landParameteres.llhref_height;
        debug("loiter_h %f OCFz %f ",loiter_h,m_landParameteres.OCFz);
        Coordinates::WGS84_Accurate::displace(m_landParameteres.OCF(0,0),m_landParameteres.OCF(1,0),m_landParameteres.OCFz,
                                              &loiter_lat,&loiter_lon,&loiter_h);
        loiter.lat = loiter_lat;
        loiter.lon = loiter_lon;
        loiter.z = loiter_h;
        loiter.z_units = IMC::Z_HEIGHT;
        loiter.speed = m_landArg.approach_speed;
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
        loiter.radius = m_landArg.Radius_end;
        loiter.duration = 0;
        loiter.setSubId(10);
        maneuverList.push_back(loiter);
      }
      //! Add the landing approach toward the net
      void
      addNetApproach(IMC::MessageList<IMC::Maneuver>& maneuverList)
      {
        //2
        addGotoPoint(m_landParameteres.WP2,m_landArg.approach_speed,maneuverList);

        //3
        addGotoPoint(m_landParameteres.WP3,m_landArg.land_speed,maneuverList);

        //4
        addGotoPoint(m_landParameteres.WP4,m_landArg.land_speed,maneuverList);

      }
      //!
      void
      addGotoPoint(const Matrix WP,const double speed,IMC::MessageList<IMC::Maneuver>& maneuverList)
      {
        IMC::Goto gotoWP;
        double wp_lat = m_landArg.net_lat;
        double wp_lon = m_landArg.net_lon;
        double wp_h = m_landArg.net_WGS84_height - WP(2,0);
        Coordinates::WGS84_Accurate::displace(WP(0,0),WP(1,0),&wp_lat,&wp_lon);
        gotoWP.lat = wp_lat;
        gotoWP.lon = wp_lon;
        gotoWP.z = wp_h;
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
      // @param Xs {x,y,z, heading} configuration of the starting point 
      // @param Xf {x,y,z, heading} configuration of the end point 
      // @param Path {x,y,z} positions for all the points along the path
      // @param OCF position of the center of the final circle/spiral
      // @output true if the generation succeded
      bool
      dubinsPath(const Matrix Xs,const Matrix Xf, std::vector<Matrix>& Path,bool &CounterClockwiseF,Matrix &OCF)
      {

        DubinsPath path;
        //! x,y,heading of start and end of Dubins path
        double q0[] = {Xs(0,0), Xs(1,0), Xs(3,0)};
        double q1[] = {Xf(0,0), Xf(1,0), Xf(3,0)};
        //! When automatic is set true the shortest path from the initial position
        // to the first wp in the net approach. Otherwise the operator can manually decide the
        // rotation direction
        if (m_landArg.automatic)
        {
          inf("Creating a plan automatic");

          dubins_init(q0,q1,m_landArg.Radius_end,&path);
          
          // set up the center and rotation of the final circle, 
          // used to create spiral if we need to lose altitude
          CounterClockwiseF = (path.type == dRSL) || (path.type == dLSL) || (path.type == dLRL);
          if (CounterClockwiseF)
          { 
            OCF(0,0) = Xf(0,0)-m_landArg.Radius_end*cos(Xf(3,0)+Math::c_pi/2);
            OCF(1,0) = Xf(1,0)-m_landArg.Radius_end*sin(Xf(3,0)+Math::c_pi/2);
          }
          else
          { 
            OCF(0,0) = Xf(0,0)-m_landArg.Radius_end*cos(Xf(3,0)-Math::c_pi/2);
            OCF(1,0) = Xf(1,0)-m_landArg.Radius_end*sin(Xf(3,0)-Math::c_pi/2);
          }

        }
        else
        {
          uint8_t type;
          inf("Creating a user specified path");
          // set up the center and rotation of the final circle, 
          // used to create spiral if we need to lose altitude
          if (m_landArg.rightFinishTurningCircle)
          {
            OCF(0,0) = Xf(0,0)-m_landArg.Radius_end*std::cos(Xf(3,0)-Math::c_pi/2);
            OCF(1,0) = Xf(1,0)-m_landArg.Radius_end*std::sin(Xf(3,0)-Math::c_pi/2);
            type = (m_landArg.rightStartTurningDirection? dRSR : dLSR);
          }
          else
          {
            OCF(0,0) = Xf(0,0)-m_landArg.Radius_end*std::cos(Xf(3,0)+Math::c_pi/2);
            OCF(1,0) = Xf(1,0)-m_landArg.Radius_end*std::sin(Xf(3,0)+Math::c_pi/2);
            type = (m_landArg.rightStartTurningDirection? dRSL : dLSL);
          }
          CounterClockwiseF = m_landArg.rightFinishTurningCircle;
          inf("Created finish turn circle");

          if (specific_dubins_init( q0, q1, m_landArg.Radius_end, &path, type) != 0)
          {
            war("Dubins Path does not exist from start position to end position");
            return false;
          }
        }

        //! Add arcs to dune path
        dubins_path_sample_many(&path, addToPath_callback,m_args.arc_segment_distance, &Path);

        inf("Constructed Dubins path");
        return true;
      }


      //! Return N angle from 0 theta_limit with fixed arc segment length
      void
      calculateTurningArcAngle(const double theta_limit,const bool startCircle,Matrix &theta)
      {
        unsigned N;
        double step;

        //! Calculate the angle between two arc segments
        if (startCircle)
        {
          step = m_args.arc_segment_distance/m_landArg.Radius_start;
        }
        else
        {
          step = m_args.arc_segment_distance/m_landArg.Radius_end;
        }

        debug("Step size %f",step);

        //! Find the number of segments that are required to construct the arc
        N = std::ceil((sign(theta_limit)*theta_limit)/step)+1;
        //! Correct the sign of the step to match theta_limit
        step = sign(theta_limit)*step;

        //! Resizing the matrix
        theta.resize(1,N);

        for (unsigned i=0;i<N;i++)
        {
          if (i==N-1)
          {
            theta(0,i) = theta_limit;
          }
          else
          {
            theta(0,i)=i*step;
          }
        }

        debug("Step limit %f",theta_limit);
        debug("Step %f",step);
        debug("Last theta %f and size theta %d N= %d",theta(0,N-1),theta.columns(),N);
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

      //! Constructs a list of {x,y} coordinates that the arc from theta[0] to theta[m_N] with R radius consists of
      void
      ConstructArc(const Matrix theta,const double theta0,const double R,const Matrix center,std::vector<Matrix>& arc)
      {
        Matrix tempP = Matrix(3,1,0.0);

        for (int i=0;i<theta.columns();i++)
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
        double descentAngle = m_landArg.approachDecent;
        bool correctHeight = false;
        Path[0](2,0) = x0(2,0);
        double distance;
        debug("Start height %f desired height %f",x0(2,0),WP(2,0));

        for (unsigned i=0;i<Path.size()-1;i++)
        {
          // Calculate the distance to the next point
          distance = sqrt(std::pow(Path[i+1](0,0)-Path[i](0,0),2)+std::pow(Path[i+1](1,0)-Path[i](1,0),2));
          debug("The distance D = %f",distance);
          debug("New angle %f descent angel %f",std::sqrt(std::pow(std::atan2(WP(2,0)-Path[i](2,0),distance),2)),
                                                std::sqrt(std::pow(descentAngle,2)));

          // Check if the correct can be reach at next point with an angle equal or less then max descentAngle
          if (!correctHeight && std::sqrt(std::pow(std::atan2(WP(2,0)-Path[i](2,0),distance),2))<std::sqrt(std::pow(descentAngle,2)))
          {
            correctHeight = true;
            inf("Reached correct height");
            descentAngle = std::atan2(WP(2,0)-Path[i](2,0),distance);
            Path[i+1](2,0) = Path[i](2,0)+distance*tan(descentAngle);
          }
          else if (!correctHeight)
          {
            Path[i+1](2,0) = Path[i](2,0) + distance*tan(descentAngle);
          }
          else
          {
            Path[i+1](2,0) = Path[i](2,0);
          }
        }

        debug("Desired height %f path height %f",WP(2,0),Path[Path.size()-1](2,0));
        return correctHeight;
      }

      //! Create a spiral path towards the desired height dHeight
      void
      createSpiral(const Matrix OF,const bool CounterClockwiseF,const double dHeight,std::vector<Matrix> &Path)
      {
        bool correctHeight = false;
        double descentAngle = m_landArg.approachDecent;
        double theta0 = std::atan2(Path[Path.size()-1](1,0)-OF(1,0),Path[Path.size()-1](0,0)-OF(0,0));
        Matrix WP1 = Path.back();
        Matrix theta = Matrix(1,1,0.0);

        if (CounterClockwiseF)
        {
          calculateTurningArcAngle(-2*Math::c_pi,false,theta);
        }
        else
        {
          calculateTurningArcAngle(2*Math::c_pi,false,theta);
        }

        Matrix WPS0 = Path.back();
        double xnn = OF(0,0) + m_landArg.Radius_end*cos(theta0+theta(0,1));
        double ynn = OF(1,0) + m_landArg.Radius_end*sin(theta0+theta(0,1));
        double distance = std::sqrt(std::pow(xnn-WPS0(0,0),2)+std::pow(ynn-WPS0(1,0),2));
        double znn = WPS0(2,0)+distance*std::tan(descentAngle);

        Matrix WPS1 = Matrix(3,1,0.0);
        WPS1(0,0) = xnn;
        WPS1(1,0) = ynn;
        WPS1(2,0) = znn;

        debug("Next height %f",m_estate.height-znn);
        debug("Desired height %f",m_landArg.netHeading-dHeight);

        int n = 2;
        Path.push_back(WPS1);
        debug("theta has %d elements",theta.size());
        debug("Glide angle is %f",descentAngle);
        int max_iter = 1000;
        int cur_iter = 0;

        while(!correctHeight && cur_iter < max_iter)
        {
          ++cur_iter;
          if (std::sqrt(std::pow(std::atan2(dHeight-WPS1(2,0),distance),2))<std::sqrt(std::pow(descentAngle,2)))
          {
            descentAngle = std::atan2(dHeight-WPS1(2,0),distance);
            correctHeight = true;
          }

          WPS0 = WPS1;
          xnn = OF(0,0) + m_landArg.Radius_end*cos(theta0+theta(0,n));
          ynn = OF(1,0) + m_landArg.Radius_end*sin(theta0+theta(0,n));
          distance = std::sqrt(std::pow(xnn-WPS0(0,0),2)+std::pow(ynn-WPS0(1,0),2));
          znn = WPS0(2,0)+distance*std::tan(descentAngle);
          debug("Next height %f",znn);

          WPS1(0,0) = xnn;
          WPS1(1,0) = ynn;
          WPS1(2,0) = znn;
          Path.push_back(WPS1);

          n = n+1;
          //! Check if n has reached the max number of segments in circle. Then set to 1, such that the 0 value is only used once
          if (n>=std::ceil((2*m_landArg.Radius_end*Math::c_pi)/m_args.arc_segment_distance))
          {
            n = 1;
          }
        }
        inf("Exited loop. N iterations: %d", cur_iter);

        double thetaH0 = std::atan2(WPS1(1,0)-OF(1,0),WPS1(0,0)-OF(0,0));
        double thetaH1 = std::atan2(WP1(1,0)-OF(1,0),WP1(0,0)-OF(0,0));

        debug("Start angle %f Finish angle %f",thetaH0,thetaH1);
        std::vector<Matrix> arc;
        if (CounterClockwiseF)
        {
          if (Angles::normalizeRadian(thetaH1-thetaH0)<=0)
          {
            calculateTurningArcAngle(-std::abs(Angles::normalizeRadian(thetaH1-thetaH0)),false,theta);
          }
          else
          {
            calculateTurningArcAngle(-(2*Math::c_pi-std::abs(Angles::normalizeRadian(thetaH1-thetaH0))),false,theta);
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
            calculateTurningArcAngle((2*Math::c_pi-std::abs(Angles::normalizeRadian(thetaH1-thetaH0))),false,theta);
          }
        }

        ConstructArc(theta,thetaH0,m_landArg.Radius_end,OF,arc);
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

      std::string
      unsignedToString ( unsigned number )
      {
        std::ostringstream oss;

        // Works just like cout
        oss<< number;

        // Return the underlying string
        return oss.str();
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
