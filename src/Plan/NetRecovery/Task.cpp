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
// Author: Jostein B. Moe, Sigurd Olav Nevstad                              *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Plan
{
  namespace NetRecovery
  {
    using DUNE_NAMESPACES;

    struct FixedWingLoiter
    {
      uint8_t altitude;
      fp32_t speed;
      double radius_first;
      double radius_second;
      fp64_t lat;
      fp64_t lon;
      double duration;
      double distance_runway;
    };

    struct VirtualRunway //Landingpath
    {
      IMC::NetRecovery nr;

      fp64_t VR_center_lat;
      fp64_t VR_center_lon;
      double VR_heading;
      double VR_length;
      double VR_altitude;
      double goto_lat;
      double goto_lon;

    };

    struct Arguments
    {
      FixedWingLoiter fw_loiter;
      double glideslope_approach_distance;
      double glideslope_distance;
      double glideslope_angle;
      double desired_speed;
      double netWGS84Height;
      bool ignoreEvasive;
      std::string automatic_generation;
      bool rightStartTurningDirection;
      bool rightFinishTurningCircle;

    };

    struct Task: public DUNE::Tasks::Task
    {
      Arguments m_args;
      VirtualRunway   virtual_runway;

      //! Last plan control state
      IMC::PlanControlState m_last_pcs;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx)
      {
        param("Loiter Altitude", m_args.fw_loiter.altitude)
        .visibility(Parameter::VISIBILITY_USER)
        .defaultValue("50");

        param("Loiter Speed", m_args.fw_loiter.speed)
        .visibility(Parameter::VISIBILITY_USER)
        .defaultValue("20");

        param("Loiter Radius first", m_args.fw_loiter.radius_first)
        .description("Lateral turning radius of UAV at first circle (m)")
        .visibility(Parameter::VISIBILITY_USER)
        .defaultValue("100");

        param("Loiter Radius second", m_args.fw_loiter.radius_second)
        .description("Lateral turning radius of UAV at second circle (m)")
        .visibility(Parameter::VISIBILITY_USER)
        .defaultValue("100");

        param("Loiter Duration", m_args.fw_loiter.duration)
        .visibility(Parameter::VISIBILITY_USER)
        .defaultValue("0");

        param("Glideslope Angle", m_args.glideslope_angle)
        .visibility(Parameter::VISIBILITY_USER)
        .units(Units::Degree)
        .defaultValue("4.0");

        param("Glideslope Approach Distance", m_args.glideslope_approach_distance)
        .visibility(Parameter::VISIBILITY_USER)
        .units(Units::Meter)
        .defaultValue("100.0");

        param("Glideslope Distance", m_args.glideslope_distance)
        .visibility(Parameter::VISIBILITY_USER)
        .units(Units::Meter)
        .defaultValue("300.0");

        param("Loiter along-track distance", m_args.fw_loiter.distance_runway)
        .visibility(Parameter::VISIBILITY_USER)
        .units(Units::Meter)
        .defaultValue("100.0")
        .description("Distance from loiter to start of runway");

        param("Automatic", m_args.automatic_generation)
        .visibility(Parameter::VISIBILITY_USER)
        .description("If true: The dune task generates the start and finish circle automatic.")
        .defaultValue("true");

        param("VR_netWGS84Height", m_args.netWGS84Height)
        .visibility(Parameter::VISIBILITY_USER)
        .description("Offset in height")
        .units(Units::Meter)
        .defaultValue("0.0");

        param("Dubins path right start direction", m_args.rightStartTurningDirection)
        .visibility(Parameter::VISIBILITY_USER)
        .defaultValue("true");

        param("Dubins path right finish direction", m_args.rightFinishTurningCircle)
        .visibility(Parameter::VISIBILITY_USER)
        .defaultValue("true");

        bind<IMC::PlanDB>(this);
        bind<IMC::PlanControl>(this);
        bind<IMC::PlanControlState>(this);
        bind<IMC::PlanSpecification>(this);
        bind<IMC::PathControlState>(this);
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
      consume(const IMC::PathControlState* pcs){

//        if (pcs->flags & IMC::PathControlState::FL_LOITERING){
//          m_args.fw_loiter.lat = pcs->end_lat;
//          m_args.fw_loiter.lon = pcs->end_lon;
//          m_args.fw_loiter.altitude = pcs->end_z;
//          m_args.fw_loiter.radius_second = pcs->lradius;
//        }
      }

      void
      consume(const IMC::PlanDB* msg)
      {
        trace("Got PlanDB:\n from '%s' at '%s'\n to '%s' at '%d'\n",
              resolveEntity(msg->getSourceEntity()).c_str(),
              resolveSystemId(msg->getSource()),
              resolveSystemId(msg->getDestination()),
              msg->getDestinationEntity());

        if(msg->plan_id == "land" && msg->op == IMC::PlanDB::DBOP_SET && msg->type == IMC::PlanDB::DBT_REQUEST){
          debug("Received requested dubinspath");
          const IMC::PlanSpecification* dubins_planspec = static_cast<const IMC::PlanSpecification*>(msg->arg.get());
          debug("Extracting PlanSpecification..");

          sendFixedWingPlan(dubins_planspec,virtual_runway.nr);
          debug("Sent FixedWing NetRecovery plan");


        }

        //msg->toText(std::cout);
        // Ignore if not reply for this entity
        /*
        if (msg->getDestination() != getSystemId()
            || msg->getDestinationEntity() != getEntityId())
        {
          trace("Ignoring PlanDB with other destination");
          return;
        }
        */
        /*
        if (msg->type == IMC::PlanDB::DBT_REQUEST)
          trace("Plan DB request\n");
        if (msg->type == IMC::PlanDB::DBT_SUCCESS)
        {
          if (msg->op == IMC::PlanDB::DBOP_GET_INFO)
            trace("Plan DB get info\n");
          if (msg->op == IMC::PlanDB::DBOP_SET)
            trace("Plan DB set");
          if (msg->op == IMC::PlanDB::DBOP_GET_STATE)
            trace("Plan DB get state\n");
        }
        */
        if (   msg->getDestination() == getSystemId()
            && msg->getSource()      != getSystemId()
            && msg->type == IMC::PlanDB::DBT_REQUEST
            && msg->op   == IMC::PlanDB::DBOP_SET)
        {
          debug(
              "Neptus setting Plan in local DB, checking if there exists a NetRecovery maneuver in the plan");
          // Check if plan has NetRecovery maneuver
          try
          {
            const IMC::PlanSpecification* planspec =
                static_cast<const IMC::PlanSpecification*>(msg->arg.get());
            IMC::MessageList<IMC::PlanManeuver>::const_iterator it =
                planspec->maneuvers.begin();
            IMC::NetRecovery nr_maneuver;
            trace("Plan: '%s'",planspec->plan_id.c_str());
            for (; it != planspec->maneuvers.end(); it++)
            {
              trace("Maneuver: '%s'",(*it)->data->getName());
              if ( (*it)->data->getId() == nr_maneuver.getId() )
              {
                debug("NetRecovery maneuver set request dispatched from Neptus");
                IMC::Maneuver* man = (*it)->data.get();
                IMC::NetRecovery* nr = static_cast<IMC::NetRecovery*>(man);
                virtual_runway.nr = *nr;
                extractVirtualRunway(nr);
                requestDubins(); //Request dubins-path from uav position to loiter.
                debug("Dubins requested!");
              //  sendFixedWingPlan(planspec->plan_id,nr);
              //  debug("Sent FixedWing NetRecovery plan");
                break;
              }
            }
          }
          catch (std::exception& e)
          {
            err(DTR(
                "Plan maneuver request failed with uncaught exception: %s"),
                e.what());
          }
        }
      }

      void
      consume(const IMC::PlanControl* msg)
      {
        trace("Got PlanControl \nfrom '%s' at '%s'",
              resolveEntity(msg->getSourceEntity()).c_str(),
              resolveSystemId(msg->getSource()));
        //msg->toText(std::cout);

        // Check if system was destination
        if (msg->getDestination() != this->getSystemId())
        {
          trace("Ignored PlanControl with wrong destination '%s'",
                resolveSystemId(msg->getDestination()));
          return;
        }

        // Ignore if not a request
        if (msg->type != IMC::PlanControl::PC_REQUEST)
        {
          trace("Ignored PlanControl (not a request)");
          return;
        }

        if (msg->op == IMC::PlanControl::PC_LOAD)
        {

        }
        else if (msg->op == IMC::PlanControl::PC_START)
        {
          debug("PlanControl START");

        }
        else if (msg->op == IMC::PlanControl::PC_STOP)
        {
          debug("PlanControl STOP");
        }
      }

      void
      consume(const IMC::PlanControlState* pcs)
      {
        spew("Got PlanControlState \nfrom '%s' at '%s'",
              resolveEntity(pcs->getSourceEntity()).c_str(),
              resolveSystemId(pcs->getSource()));
        //pcs->toText(std::cout);

        // Check if we just stopped executing a plan
        if (m_last_pcs.state == IMC::PlanControlState::PCS_EXECUTING
            && pcs->state == IMC::PlanControlState::PCS_READY)
        {
          debug("Plan stopped");
          // Check if success
          if (pcs->last_outcome == IMC::PlanControlState::LPO_SUCCESS)
          {
            debug("Plan success");
          }
        }

        m_last_pcs = *pcs;

      }

      void
      consume(const IMC::PlanSpecification* spec)
      {
        spew("Got PlanControlState \nfrom '%s' at '%s'",
              resolveEntity(spec->getSourceEntity()).c_str(),
              resolveSystemId(spec->getSource()));
        trace("PlanDB destination \n '%s' at '%d'",
              resolveSystemId(spec->getDestination()),
              spec->getDestinationEntity());
        if (spec->getSource() != getSystemId()
            && spec->getDestination() == getSystemId())
        {
          trace("PlanSpecification from remote to me");
        }
        else
        {
          return;
        }
      }

      void extractVirtualRunway(IMC::NetRecovery* maneuver){

        double bearing;
        double range;

        WGS84::getNEBearingAndRange(maneuver->start_lat,maneuver->start_lon,
                                    maneuver->end_lat,maneuver->end_lon,
                                             &bearing,&range);

        virtual_runway.VR_heading    = bearing;
        virtual_runway.VR_length     = range;
        virtual_runway.VR_altitude   = (maneuver->z + maneuver->z_off);

        double N_offset   = virtual_runway.VR_length/2 * cos(virtual_runway.VR_heading);
        double E_offset   = virtual_runway.VR_length/2 * sin(virtual_runway.VR_heading);

        double lat_offset = maneuver->start_lat;
        double lon_offset = maneuver->start_lon;
        double height_offset = 0.0;

        WGS84::displace(N_offset, E_offset, 0.0,
                  &lat_offset, &lon_offset, &height_offset);

        virtual_runway.VR_center_lat = Angles::degrees(lat_offset);
        virtual_runway.VR_center_lon = Angles::degrees(lon_offset);
        m_args.desired_speed = maneuver->speed;

        inf("VR center lat = %f, VR ceter lon = %f",virtual_runway.VR_center_lat,virtual_runway.VR_center_lon);

      }

      void
      requestDubins(){
        IMC::PlanGeneration msg;

        msg.params.append("land_lat=").append(DoubleToString(virtual_runway.VR_center_lat)).append(";");
        msg.params.append("land_lon=").append(DoubleToString(virtual_runway.VR_center_lon)).append(";");

        msg.params.append("land_heading=").append(DoubleToString(Angles::degrees(Angles::normalizeRadian(virtual_runway.VR_heading+Math::c_pi)))).append(";");

        msg.params.append("net_height=").append(DoubleToString(-virtual_runway.VR_altitude)).append(";");


        msg.params.append("start_turning_circle_radius=").append(DoubleToString(m_args.fw_loiter.radius_first)).append(";");
        msg.params.append("final_turning_circle_radius=").append(DoubleToString(m_args.fw_loiter.radius_second)).append(";");


        msg.params.append("final_approach_angle=").append(DoubleToString(0.0)).append(";");
        msg.params.append("glide_slope_anglee=").append(DoubleToString(m_args.glideslope_angle)).append(";");

        msg.params.append("dist_behind=").append(DoubleToString((virtual_runway.VR_length/2.0))).append(";");
        msg.params.append("final_approach=").append(DoubleToString((virtual_runway.VR_length/2.0))).append(";");

        msg.params.append("glideslope=").append(DoubleToString(m_args.glideslope_distance)).append(";");

        msg.params.append("approach=").append(DoubleToString(m_args.glideslope_approach_distance)).append(";");
        msg.params.append("speed12=").append(DoubleToString(m_args.desired_speed)).append(";");
        msg.params.append("speed345=").append(DoubleToString(m_args.desired_speed)).append(";");

        msg.params.append("z_unit=height;");// "height" or "altitude"
        msg.params.append("net_WGS84_height=").append(DoubleToString(m_args.netWGS84Height)).append(";");

      //  msg.params += "auxiliary_WPa_side=" + auxiliaryWpaRight + ";";
      //   msg.params += "ignore_evasive=" + m_args.ignoreEvasive + ";";

        msg.params.append("automatic=").append(m_args.automatic_generation).append(";");

        msg.params.append("start_turning_circle_counter_clockwise=").append(DoubleToString(m_args.rightStartTurningDirection)).append(";");
        msg.params.append("final_turning_circle_counter_clockwise=").append(DoubleToString(m_args.rightFinishTurningCircle)).append(";");

        msg.op = IMC::PlanGeneration::OP_REQUEST;
        msg.cmd = IMC::PlanGeneration::CMD_GENERATE;
        msg.plan_id = "land";

        dispatch(msg);

      }
      void
      sendFixedWingPlan(const IMC::PlanSpecification* dubins_planspec, IMC::NetRecovery maneuver)
      {
        // Create plan set request
        IMC::PlanDB plan_db;
        plan_db.type = IMC::PlanDB::DBT_REQUEST;
        plan_db.op = IMC::PlanDB::DBOP_SET;
        plan_db.plan_id = dubins_planspec->plan_id + "_fixedwing";
        plan_db.request_id = 0;
        debug("Created a new plan DB..");

        debug("Trying to create new PlanSpecification...");
        IMC::PlanSpecification plan_spec = *dubins_planspec;
        debug("Created new PlanSpecification...");
        plan_spec.description = "A fixed wing recovery plan";
        plan_spec.plan_id = dubins_planspec->plan_id + "_fixedwing";
        plan_spec.start_man_id = "1";

//        // Add current loiter as initial maneuver
//
//        IMC::PlanManeuver loiter_man;
//        loiter_man.maneuver_id = 1;
//
//        IMC::Loiter man_loit;
//        man_loit.type = IMC::Loiter::LT_CIRCULAR;
//        man_loit.direction = IMC::Loiter::LD_CLOCKW;
//
//        man_loit.z             = m_args.fw_loiter.altitude;
//        man_loit.z_units       = IMC::Z_ALTITUDE;
//        man_loit.speed         = m_args.fw_loiter.speed;
//        man_loit.speed_units   = IMC::SUNITS_METERS_PS;
//        man_loit.duration      = m_args.fw_loiter.duration;
//        man_loit.radius        = m_args.fw_loiter.radius;
//        man_loit.lat           = m_args.fw_loiter.lat;
//        man_loit.lon           = m_args.fw_loiter.lon;

        //Calculate go-to approach point based on desired along-track position behind the start point of the virtual runway

//        double bearing;
//        double range;
//
//
//        WGS84::getNEBearingAndRange(maneuver->start_lat,maneuver->start_lon,
//                                    maneuver->end_lat,maneuver->end_lon,
//                                    &bearing,&range);
//
//        double z_diff = abs(m_args.fw_loiter.altitude - maneuver->z);
//        double distance = z_diff / tan(Angles::radians(m_args.fw_loiter.glideslope_angle));
        double z_loiter = (maneuver.z - maneuver.z_off) + tan(Angles::radians(m_args.glideslope_angle)) * m_args.glideslope_distance;
        //GO-TO Glideslope START POINT
        double N_offset   = -m_args.glideslope_distance * cos(virtual_runway.VR_heading);
        double E_offset   = -m_args.glideslope_distance * sin(virtual_runway.VR_heading);
        double lat_offset = maneuver.start_lat;
        double lon_offset = maneuver.start_lon;
        double height_offset = 0.0;

        WGS84::displace(N_offset, E_offset, 0.0,
                  &lat_offset, &lon_offset, &height_offset);

        //man_loit.lat = lat_offset;
        //man_loit.lon = lon_offset;

        // Add goto-point
        IMC::Goto man_goto_glideslope;

        man_goto_glideslope.lat           = lat_offset;
        man_goto_glideslope.lon           = lon_offset;
        man_goto_glideslope.speed         = m_args.fw_loiter.speed;
        man_goto_glideslope.speed_units   = IMC::SUNITS_METERS_PS;
        man_goto_glideslope.z             = z_loiter;
        man_goto_glideslope.z_units       = IMC::Z_ALTITUDE;


        debug("Goto created...");

        //GO-TO APPROACH GLIDESLOPE POINT
        N_offset   = -(m_args.glideslope_distance+m_args.glideslope_approach_distance) * cos(virtual_runway.VR_heading);
        E_offset   = -(m_args.glideslope_distance+m_args.glideslope_approach_distance) * sin(virtual_runway.VR_heading);
        lat_offset = maneuver.start_lat;
        lon_offset = maneuver.start_lon;
        height_offset = 0.0;

        WGS84::displace(N_offset, E_offset, 0.0,
                  &lat_offset, &lon_offset, &height_offset);

        //man_loit.lat = lat_offset;
        //man_loit.lon = lon_offset;

        // Add goto-point

        IMC::Goto man_goto_approach_glideslope;

        man_goto_approach_glideslope.lat           = lat_offset;
        man_goto_approach_glideslope.lon           = lon_offset;
        man_goto_approach_glideslope.speed         = m_args.fw_loiter.speed;
        man_goto_approach_glideslope.speed_units   = IMC::SUNITS_METERS_PS;
        man_goto_approach_glideslope.z             = z_loiter;
        man_goto_approach_glideslope.z_units       = IMC::Z_ALTITUDE;
        //
       // IMC::MessageList<IMC::PlanManeuver>  manlist = plan_spec.maneuvers;


        //
        IMC::PlanManeuver* pman1 = new IMC::PlanManeuver();
        IMC::InlineMessage<IMC::Maneuver> pman1_inline;
        pman1_inline.set(man_goto_approach_glideslope);
        pman1->maneuver_id = "3";
        pman1->data = pman1_inline;

        IMC::PlanManeuver* pman2  = new IMC::PlanManeuver();
        IMC::InlineMessage<IMC::Maneuver> pman2_inline;
        pman2_inline.set(man_goto_glideslope); // SEGMENTATION FAULT
        pman2->maneuver_id = "4";
        pman2->data = pman2_inline;

        IMC::PlanManeuver* pman3 = new IMC::PlanManeuver();
        IMC::InlineMessage<IMC::Maneuver> pman3_inline;
        pman3_inline.set(maneuver);
        pman3->maneuver_id = "5";
        pman3->data = pman3_inline;

        IMC::PlanTransition* transition1 = new IMC::PlanTransition;
        transition1->source_man = "2";
        transition1->dest_man   = "3";
        transition1->conditions = "ManeuverIsDone";

        debug("PlanTransition 1 created");

        IMC::PlanTransition* transition2 = new IMC::PlanTransition;
        transition2->source_man = "3";
        transition2->dest_man   = "4";
        transition2->conditions = "ManeuverIsDone";

        IMC::PlanTransition* transition3 = new IMC::PlanTransition;
        transition3->source_man = "4";
        transition3->dest_man   = "5";
        transition3->conditions = "ManeuverIsDone";



        //IMC::MessageList<IMC::PlanTransition> translist = plan_spec.transitions;
        debug("Push back trans list");
        plan_spec.transitions.push_back(transition1);
        plan_spec.transitions.push_back(transition2);
        plan_spec.transitions.push_back(transition3);

        debug("Push back manuvers list");
        plan_spec.maneuvers.push_back(pman1);
        plan_spec.maneuvers.push_back(pman2);
        plan_spec.maneuvers.push_back(pman3);

//        plan_spec.start_man_id = "1";
//        plan_spec.maneuvers = manlist;
//        plan_spec.transitions = translist;

        // Set plan
        debug("Setting plan spec..");
        plan_db.arg.set(plan_spec);
        debug("plan spec is set");
        // Send set plan request
        debug("Trying to send planDB");
        dispatch(plan_db);
        debug("PlanDB sent");

        // Create and send plan start request
        /*
        IMC::PlanControl plan_ctrl;
        plan_ctrl.type = IMC::PlanControl::PC_REQUEST;
        plan_ctrl.op = IMC::PlanControl::PC_START;
        plan_ctrl.plan_id = plan_spec.plan_id;
        plan_ctrl.request_id = 0;
        plan_ctrl.arg.set(plan_spec);
        dispatch(plan_ctrl);
        */
      }

      std::string DoubleToString ( double number )
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
    };
  }
}

DUNE_TASK
