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
// Author: Jostein B. Moe                                                   *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

#include <vector>
#include <queue>

namespace Maneuver
{
  namespace NetCatchCoordinator
  {
    using DUNE_NAMESPACES;
    
    //! %Task arguments.
    struct Arguments
    {
      //! Enable coordinated catch
      bool enable_coord;
      //! Enable catch-state
      bool enable_catch;
      //! Stop and hold at end of runway
      bool enable_stop_endRunway;
      //! Enable this path controller or not
      bool use_controller;
      //! Enable mean window for aircraft states
      bool use_mean_window_aircraft;
      //! Disable Z flag, this will utilize new rate controller on some targets
      bool disable_Z;

      //! Moving mean window size
      double mean_ws;
      //! Desired velocity of net at impact
      double m_ud_impact;
      //! Desired time to accelerate to desired velocity of net at impact
      double m_td_acc;
      //! Desired offset-cross track
      double m_crosstrack_offset;
      //! Desired collision radius
      double m_coll_r;
      //! Radius to stop at end of runway
      double m_endCatch_radius;

      //!position Controller parameters
      Matrix Kp;
	  Matrix Ki;
	  Matrix Kd;

	  double max_norm_v;

      //! Frequency of controller
      double m_freq;

      //! Maximum cross-track error aircraft
      Matrix eps_ct_a;
      //! Maximum cross-track error net
      Matrix eps_ct_n;
    };

    struct VirtualRunway
    {
	  fp64_t lat_start, lon_start, hae_start;
	  fp64_t lat_end, lon_end, hae_end;

	  fp32_t box_height, box_width, box_length;

      //! Course of runway
      double alpha;
      //! Pitch of runway
      double theta;

      Matrix start_NED;
      Matrix end_NED;
    };

    struct Vehicles
    {
    	int no_vehicles;
    	std::string aircraft;
    	std::vector<std::string> copters;
    };

    enum Vehicle {AIRCRAFT=0, COPTER_LEAD, COPTER_FOLLOW, INVALID=-1};

    struct Task: public DUNE::Control::PeriodicUAVAutopilot
    {
      //! Task arguments.
      Arguments m_args;
      //! Current state
      IMC::NetRecoveryState::NetRecoveryLevelEnum m_curr_state;

      VirtualRunway m_runway;
      Vehicles m_vehicles;

  	  //! Localization origin (WGS-84)
  	  fp64_t m_ref_lat, m_ref_lon;
  	  fp32_t m_ref_hae;
      bool m_ref_valid;

      //! Ready to read state-messages
      bool m_coordinatorEnabled;
      //! Vehicles initialized
      std::vector<bool> m_initialized;
      //! All parameters well defined
      bool m_initializedCoord;

      //! Last EstimatedLocalState received
      std::vector<IMC::EstimatedLocalState> m_estate;
      
      //! Last positions in NED
      std::vector<Matrix> m_p;
      //! Last velocities in NED
      std::vector<Matrix> m_v;
      
      //! Cross track errors
      std::vector<Matrix> m_p_path;
      std::vector<std::queue<Matrix> > m_cross_track_window;
      std::vector<Matrix> m_p_path_mean;
      //! Cross track errors derivative
      std::vector<Matrix> m_v_path;
      std::vector<std::queue<Matrix> > m_cross_track_d_window;
      std::vector<Matrix> m_v_path_mean;

      //! Position difference along path
      double delta_p_path_x;
      double delta_p_path_x_mean;
      //! Velocity difference along path
      double delta_v_path_x;
      double delta_v_path_x_mean;

      Matrix m_p_int_value;

     //! Radius to start net-catch (calculate based on net-acceleration)
      double m_startCatch_radius;

      //! Timeout for operation
      uint16_t m_timeout;
      //! Along-track velocity reference value
      fp32_t m_u_ref;
      //! Desired along-track velocity
      fp32_t m_ud;
      //! Desired along-track acceleration
      fp32_t m_ad;

      //! Control loops last reference
      uint32_t m_scope_ref;

      //! To print the control-loop frequency
      uint64_t m_time_end;

      //! To print the control-loop frequency
	  uint64_t m_time_diff;

      //! Controllable loops.
      static const uint32_t c_controllable = IMC::CL_PATH;
      //! Required loops.
      static const uint32_t c_required = IMC::CL_SPEED;


      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
    	PeriodicUAVAutopilot(name, ctx, c_controllable, c_required),
        m_ud(0),
        m_initializedCoord(false),
        m_scope_ref(0),
        m_ref_lat(0.0),
		m_ref_lon(0.0),
		m_ref_hae(0.0),
        m_ref_valid(false),
        m_coordinatorEnabled(false),
		delta_p_path_x(0),
		delta_p_path_x_mean(0),
		delta_v_path_x(0),
		delta_v_path_x_mean(0),
		m_p_int_value(3,1,0.0),
	    m_startCatch_radius(0),
		m_timeout(0),
        m_u_ref(0.0),
        m_ad(0.0),
        m_time_end(Clock::getMsec()),
		m_time_diff(0)
      {
 	    param("Path Controller", m_args.use_controller)
	    .visibility(Tasks::Parameter::VISIBILITY_USER)
	    .scope(Tasks::Parameter::SCOPE_MANEUVER)
	    .defaultValue("false")
	    .description("Enable Path Controller");

 	    param("Offset cross-track", m_args.m_crosstrack_offset)
	    .visibility(Tasks::Parameter::VISIBILITY_USER)
	    .scope(Tasks::Parameter::SCOPE_MANEUVER)
	    .defaultValue("0.0")
	    .description("Cross-track offset");

	    param("Stop at end-of-runway", m_args.enable_stop_endRunway)
	    .visibility(Tasks::Parameter::VISIBILITY_USER)
	    .scope(Tasks::Parameter::SCOPE_MANEUVER)
	    .defaultValue("false")
	    .description("Enable stop at end of runway");

 	    param("Enable Mean Window Aircraft", m_args.use_mean_window_aircraft)
	    .visibility(Tasks::Parameter::VISIBILITY_USER)
	    .scope(Tasks::Parameter::SCOPE_MANEUVER)
 	    .defaultValue("false")
 	    .description("Use mean window on aircraft states");

 	    param("Frequency", m_args.m_freq)
	    .visibility(Tasks::Parameter::VISIBILITY_USER)
	    .scope(Tasks::Parameter::SCOPE_MANEUVER)
        .defaultValue("1.0")
        .description("Controller frequency");

 	    param("Disable Z flag", m_args.disable_Z)
        .defaultValue("false")
        .visibility(Tasks::Parameter::VISIBILITY_USER)
        .description("Choose whether to disable Z flag. In turn, this will utilize new rate controller on some targets");

        param("Coordinated Catch", m_args.enable_coord)
	    .visibility(Tasks::Parameter::VISIBILITY_USER)
	    .scope(Tasks::Parameter::SCOPE_MANEUVER)
        .defaultValue("false")
        .description("Flag to enable net catch with two multicopters");

        param("Enable Catch", m_args.enable_catch)
	    .visibility(Tasks::Parameter::VISIBILITY_USER)
	    .scope(Tasks::Parameter::SCOPE_MANEUVER)
        .defaultValue("false")
        .description("Flag to enable catch state of the state-machine");
        
        param("Mean Window Size", m_args.mean_ws)
	    .visibility(Tasks::Parameter::VISIBILITY_USER)
	    .scope(Tasks::Parameter::SCOPE_MANEUVER)
        .defaultValue("1.0")
        .description("Number of samples in moving average window");

        param("Maximum Cross-Track Error Aircraft", m_args.eps_ct_a)
        .units(Units::Meter);

        param("Maximum Cross-Track Error Net", m_args.eps_ct_n)
        .units(Units::Meter);

        param("Desired collision radius", m_args.m_coll_r)
        .defaultValue("100.0")
        .units(Units::Meter);

        param("Desired switching radius", m_args.m_endCatch_radius)
	    .visibility(Tasks::Parameter::VISIBILITY_USER)
	    .scope(Tasks::Parameter::SCOPE_MANEUVER)
        .defaultValue("10.0")
        .units(Units::Meter);

        param("Kp Position Control", m_args.Kp)
	    .visibility(Tasks::Parameter::VISIBILITY_USER)
        .defaultValue("1.0,1.0,1.0")
        .description("Position Controller tuning parameter Kp");

        param("Ki Position Control", m_args.Ki)
        .visibility(Tasks::Parameter::VISIBILITY_USER)
        .defaultValue("0.0,0.0,0.0")
        .description("Position Controller tuning parameter Ki");

        param("Kd Position Control", m_args.Kd)
        .visibility(Tasks::Parameter::VISIBILITY_USER)
        .defaultValue("0.0,0.0,0.0")
        .description("Position Controller tuning parameter Kd");

        param("Maximum Normalised Velocity", m_args.max_norm_v)
        .defaultValue("5.0")
        .description("Maximum Normalised Velocity of the Copter");

        // Bind incoming IMC messages
        bind<IMC::EstimatedLocalState>(this);
        bind<IMC::DesiredNetRecoveryPath>(this);
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
		debug("Current frequency: %f",getFrequency());
		setFrequency(m_args.m_freq);
		debug("Frequency changed to : %f",getFrequency());
      }

 
      void
      consume(const IMC::DesiredNetRecoveryPath* msg)
      {
   
	   if (!isActive())
	   {
		 err(DTR("not active"));
		 return;
	   }

        trace("Got DesiredNetRecovery \nfrom '%s' at '%s'",
		     resolveEntity(msg->getSourceEntity()).c_str(),
		     resolveSystemId(msg->getSource()));
  	    inf("Initialize net recovery maneuver");

    	debug("Start (ll): [%f,%f]",msg->start_lat,msg->start_lon);
    	debug("End (ll): [%f,%f]",msg->end_lat,msg->end_lon);
    	debug("z: %f",msg->z);
    	debug("z_off: %f",msg->z);
    	debug("box [h x w]=[%f x %f]=",msg->lbox_height,msg->lbox_width);
    	debug("Speed: %f",msg->speed);
    	debug("Max acceleration: %f",msg->max_acc);
    	debug("\n\n");

  	    m_vehicles.aircraft = msg->aircraft;
  	    m_vehicles.no_vehicles = 1;
  	    m_vehicles.copters = std::vector<std::string>();
        std::stringstream  lineStream(msg->multicopters.c_str());
        std::string        copter;
        while(std::getline(lineStream,copter,','))
        {
        	m_vehicles.no_vehicles++;
        	m_vehicles.copters.push_back(copter);
        }
        inf("No vehicles: %d",m_vehicles.no_vehicles);
        inf("Aircraft: %s",m_vehicles.aircraft.c_str());
        for(unsigned int i=0; i < m_vehicles.copters.size(); i++)
        {
        	inf("Multicopter[%d]: %s",i,m_vehicles.copters[i].c_str());
        }

  	    m_runway.lat_start = msg->start_lat;
  	    m_runway.lon_start = msg->start_lon;
  	    m_runway.hae_start = msg->z + msg->z_off;
  	    m_runway.lon_end = msg->end_lon;
  	    m_runway.lat_end = msg->end_lat;
  	    m_runway.hae_end = msg->z + msg->z_off;

  	    m_runway.box_height = msg->lbox_height;
  	    m_runway.box_width = msg->lbox_width;

        m_u_ref   = msg->speed;
        m_ad 	  = msg->max_acc;

  	    m_coordinatorEnabled = true;
  	    inf("Maneuver enabled");
      }

      void
      consume(const IMC::EstimatedLocalState* estate)
      {
        if (!m_coordinatorEnabled)
        	return;

    	if (!m_initializedCoord)
        {
    	  debug("Initializing coordinator after EstimatedLocalState");
    	  m_ref_lat = estate->lat;
		  m_ref_lon = estate->lon;
		  m_ref_hae = estate->height;
		  m_ref_valid = true;
          initCoordinator();
          initRunwayPath();
          //return;
          debug("Initialized coordinator after EstimatedLocalState");
        }

        //spew("Got EstimatedState \nfrom '%s' at '%s'",
        //     resolveEntity(estate->getSourceEntity()).c_str(),
        //     resolveSystemId(estate->getSource()));

        std::string vh_id  = resolveSystemId(estate->getSource());
        //spew("Received EstimatedLocalState from %s: ",vh_id.c_str());
        int s = getVehicle(vh_id);
        //spew("s=%d",s);
        if (s == INVALID)  //invalid vehicle
          return;

        Matrix p = Matrix(3,1,0); //position in NED
        Matrix v = Matrix(3,1,0); //velocity in NED

        p(0) = estate->x;
        p(1) = estate->y;
        p(2) = estate->z;
        m_p[s] = p;

        v(0) = estate->vx;
        v(1) = estate->vy;
        v(2) = estate->vz;
        m_v[s] = v;

        m_initialized[s] = true;
        m_estate[s]       = *estate;

        calcPathErrors(p, v, s);
        updateMeanValues(s);

        IMC::NetRecoveryState::NetRecoveryLevelEnum last_state = m_curr_state;
        switch(m_curr_state)
        {
          case IMC::NetRecoveryState::NR_INIT:
          {

            if (allInitialized())
            {
              m_curr_state = IMC::NetRecoveryState::NR_STANDBY;

              /**********************
              //!!!!DEBUG!!!
              ***********************/
            		//m_curr_state = IMC::NetRecoveryState::NR_STOP;
              inf("Initialized, at standby");
            }
            break;
          }
          case IMC::NetRecoveryState::NR_STANDBY:
          {
        	updateMeanValues(s);
            if (aircraftApproaching() && m_args.enable_catch)
            	m_curr_state = IMC::NetRecoveryState::NR_APPROACH; //requires that the net is standby at the start of the runway

            break;
          }
          case IMC::NetRecoveryState::NR_APPROACH:
          {
              updateMeanValues(s);

              double v_a = abs(m_v_path[AIRCRAFT](0));
              m_startCatch_radius = updateStartRadius(v_a,0, m_u_ref, m_ad, m_args.m_coll_r);
              if (m_startCatch_radius == -1)
              {
              	err("Unable to calculate the desired start-radius");
              	return;
              }
              if (s==AIRCRAFT)
              {
              	/*
              	spew("v_a: %f",v_a);
              	spew("p[%d]: [%f,%f,%f]",s,p(0),p(1),p(2));
                  spew("v[%d]: [%f,%f,%f]",s,v(0),v(1),v(2));
                  spew("m_runway.start_NED[%d]: [%f,%f,%f]",s,m_runway.start_NED(0),m_runway.start_NED(1),m_runway.start_NED(2));
                  spew("m_runway.end_NED [%d]: [%f,%f,%f]",s,m_runway.end_NED(0),m_runway.end_NED(1),m_runway.end_NED(2));
  				spew("m_p_path[%d]: [%f,%f,%f]",s,m_p_path[s](0),m_p_path[s](1),m_p_path[s](2));
  				spew("m_v_path[%d]: [%f,%f,%f]",s,m_v_path[s](0),m_v_path[s](1),m_v_path[s](2));
  				spew("m_p_path_mean[%d]: [%f,%f,%f]",s,m_p_path_mean[s](0),m_p_path_mean[s](1),m_p_path_mean[s](2));
  				spew("m_v_path_mean[%d]: [%f,%f,%f]",s,m_v_path_mean[s](0),m_v_path_mean[s](1),m_v_path_mean[s](2));
  		        spew("delta_p_path_x = %f",delta_p_path_x);
  		        spew("delta_v_path_x = %f",delta_v_path_x);
  		        spew("StartCatch radius: %f",m_startCatch_radius);
  				spew("\n\n");
  				*/
              }

        	  if (startNetRecovery())
        		 m_curr_state = IMC::NetRecoveryState::NR_START;

        	  break;
          }
          case IMC::NetRecoveryState::NR_START:
          {
            updateMeanValues(s);
            m_ud = getPathVelocity(0, m_u_ref, m_ad, false);

            if (catched())
              m_curr_state = IMC::NetRecoveryState::NR_CATCH;

            if (endAtRunway())
              m_curr_state = IMC::NetRecoveryState::NR_END;

            break;
          }
          case IMC::NetRecoveryState::NR_CATCH:
          {
        	if (endAtRunway())
			  m_curr_state = IMC::NetRecoveryState::NR_END;

        	break;
          }
          case IMC::NetRecoveryState::NR_END:
          {
        	m_curr_state = IMC::NetRecoveryState::NR_STOP;
            break;
          }
          case IMC::NetRecoveryState::NR_STOP:
          {
        	 //signal the supervisor that the maneuver is done
        	 break;
          }
          case IMC::NetRecoveryState::NR_ABORT:
          {
            break;
          }                            
        }        
        //inf("Send current state");
        sendCurrentState();
        //inf("Current state sent");
        if (last_state != m_curr_state)
        	inf("Current state: %d",static_cast<NetRecoveryState::NetRecoveryLevelEnum>(m_curr_state));
        if (m_curr_state == IMC::NetRecoveryState::NR_STOP)
        {
        	//inf("test state %d", m_curr_state);
        	//onAutopilotDeactivation();
        	//inf("test state %d", m_curr_state);
        	//m_curr_state = IMC::NetRecoveryState::NR_INIT;
        }
        //inf("Here");
      }

      void
      initCoordinator()
      {
        if (m_initializedCoord)
        {
          inf("Coordinator already initialized");
          return;
        }
        unsigned int no_vehicles = m_vehicles.no_vehicles;
        
        m_estate        = std::vector<IMC::EstimatedLocalState>(no_vehicles);
        m_p             = std::vector<Matrix>(no_vehicles);
        m_v             = std::vector<Matrix>(no_vehicles);

        m_p_path          = std::vector<Matrix>(no_vehicles);
        m_v_path        = std::vector<Matrix>(no_vehicles);
        m_p_path_mean   = std::vector<Matrix>(no_vehicles);
        m_v_path_mean = std::vector<Matrix>(no_vehicles);
        m_cross_track_window   = std::vector<std::queue<Matrix> >(no_vehicles);
        m_cross_track_d_window = std::vector<std::queue<Matrix> >(no_vehicles);
        m_initialized = std::vector<bool>(no_vehicles);

        for (unsigned int i=0; i<no_vehicles; i++) {
          m_p[i] = Matrix(3,1,0);
          m_v[i] = Matrix(3,1,0);
          m_p_path[i]   = Matrix(3,1,0);
          m_v_path[i] = Matrix(3,1,0);
          m_p_path_mean[i]   = Matrix(3,1,0);
          m_v_path_mean[i] = Matrix(3,1,0);
          debug("m_p_path[%d]: Rows: %d, Cols: %d",i,m_p_path[i].rows(),m_p_path[i].columns());
          debug("m_v_path[%d]: Rows: %d, Cols: %d",i,m_v_path[i].rows(),m_v_path[i].columns());
          debug("m_p_path_mean[%d]: Rows: %d, Cols: %d",i,m_p_path_mean[i].rows(),m_p_path_mean[i].columns());
          debug("m_v_path_mean[%d]: Rows: %d, Cols: %d",i,m_v_path_mean[i].rows(),m_v_path_mean[i].columns());
          m_cross_track_window[i]   = std::queue<Matrix>();
          m_cross_track_d_window[i] = std::queue<Matrix>();
          for (unsigned int j=0; j < m_args.mean_ws; j++)
          {
            m_cross_track_window[i].push(Matrix(3,1,0));
            m_cross_track_d_window[i].push(Matrix(3,1,0));
          }
          m_initialized[i] = false;
        }
        m_runway.end_NED = Matrix(3,1,0);
        m_runway.start_NED = Matrix(3,1,0);

        m_curr_state = IMC::NetRecoveryState::NR_INIT;
        m_initializedCoord = true;
        inf("Coordinator initialized");
      }

      void
      initRunwayPath()
      {
          WGS84::displacement(m_ref_lat, m_ref_lon, 0,
          				    m_runway.lat_start, m_runway.lon_start, 0,
          					&m_runway.start_NED(0), &m_runway.start_NED(1));
          m_runway.start_NED(2) = -m_runway.hae_start;

          WGS84::displacement(m_ref_lat, m_ref_lon, 0,
          					m_runway.lat_end, m_runway.lon_end, 0,
                              &m_runway.end_NED(0), &m_runway.end_NED(1));
          m_runway.end_NED(2) = -m_runway.hae_end;

          Matrix deltaWP = m_runway.end_NED - m_runway.start_NED;


          m_runway.box_length = deltaWP.norm_2();

          double deltaWP_NE = deltaWP.get(0,1,0,0).norm_2(); 

          m_runway.alpha=  atan2(deltaWP(1),deltaWP(0));
          m_runway.theta = -atan2(deltaWP_NE,deltaWP(2)) + Angles::radians(90);
      }

      void
      calcPathErrors(Matrix p, Matrix v, int s)
      {
    	  //spew("p_NED[%d]: [%f,%f,%f]",s,p(0),p(1),p(2));

          Matrix R       = transpose(Rzyx(0.0, -m_runway.theta, m_runway.alpha));
          Matrix eps     = R*(p-m_runway.start_NED);
          Matrix eps_dot = R*v;

          m_p_path[s] = eps;
          m_v_path[s] = eps_dot;

          //spew("m_p_path[%d]: [%f,%f,%f]",s,m_p_path[s](0),m_p_path[s](1),m_p_path[s](2));

          Matrix p_n = getNetPosition(m_p);
          Matrix v_n = getNetVelocity(m_v);

          Matrix delta_p_path = R*(p_n-m_p[AIRCRAFT]);
          Matrix delta_v_path = R*(v_n-m_v[AIRCRAFT]);

          delta_p_path_x = delta_p_path(0);
          delta_v_path_x = delta_v_path(0);
      }

      void
      updateMeanValues(int s)
      {
        if (m_cross_track_window[s].size() == 0)
        {
          err("Mean window queue empty");
          return;
        }
        Matrix weightedAvg      = m_p_path[s]/m_args.mean_ws;
        Matrix firstWeightedAvg = m_cross_track_window[s].front();
        Matrix newMean = m_p_path_mean[s] + weightedAvg - firstWeightedAvg;

        m_p_path_mean[s] = newMean;
        m_cross_track_window[s].pop();
        m_cross_track_window[s].push(weightedAvg);
        if (m_cross_track_d_window[s].size() == 0)
        {
          debug("Mean window d queue empty");
          return;
        }

        Matrix weightedAvg_d      = m_v_path[s]/m_args.mean_ws;
        Matrix firstWeightedAvg_d = m_cross_track_d_window[s].front();
        Matrix newMean_d = m_v_path_mean[s] + weightedAvg_d - firstWeightedAvg_d;
        m_v_path_mean[s]   = newMean_d;
        
        
        m_cross_track_d_window[s].pop();
        m_cross_track_d_window[s].push(weightedAvg_d);

        delta_p_path_x_mean = delta_p_path_x;
        delta_v_path_x_mean = delta_v_path_x;
      }

      bool
      abort()
      {
        bool aircraftOff = false;
        bool netOff = false;
        if (abs(m_p_path_mean[AIRCRAFT](1))  >= m_args.eps_ct_a(0) ||
            abs(m_p_path_mean[AIRCRAFT](2))  >= m_args.eps_ct_a(1) )
        {
          aircraftOff = true;
        }
        Matrix p_n = getNetPosition(m_p_path);
        if (abs(p_n(1))  >= m_args.eps_ct_n(0) ||
            abs(p_n(2))  >= m_args.eps_ct_n(1) )
        {
          netOff = true;
        }

        return (aircraftOff || netOff);
      }
      
      Vehicle
      getVehicle(std::string src_entity)
      {
    	  if (src_entity == m_vehicles.aircraft)
    	  {
    		  return AIRCRAFT;
    	  }
    	  for(unsigned int i=0; i < m_vehicles.copters.size(); i++)
    	  {
        	  if (src_entity == m_vehicles.copters[i])
        	  {
        		  if (i==0)
        			  return COPTER_LEAD;
        		  else
        			  return COPTER_FOLLOW;
        	  }
    	  }
		  return INVALID;
      }

      bool
      allInitialized()
      {
    	unsigned int vehicles = m_vehicles.no_vehicles;
    	if (!m_args.enable_coord)
    		vehicles = 2;

        for(unsigned int i=0; i < vehicles; i++)
        {
          if (!m_initialized[i])
            return false;
        }        
        return true;
      }

      bool
      aircraftApproaching()
      {
    	  if (m_v_path[AIRCRAFT](0) > 0 && m_p_path[AIRCRAFT](0) < 0)
    		  return true;
    	  return false;
      }

      bool
      startNetRecovery()
      {
        //monitor the path-along distance between the net and the aircraft
          // when at a given boundary, start the net-catch mission
        // this requires that the net are stand-by at the first WP at the runway
    	double delta_p = delta_p_path_x;
    	if (m_args.use_mean_window_aircraft)
    		delta_p = delta_p_path_x_mean;

        if (abs(delta_p) <= m_startCatch_radius)
        {
          //inf("Start net-catch mission: delta_p_path_x_mean=%f", delta_p_path_x_mean);
          return true;
        }
        return false;
      }

      bool
      catched()
      {
    	  //simulation: position check
    	  //real life: weight cell in combination with rotary sensor
    	  double eps = 0.1;
    	  Matrix diff = m_p_path[COPTER_LEAD]-m_p_path[AIRCRAFT];
    	  if ( diff.norm_2() < eps && m_v_path[AIRCRAFT](0) > 0)
    		  return true;
    	  return false;
      }

      bool
      endAtRunway()
      {
    	Matrix p_n = getNetPosition(m_p);
      	Matrix p_to_end = p_n - m_runway.end_NED;
      	double dist_left = p_to_end.norm_2();
      	if (dist_left <= m_args.m_endCatch_radius)
    	  return true;
      	return false;
      }

      double
      getPathVelocity(double v0, double v_ref, double a_n, bool reset_ramp)
      {
    	double deltaT = (v_ref-v0)/a_n;
        static bool rampEnabled = false;
        static double startTime = -1;
        static double deltaV = (v_ref-v0)/deltaT;
        if (reset_ramp)
        {
        	startTime = -1;
        	rampEnabled = false;
        }
        //when starting net-catch operation, this should be a ramp in velocity
        if (!reset_ramp)
            rampEnabled = true;
        if (rampEnabled && startTime == -1)
          startTime=Clock::get();
        double vel = v0;
        double deltaTime = Clock::get() - startTime;
        if (rampEnabled)
        {
          if (deltaTime < deltaT)
            vel = v0 + deltaV*deltaTime;
          else
            vel = v_ref;
        }
        else
        {
            vel = v0;
        }
        static double startPrint = 0;
        if (Clock::get() - startPrint > 1)
        {
        	spew("deltaV: %f\n");
        	spew("getPathVelocity: \n\t u_d=%f\n\t deltaTime=%f\n\t enabled=%d",vel,deltaTime,rampEnabled);
        	startPrint = Clock::get();
        }

        return vel;
      }

      double
      updateStartRadius(double v_a, double v0_n, double v_ref_n, double a_n, double r_impact)
      {
        // calculate the radius which the net-catch maneuver should start, based on the mean velocity of the airplane
        // and the ramp reference velocity and max velocity
    	if (a_n == 0.0)
    		return -1;

        double deltaT_n = (v_ref_n-v0_n)/a_n;
        double r_n_delta_t = (std::pow(v_ref_n, 2),std::pow(v0_n, 2))/(2*a_n);
        double Delta_r_impact = r_impact - r_n_delta_t;
        //inf("Delta_r_impact=%f",Delta_r_impact);
        if (Delta_r_impact < 0)
        {
          err("Desired impact position should be at least %f m from the start of the runway",r_n_delta_t);
          Delta_r_impact = 0;          
        }
        double r_start = abs(r_impact - v_a*(deltaT_n + Delta_r_impact/v_ref_n));

        static double startPrint = 0;
        if (Clock::get() - startPrint > 1)
        {
        	spew("desiredStartRadius: %f", r_start);
        	startPrint = Clock::get();
        }
        return r_start;
      }

      void
      sendCurrentState()
      {
    	  IMC::NetRecoveryState state;
    	  state.flags = m_curr_state;	// Should use the IMC flags for state
    	  Matrix p_n = getNetPosition(m_p_path);
    	  Matrix v_n = getNetVelocity(m_v_path);
    	  state.x_n = p_n(0);
		  state.y_n = p_n(1);
		  state.z_n = p_n(2);

    	  state.vx_n = v_n(0);
		  state.vy_n = v_n(1);
		  state.vz_n = v_n(2);

    	  state.x_a = m_p_path[AIRCRAFT](0);
    	  state.y_a = m_p_path[AIRCRAFT](1);
    	  state.z_a = m_p_path[AIRCRAFT](2);

    	  state.vx_a = m_v_path[AIRCRAFT](0);
    	  state.vy_a = m_v_path[AIRCRAFT](1);
    	  state.vz_a = m_v_path[AIRCRAFT](2);

    	  state.delta_p  = delta_p_path_x;
    	  state.delta_vp = delta_v_path_x;

    	  state.course_error_a = 0;
    	  state.course_error_n = 0;

    	  dispatch(state);
      }

      void
      sendDesiredLocalVelocity(Matrix vel)
      {
  		IMC::DesiredVelocity m_desired_vel;

  		m_desired_vel.u = vel(0);
  		m_desired_vel.v = vel(1);
  		m_desired_vel.w = vel(2);

  		if (m_args.disable_Z)
  		  m_desired_vel.flags = IMC::DesiredVelocity::FL_SURGE| IMC::DesiredVelocity::FL_SWAY;
        else
          m_desired_vel.flags = IMC::DesiredVelocity::FL_SURGE| IMC::DesiredVelocity::FL_SWAY | IMC::DesiredVelocity::FL_HEAVE;

  		dispatch(m_desired_vel);
      }

      Matrix
      getNetPosition(std::vector<Matrix> p)
      {
    	Matrix p_n;
  		if (m_args.enable_coord && m_vehicles.no_vehicles == 3)
  		{
  			p_n = 0.5*(p[COPTER_LEAD]+ p[COPTER_FOLLOW]);
  		}
  		else
  		{
  			p_n = p[COPTER_LEAD];
  		}
  		return p_n;
      }

      Matrix
      getNetVelocity(std::vector<Matrix> v)
      {
    	Matrix v_n;
  		if (m_args.enable_coord && m_vehicles.no_vehicles == 3)
  		{
  			v_n = 0.5*(v[COPTER_LEAD] + v[COPTER_FOLLOW]);
  		}
  		else
  		{
  			v_n = v[COPTER_LEAD];
  		}
  		return v_n;
      }


      //! Control loop in path-frame
      Matrix
      getDesiredPathVelocity(double u_d_along_path, Matrix p_a_path, Matrix v_a_path, Matrix p_n_path, Matrix v_n_path)
      {
    	//_a: aircraft
    	//_n: net (copter)
    	Matrix p_ref_path = Matrix(3,1,0.0);
      	Matrix v_ref_path = Matrix(3,1,0.0);
    	Matrix v_path = Matrix(3,1,0.0);

    	Matrix p_max_path = Matrix(3,1,0.0);
    	p_max_path(0) = m_runway.box_length;
    	p_max_path(1) = m_runway.box_width/2;
    	p_max_path(2) = m_runway.box_height/2;

    	if (u_d_along_path > 0)
    	{
    		p_ref_path(0) = p_max_path(0);
    	}

    	for (int i = 1; i <= 2; i = i+1)
    	{
    		if (abs(p_a_path(i)) < p_max_path(i))
    		{
    			p_ref_path(i) = p_a_path(i);
    			v_ref_path(i) = v_a_path(i);
    		}
    		else
    		{
    			p_ref_path(i) = p_a_path(i)/abs(p_a_path(i)) * p_max_path(i);
    			v_ref_path(i) = 0;
    		}
    	}

    	Matrix e_p_path = p_ref_path-p_n_path;
    	Matrix e_v_path = v_ref_path-v_n_path;
    	m_p_int_value = m_p_int_value + e_p_path*m_time_diff;



    	if (p_n_path(0) < p_max_path(0)-m_args.m_endCatch_radius && u_d_along_path > 0)
		{
    		e_p_path(0) = 0;
    		v_path(0) = u_d_along_path;

        	Matrix v_path_yz = Matrix(2,1,0.0);
    		v_path_yz(0) = m_args.Kp(1)*e_p_path(1) + m_args.Ki(1)*m_p_int_value(1) + m_args.Kd(1)*e_v_path(1);
    		v_path_yz(1) = m_args.Kp(2)*e_p_path(2) + m_args.Ki(2)*m_p_int_value(2) + m_args.Kd(2)*e_v_path(2);

        	//limit velocity
    		if (v_path_yz.norm_2() > sqrt(pow(m_args.max_norm_v,2)-pow(m_u_ref,2)))
    		{
				v_path_yz = sqrt(pow(m_args.max_norm_v,2)-pow(m_u_ref,2)) * v_path_yz/v_path_yz.norm_2();
    		}

			v_path(1) = v_path_yz(0);
			v_path(2) = v_path_yz(1);
		}
    	else
    	{
    		v_path(0) = m_args.Kp(0)*e_p_path(0) + m_args.Ki(0)*m_p_int_value(0) + m_args.Kd(0)*e_v_path(0);
        	v_path(1) = m_args.Kp(1)*e_p_path(1) + m_args.Ki(1)*m_p_int_value(1) + m_args.Kd(1)*e_v_path(1);
        	v_path(2) = m_args.Kp(2)*e_p_path(2) + m_args.Ki(2)*m_p_int_value(2) + m_args.Kd(2)*e_v_path(2);

        	//limit velocity
    		if (v_path.norm_2() > m_args.max_norm_v)
    		{
    			v_path = abs(m_args.max_norm_v) * v_path/v_path.norm_2();
    		}
    	}

    	static double startPrint = 0;
        if (Clock::get() - startPrint > 1)
        {
        	spew("Kp: [%f,%f,%f]", m_args.Kp(0),m_args.Kp(1),m_args.Kp(2));
        	spew("m_time_diff: %d",m_time_diff);
        	spew("m_p_int_value: [%f,%f,%f]", m_p_int_value(0),m_p_int_value(1),m_p_int_value(2));
        	spew("p_ref_path: [%f,%f,%f]"  ,p_ref_path(0),p_ref_path(1),p_ref_path(2));
        	spew("v_ref_path: [%f,%f,%f]"  ,v_ref_path(0),v_ref_path(1),v_ref_path(2));
        	spew("p_a_path: [%f,%f,%f]"  ,p_a_path(0),p_a_path(1),p_a_path(2));
        	spew("p_n_path: [%f,%f,%f]"  ,p_n_path(0),p_n_path(1),p_n_path(2));
        	spew("  v_path: [%f,%f,%f]\n\n",v_path(0),v_path(1),v_path(2));

        	startPrint = Clock::get();
        }

    	return v_path;
      }

      //! Get desired local
      Matrix
      getDesiredLocalVelocity(Matrix v_p, double course, double pitch)
      {
    	return Rzyx(course, pitch, 0)*v_p;
      }

      virtual void
      reset(void)
      {
    	  inf("Reset coordinator");
    	  m_curr_state = IMC::NetRecoveryState::NR_INIT;
          m_initializedCoord = false;
          m_ref_valid = false;
          m_coordinatorEnabled = false;
          m_time_end = Clock::getMsec();
          m_time_diff = 0.0;
          m_p_int_value = Matrix(3,1,0.0);
          //reset ramp
          m_ud = 0;
          getPathVelocity(0, m_u_ref, m_ad, true);
          //initCoordinator();
      }

      virtual void
	  onAutopilotActivation(void)
      {
    	  inf("AutopilotActivating");
          if (!m_args.use_controller)
          {
            debug("Path activated, but not active: Requesting deactivation");
            requestDeactivation();
            return;
          }
      }

      virtual void
	  onAutopilotDeactivation(void)
      {
    	  debug("Deactivation");
          //requestDeactivation();
      }

      //! Main loop.
      void
      task(void)
      {
  	    if(!m_args.use_controller || !isActive() || !m_coordinatorEnabled)
  	      //spew("isActive: %d",isActive());
		  return;

  	    m_time_diff = Clock::getMsec() - m_time_end;
  	    m_time_end = Clock::getMsec();
		//spew("Frequency: %1.1f, %d", 1000.0/m_time_diff,m_coordinatorEnabled);

        //dispatch control if ready

    	if(m_curr_state == IMC::NetRecoveryState::NR_STANDBY  ||
    	   m_curr_state == IMC::NetRecoveryState::NR_APPROACH ||
    	   m_curr_state == IMC::NetRecoveryState::NR_START    ||
    	   m_curr_state == IMC::NetRecoveryState::NR_END	)
    	{
    		Matrix p_a_path;
    		Matrix v_a_path;

    		Matrix p_n_path;
    		Matrix v_n_path;

    		if (m_curr_state == IMC::NetRecoveryState::NR_STANDBY)
    		{
    			p_a_path = Matrix(3,1,0.0);
    			v_a_path = Matrix(3,1,0.0);
    		}
    		else
    		{
				if (m_args.use_mean_window_aircraft)
				{
					p_a_path = m_p_path_mean[AIRCRAFT];
					v_a_path = m_v_path_mean[AIRCRAFT];
				}
				else
				{
					p_a_path = m_p_path[AIRCRAFT];
					v_a_path = m_v_path[AIRCRAFT];
				}
    		}

    		p_n_path = getNetPosition(m_p_path);
    		v_n_path = getNetVelocity(m_v_path);

    		Matrix v_path   = getDesiredPathVelocity(m_ud, p_a_path, v_a_path, p_n_path, v_n_path);

    		Matrix v_d_local = getDesiredLocalVelocity(v_path,m_runway.alpha,m_runway.theta);

    		sendDesiredLocalVelocity(v_d_local);
    	}

      }

      //! @return  Rotation matrix.
      Matrix Rzyx(double phi, double theta, double psi) const
      {
        double R_en_elements[] = {cos(psi)*cos(theta), (-sin(psi)*cos(phi))+(cos(psi)*sin(theta)*sin(phi)), ( sin(psi)*sin(phi))+(cos(psi)*cos(phi)*sin(theta)) ,
        						  sin(psi)*cos(theta), ( cos(psi)*cos(phi))+(sin(phi)*sin(theta)*sin(psi)), (-cos(psi)*sin(phi))+(sin(theta)*sin(psi)*cos(phi)),
								 -sin(theta), 			 cos(theta)*sin(phi), 								  cos(theta)*cos(phi)};
        return Matrix(R_en_elements,3,3);
      }

    };
  }
}

DUNE_TASK
