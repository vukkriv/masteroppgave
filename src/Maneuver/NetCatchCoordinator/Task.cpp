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
      //! Target producer
      std::string m_trg_prod;

      //! Enable coordinated catch
      bool enable_coord;
      //! Moving mean window size
      double mean_ws;
      
      //! Maximum cross-track error aircraft
      Matrix eps_ct_a;
      //! Maximum cross-track error net
      Matrix eps_ct_n;

      //! Reference latitude
      fp64_t ref_lat;
      //! Reference longitude
      fp64_t ref_lon;
      //! Reference height (above elipsoid)
      fp32_t ref_hae;
      
      //! Radius to stop at end of runway
      unsigned int m_endCatch_radius;

      //! Desired velocity of net at impact
      double m_ud_impact;
      //! Desired time to accelerate to desired velocity of net at impact
      double m_td_acc;
      //! Desired collision radius
      double m_coll_r;

      //! Vehicles
      std::string m_copter_id;
      std::string m_aircraft_id;
    };

    //! % Coordinator states
    enum CoordState {INIT=0, GOTO_RUNW, STANDBY_RUNW, EN_CATCH, END_RUNW, CATCH_ABORT};
    enum Vehicle {AIRCRAFT=0, COPTER, INVALID=-1};

    struct Task: public DUNE::Tasks::Task
    {
      //! Task arguments.
      Arguments m_args;
      //! Current state
      CoordState m_curr_state;
  	  //! Localization origin (WGS-84)
  	  fp64_t m_ref_lat, m_ref_lon;
  	  fp32_t m_ref_hae;
      bool m_ref_valid;


      bool m_maneuverEnabled;

      //! Vehicles initialized
      std::vector<bool> m_initialized;
      //! Task ready to receive messages
      bool m_initializedCoord;

      //! Last FormPos received
      std::vector<IMC::FormPos> m_estate;
      
      //! Last position in NED
      std::vector<Matrix> m_p;
      //! Last velocity in NED
      std::vector<Matrix> m_v;
      
      //! Cross track errors
      std::vector<Matrix> m_cross_track;
      std::vector<std::queue<Matrix> > m_cross_track_window;
      std::vector<Matrix> m_cross_track_mean;
      //! Cross track errors derivative
      std::vector<Matrix> m_cross_track_d;
      std::vector<std::queue<Matrix> > m_cross_track_d_window;
      std::vector<Matrix> m_cross_track_d_mean;

      //! Position difference along path
      double delta_p_path_x;
      double delta_p_path_x_mean;
      //! Velocity difference along path
      double delta_v_path_x;
      double delta_v_path_x_mean;

      //! Course of runway
      double alpha_runway;
      //! Pitch of runway
      double theta_runway;

      //! Virtual runway start WP NED
      Matrix m_WP_start;
      //! Virtual runway end WP nED
      Matrix m_WP_end;
      //! Radius to change to next WP
      unsigned int m_WP_radius;
      //! Current WP
      unsigned int m_WP;

      //! Radius to start net-catch (calculate based on net-acceleration)
      unsigned int m_startCatch_radius;

      //! Desired along-track velocity
      double m_ud;

      //! Control loops last reference
      uint32_t m_scope_ref;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_WP(0),
        m_ud(0),
        m_initializedCoord(false),
        m_scope_ref(0),
        m_ref_lat(0.0),
		m_ref_lon(0.0),
		m_ref_hae(0.0),
        m_ref_valid(false),
        m_maneuverEnabled(false)
      {
        param("Coordinated Catch", m_args.enable_coord)
        .defaultValue("false")
        .description("Flag to enable net catch with two multicopters");

        param("Latitude", m_args.ref_lat)
        .defaultValue("-999.0")
        .units(Units::Degree)
        .description("Reference Latitude");

        param("Longitude", m_args.ref_lon)
        .defaultValue("0.0")
        .units(Units::Degree)
        .description("Reference Longitude");

        param("Height", m_args.ref_hae)
        .defaultValue("0.0")
        .units(Units::Meter)
        .description("Reference Height (above elipsoid)");
        
        param("Mean Window Size", m_args.mean_ws)
        .defaultValue("1.0")
        .description("Number of samples in moving average window");

        param("Maximum Cross-Track Error Aircraft", m_args.eps_ct_a)
        .units(Units::Meter);

        param("Maximum Cross-Track Error Net", m_args.eps_ct_n)
        .units(Units::Meter);

        param("Copter", m_args.m_copter_id);
        param("Aircraft", m_args.m_aircraft_id);

        param("Radius end-of runway", m_args.m_endCatch_radius)
        .defaultValue("10.0")
        .units(Units::Meter);

        param("Desired net-vel catch", m_args.m_ud_impact)
        .defaultValue("10.0");

        param("Desired acceleration time", m_args.m_td_acc)
        .defaultValue("10.0")
        .units(Units::Second);

        param("Desired collision radius", m_args.m_coll_r)
        .defaultValue("100.0")
        .units(Units::Meter);



        // Bind incoming IMC messages
        //bind<IMC::EstimatedState>(this);
        bind<IMC::FormPos>(this);
        bind<IMC::NetRecovery>(this);
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
      consume(const IMC::NetRecovery* msg)
      {
  	    inf("Got NetRecovery \nfrom '%s' at '%s'",
		     resolveEntity(msg->getSourceEntity()).c_str(),
		     resolveSystemId(msg->getSource()));
  	    inf("Intialize net recovery maneuver");

  	    if (!m_initializedCoord)
        {
  	      inf("Intialize coordinator");
   		  m_ref_lat = m_args.ref_lat;
		  m_ref_lon = m_args.ref_lon;
		  m_ref_hae = m_args.ref_hae;
          m_ref_valid = true;
          initCoordinator();
        }


        WGS84::displacement(m_ref_lat, m_ref_lon, 0,
        					msg->start_lat, msg->start_lon, 0,
        					&m_WP_start(0), &m_WP_start(1));
        m_WP_start(2) = msg->z + msg->z_off;

        WGS84::displacement(m_ref_lat, m_ref_lon, 0,
        					msg->end_lat, msg->end_lon, 0,
                            &m_WP_end(0), &m_WP_end(1));
        m_WP_end(2) = msg->z + msg->z_off;

        initCoordinator();
        initRunwayPath(m_WP_start, m_WP_end);

  	    m_maneuverEnabled = true;
  	    inf("Maneuver enabled");
  	    enablePathController();
      }

      void
      consume(const IMC::FormPos* estate)
      {
        if (!m_initializedCoord)
        {
   		  m_ref_lat = m_args.ref_lat;
		  m_ref_lon = m_args.ref_lon;
		  m_ref_hae = m_args.ref_hae;
          m_ref_valid = true;
          initCoordinator();
          return;
        }

        spew("Got FormPos \nfrom '%s' at '%s'",
             resolveEntity(estate->getSourceEntity()).c_str(),
             resolveSystemId(estate->getSource()));

        std::string vh_id  = resolveSystemId(estate->getSource());
        int s = getVehicle(vh_id);
        
        spew("Position NED: [%f,%f,%f]",estate->x,estate->y,estate->z);

        trace("s: %d",s);
        
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

        spew("Position aircraft NED: [%f,%f,%f]",m_p[AIRCRAFT](0),m_p[AIRCRAFT](1),m_p[AIRCRAFT](2));
        spew("Position copter NED: [%f,%f,%f]",m_p[COPTER](0),m_p[COPTER](1),m_p[COPTER](2));

        if (!m_maneuverEnabled)
        	return;

        m_estate[s]       = *estate;
        calcPathErrors(p, v, s);


        updateMeanValues(s);
        //trace("Curr state: %d",static_cast<int>(m_curr_state));

/*
        if (changeWP(m_p[s]))
          inf("WP update");
          m_WP += m_WP;
*/
        // should be called only when waypoint/velocity update
        //sendDesiredPath(m_args.WP1,m_args.WP2, m_ud);

        CoordState last_state = m_curr_state;
        switch(m_curr_state)
        {
          case INIT:
          {
            if (allInitialized())
            {
              m_curr_state = GOTO_RUNW; 
            }
            break;
          }
          case GOTO_RUNW:
          {
            m_curr_state = STANDBY_RUNW;
            break;
          }
          case STANDBY_RUNW:
          {
            updateMeanValues(s);

            double v_a = 20;
            m_startCatch_radius = updateStartRadius(v_a,0, m_args.m_ud_impact, m_args.m_td_acc, m_args.m_coll_r);
            spew("StartCatch radius: %d",m_startCatch_radius);
            spew("Delta_p_x: %f",delta_p_path_x);
            checkPositionsAtRunway(); //requires that the net is standby at the start of the runway

            //test
            m_curr_state = EN_CATCH;
            break;
          }
          case EN_CATCH:
          {
            updateMeanValues(s);
            m_ud = getPathVelocity(0, m_args.m_ud_impact, m_args.m_td_acc, true);
            //checkAbortCondition();
            //test
            //m_curr_state = STANDBY_RUNW;
            break;
          }                            
          case END_RUNW:
          {
            break;
          }
          case CATCH_ABORT:
          {
            break;
          }                            
        }        

        if (last_state != m_curr_state)
        	inf("Curr state: %d",static_cast<int>(m_curr_state));

      }

      void
      initCoordinator()
      {
        if (m_initializedCoord)
          return;

        unsigned int no_vehicles = 2;
        
        m_estate        = std::vector<IMC::FormPos>(no_vehicles);
        m_p             = std::vector<Matrix>(no_vehicles);
        m_v             = std::vector<Matrix>(no_vehicles);

        m_cross_track          = std::vector<Matrix>(no_vehicles);
        m_cross_track_d        = std::vector<Matrix>(no_vehicles);
        m_cross_track_mean   = std::vector<Matrix>(no_vehicles);
        m_cross_track_d_mean = std::vector<Matrix>(no_vehicles);
        m_cross_track_window   = std::vector<std::queue<Matrix> >(no_vehicles);
        m_cross_track_d_window = std::vector<std::queue<Matrix> >(no_vehicles);
        m_initialized = std::vector<bool>(no_vehicles);

        for (unsigned int i=0; i<no_vehicles; i++) {
          m_p[i] = Matrix(3,1,0);
          m_v[i] = Matrix(3,1,0);
          m_cross_track[i]   = Matrix(2,1,0);
          m_cross_track_d[i] = Matrix(2,1,0);
          m_cross_track_mean[i]   = Matrix(2,1,0);
          m_cross_track_d_mean[i] = Matrix(2,1,0);
          debug("m_cross_track[%d]: Rows: %d, Cols: %d",i,m_cross_track[i].rows(),m_cross_track[i].columns());
          debug("m_cross_track_d[%d]: Rows: %d, Cols: %d",i,m_cross_track_d[i].rows(),m_cross_track_d[i].columns());
          debug("m_cross_track_mean[%d]: Rows: %d, Cols: %d",i,m_cross_track_mean[i].rows(),m_cross_track_mean[i].columns());
          debug("m_cross_track_d_mean[%d]: Rows: %d, Cols: %d",i,m_cross_track_d_mean[i].rows(),m_cross_track_d_mean[i].columns());

          m_cross_track_window[i]   = std::queue<Matrix>();
          m_cross_track_d_window[i] = std::queue<Matrix>();
          for (unsigned int j=0; j < m_args.mean_ws; j++)
          {
            m_cross_track_window[i].push(Matrix(2,1,0));
            m_cross_track_d_window[i].push(Matrix(2,1,0));
          }
          m_initialized[i] = false;
        }

        m_WP_start = Matrix(3,1,0);
        m_WP_end = Matrix(3,1,0);

        // TEMP: set current end WP as end of runway
        m_WP_radius = m_args.m_endCatch_radius;
        // For now: set the state directly to standby at runway
        m_curr_state = INIT;
        m_initializedCoord = true;
        inf("Coordinator initialized");
      }

      void
      initRunwayPath(Matrix WP_start, Matrix WP_end)
      {
          Matrix deltaWP = WP_end - WP_start;
          debug("Rows: %d, Cols: %d",deltaWP.rows(),deltaWP.columns());
          double deltaWP_NE = deltaWP.get(0,1,0,0).norm_2(); 
          debug("deltaWP_NE: %f",deltaWP_NE);
          alpha_runway =  atan2(deltaWP(1),deltaWP(0));
          theta_runway = -atan2(deltaWP_NE,deltaWP(2)) + Angles::radians(90);
      }

      void
      calcPathErrors(Matrix p, Matrix v, int s)
      {
          //Calculate position based on the current global position and the reference point

          //WGS84::displacement(Angles::radians(m_args.NED_origin(0)), Angles::radians(m_args.NED_origin(1)), 0,
          //		  	  	  	  estate.lat, estate.lon, 0,
          //		  	  	  	  &p(0), &p(1));
          // p(2) = m_args.WP1(2);


          Matrix R       = transpose(Rzyx(0.0, -theta_runway, alpha_runway));
          Matrix eps     = R*(p-m_WP_start);
          Matrix eps_dot = R*v;

          m_cross_track[s]    = eps.get(1,2,0,0); 
          m_cross_track_d[s]  = eps_dot.get(1,2,0,0); 

          debug("Cross-track e_a:   [%f,%f]",m_cross_track[AIRCRAFT](0),m_cross_track[AIRCRAFT](1));
          debug("Cross-track e_c1:  [%f,%f]",m_cross_track[COPTER](0),m_cross_track[COPTER](1));
          debug("Cross-track_d e_a:   [%f,%f]",m_cross_track_d[AIRCRAFT](0),m_cross_track_d[AIRCRAFT](1));
          debug("Cross-track_d e_c1:  [%f,%f]",m_cross_track_d[COPTER](0),m_cross_track_d[COPTER](1));

          //debug("Position in NED from '%d': [%f,%f,%f]",s,p(0),p(1),p(2));

          Matrix delta_p_path = R*(m_p[COPTER]-m_p[AIRCRAFT]);
          Matrix delta_v_path = R*(m_v[COPTER]-m_v[AIRCRAFT]);

          delta_p_path_x = delta_p_path(0);
          delta_v_path_x = delta_v_path(0);

          debug("delta_p_path_x = %f",delta_p_path_x);
          debug("delta_v_path_x = %f",delta_v_path_x);

      }

      void
      updateMeanValues(int s)
      {
        debug("Update mean values");
        if (m_cross_track_window[s].size() == 0)
        {
          debug("Mean window queue empty");
          return;
        }
        Matrix weightedAvg      = m_cross_track[s]/=m_args.mean_ws;
        Matrix firstWeightedAvg = m_cross_track_window[s].front();

        /*
        debug("weightedAvg: Rows: %d, Cols: %d",weightedAvg.rows(),weightedAvg.columns());
        debug("firstWeightedAvg: Rows: %d, Cols: %d",firstWeightedAvg.rows(),firstWeightedAvg.columns());
        debug("m_cross_track_mean[%d]: Rows: %d, Cols: %d",s,m_cross_track_mean[s].rows(),m_cross_track_mean[s].columns());
        */
        
        //debug("oldMean: [%f,%f]",m_cross_track_mean[s](0),m_cross_track_mean[s](1));
        Matrix newMean = m_cross_track_mean[s] + weightedAvg - firstWeightedAvg;
        //debug("newMean: [%f,%f]",newMean(0),newMean(1));
        m_cross_track_mean[s] = newMean;
        m_cross_track_window[s].pop();
        m_cross_track_window[s].push(weightedAvg);
        if (m_cross_track_d_window[s].size() == 0)
        {
          debug("Mean window d queue empty");
          return;
        }

        Matrix weightedAvg_d      = m_cross_track_d[s]/=m_args.mean_ws;
        Matrix firstWeightedAvg_d = m_cross_track_d_window[s].front();
        Matrix newMean_d = m_cross_track_d_mean[s] + weightedAvg_d - firstWeightedAvg_d;
        m_cross_track_d_mean[s]   = newMean_d;
        
        
        m_cross_track_d_window[s].pop();
        m_cross_track_d_window[s].push(weightedAvg_d);

        //m_cross_track_mean[s]   = m_cross_track[s];
        //m_cross_track_d_mean[s] = m_cross_track_d[s];
        delta_p_path_x_mean = delta_p_path_x;
        delta_v_path_x_mean = delta_v_path_x;
      }

      void
      checkAbortCondition()
      {
        //cḧeck the mean values of the cross-track errors, if too high, possible abort
        // if the erros are too high and increasing, at a given radius between the vehicles, send abort catch
        // requires that the net-catch mission has started (net at runway)

        //TODO: use a string enumeration to get position of vehicle in the vector lists  
        bool aircraftOff = false;
        bool netOff = false;
        if (abs(m_cross_track_mean[AIRCRAFT](0))  >= m_args.eps_ct_a(0) ||
            abs(m_cross_track_mean[AIRCRAFT](1))  >= m_args.eps_ct_a(1) )
        {
          aircraftOff = true;
        }
        if (abs(m_cross_track_mean[COPTER](0))  >= m_args.eps_ct_n(0) ||
            abs(m_cross_track_mean[COPTER](1))  >= m_args.eps_ct_n(1) )
        {
          netOff = true;
        }

        if (aircraftOff || netOff) 
        {
          m_curr_state = CATCH_ABORT;
        }
      }
      
      bool 
      changeWP(Matrix currPos)
      {
        if ( (m_WP_end-currPos).norm_2() <= m_WP_radius*m_WP_radius )
          return true;
        return false;
      }

      Vehicle
      getVehicle(std::string src_entity)
      {
    	  if (src_entity == m_args.m_copter_id)
    	  {
    		  return COPTER;
    	  }
    	  else if (src_entity == m_args.m_aircraft_id)
    	  {
    		  return AIRCRAFT;
    	  }
    	  else
    	  {
    		  return INVALID;
    	  }
      }

      bool
      allInitialized()
      {
        for(unsigned int i=0; i < m_initialized.size(); i++) 
        {
          if (!m_initialized[i])
            return false;
        }        
        return true;
      }

      void
      checkPositionsAtRunway()
      {
        //monitor the path-along distance between the net and the aircraft
          // when at a given boundary, start the net-catch mission
        // this requires that the net are stand-by at the first WP at the runway
        if (abs(delta_p_path_x_mean) <= m_startCatch_radius)
        {
          inf("Start net-catch mission: delta_p_path_x_mean=%f", delta_p_path_x_mean);
          m_curr_state = EN_CATCH;
        }
      }

      void
      sendDesiredPath(Matrix WP_start, Matrix WP_end, double u_d)
      {
        IMC::DesiredPath dp;
        dp.start_lat = WP_start(0);
        dp.start_lon = WP_start(1);
        dp.start_z   = WP_start(2);
        
        dp.end_lat = WP_end(0);
        dp.end_lon = WP_end(1);
        dp.end_z   = WP_end(2);

        dp.speed = u_d;

        dispatch(dp);
      }

      double 
      getPathVelocity(double v0, double v_ref, double deltaT, bool active_ramp)
      {
        static bool rampEnabled = false;
        static double startTime = -1;
        static double deltaV = (v_ref-v0)/deltaT;
        //when starting net-catch operation, this should be a ramp in velocity
        if (active_ramp)
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
        spew("getPathVelocity: u_d=%f,deltaTime=%f,startTime=%f",vel,deltaTime,startTime);
        return vel;
      }

      double
      updateStartRadius(double v_a, double v0_n, double v_ref_n, double deltaT_n, double r_impact)
      {
        // calculate the radius which the net-catch maneuver should start, based on the mean velocity of the airplane
        // and the ramp reference velocity and max velocity
        double a_n = (v_ref_n-v0_n)/deltaT_n;
        double r_n_delta_t = (std::pow(v_ref_n, 2),std::pow(v0_n, 2))/(2*a_n);
        double Delta_r_impact = r_impact - r_n_delta_t;
        inf("Delta_r_impact=%f",Delta_r_impact);
        if (Delta_r_impact < 0)
        {
          err("Desired impact position should be at least %f m from the start of the runway",r_n_delta_t);
          Delta_r_impact = 0;          
        }
        double r_start = abs(r_impact - v_a*(deltaT_n + Delta_r_impact/v_ref_n));
        return r_start;
      }

      void
      enablePathController()
      {
    	  //activate path controller
    	  IMC::ControlLoops m_cloops;
          m_cloops.enable = IMC::ControlLoops::CL_ENABLE;
          m_cloops.mask = IMC::CL_PATH;
          m_cloops.scope_ref = m_scope_ref;
          dispatch(m_cloops);
          trace("Path controller activated from coordinator");
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
      Matrix Rzyx(double phi, double theta, double psi) const
      {
        double R_en_elements[] = {cos(psi)*cos(theta), (-sin(psi)*cos(phi))+(cos(psi)*sin(theta)*sin(phi)), (sin(psi)*sin(phi))+(cos(psi)*cos(phi)*sin(theta)) ,
            sin(psi)*cos(theta), (cos(psi)*cos(phi))+(sin(phi)*sin(theta)*sin(psi)), (-cos(psi)*sin(phi))+(sin(theta)*sin(psi)*cos(phi)),
            -sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi)};
        return Matrix(R_en_elements,3,3);
      }

    };
  }
}

DUNE_TASK
