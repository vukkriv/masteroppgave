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

      //! NED origin
      Matrix NED_origin;
      //! WP 1 Runway llh
      Matrix WP1;
      //! WP 2 Runway llh
      Matrix WP2;
      
      //! Radius to stop at end of runway
      unsigned int m_endCatch_radius;

      //! Vehicle list
      std::vector<std::string> m_vehicles;
    };
    
    //! % Coordinator states
    enum CoordState {INIT=0, GOTO_RUNW, STANDBY_RUNW, EN_CATCH, END_RUNW, CATCH_ABORT};

    struct Task: public DUNE::Tasks::Task
    {
      //! Task arguments.
      Arguments m_args;
      
      //! Current state
      CoordState m_curr_state;

      //! Vehicles initialized
      std::vector<bool> m_initialized;

      //! Last Estimated State received
      std::vector<IMC::EstimatedState> m_estate;
      
      //! Last position in NED
      std::vector<Matrix> m_p;
      //! Last velocity in NED
      std::vector<Matrix> m_v;
      
      //! Cross track errors
      std::vector<Matrix> m_cross_track;
      std::vector<Matrix> m_cross_track_mean;
      //! Cross track errors derivative
      std::vector<Matrix> m_cross_track_d;
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

      //! Current WP NED
      Matrix m_WP_curr;
      //! Next WP NED
      Matrix m_WP_next;
      //! Radius to change to next WP
      unsigned int m_WP_radius;
      //! Current WP
      unsigned int m_WP;

      //! Radius to start net-catch (calculate based on net-acceleration)
      unsigned int m_startCatch_radius;


      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_WP(0)
      {
        param("Coordinated Catch", m_args.enable_coord)
        .defaultValue("false")
        .description("Flag to enable net catch with two multicopters");

        param("Target Producer", m_args.m_trg_prod)
        .defaultValue("test")
        .description("Producer to read from");

        param("NED origin", m_args.NED_origin)
        .defaultValue("0.0, 0.0, 0.0")
        .units(Units::Meter)
        .description("Origin of NED in llh");

        param("WP runway 1 llh", m_args.WP1)
        .defaultValue("0.0, 0.0, 0.0")
        .units(Units::Meter)
        .description("First WP of runway");

        param("WP runway 2 llh", m_args.WP2)
        .defaultValue("0.0, 0.0, 0.0")
        .units(Units::Meter)
        .description("Second WP of runway");
        
        param("Mean Window Size", m_args.mean_ws)
        .defaultValue("1.0")
        .description("Number of samples in moving average window");

        param("Maximum Cross-Track Error Aircraft", m_args.eps_ct_a)
        .units(Units::Meter);

        param("Maximum Cross-Track Error Net", m_args.eps_ct_n)
        .units(Units::Meter);

        param("Vehicles", m_args.m_vehicles);

        param("Radius end-of runway", m_args.m_endCatch_radius)
        .defaultValue("10.0")
        .units(Units::Meter);


        // Bind incoming IMC messages
        bind<IMC::EstimatedState>(this);
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
        initCoordinator();
        initRunwayPath(m_WP_curr, m_WP_next);
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
      consume(const IMC::EstimatedState* estate)
      {
        trace("Got EstimatedState \nfrom '%s' at '%s'",
             resolveEntity(estate->getSourceEntity()).c_str(),
             resolveSystemId(estate->getSource()));
        
        debug("Target producer: %s",m_args.m_trg_prod.c_str());
        // Ignored if sent by self
        //if (estate->getSource() == getSystemId())
        //  return;

        debug("Alpha: %f",alpha_runway);
        debug("Theta: %f",theta_runway);

        std::string trg_prod  = resolveEntity(estate->getSourceEntity());
        int s = getVehicle(trg_prod);
        
        debug("s: %d",s);
        
        if (s == -1)  //invalid vehicle
          return;

        m_initialized[s] = true;
        m_estate[s]       = *estate;
        calcPathErrors(m_estate[s], s);          
        updateMeanValues(s);
        trace("Curr state: %d",static_cast<int>(m_curr_state));


        if (changeWP(m_p[s]))
          m_WP += m_WP;

        // should be called only when waypoint/velocity update
        sendDesiredPath(m_args.WP1,m_args.WP2, 1.1);

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
            checkPositionsAtRunway(); //requires that the net is standby at the start of the runway
            break;
          }
          case EN_CATCH:
          {
            checkAbortCondition();    //requires that the net is along the runway
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
      }

      void
      initCoordinator()
      {
        unsigned int no_vehicles = m_args.m_vehicles.size();

        debug("# Vehicles: %d",no_vehicles); 
        
        m_estate        = std::vector<IMC::EstimatedState>(no_vehicles);
        m_p             = std::vector<Matrix>(no_vehicles);
        m_v             = std::vector<Matrix>(no_vehicles);

        m_cross_track   = std::vector<Matrix>(no_vehicles);
        m_cross_track_d = std::vector<Matrix>(no_vehicles);
        m_cross_track_mean   = std::vector<Matrix>(no_vehicles);
        m_cross_track_d_mean = std::vector<Matrix>(no_vehicles);
        m_initialized = std::vector<bool>(no_vehicles);

        for (unsigned int i=0; i<no_vehicles; i++) {
          m_p[i] = Matrix(3,1,0);
          m_v[i] = Matrix(3,1,0);
          m_cross_track[i]   = Matrix(2,1,0);
          m_cross_track_d[i] = Matrix(2,1,0);
          m_cross_track_mean[i]   = Matrix(2,1,0);
          m_cross_track_d_mean[i] = Matrix(2,1,0);
          m_initialized[i] = false;
        }

        m_WP_curr = Matrix(3,1,0);
        m_WP_next = Matrix(3,1,0);

        WGS84::displacement(m_args.NED_origin(0), m_args.NED_origin(1), 0,
                    m_args.WP1(0), m_args.WP1(1), 0,
                    &m_WP_curr(0), &m_WP_curr(1));
        m_WP_curr(2) = m_args.WP1(2);

        WGS84::displacement(m_args.NED_origin(0), m_args.NED_origin(1), 0,
                            m_args.WP2(0), m_args.WP2(1), 0,
                            &m_WP_next(0), &m_WP_next(1));
        m_WP_next(2) = m_args.WP2(2);


        // TODO: calculate the desired net-catch radius
        m_startCatch_radius = 100;
        // TEMP: set current end WP as end of runway
        m_WP_radius = m_args.m_endCatch_radius;
        // For now: set the state directly to standby at runway
        m_curr_state = INIT;
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
      calcPathErrors(const IMC::EstimatedState estate, int s)
      {
          Matrix p = Matrix(3,1,0); //position in NED
          Matrix v = Matrix(3,1,0); //velocity in NED

          p(0) = estate.x;
          p(1) = estate.y;
          p(2) = estate.z;          
          m_p[s] = p;

          v(0) = estate.vx;
          v(1) = estate.vy;
          v(2) = estate.vz;          
          m_v[s] = v;

          Matrix R       = transpose(Rzyx(0.0, -theta_runway, alpha_runway));
          Matrix eps     = R*(p-m_WP_curr); 
          Matrix eps_dot = R*v;

          m_cross_track[s]    = eps.get(1,2,0,0); 
          m_cross_track_d[s]  = eps_dot.get(1,2,0,0); 
          debug("Cross-track e_a:   [%f,%f]",m_cross_track[0](0),m_cross_track[0](1));
          debug("Cross-track e_c1:  [%f,%f]",m_cross_track[1](0),m_cross_track[1](1));
          debug("Cross-track_d e_a:   [%f,%f]",m_cross_track_d[0](0),m_cross_track_d[0](1));
          debug("Cross-track_d e_c1:  [%f,%f]",m_cross_track_d[1](0),m_cross_track_d[1](1));

          debug("Position in NED from '%d': [%f,%f,%f]",s,estate.x,estate.y,estate.z);

          Matrix delta_p_path = R*(m_p[1]-m_p[0]);
          Matrix delta_v_path = R*(m_v[1]-m_v[0]);

          delta_p_path_x = delta_p_path(0);
          delta_v_path_x = delta_v_path(0);
          debug("delta_p_path_x = %f",delta_p_path_x);
          debug("delta_v_path_x = %f",delta_v_path_x);
      }

      void
      updateMeanValues(int s)
      {
        // should use a ring-buffer to calculate the mean value based on the last value in the buffer
        m_cross_track_mean[s]   = m_cross_track[s];
        m_cross_track_d_mean[s] = m_cross_track_d[s];
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
        if (abs(m_cross_track_mean[0](0))  >= m_args.eps_ct_a(0) ||
            abs(m_cross_track_mean[0](1))  >= m_args.eps_ct_a(1) )
        {
          aircraftOff = true;
        }
        if (abs(m_cross_track_mean[1](0))  >= m_args.eps_ct_n(0) ||
            abs(m_cross_track_mean[1](1))  >= m_args.eps_ct_n(1) )
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
        if ( (m_WP_next-currPos).norm_2() <= m_WP_radius*m_WP_radius )
          return true;
        return false;
      }

      int
      getVehicle(std::string src_entity)
      {
        for(unsigned int i=0; i < m_args.m_vehicles.size(); i++) 
        {
          //compare 
          if ( m_args.m_vehicles[i] == src_entity )
          {
            return i;
          }
        }
        return -1;
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
        double R_en_elements[] = {cos(psi)*cos(theta), (-sin(psi)*cos(phi))+(cos(psi)*sin(theta)*sin(psi)), (sin(psi)*sin(phi))+(cos(psi)*cos(phi)*sin(theta)) ,
            sin(psi)*cos(theta), (cos(psi)*cos(phi))+(sin(phi)*sin(theta)*sin(psi)), (-cos(psi)*sin(phi))+(sin(theta)*sin(psi)*cos(phi)),
            -sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi)};
        return Matrix(R_en_elements,3,3);
      }

    };
  }
}

DUNE_TASK
