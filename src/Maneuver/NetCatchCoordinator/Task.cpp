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

      //! WP 1 Runway NED
      Matrix WP1;
      //! WP 2 Runway NED
      Matrix WP2;
    };

    //! % Vehicles
    enum Vehicle { A, C1, C2 };
    
    struct Task: public DUNE::Tasks::Task
    {
      //! Task arguments.
      Arguments m_args;
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
      
      //! Aircraft vehicle initialized
      bool initializedA;
      //! Copter vehicles initialized
      bool initializedC;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx)
      {
        param("Coordinated Catch", m_args.enable_coord)
        .defaultValue("false")
        .description("Flag to enable net catch with two multicopters");

        param("Target Producer", m_args.m_trg_prod)
        .defaultValue("test")
        .description("Producer to read from");

        param("WP runway 1 NED", m_args.WP1)
        .defaultValue("0.0, 0.0, 0.0")
        .units(Units::Meter)
        .description("First WP of runway");

        param("WP runway 2 NED", m_args.WP2)
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
        initRunwayPath(m_args.WP1, m_args.WP2);
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
        
        inf("Should use coordinated: %d",m_args.enable_coord);
        inf("Target producer: %s",m_args.m_trg_prod.c_str());
        // Ignored if sent by self
        //if (estate->getSource() == getSystemId())
        //  return;

        debug("Alpha: %f",alpha_runway);
        debug("Theta: %f",theta_runway);
        
        int s = estate->getSourceEntity();
        switch (s)
        {
          case A:
          {
            initializedA = true;
            break;
          }
          case C1:
          {
            initializedC = true;
            break;
          }
        } 
        if (s <= static_cast<int>(m_estate.size()) )
        {
          m_estate[s-1]       = *estate;
          calcPathErrors(m_estate[s-1], s);          
          if( initializedA && initializedC )
          {
            updateMeanValues(s);

            // should implement a state-machine here, run different checks
            checkAbortCondition();    //requires that the net is along the runway
          }
        }
        
      }


      void
      initCoordinator()
      {
        unsigned int no_vehicles = 2;
        if (m_args.enable_coord)
          no_vehicles = 3;
        initializedA = false;
        initializedC = true;

        inf("# Vehicles: %d",no_vehicles); 
        
        m_estate        = std::vector<IMC::EstimatedState>(no_vehicles);
        m_p             = std::vector<Matrix>(no_vehicles);
        m_v             = std::vector<Matrix>(no_vehicles);

        m_cross_track   = std::vector<Matrix>(no_vehicles);
        m_cross_track_d = std::vector<Matrix>(no_vehicles);

        for (unsigned int i=0; i<no_vehicles; i++) {
          m_p[i] = Matrix(3,1,0);
          m_v[i] = Matrix(3,1,0);
          m_cross_track[i]   = Matrix(2,1,0);
          m_cross_track_d[i] = Matrix(2,1,0);
        }

        inf("# Length of vectors: %d",static_cast<int>(m_estate.size()) );
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
          m_p[s-1] = p;

          v(0) = estate.vx;
          v(1) = estate.vy;
          v(2) = estate.vz;          
          m_v[s-1] = v;

          Matrix R       = transpose(Rzyx(0.0, -theta_runway, alpha_runway));
          Matrix eps     = R*(p-m_args.WP1); 
          Matrix eps_dot = R*v;

          m_cross_track[s-1]    = eps.get(1,2,0,0); 
          m_cross_track_d[s-1]  = eps_dot.get(1,2,0,0); 
          inf("Cross-track e_a:   [%f,%f]",m_cross_track[0](0),m_cross_track[0](1));
          inf("Cross-track e_c1:  [%f,%f]",m_cross_track[1](0),m_cross_track[1](1));
          inf("Cross-track_d e_a:   [%f,%f]",m_cross_track_d[0](0),m_cross_track_d[0](1));
          inf("Cross-track_d e_c1:  [%f,%f]",m_cross_track_d[1](0),m_cross_track_d[1](1));

          inf("Position in NED from '%d': [%f,%f,%f]",s,estate.x,estate.y,estate.z);

          Matrix delta_p_path = R*(m_p[1]-m_p[0]);
          Matrix delta_v_path = R*(m_v[1]-m_v[0]);

          delta_p_path_x = delta_p_path(0);
          delta_v_path_x = delta_v_path(0);
          inf("delta_p_path_x = %f",delta_p_path_x);
          inf("delta_v_path_x = %f",delta_v_path_x);
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
        bool aircraftOff = false;
        bool netOff = false;
        if (abs(m_cross_track_mean[A](0))  >= m_args.eps_ct_a(0) ||
            abs(m_cross_track_mean[A](1))  >= m_args.eps_ct_a(1) )
        {
          aircraftOff = true;
        }
        if (abs(m_cross_track_mean[C1](0))  >= m_args.eps_ct_n(0) ||
            abs(m_cross_track_mean[C1](1))  >= m_args.eps_ct_n(1) )
        {
          netOff = true;
        }
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
