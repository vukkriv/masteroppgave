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

namespace Control
{
  namespace NetCatchPath
  {
    using DUNE_NAMESPACES;

    //! %Task arguments.
    struct Arguments
    {
      double m_Delta_x;
      double m_Delta_z;
    };

    //! Controllable loops.
    static const uint32_t c_controllable = 1;
    //! Required loops.
    static const uint32_t c_required = 1;

    struct Task: public DUNE::Control::BasicUAVAutopilot
    {
      //! Task arguments.
      Arguments m_args;

      //! Last desired path received
      IMC::DesiredPath m_dp;

      //! Last received start WP
      Matrix WP_start;
      //! Last received end WP
      Matrix WP_end;
      //! Course of current path
      double m_alpha_k;
      //! Pitch of current path
      double m_theta_k;
      //! Desired velocity along the path
      double m_ud;
      //! Desired path receceived
      bool m_initialized;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Control::BasicUAVAutopilot(name,ctx, c_controllable, c_required)
      {
        param("Delta X", m_args.m_Delta_x)
        .defaultValue("1.0")
        .units(Units::Meter)
        .description("Look-a-head distance LOS x-direction");

        param("Delta Z", m_args.m_Delta_z)
        .defaultValue("1.0")
        .units(Units::Meter)
        .description("Look-a-head distance LOS z-direction");

        // Bind incoming IMC messages
        bind<IMC::DesiredPath>(this);        
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
        m_initialized = false;
        BasicUAVAutopilot::onResourceAcquisition();
      }

      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
        BasicUAVAutopilot::onResourceInitialization();
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
        BasicUAVAutopilot::onResourceRelease();
      }      

      // consume desired path from NetCatchCoordinator
      void
      consume(const IMC::DesiredPath* dp)
      {
        trace("Got DesiredPath \nfrom '%s' at '%s'",
             resolveEntity(dp->getSourceEntity()).c_str(),
             resolveSystemId(dp->getSource()));

        //TODO: check somehow if this is sent from the NetCatchCoordinator 
        m_dp = *dp;
        initDesiredPath(dp);
        m_initialized = true;

        //Testing
        /*
        if (m_initialized)
        {
          Matrix position = Matrix(3,1,0);
          Matrix u = getDesiredVelocity(position);
          sendDesiredVelocity(u);
        } 
        */       
      }

      void 
      initDesiredPath(const IMC::DesiredPath* dp)
      {
        WP_start = Matrix(3,1,0);
        WP_end   = Matrix(3,1,0);

        Matrix deltaWP = WP_end - WP_start;
        double deltaWP_NE = deltaWP.get(0,1,0,0).norm_2(); 
        
        m_alpha_k =  atan2(deltaWP(1),deltaWP(0));
        m_theta_k = -atan2(deltaWP_NE,deltaWP(2)) + Angles::radians(90);
        m_ud = dp->speed;
        debug("m_alpha_k: %f",m_alpha_k);
        debug("m_theta_k: %f",m_theta_k);        
        debug("m_ud: %f",m_ud);        
      }

      Matrix
      getPositionOfNet(const IMC::EstimatedState* msg)
      {
        Matrix pos(3,1,0);
        pos(0)= msg->x;
        pos(1)= msg->y;
        pos(2)= msg->z;
        return pos;
      }

      Matrix
      getDesiredVelocity(Matrix position)
      {
        Matrix u_d_p(3,1,0);
        u_d_p(0) = m_ud;

        Matrix eps = transpose(Rzyx(0,-m_theta_k,m_alpha_k))*(position-WP_start);

        double theta_d = 0;
        double psi_d = 0;

        Matrix u_d_n = Rzyx(0,-theta_d,psi_d)*u_d_p;

        return u_d_n;
      }

      void 
      sendDesiredVelocity(Matrix velocity)
      {
            IMC::DesiredVelocity d_vel;
            d_vel.u = velocity(0);
            d_vel.v = velocity(1);
            d_vel.w = velocity(2);
            debug("Dispatch desired velocity: [%f,%f,%f]: ",d_vel.u,d_vel.v,d_vel.w);
            dispatch(d_vel);
      }

      void
      onEstimatedState(const double timestep, const IMC::EstimatedState* msg)
      {
          //calculate desired velocity to be dispatched to inner control loop
          //assume IMC::EstimatedState* msg gives the state of the net?
          // or should we calculate the net position here?

          trace("Timestep: %f",timestep);
          if (m_initialized)
          {
            Matrix position = getPositionOfNet(msg);
            Matrix u = getDesiredVelocity(position);
            sendDesiredVelocity(u);
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
