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
#include <DUNE/Control/DiscretePID.hpp>
namespace Control
{
  namespace NetCatchControl
  {
    using DUNE_NAMESPACES;

    struct PID_Kp 
    {
      double x;
      double y;
      double z;
    };

    struct PID_Ki 
    {
      double x;
      double y;
      double z;
    };

    struct PID_Kd
    {
      double x;
      double y;
      double z;
    };        

    //! %Task arguments.
    struct Arguments
    {
      bool use_controller;

      PID_Kp Kp;
      PID_Ki Ki;
      PID_Kd Kd;  

      float m_maxVx;
      float m_maxVy;
      float m_maxVz;

      float m_maxIntx;
      float m_maxInty;
      float m_maxIntz;
    };

    //! Controllable loops.
    static const uint32_t c_controllable = IMC::CL_SPEED;
    //! Required loops.
    static const uint32_t c_required = IMC::CL_FORCE;

    struct Task: public DUNE::Control::BasicUAVAutopilot
    {
      //! Task arguments.
      Arguments m_args;

      //! Last desired velocity received
      IMC::DesiredVelocity m_dv;

      //! Last desired control sent
      IMC::DesiredControl m_f;

      //! Discrete velocity PID control x-direction
      DiscretePID m_PID_vx;
      
      //! Discrete velocity PID control x-direction
      DiscretePID m_PID_vy;

      //! Discrete velocity PID control x-direction
      DiscretePID m_PID_vz;

      //! Desired velocity receceived
      bool m_initialized;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Control::BasicUAVAutopilot(name,ctx, c_controllable, c_required)
      {
		param("Velocity Controller", m_args.use_controller)
		.defaultValue("false")
		.description("Enable Velocity Controller");

        param("Kp - X", m_args.Kp.x)
        .defaultValue("1.0")
        .description("PID Kp x");

        param("Kp - Y", m_args.Kp.y)
        .defaultValue("1.0")
        .description("PID Kp y");

        param("Kp - Z", m_args.Kp.z)
        .defaultValue("1.0")
        .description("PID Kp z");

        param("Ki - X", m_args.Ki.x)
        .defaultValue("1.0")
        .description("PID Ki x");

        param("Ki - Y", m_args.Ki.y)
        .defaultValue("1.0")
        .description("PID Ki y");

        param("Ki - Z", m_args.Ki.z)
        .defaultValue("1.0")
        .description("PID Ki z");

        param("Kd - X", m_args.Kd.x)
        .defaultValue("1.0")
        .description("PID Kd x");

        param("Kd - Y", m_args.Kd.y)
        .defaultValue("1.0")
        .description("PID Kd y");

        param("Kd - Z", m_args.Kd.z)
        .defaultValue("1.0")
        .description("PID Kd z");        

        param("Max Vel X", m_args.m_maxVx)
        .defaultValue("0.0")
        .description("Maximum velocity x-direction");     

        param("Max Vel Y", m_args.m_maxVy)
        .defaultValue("0.0")
        .description("Maximum velocity y-direction");     

        param("Max Vel Z", m_args.m_maxVz)
        .defaultValue("0.0")
        .description("Maximum velocity z-direction");   

        param("Max Integral X", m_args.m_maxIntx)
        .defaultValue("0.0")
        .description("Maximum integral x-direction");     

        param("Max Integral Y", m_args.m_maxInty)
        .defaultValue("0.0")
        .description("Maximum integral y-direction");     

        param("Max Integral Z", m_args.m_maxIntz)
        .defaultValue("0.0")
        .description("Maximum integral z-direction");            

        // Bind incoming IMC messages
        bind<IMC::DesiredVelocity>(this);                
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
        m_initialized = false;
        BasicUAVAutopilot::onResourceRelease();        
      }

      // consume desired path from NetCatchPath
      void
      consume(const IMC::DesiredVelocity* dv)
      {
        trace("Got DesiredVelocity \nfrom '%s' at '%s'",
             resolveEntity(dv->getSourceEntity()).c_str(),
             resolveSystemId(dv->getSource()));

        debug("Got desired velocity: [%f,%f,%f]: ",dv->u,dv->v,dv->w);
        m_dv = *dv;

        m_initialized = true;
      }

      void 
      initialize()
      {
        m_PID_vx.setOutputLimits(0, m_args.m_maxVx);
        m_PID_vy.setOutputLimits(0, m_args.m_maxVy);
        m_PID_vz.setOutputLimits(0, m_args.m_maxVz);

        m_PID_vx.setIntegralLimits(m_args.m_maxIntx);
        m_PID_vy.setIntegralLimits(m_args.m_maxInty);
        m_PID_vz.setIntegralLimits(m_args.m_maxIntz);

        m_PID_vx.setProportionalGain(m_args.Kp.x);
        m_PID_vy.setProportionalGain(m_args.Kp.y);
        m_PID_vz.setProportionalGain(m_args.Kp.z);

        m_PID_vx.setIntegralGain(m_args.Ki.x);
        m_PID_vy.setIntegralGain(m_args.Ki.y);
        m_PID_vz.setIntegralGain(m_args.Ki.z);

        m_PID_vx.setDerivativeGain(m_args.Kd.x);
        m_PID_vy.setDerivativeGain(m_args.Kd.y);
        m_PID_vz.setDerivativeGain(m_args.Kd.z);
      }

      void
      onEstimatedState(const double timestep, const IMC::EstimatedState* msg)
      {
          //calculate desired inertial force, input to coordinated control
    	  debug("Timestep: %f",timestep);
          if (m_initialized)
          {
            m_f.x = m_PID_vx.step(timestep, m_dv.u - msg->vx);
            m_f.y = m_PID_vy.step(timestep, m_dv.v - msg->vy);
            m_f.z = m_PID_vz.step(timestep, m_dv.w - msg->vz);
          }
      }
    };
  }
}

DUNE_TASK
