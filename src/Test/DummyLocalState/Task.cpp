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

namespace Test
{
  namespace DummyLocalState
  {
    using DUNE_NAMESPACES;

    //! %Task arguments.
    struct Arguments
    {
      Matrix pos_ned;
      Matrix vel_ned;
      Matrix vel_body;
      Matrix euler_orient;
      Matrix omega;
      Matrix acc_body;
      Matrix LLH;
    };

    struct Task: public Tasks::Periodic
    {
      Arguments m_args;
      IMC::EstimatedLocalState m_state;
      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        Tasks::Periodic(name, ctx)
      {
          param("Position", m_args.pos_ned)
          .defaultValue("0.0, 0.0, 0.0");
          
          param("Velocity", m_args.vel_ned)
          .defaultValue("0.0, 0.0, 0.0");

          param("Velocity body", m_args.vel_body)
          .defaultValue("0.0, 0.0, 0.0");

          param("Acceleration body", m_args.acc_body)
          .defaultValue("0.0, 0.0, 0.0");          

          param("Euler orientation", m_args.euler_orient)
          .defaultValue("0.0, 0.0, 0.0");                    

          param("Omega", m_args.omega)
          .defaultValue("0.0, 0.0, 0.0");  

          param("LLH", m_args.LLH)
          .defaultValue("0.0, 0.0, 0.0");  
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      { 
        m_state.x      = m_args.pos_ned(0);
        m_state.y      = m_args.pos_ned(1);
        m_state.z      = m_args.pos_ned(2);
        m_state.vx     = m_args.vel_ned(0);
        m_state.vy     = m_args.vel_ned(1);
        m_state.vz     = m_args.vel_ned(2);
        m_state.u      = m_args.vel_body(0);
        m_state.v      = m_args.vel_body(1);
        m_state.w      = m_args.vel_body(2);
        m_state.ax     = m_args.acc_body(0);
        m_state.ay     = m_args.acc_body(1);
        m_state.az     = m_args.acc_body(2);
        m_state.phi    = m_args.euler_orient(0);
        m_state.theta  = m_args.euler_orient(1);
        m_state.psi    = m_args.euler_orient(2);    
        m_state.p      = m_args.omega(0);
        m_state.q      = m_args.omega(1);
        m_state.r      = m_args.omega(2);                
        m_state.lat    = m_args.LLH(0);
        m_state.lon    = m_args.LLH(1);
        m_state.height = m_args.LLH(2);          
        inf("Frequency: %f",this->getFrequency());              
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
      task(void)
      {
        //if (!isActive())
        //  return;
        spew("Dummy EstimatedLocalState sent");
        dispatch(m_state);                
      }
    };
  }
}

DUNE_TASK
