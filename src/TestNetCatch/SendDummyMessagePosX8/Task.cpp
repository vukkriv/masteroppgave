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
// Author: DuneAuthor                                                       *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace TestNetCatch
{
  namespace SendDummyMessagePosX8
  {
    using DUNE_NAMESPACES;

    struct Task: public DUNE::Tasks::Task
    {
      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx)
      {
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

      //! Main loop.
      void
      onMain(void)
      {
    	  IMC::NavigationData msg;   // use NavigationData message from IMC

    	  //1 = x8, 0 = Copter
    	  double x8orCopter = 1;
    	  msg.cog = x8orCopter;

    	  //position x8 in NED;
    	  msg.custom_x = -10;
    	  msg.custom_y = -10;
    	  msg.custom_z = -20;

    	  //desired speed along path
    	  double speed_des = 2;
    	  msg.cyaw = speed_des;
    	  //velocity x8 in NED:
    	  msg.bias_psi = 0; //xvel
    	  msg.bias_r = 0; //yvel

    	  double err_x = 1;
    	  double err_y = -1;
    	  double err_z = -1;

		  while (!stopping())
		  {
			  dispatch(msg);       // Dispatch the value to the message bus
	          debug("pos est x8: %f; %f; %f\n", msg.custom_x, msg.custom_y, msg.custom_z);

			  Delay::wait(40);    // Wait doing nothing.

			  err_x = err_x*-1;
			  err_y = err_y*-1;
			  err_z = err_z*-1;

	    	  msg.custom_x = msg.custom_x+err_x;
	    	  msg.custom_y = msg.custom_y+err_y;
	    	  msg.custom_z = msg.custom_z+err_z;
		  }
       }

    };
  }
}

DUNE_TASK
