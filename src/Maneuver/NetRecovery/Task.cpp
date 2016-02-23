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
// Author: Kristian Klausen                                                 *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Maneuver
{
  namespace NetRecovery
  {
    using DUNE_NAMESPACES;

    struct Task: public DUNE::Maneuvers::Maneuver
    {
      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Maneuvers::Maneuver(name, ctx)
      {
        bindToManeuver<Task, IMC::NetRecovery>();
        bind<IMC::NetRecoveryState>(this);
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
      }

      void
      consume(const IMC::NetRecovery* maneuver)
      {
      	inf("NetRecovery maneuver received");

        // check the source with vehicle
        std::string aircraft             = maneuver->aircraft;
        std::vector<std::string> copters = std::vector<std::string>();
        std::stringstream lineStream(maneuver->multicopters.c_str());
        std::string copter;
        while (std::getline(lineStream, copter, ','))
        {
          copters.push_back(copter);
        }

        inf("Aircraft: %s", aircraft.c_str());
        for (unsigned int i = 0; i < copters.size(); i++)
        {
          inf("Multicopter[%d]: %s", i, copters[i].c_str());
        }

        std::string vh_id = resolveSystemId(maneuver->getSource());

        if (vh_id == aircraft)
        {
          //aircraft
          debug("Aircraft maneuver");
        }
        else
        {
          //assume copter
          debug("Copter maneuver");
        }

    	  // First disable all
    	  setControl(0);
        // Enable control loops
        setControl(IMC::CL_PATH);

        IMC::DesiredNetRecoveryPath d_path;
        d_path.start_lat = maneuver->start_lat;
        d_path.start_lon = maneuver->start_lon;
        d_path.end_lat = maneuver->end_lat;
        d_path.end_lon = maneuver->end_lon;
        d_path.z = maneuver->z;
        d_path.lbox_height = maneuver->lbox_height;
        d_path.lbox_width = maneuver->lbox_width;
        d_path.max_acc	= maneuver->max_acc;
        d_path.speed = maneuver->speed;
        d_path.z_off = maneuver->z_off;
        d_path.z_units = maneuver->z_units;
        d_path.aircraft = maneuver->aircraft;
        d_path.multicopters = maneuver->multicopters;

        dispatch(d_path);
        inf("DesiredNetRecoveryPath dispatched");
      }

      void
      consume(const IMC::NetRecoveryState* state)
      {
    	  spew("NetRecoveryState received");
    	  if (state->flags == IMC::NetRecoveryState::NR_STOP)
    	  {
    		  inf("NetRecoveryState STOP, maneuver is done");
    		  signalCompletion();
    	      setControl(0); // Becouse f* the other controllers.

    	  }
    	  else
    	  {

    	  }
      }
    };
  }
}

DUNE_TASK
