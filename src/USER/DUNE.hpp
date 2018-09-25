//***************************************************************************
// Copyright 2007-2018 Norwegian University of Science and Technology (NTNU)*
// Centre for Autonomous Marine Operations and Systems (AMOS)               *
// Department of Engineering Cybernetics (ITK)                              *
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

#ifndef USER_DUNE_HPP_INCLUDED_
#define USER_DUNE_HPP_INCLUDED_

// USER module headers.
#include <USER/Control.hpp>
#include <USER/Coordinates.hpp>
#include <USER/Hardware.hpp>


//! DUNE: Uniform Navigational Environment.
namespace DUNE
{ }

//! Convenience macro to import DUNE's most commonly used namespaces
//! into global scope. Use it like this: 'using USER_NAMESPACES;'
#define USER_NAMESPACES                         \
  namespace DUNE;                               \
  using namespace DUNE::Cooordinates            \
  using namespace DUNE::Control                 \
  using namespace DUNE::Hardware

#endif
