/*
 * Enums.hpp
 *
 *  Created on: Oct 21, 2016
 *      Author: siri
 */

#ifndef ENUMS_HPP_
#define ENUMS_HPP_

using DUNE_NAMESPACES;

// The enums used to choose which approach method will be used to calculate the CARP.
enum Modes {LONG_STRETCH = 0, OPTIMAL_LONG_STRETCH, OWSI};

// The enums for the state machine that controls the drop maneuvre.
enum States {IDLE=-1, GOING_TO_START_POINT, GOING_TO_DROP_POINT, GOING_TO_SAFE_HEIGHT, GLIDE_MODE, LAST_STATE};



#endif /* ENUMS_HPP_ */
