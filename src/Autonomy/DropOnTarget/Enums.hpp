/*
 * Enums.hpp
 *
 *  Created on: Oct 21, 2016
 *      Author: siri
 */

#ifndef ENUMS_HPP_
#define ENUMS_HPP_

using DUNE_NAMESPACES;

enum Modes {LONG_STRETCH = 0, INCREMENTED_LONG_STRETCH, OPTIMAL_LONG_STRETCH, OWSI};

enum States {IDLE=-1, GOING_TO_START_POINT, GOING_TO_DROP_POINT, GOING_TO_SAFE_HEIGHT, GLIDE_MODE, LAST_STATE};



#endif /* ENUMS_HPP_ */
