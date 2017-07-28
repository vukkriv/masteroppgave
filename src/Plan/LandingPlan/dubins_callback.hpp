#ifndef LANDING_CALLBACK
#define LANDING_CALLBACK  
#include <aw-dubins-curves/dubins.h>

// mapping from aw-dubins words
#define dRSR LSL 
#define dRSL LSR
#define dRLR LRL 
#define dLSL RSR
#define dLSR RSL
#define dLRL RLR 

//! Callback function called from dubins_path_sample_many
// adds the path to a vector for further processing
// @param q configuration parameter {x,y,heading} of the current sample
// @param x distance along the path at the current sample
// @param user_dat pointer to the vector the elements will be added to
// @output success(0) or sampling should be stopped (1)
int
addToPath_callback(double q[3], double x, void* user_data);
#endif /* ifndef LANDING_CALLBACK */
