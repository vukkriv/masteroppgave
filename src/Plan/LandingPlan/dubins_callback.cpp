
// This task generates a dubins path that can be used for landing
// DUNE headers.
#include <DUNE/DUNE.hpp>

// USER headers
#include <USER/DUNE.hpp>

#include <vector>
#include <cmath>

#include <aw-dubins-curves/dubins.h>

//! Callback function called from dubins_path_sample_many
// adds the path to a vector for further processing
// @param q configuration parameter {x,y,heading} of the current sample
// @param x distance along the path at the current sample
// @param user_dat pointer to the vector the elements will be added to
// @output success(0) or sampling should be stopped (1)
int
addToPath_callback(double q[3], double x, void* user_data) {
  /*const int* c = user_data;*/
  std::vector<Matrix> arc = *((std::vector<Matrix> *)user_data);
  Matrix tempP = Matrix(3,1,0.0); //additional (3rd) index is needed for z
  tempP(0,0) = q[0];
  tempP(1,0) = q[1];
  //DUNE::Tasks::Task::trace("%f,%f,%f,%f\n", q[0], q[1], q[2], x);
  arc.push_back(tempP);
  return 0;
}

