#include "eigen-dir/Eigen/Dense"

class Boid {

public:

  Boid() {
    position.x() = ((float) rand()) / (float) RAND_MAX - 0.5; //generate float between -0.5 and 0.5                                                                                                         
    position.x() = (((float) rand()) / (float) RAND_MAX) / 2 - 0.25; //generate float between -0.25 and 0.25                                                                                                
  }
  Boid(Eigen::Vector3d position_input) {
    position = position_input;
  }
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
};
