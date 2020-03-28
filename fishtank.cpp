#include <cmath>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <time.h>
#include <vector>
#include "eigen-dir/Eigen/Dense"

using namespace std;

int NUM_FRAMES = 600;

class Boid {

public:
  Boid(Eigen::Vector3d position_arg, Eigen::Vector3d velocity_arg) {
    position = position_arg;
    velocity = velocity_arg;
  } 

  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
};

vector<Boid*> boids;

void write_to_file();
void change_positions_of_boids(int num_neighbors, double neighbor_radius);
Eigen::Vector3d collision_avoidance(Boid* b);
Eigen::Vector3d velocity_matching(Boid* b, int num_neighbors, double neighbor_radius);
Eigen::Vector3d flock_centering(Boid* b, int num_neighbors, double neighbor_radius);

int main(int argc, char** argv) {
  srand(time(NULL));

  //input validation later
  string input_file_name = argv[1];
  string output_file_name = argv[2];

  fstream input_file(input_file_name);
  ofstream output_file(output_file_name);

  double size, neighbor_radius, mass, collision, centering, velocity, hunger, damping, dt, length;
  int num_neighbors, num_boids;
  input_file >> size >> neighbor_radius >> num_neighbors >> mass >> collision >> centering >> velocity >> hunger >> damping >> dt >> length;
  //input_file.ignore(); //ignore newline character
  input_file >> num_boids;
  cout << num_boids << endl;
  input_file.ignore(); //ignore newline character
  
  string pos_x_string;
  string pos_y_string;
  string pos_z_string;
  string vel_x_string;
  string vel_y_string;
  string vel_z_string;

  double pos_x;
  double pos_y;
  double pos_z;
  double vel_x;
  double vel_y;
  double vel_z;

  for (int i=0; i < num_boids; i++) {
    input_file.ignore();
    //input_file.ignore('[');
    getline(input_file, pos_x_string, ',');
    getline(input_file, pos_y_string, ',');
    getline(input_file, pos_z_string, ']');

    //input_file.ignore(' ');
    //input_file.ignore('[');
    getline(input_file, vel_x_string, ',');
    getline(input_file, vel_y_string, ',');
    getline(input_file, vel_z_string, ']');

    //cout << pos_z_string << endl;
    //cout << vel_x_string << endl;
    //cout << vel_y_string << endl;
    //cout << vel_z_string << endl;
    pos_x = stod(pos_x_string.substr(1, pos_x_string.length() - 1));
    //cout << pos_x << endl;
    pos_y = stod(pos_y_string);
    pos_z = stod(pos_z_string);
    
    //cout << pos_x << " " << pos_y << " " << pos_z << endl;

    vel_x = stod(vel_x_string.substr(2, vel_x_string.length() - 1));
    vel_y = stod(vel_y_string);
    vel_z = stod(vel_z_string);

    //cout << vel_x << " " << vel_y << " " << vel_z << endl;

    Eigen::Vector3d position(pos_x, pos_y, pos_z);
    Eigen::Vector3d velocity(vel_x, vel_y, vel_z);

    Boid* new_boid = new Boid(position, velocity);
    boids.push_back(new_boid);
  }

  //write NUM_FRAMES to sample.out
  output_file << NUM_FRAMES << "\n";

  for (int i=0; i < NUM_FRAMES; i++) {
    //write new position and velocity for each boid to sample.out
    output_file << num_boids << "\n";
    for (int i=0; i < num_boids; i++) {
      output_file << "[" << boids[i]->position.x() << ',' << boids[i]->position.y() << ',' << boids[i]->position.z() << ']' << ' ';
      output_file << "[" << boids[i]->velocity.x() << ',' << boids[i]->velocity.y() << ',' << boids[i]->velocity.z() << ']' << ' ' << '\n';
    }
    output_file << 0 << "\n";

    change_positions_of_boids(num_neighbors, neighbor_radius);
  }
}

void change_positions_of_boids(int num_neighbors, double neighbor_radius) {
  Eigen::Vector3d v1, v2, v3;
  //Boid* b;

  //traverse boids and apply rules to each boid
  for (int i=0; i < (int) boids.size(); i++) {
    //b = boids[i];
    //cout << boids[i]->position << endl;
    v1 = collision_avoidance(boids[i]);
    v2 = velocity_matching(boids[i], num_neighbors, neighbor_radius);
    v3 = flock_centering(boids[i], num_neighbors, neighbor_radius);
    //cout << v1 << endl << v2 << endl << v3 << endl;
    boids[i]->velocity = boids[i]->velocity + v1 + v2 + v3;
    boids[i]->velocity = boids[i]->velocity * .999;

    double new_position_x = (double) (boids[i]->position.x() + boids[i]->velocity.x());
    double new_position_y = (double) (boids[i]->position.y() + boids[i]->velocity.y());
    double new_position_z = (double) (boids[i]->position.z() + boids[i]->velocity.z());
    if (new_position_x >= .5 || new_position_x < -.5)
      boids[i]->velocity.x() = boids[i]->velocity.x() * -1;
    if (new_position_y >= .5 || new_position_y < -.5)
      boids[i]->velocity.y() = boids[i]->velocity.y() * -1;
    if (new_position_z >= .5 || new_position_z < -.5)
      boids[i]->velocity.z() = boids[i]->velocity.z() * -1;
    boids[i]->position = boids[i]->position + boids[i]->velocity; 
  }
}

//rule for collision avoidance
Eigen::Vector3d collision_avoidance(Boid* b) {
  Eigen::Vector3d delta(0,0,0);
  Boid* curr_boid;

  int neighbors_seen = 0;
  for (int i=0; i < (int) boids.size(); i++) {
    curr_boid = boids[i];
    if (curr_boid != b) {
      Eigen::Vector3d distance;
      distance.x() = curr_boid->position.x() - b->position.x();
      distance.y() = curr_boid->position.y() - b->position.y();
      distance.z() = curr_boid->position.z() - b->position.z();

      // modify delta if distance between curr_boid and b is less than 0.001
      if (sqrt(distance.x() * distance.x() + distance.y() * distance.y() + distance.z() * distance.z()) < .001) {
	delta = delta - (curr_boid->position - b-> position);
      }
      
    }
  }

  //do collision avoidance on edges too?

  return delta;
}


//rule for velocity matching
Eigen::Vector3d velocity_matching(Boid* b, int num_neighbors, double neighbor_radius) {
  Eigen::Vector3d perceived_velocity(0,0,0);
  Boid* curr_boid;

  for (int i=0; i < (int) boids.size(); i++) {
    curr_boid = boids[i];

    Eigen::Vector3d distance;
    distance.x() = curr_boid->position.x() - b->position.x();
    distance.y() = curr_boid->position.y() - b->position.y();
    distance.z() = curr_boid->position.z() - b->position.z();
    double real_radius = (double) sqrt(distance.x() * distance.x() + distance.y() * distance.y() + distance.z() * distance.z());

    int neighbors_count = 0;
    if (curr_boid != b && sqrt(distance.x() * distance.x() + distance.y() * distance.y() + distance.z() * distance.z()) < neighbor_radius && neighbors_count < num_neighbors) {
      perceived_velocity = perceived_velocity + curr_boid->velocity;
      neighbors_count++;
    }
  }

  perceived_velocity = perceived_velocity / (boids.size() - 1);

  return (perceived_velocity - b->velocity) / 8;
}

//rule for flock centering
Eigen::Vector3d flock_centering(Boid* b, int num_neighbors, double neighbor_radius) {
  Eigen::Vector3d perceived_center(0,0,0);
  Boid* curr_boid;
  int neighbors_count = 0;
  
  for (int i=0; i < (int) boids.size(); i++) {
    curr_boid = boids[i];

    Eigen::Vector3d distance;
    distance.x() = curr_boid->position.x() - b->position.x();
    distance.y() = curr_boid->position.y() - b->position.y();
    distance.z() = curr_boid->position.z() - b->position.z();
    double real_radius = (double) sqrt(distance.x() * distance.x() + distance.y() * distance.y() + distance.z() * distance.z());

    if (b != curr_boid && real_radius < neighbor_radius && neighbors_count < num_neighbors) {
      perceived_center += curr_boid->position;
      neighbors_count++;
    }
  }

  perceived_center /= ((int) boids.size() - 1);
  return perceived_center / 100;
}

