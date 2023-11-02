#ifndef MY_ALGORITHMS_H_
#define MY_ALGORITHMS_H_


unsigned int expected_reactive (unsigned int current_vertex, vertex *vertex_web, double current_time, double x_robot, double y_robot, double vel_robot, double *estimate_last_visit, double edge_avg);

unsigned int expected_cognitive (unsigned int current_vertex, vertex *vertex_web, double current_time, double x_robot, double y_robot, double vel_robot, double *estimate_last_visit, double edge_avg, unsigned int dimension, double *instantaneous_idleness, unsigned int & goal_vertex, double & goal_time);

unsigned int expected_cognitive_around (unsigned int current_vertex, vertex *vertex_web, double current_time, double x_robot, double y_robot, double vel_robot, double *estimate_last_visit, double edge_avg, unsigned int dimension, double *instantaneous_idleness, unsigned int & goal_vertex, double & goal_time);

unsigned int mixed_reactive (unsigned int current_vertex, vertex *vertex_web, double current_time, double x_robot, double y_robot, double vel_robot, double *estimate_last_visit, double edge_avg, unsigned int dimension);

void add_adjacent_vertex(std::set<unsigned int> & vertex_set, unsigned int current_vertex, vertex *vertex_web);

unsigned int my_conscientious_reactive (unsigned int current_vertex, vertex *vertex_web, double *instantaneous_idleness);


#endif  // MY_ALGORITHMS_H_
