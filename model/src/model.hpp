//####################################################
//#                                                  #
//#Author: FERNANDEZ LORDEN Christian                #
//#Created: 26/11/2024                               #
//#Version: 1.0.0                                    #
//#Description: Model Template For Simulation 		 #
//#                                                  #
//####################################################

#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"

class Model {
public:
	Model() {};
	// Setup functions (for simulator)
	virtual void setup(int &nb_state, int &nb_input, int &nb_output) = 0;
	virtual void initial_value_setup(double *state, double *input) = 0;
	virtual void setup_pub_sub(rclcpp::Node *simulator) = 0;
	virtual void get_temp_timer(double &timer, int &sim_nb_steps) = 0;
	// Simulation functions
	virtual void sim_com(double *input, double *output)=0;
	virtual void dyn_system(double *input, double *state, double *dstate, double *output)=0;
	// Stop simulation function
	virtual void stop_model() = 0;
	~Model() {};
private:
};
