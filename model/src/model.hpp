//####################################################
//#                                                  #
//#Autor: BLAYES Hugo                                #
//#Created: 8/7                                      #
//#Version: 1.0.0                                    #
//#Description: Interface for neuromorphic model     #
//#                                                  #
//####################################################

#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"

class Model {
public:
	Model() {};

	virtual void setup(int *nb_state, int *nb_input, int *nb_output)=0;
	virtual void initial_value_setup(double **state, double **input)=0;
	virtual void system_update()=0;
	virtual void sim_com(double *input, double *output)=0;
	virtual void dyn_system(double *input, double *state, double *dstate, double *output)=0;
	virtual void get_temp_timer(double *timer)=0;
	virtual void setup_pub_sub(rclcpp::Node *simulator) = 0;
	virtual void stop_model() = 0;

private:
};
