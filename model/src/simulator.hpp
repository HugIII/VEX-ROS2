//####################################################
//#                                                  #
//#Autor: FERNANDEZ LORDEN Christian                 #
//#Created: 26/11/2024                               #
//#Version: 1.0.0                                    #
//#Description: Simulation Manager Header 			 #
//#                                                  #
//####################################################

#pragma once

#include "model.hpp"

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "vex_message/msg/vexmsg.hpp"
#include "std_msgs/msg/empty.hpp"

//change to the model we want to use
class Simulator : public rclcpp::Node
{
public:
	//Change to the model we want to use
	Simulator(std::shared_ptr<Model> m);
	~Simulator();
	static void launch(int argc, char *argv[], std::shared_ptr<Model> m);

private:
	std::shared_ptr<Model> m;

	// Simulation loop callback
	rclcpp::TimerBase::SharedPtr loop_rate;

	// Stop simulation callback
	rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr terminate_;

	int nb_state, nb_input, nb_output;
	
	double *input, *state, *state2, *dstate, *dstate2, *output;

	double sim_timer;
	int sim_nb_steps;
	double step_length;
	
	void sim_function();
	void stop_node_callback(const std_msgs::msg::Empty msg);
};