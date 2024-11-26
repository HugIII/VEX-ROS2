//####################################################
//#                                                  #
//#Autor: BLAYES Hugo                                #
//#Created: 8/7                                      #
//#Version: 1.0.0                                    #
//#Description: Model manager for neuromorphic model #
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
	//change to the model we want to use
	Simulator(std::shared_ptr<Model> m);

	~Simulator();

	static void launch(int argc, char *argv[], std::shared_ptr<Model> m);

private:
	std::shared_ptr<Model> m;

	rclcpp::TimerBase::SharedPtr loop_rate;

	rclcpp::Subscription<vex_message::msg::Vexmsg>::SharedPtr subscription_;

	rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr terminate_;

	int* nb_state;
	int* nb_input;
	int* nb_output;
	
	double *input, *state, *state2, *dstate, *dstate2, *output;

	double *sim_timer;
	int *sim_nb_steps;
	double *step_length;

	void topic_callback(const vex_message::msg::Vexmsg msg);
	
	void sim_function();

	void stop_node_callback(const std_msgs::msg::Empty msg);
};