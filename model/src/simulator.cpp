//####################################################
//#                                                  #
//#Autor: BLAYES Hugo                                #
//#Created: 8/7                                      #
//#Version: 1.0.0                                    #
//#Description: Model manager for neuromorphic model #
//#                                                  #
//####################################################

#include "simulator.hpp"
#include "model.hpp"

#include <string>
#include <chrono>
#include <thread>
#include <iostream>
#include <cstdint>

using namespace std::chrono;
using namespace std::chrono_literals;
using std::placeholders::_1;

void Simulator::launch(int argc, char *argv[], std::shared_ptr<Model> m)
{
	rclcpp::init(argc, argv);
	try
	{
		rclcpp::spin(std::make_shared<Simulator>(m));
	}
	catch (const std::runtime_error &ex)
	{
		// System is terminated
	}
	rclcpp::shutdown();
}

Simulator::Simulator(std::shared_ptr<Model> m) : Node("Simulator")
{
	this->m = m;

	nb_state = (int*)malloc(sizeof(int));
	nb_input = (int*)malloc(sizeof(int));
	nb_output = (int*)malloc(sizeof(int));
	sim_timer = (double *)malloc(sizeof(double));
	sim_nb_steps = (int *)malloc(sizeof(int));
	step_length = (double *)malloc(sizeof(double));

	this->m->setup(nb_state,nb_input,nb_output);
	
	input = (double*)malloc(*nb_input * sizeof(double));
	state = (double*)malloc(*nb_state * sizeof(double));
	state2 = (double*)malloc(*nb_state * sizeof(double));
	dstate = (double*)malloc(*nb_state * sizeof(double));
	dstate2 = (double*)malloc(*nb_state * sizeof(double));
	output = (double*)malloc(*nb_output * sizeof(double));
	
	
	this->m->initial_value_setup(&state,&input);
	
	//std::cout << "error before"  << std::endl;

	this->m->get_temp_timer(sim_timer, sim_nb_steps);

	subscription_ = this->create_subscription<vex_message::msg::Vexmsg>("out_serial", 10, std::bind(&Simulator::topic_callback, this, _1));

	auto time = (*sim_timer) * 1us * (*sim_nb_steps);
	step_length = (*sim_timer) * 0.000001;
	loop_rate = this->create_wall_timer(time, std::bind(&Simulator::sim_function, this));

	this->m->setup_pub_sub(this);

	terminate_ = this->create_subscription<std_msgs::msg::Empty>("terminate_sim", 10, std::bind(&Simulator::stop_node_callback, this, _1));
}

Simulator::~Simulator()
{
	free(nb_state);
	free(nb_input);
	free(nb_output);
	free(timer);
	free(input);
	free(state);
	free(state2);
	free(dstate);
	free(dstate2);
	free(output);
}

void Simulator::topic_callback(const vex_message::msg::Vexmsg msg)
{
	this->m->system_update();
}

void Simulator::sim_function()
{
	//std::cout << "timer entry " << std::endl;

	auto step = this->step_length;

	this->m->sim_com(input,output);
	for (int j = 0; j < *sim_nb_steps; j++)
	{
		// Ode2 step (Heun method)
		this->m->dyn_system(input,state,dstate,output);
			
		for (int i = 0 ; i < *nb_state ; i++) {
			state2[i] = state[i] + step*dstate[i];
		}

		this->m->dyn_system(input,state2,dstate2,output);

		for (int i = 0 ; i < *nb_state ; i++) {
			state[i] = state[i] + step*(dstate[i] + dstate2[i])/2;
		}
	}
}

void Simulator::stop_node_callback(const std_msgs::msg::Empty msg)
{
	//delete subscription_;
	//delete terminate_;
	//delete loop_rate;
	this->m->stop_model();
	throw std::runtime_error("Terminated");
}
