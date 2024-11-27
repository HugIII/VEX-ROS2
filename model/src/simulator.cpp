//####################################################
//#                                                  #
//#Autor: FERNANDEZ LORDEN Christian                 #
//#Created: 26/11/2024                               #
//#Version: 1.0.0                                    #
//#Description: Simulation Manager Class 			 #
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

	this->m->setup(nb_state, nb_input, nb_output); // Values passed by reference

	input = (double*)malloc(nb_input * sizeof(double));
	state = (double*)malloc(nb_state * sizeof(double));
	state2 = (double*)malloc(nb_state * sizeof(double));
	dstate = (double*)malloc(nb_state * sizeof(double));
	dstate2 = (double*)malloc(nb_state * sizeof(double));
	output = (double*)malloc(nb_output * sizeof(double));

	this->m->initial_value_setup(state, input);

	// Model Specific Publisher and Subscriber
	this->m->setup_pub_sub(this); 

	// Simulation loop Timer
	this->m->get_temp_timer(sim_timer, sim_nb_steps); // Values passed by reference
	auto time = sim_timer * 1us * sim_nb_steps;
	step_length = sim_timer * 0.000001;
	loop_rate = this->create_wall_timer(time, std::bind(&Simulator::sim_function, this));

	// Terminate simulation Subscriber
	terminate_ = this->create_subscription<std_msgs::msg::Empty>("terminate_sim", 10, std::bind(&Simulator::stop_node_callback, this, _1));
}

Simulator::~Simulator()
{
	free(input);
	free(state);
	free(state2);
	free(dstate);
	free(dstate2);
	free(output);
}

void Simulator::sim_function()
{
	auto step = this->step_length;

	this->m->sim_com(input,output);
	for (int j = 0; j < sim_nb_steps; j++)
	{
		// Ode2 step (Heun method)
		this->m->dyn_system(input,state,dstate,output);
			
		for (int i = 0 ; i < nb_state ; i++) {
			state2[i] = state[i] + step*dstate[i];
		}

		this->m->dyn_system(input,state2,dstate2,output);

		for (int i = 0 ; i < nb_state ; i++) {
			state[i] = state[i] + step*(dstate[i] + dstate2[i])/2;
		}
	}
}

void Simulator::stop_node_callback(const std_msgs::msg::Empty msg)
{
	//this->m->stop_model();
	// Trhrowing error terminate the program
	// Let destructors handle the cleanup
	throw std::runtime_error("Terminated");
}
