#include "simulator.hpp"
#include "model.hpp"
#include "fast_tanh.cpp"

#include <chrono>

using std::placeholders::_1;
using namespace std::chrono_literals;

#include <iostream>

class HCOModel : public Model
{
public:
	HCOModel() : Model() {}

	void setup(int &nb_state, int &nb_input, int &nb_output) override
	{
		nb_state = 12;
		nb_input = 6;
		nb_output = 3;
	}

	void initial_value_setup(double *state, double *input) override
	{
		state[0] = 0;
		state[1] = 0;
		state[2] = 0;
		state[3] = 0;
		state[4] = -0.5;
		state[5] = -0.5;
		state[6] = -0.5;
		state[7] = -0.5;
		state[8] = 0;
		state[9] = 0;
		state[10] = 0;
		state[11] = 0;

		input[0] = 0; // angle
		input[1] = 0; // speed

		input[2] = -3.5; // gsm

		input[3] = 0; // Iapp

		input[4] = 12000.0; // gsyn
		input[5] = 0.0;	   // dsyn
	}

	void get_temp_timer(double &timer, int &sim_nb_steps) override
	{
		timer = 100;
		sim_nb_steps = 1;
	}

	void setup_pub_sub(rclcpp::Node *simulator) override
	{
		subscription_ = simulator->create_subscription<std_msgs::msg::Empty>("cool_topic", 10, std::bind(&HCOModel::cool_callback, this, _1));
		publisher_ = simulator->create_publisher<std_msgs::msg::Empty>("cool_topic", 10);
	}

	void sim_com([[maybe_unused]] double *input, [[maybe_unused]] double *output) override
	{
		publisher_->publish(std_msgs::msg::Empty());
		// std::cout << "sim_com" << std::endl;
	}

	void dyn_system(double *input, double *state, double *dstate, double *output) override
	{
		// Named Constants
		double gfm = -2.0;
		double gsp = 6.0;
		double gup = 5.0;
		double dfm = 0.0;
		double dsp = 0.5;
		double dsm = -0.5;
		double dup = -0.5;
		double V0 = -0.85;
		double tau_V_inv = 1000.0;
		double tau_vf_inv = 1000.0;
		double tau_vs_inv = 25.0;
		double tau_vu_inv = 1.25;
		double tau_vsyn_out_inv = 25.0;
		double gsyn = -1.0;
		double dsyn = 0.0;
		double tau_vsyn_inv = 25.0;
		double gspeed = 10.0;
		double gpos = 20.0;
		double dpos = 0.1;
		double Kfeed = 1.0;

		// Input reading
		double pos = input[0];
		double speed = input[1];
		double gsm = input[2];
		double Iapp = input[3];
		double gsyn_out = input[4];
		double dsyn_out = input[5];

		// State reading
		double synapse_out_1_s_1 = state[10];
		double neuron2_s_1 = state[4];
		double neuron2_s_2 = state[5];
		double neuron2_s_3 = state[6];
		double neuron2_s_4 = state[7];
		double synapse_out_2_s_1 = state[11];
		double synapse1_s_1 = state[8];
		double synapse2_s_1 = state[9];
		double neuron1_s_1 = state[0];
		double neuron1_s_2 = state[1];
		double neuron1_s_3 = state[2];
		double neuron1_s_4 = state[3];

		// Output Computing
		double synapse_out_1_i_1 = (gsyn_out * ((fast_tanh(((synapse_out_1_s_1 - dsyn_out) * (2.0))) + (1.0)) * (0.5)));
		double neuron2_i_1 = (gfm * (fast_tanh((neuron2_s_2 - dfm)) - fast_tanh((V0 - dfm))));
		double neuron2_i_2 = (gsp * (fast_tanh((neuron2_s_3 - dsp)) - fast_tanh((V0 - dsp))));
		double neuron2_i_3 = (gsm * (fast_tanh((neuron2_s_3 - dsm)) - fast_tanh((V0 - dsm))));
		double neuron2_i_4 = (gup * (fast_tanh((neuron2_s_4 - dup)) - fast_tanh((V0 - dup))));
		double synapse_out_2_i_1 = (gsyn_out * ((fast_tanh(((synapse_out_2_s_1 - dsyn_out) * (2.0))) + (1.0)) * (0.5)));
		double synapse1_i_1 = (gsyn * ((fast_tanh(((synapse1_s_1 - dsyn) * (2.0))) + (1.0)) * (0.5)));
		double synapse2_i_1 = (gsyn * ((fast_tanh(((synapse2_s_1 - dsyn) * (2.0))) + (1.0)) * (0.5)));
		double feedback_i_1 = sin(pos);
		double feedback_i_2 = fast_tanh((gspeed * speed));
		double feedback_i_3 = ((fast_tanh((gpos * (feedback_i_1 + dpos))) - fast_tanh((gpos * (feedback_i_1 - dpos)))) * (0.5));
		double feedback_i_4 = (Kfeed * (feedback_i_3 + ((((1.0) - feedback_i_2) * (0.5)) - (1.0))));
		double feedback_i_5 = (Kfeed * (feedback_i_3 + ((((1.0) + feedback_i_2) * (0.5)) - (1.0))));
		double neuron1_i_1 = (gfm * (fast_tanh((neuron1_s_2 - dfm)) - fast_tanh((V0 - dfm))));
		double neuron1_i_2 = (gsp * (fast_tanh((neuron1_s_3 - dsp)) - fast_tanh((V0 - dsp))));
		double neuron1_i_3 = (gsm * (fast_tanh((neuron1_s_3 - dsm)) - fast_tanh((V0 - dsm))));
		double neuron1_i_4 = (gup * (fast_tanh((neuron1_s_4 - dup)) - fast_tanh((V0 - dup))));
		double adder_i_1 = (Iapp + (synapse1_i_1 + feedback_i_4));
		double adder_i_2 = (Iapp + (synapse2_i_1 + feedback_i_5));
		double adder_i_3 = (synapse_out_1_i_1 - synapse_out_2_i_1);

		// Derivative Computing
		dstate[10] = (tau_vsyn_out_inv * (neuron1_s_1 - synapse_out_1_s_1));
		dstate[6] = (tau_vs_inv * (neuron2_s_1 - neuron2_s_3));
		dstate[5] = (tau_vf_inv * (neuron2_s_1 - neuron2_s_2));
		dstate[7] = (tau_vu_inv * (neuron2_s_1 - neuron2_s_4));
		dstate[4] = (tau_V_inv * ((((((V0 + adder_i_1) - neuron2_i_1) - neuron2_i_2) - neuron2_i_3) - neuron2_i_4) - neuron2_s_1));
		dstate[11] = (tau_vsyn_out_inv * (neuron2_s_1 - synapse_out_2_s_1));
		dstate[8] = (tau_vsyn_inv * (neuron1_s_1 - synapse1_s_1));
		dstate[9] = (tau_vsyn_inv * (neuron2_s_1 - synapse2_s_1));
		dstate[1] = (tau_vf_inv * (neuron1_s_1 - neuron1_s_2));
		dstate[0] = (tau_V_inv * ((((((V0 + adder_i_2) - neuron1_i_1) - neuron1_i_2) - neuron1_i_3) - neuron1_i_4) - neuron1_s_1));
		dstate[3] = (tau_vu_inv * (neuron1_s_1 - neuron1_s_4));
		dstate[2] = (tau_vs_inv * (neuron1_s_1 - neuron1_s_3));

		// Output Setting
		output[0] = adder_i_3;
		output[1] = neuron1_s_1;
		output[2] = neuron2_s_1;
	}

	void stop_model() override
	{
		// Let empty, try to only use the destructor
	}

	~HCOModel()
	{
		std::cout << "Simulation Terminated" << std::endl;
	}

private:
	rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscription_;
	rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_;

	void cool_callback([[maybe_unused]] const std_msgs::msg::Empty msg)
	{
		this->count++;
	}

	int count;
};

int main(int argc, char *argv[])
{
	Simulator::launch(argc, argv, std::make_shared<HCOModel>());
}