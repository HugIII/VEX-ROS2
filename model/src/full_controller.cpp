#include "simulator.hpp"
#include "model.hpp"
#include "fast_tanh.cpp"

#include <chrono>

using std::placeholders::_1;
using namespace std::chrono_literals;

#include <iostream>

#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/float64.hpp"

class FULLModel : public Model
{
public:
	FULLModel() : Model() {}
	/*
	 * INPUTS
	 * ======
	 * input : double vector of length 1
	 * state : double vector of length 35
	 *
	 * OUTPUTS
	 * =======
	 * dstate : double vector of length 35
	 * output : double vector of length 1
	 */

	void setup(int &nb_state, int &nb_input, int &nb_output) override
	{
		nb_state = 35;
		nb_input = 1;
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
		state[12] = 0;
		state[13] = 0;
		state[14] = 0;
		state[15] = 0;
		state[16] = -0.5;
		state[17] = -0.5;
		state[18] = -0.5;
		state[19] = -0.5;
		state[20] = 0;
		state[21] = 0;
		state[22] = 0;
		state[23] = 0;
		state[24] = 0;
		state[25] = 0;
		state[26] = 0;
		state[27] = 0;
		state[28] = -0.5;
		state[29] = -0.5;
		state[30] = -0.5;
		state[31] = -0.5;
		state[32] = 0;
		state[33] = 0;
		state[34] = 0;

		input[0] = 0; // angle
	}

	void get_temp_timer(double &timer, int &sim_nb_steps) override
	{
		timer = 10;
		sim_nb_steps = 50;
	}

	void setup_pub_sub(rclcpp::Node *simulator) override
	{
		//subscription_ = simulator->create_subscription<std_msgs::msg::Empty>("freq_topic", 10, std::bind(&FULLModel::freq_callback, this, _1));
		publisher_ = simulator->create_publisher<std_msgs::msg::Float64>("count_topic", 10);
	}

	void sim_com([[maybe_unused]] double *input, [[maybe_unused]] double *output) override
	{
		auto message = std_msgs::msg::Float64();
		message.data = output[1];
		publisher_->publish(message);
		// std::cout << "sim_com" << std::endl;
	}

	void dyn_system(double *input, double *state, double *dstate, double *output) override
	{
		this->count++;
		// Named Constants
		const double V0 = -1.2;
		const double angle_ref = 2.0;
		const double tau_angle_inv = 200.0;
		const double cos_sin_amp = 2.5;
		const double tau_diff_fast_inv = 25000.0;
		const double gain_diff_fast = 2778.0;
		const double tau_diff_slow_inv = 2500.0;
		const double gain_diff_slow = 2778.0;
		const double gsyn_sin_pos = 20.0;
		const double dsyn_sin_pos = 0.0;
		const double gain_sin_pos = 100.0;
		const double gsyn_sin_neg = 20.0;
		const double dsyn_sin_neg = 0.0;
		const double gain_sin_neg = 100.0;
		const double gsyn_dcos_neg = -4.0;
		const double dsyn_dcos_neg = 0.05;
		const double gain_dcos_neg = 100.0;
		const double gfm_rebound_inhib = -3.0;
		const double gsp_rebound_inhib = 6.0;
		const double dfm_rebound_inhib = 0.0;
		const double dsp_rebound_inhib = 0.5;
		const double tau_V_rebound_inhib_inv = 2500.0;
		const double tau_vf_rebound_inhib_inv = 2500.0;
		const double tau_vs_rebound_inhib_inv = 62.5;
		const double Iapp_rebound_inhib = 0.2;
		const double gsyn_rebound_inhib = -2.0;
		const double dsyn_rebound_inhib = -1.0;
		const double tau_syn_rebound_inhib_inv = 25.0;
		const double gfm_rebound_out = -3.0;
		const double gsp_rebound_out = 6.0;
		const double dfm_rebound_out = 0.0;
		const double dsp_rebound_out = 0.5;
		const double tau_V_rebound_out_inv = 2500.0;
		const double tau_vf_rebound_out_inv = 2500.0;
		const double tau_vs_rebound_out_inv = 62.5;
		const double Iapp_rebound_out = 0.1;
		const double dsyn_event_pos = 0.5;
		const double dsyn_event_neg = 0.5;
		const double gain_high_sin_pos = 100.0;
		const double dsyn_high_sin_pos = 0.0;
		const double gain_low_sin_pos = 100.0;
		const double dsyn_low_sin_pos = 0.0;
		const double gain_high_sin_neg = 100.0;
		const double dsyn_high_sin_neg = 0.0;
		const double gain_low_sin_neg = 100.0;
		const double dsyn_low_sin_neg = 0.0;
		const double dsyn_amp_pos_dec = 0.5;
		const double dsyn_amp_pos_inc = 0.5;
		const double dsyn_amp_neg_dec = 0.5;
		const double dsyn_amp_neg_inc = 0.5;
		const double gsyn_bistable_in_pos = 20.0;
		const double dsyn_bistable_in_pos = 5.0;
		const double gsyn_bistable_in_neg = 20.0;
		const double dsyn_bistable_in_neg = 5.0;
		const double gsyn_bistable = 5.0;
		const double dsyn_bistable = 0.0;
		const double tau_bistable_inv = 2500.0;
		const double gsyn_bistable_out_pos = 20.0;
		const double dsyn_bistable_out_pos = 0.0;
		const double gsyn_bistable_out_neg = 20.0;
		const double dsyn_bistable_out_neg = 0.0;
		const double dsyn_freq_pos_dec = 5.0;
		const double dsyn_freq_pos_inc = 5.0;
		const double dsyn_freq_neg_dec = 5.0;
		const double dsyn_freq_neg_inc = 5.0;
		const double gsyn_hco_in_pos = 0.2;
		const double dsyn_hco_in_pos = 5.0;
		const double gsyn_hco_in_neg = 0.2;
		const double dsyn_hco_in_neg = 5.0;
		const double gfm_hco_pos = -3.0;
		const double gsp_hco_pos = 6.0;
		const double gsm_hco_pos = -3.0;
		const double gup_hco_pos = 5.0;
		const double gum_hco_pos = -2.0;
		const double dfm_hco_pos = 0.0;
		const double dsp_hco_pos = 0.5;
		const double dsm_hco_pos = -0.5;
		const double dup_hco_pos = -0.5;
		const double dum_hco_pos = -1.0;
		const double tau_V_hco_pos_inv = 2500.0;
		const double tau_vf_hco_pos_inv = 2500.0;
		const double tau_vs_hco_pos_inv = 62.5;
		const double tau_vu_hco_pos_inv = 1.25;
		const double Iapp_hco_pos = 0.2;
		const double gfm_hco_neg = -3.0;
		const double gsp_hco_neg = 6.0;
		const double gsm_hco_neg = -3.0;
		const double gup_hco_neg = 5.0;
		const double gum_hco_neg = -2.0;
		const double dfm_hco_neg = 0.0;
		const double dsp_hco_neg = 0.5;
		const double dsm_hco_neg = -0.5;
		const double dup_hco_neg = -0.5;
		const double dum_hco_neg = -1.0;
		const double tau_V_hco_neg_inv = 2500.0;
		const double tau_vf_hco_neg_inv = 2500.0;
		const double tau_vs_hco_neg_inv = 62.5;
		const double tau_vu_hco_neg_inv = 1.25;
		const double Iapp_hco_neg = 0.2;
		const double gsyn_hco_pos = -2.0;
		const double dsyn_hco_pos = 0.0;
		const double tau_syn_hco_pos_inv = 25.0;
		const double gsyn_hco_neg = -2.0;
		const double dsyn_hco_neg = 0.0;
		const double tau_syn_hco_neg_inv = 25.0;
		const double gsyn_hco_pos_mod_inc = 1.5;
		const double dsyn_hco_pos_mod_inc = 5.0;
		const double gsyn_hco_pos_mod_dec = 1.5;
		const double dsyn_hco_pos_mod_dec = 5.0;
		const double tau_hco_pos_mod_inv = 0.0025;
		const double gsyn_hco_neg_mod_inc = 1.5;
		const double dsyn_hco_neg_mod_inc = 5.0;
		const double gsyn_hco_neg_mod_dec = 1.5;
		const double dsyn_hco_neg_mod_dec = 5.0;
		const double tau_hco_neg_mod_inv = 0.0025;
		const double gsyn_hco_out_pos = 20.0;
		const double dsyn_hco_out_pos = 1.0;
		const double gsyn_hco_out_neg = 20.0;
		const double dsyn_hco_out_neg = 1.0;
		const double gsyn_mot_pos = 4.0;
		const double dsyn_mot_pos = 0.0;
		const double gsyn_mot_pos_dep = 100.0;
		const double dsyn_mot_pos_dep = 10.0;
		const double tau_syn_mot_pos_inv = 2.5;
		const double gsyn_mot_neg = 4.0;
		const double dsyn_mot_neg = 0.0;
		const double gsyn_mot_neg_dep = 100.0;
		const double dsyn_mot_neg_dep = 10.0;
		const double tau_syn_mot_neg_inv = 2.5;
		const double gfm_mot_pos = -3.0;
		const double gsp_mot_pos = 6.0;
		const double gsm_mot_pos = -2.3;
		const double gup_mot_pos = 5.0;
		const double gum_mot_pos = -2.0;
		const double dfm_mot_pos = 0.0;
		const double dsp_mot_pos = 0.5;
		const double dsm_mot_pos = -0.5;
		const double dup_mot_pos = -0.5;
		const double dum_mot_pos = -1.0;
		const double tau_V_mot_pos_inv = 2500.0;
		const double tau_vf_mot_pos_inv = 2500.0;
		const double tau_vs_mot_pos_inv = 62.5;
		const double tau_vu_mot_pos_inv = 1.25;
		const double Iapp_mot_pos = -0.5;
		const double gfm_mot_neg = -3.0;
		const double gsp_mot_neg = 6.0;
		const double gsm_mot_neg = -2.3;
		const double gup_mot_neg = 5.0;
		const double gum_mot_neg = -2.0;
		const double dfm_mot_neg = 0.0;
		const double dsp_mot_neg = 0.5;
		const double dsm_mot_neg = -0.5;
		const double dup_mot_neg = -0.5;
		const double dum_mot_neg = -1.0;
		const double tau_V_mot_neg_inv = 2500.0;
		const double tau_vf_mot_neg_inv = 2500.0;
		const double tau_vs_mot_neg_inv = 62.5;
		const double tau_vu_mot_neg_inv = 1.25;
		const double Iapp_mot_neg = -0.5;
		const double gsyn_gsm_mot_pos_mod_inc = 1.5;
		const double dsyn_gsm_mot_pos_mod_inc = 5.0;
		const double gsyn_gsm_mot_pos_mod_dec = 1.5;
		const double dsyn_gsm_mot_pos_mod_dec = 5.0;
		const double tau_mot_pos_mod_inv = 0.0025;
		const double gsyn_gsm_mot_neg_mod_inc = 1.5;
		const double dsyn_gsm_mot_neg_mod_inc = 5.0;
		const double gsyn_gsm_mot_neg_mod_dec = 1.5;
		const double dsyn_gsm_mot_neg_mod_dec = 5.0;
		const double tau_mot_neg_mod_inv = 0.0025;
		const double gsyn_mot_out_pos = 20.0;
		const double dsyn_mot_out_pos = 1.0;
		const double gsyn_mot_out_neg = 20.0;
		const double dsyn_mot_out_neg = 1.0;
		const double gsyn_out_inv = 0.05;

		// Input Reading
		double angle = input[0];

		// State Reading
		double neuron_mot_pos_s_1 = state[25];
		double neuron_mot_pos_s_2 = state[26];
		double neuron_mot_pos_s_3 = state[27];
		double neuron_mot_pos_s_4 = state[28];
		double neuron_hco_pos_s_1 = state[11];
		double neuron_hco_pos_s_2 = state[12];
		double neuron_hco_pos_s_3 = state[13];
		double neuron_hco_pos_s_4 = state[14];
		double preprocessing_s_1 = state[0];
		double lowpass_diff_s_1 = state[1];
		double lowpass_diff_s_2 = state[2];
		double neuron_rebound_out_s_1 = state[7];
		double neuron_rebound_out_s_2 = state[8];
		double neuron_rebound_out_s_3 = state[9];
		double gsm_mot_pos_mod_s_1 = state[33];
		double synapse_mot_neg_s_1 = state[24];
		double synapse_mot_pos_s_1 = state[23];
		double bistable_s_1 = state[10];
		double neuron_hco_neg_s_1 = state[15];
		double neuron_hco_neg_s_2 = state[16];
		double neuron_hco_neg_s_3 = state[17];
		double neuron_hco_neg_s_4 = state[18];
		double synapse_hco_pos_s_1 = state[19];
		double synapse_hco_neg_s_1 = state[20];
		double neuron_rebound_inhib_s_1 = state[3];
		double neuron_rebound_inhib_s_2 = state[4];
		double neuron_rebound_inhib_s_3 = state[5];
		double synapse_rebound_inhib_s_1 = state[6];
		double gsyn_hco_neg_mod_s_1 = state[22];
		double neuron_mot_neg_s_1 = state[29];
		double neuron_mot_neg_s_2 = state[30];
		double neuron_mot_neg_s_3 = state[31];
		double neuron_mot_neg_s_4 = state[32];
		double gsyn_hco_pos_mod_s_1 = state[21];
		double gsm_mot_neg_mod_s_1 = state[34];

		// Intermediate Value Computing
		double reference_cos_i_1 = (cos_sin_amp * cos(angle_ref));
		double neuron_mot_pos_i_1 = (gfm_mot_pos * (fast_tanh((neuron_mot_pos_s_2 - dfm_mot_pos)) - fast_tanh((V0 - dfm_mot_pos))));
		double neuron_mot_pos_i_2 = (gsp_mot_pos * (fast_tanh((neuron_mot_pos_s_3 - dsp_mot_pos)) - fast_tanh((V0 - dsp_mot_pos))));
		double neuron_mot_pos_i_3 = (gsm_mot_pos_mod_s_1 * (fast_tanh((neuron_mot_pos_s_3 - dsm_mot_pos)) - fast_tanh((V0 - dsm_mot_pos))));
		double neuron_mot_pos_i_4 = (gup_mot_pos * (fast_tanh((neuron_mot_pos_s_4 - dup_mot_pos)) - fast_tanh((V0 - dup_mot_pos))));
		double neuron_mot_pos_i_5 = (gum_mot_pos * (fast_tanh((neuron_mot_pos_s_4 - dum_mot_pos)) - fast_tanh((V0 - dum_mot_pos))));
		double neuron_hco_pos_i_1 = (gfm_hco_pos * (fast_tanh((neuron_hco_pos_s_2 - dfm_hco_pos)) - fast_tanh((V0 - dfm_hco_pos))));
		double neuron_hco_pos_i_2 = (gsp_hco_pos * (fast_tanh((neuron_hco_pos_s_3 - dsp_hco_pos)) - fast_tanh((V0 - dsp_hco_pos))));
		double neuron_hco_pos_i_3 = (gsm_hco_pos * (fast_tanh((neuron_hco_pos_s_3 - dsm_hco_pos)) - fast_tanh((V0 - dsm_hco_pos))));
		double neuron_hco_pos_i_4 = (gup_hco_pos * (fast_tanh((neuron_hco_pos_s_4 - dup_hco_pos)) - fast_tanh((V0 - dup_hco_pos))));
		double neuron_hco_pos_i_5 = (gum_hco_pos * (fast_tanh((neuron_hco_pos_s_4 - dum_hco_pos)) - fast_tanh((V0 - dum_hco_pos))));
		double mot_out_i_1 = (gsyn_mot_out_pos * ((fast_tanh(2 * ((neuron_mot_pos_s_1 - dsyn_mot_out_pos))) + 1) / 2));
		double mot_out_i_2 = (gsyn_mot_out_neg * ((fast_tanh(2 * ((neuron_mot_neg_s_1 - dsyn_mot_out_neg))) + 1) / 2));
		double preprocessing_i_1 = (cos_sin_amp * sin(preprocessing_s_1));
		double preprocessing_i_2 = (cos_sin_amp * cos(preprocessing_s_1));
		double reference_cos_i_2 = (reference_cos_i_1 - preprocessing_i_2);
		double reference_cos_i_3 = -(reference_cos_i_2);
		double lowpass_diff_i_1 = (lowpass_diff_s_1 - lowpass_diff_s_2);
		double neuron_rebound_out_i_1 = (gfm_rebound_out * (fast_tanh((neuron_rebound_out_s_2 - dfm_rebound_out)) - fast_tanh((V0 - dfm_rebound_out))));
		double neuron_rebound_out_i_2 = (gsp_rebound_out * (fast_tanh((neuron_rebound_out_s_3 - dsp_rebound_out)) - fast_tanh((V0 - dsp_rebound_out))));
		double synapse_mot_neg_i_1 = (gsyn_mot_neg_dep * ((fast_tanh(2 * ((neuron_hco_neg_s_1 - dsyn_mot_neg))) + 1) / 2));
		double synapse_mot_neg_i_2 = (gsyn_mot_neg * ((fast_tanh(2 * ((synapse_mot_neg_s_1 - dsyn_mot_neg_dep))) + 1) / 2));
		double synapse_mot_neg_i_3 = ((gsyn_mot_neg - synapse_mot_neg_i_2) * ((fast_tanh(2 * ((neuron_hco_neg_s_1 - dsyn_mot_neg))) + 1) / 2));
		double sin_side_i_1 = (gsyn_sin_pos * ((fast_tanh(2 * ((gain_sin_pos * (preprocessing_i_1 - dsyn_sin_pos)))) + 1) / 2));
		double events_i_2 = (sin_side_i_1 * ((fast_tanh(2 * ((neuron_rebound_out_s_1 - dsyn_event_neg))) + 1) / 2));
		double sin_side_i_2 = (gsyn_sin_neg * ((fast_tanh(2 * ((gain_sin_neg * (-(preprocessing_i_1)-dsyn_sin_neg)))) + 1) / 2));
		double events_i_1 = (sin_side_i_2 * ((fast_tanh(2 * ((neuron_rebound_out_s_1 - dsyn_event_pos))) + 1) / 2));
		double sin_dcos_comparator_i_1 = (sin_side_i_1 * ((fast_tanh(2 * ((gain_high_sin_pos * (reference_cos_i_2 - dsyn_high_sin_pos)))) + 1) / 2));
		double sin_dcos_comparator_i_2 = (sin_side_i_1 * ((fast_tanh(2 * ((gain_low_sin_pos * (reference_cos_i_3 - dsyn_low_sin_pos)))) + 1) / 2));
		double sin_dcos_comparator_i_3 = (sin_side_i_2 * ((fast_tanh(2 * ((gain_high_sin_neg * (reference_cos_i_2 - dsyn_high_sin_neg)))) + 1) / 2));
		double sin_dcos_comparator_i_4 = (sin_side_i_2 * ((fast_tanh(2 * ((gain_low_sin_neg * (reference_cos_i_3 - dsyn_low_sin_neg)))) + 1) / 2));
		double amp_pos_mod_i_1 = (sin_dcos_comparator_i_1 * ((fast_tanh(2 * ((neuron_rebound_out_s_1 - dsyn_amp_pos_dec))) + 1) / 2));
		double gsm_mot_pos_mod_i_1 = (gsyn_gsm_mot_pos_mod_inc * ((fast_tanh(2 * ((amp_pos_mod_i_1 - dsyn_gsm_mot_pos_mod_inc))) + 1) / 2));
		double amp_pos_mod_i_2 = (sin_dcos_comparator_i_2 * ((fast_tanh(2 * ((neuron_rebound_out_s_1 - dsyn_amp_pos_inc))) + 1) / 2));
		double gsm_mot_pos_mod_i_2 = (gsyn_gsm_mot_pos_mod_dec * ((fast_tanh(2 * ((amp_pos_mod_i_2 - dsyn_gsm_mot_pos_mod_dec))) + 1) / 2));
		double gsm_mot_pos_mod_i_3 = (gsm_mot_pos_mod_i_1 - gsm_mot_pos_mod_i_2);
		double synapse_mot_pos_i_1 = (gsyn_mot_pos_dep * ((fast_tanh(2 * ((neuron_hco_pos_s_1 - dsyn_mot_pos))) + 1) / 2));
		double synapse_mot_pos_i_2 = (gsyn_mot_pos * ((fast_tanh(2 * ((synapse_mot_pos_s_1 - dsyn_mot_pos_dep))) + 1) / 2));
		double synapse_mot_pos_i_3 = ((gsyn_mot_pos - synapse_mot_pos_i_2) * ((fast_tanh(2 * ((neuron_hco_pos_s_1 - dsyn_mot_pos))) + 1) / 2));
		double bistable_i_2 = (gsyn_bistable * fast_tanh((bistable_s_1 - dsyn_bistable)));
		double neuron_hco_neg_i_1 = (gfm_hco_neg * (fast_tanh((neuron_hco_neg_s_2 - dfm_hco_neg)) - fast_tanh((V0 - dfm_hco_neg))));
		double neuron_hco_neg_i_2 = (gsp_hco_neg * (fast_tanh((neuron_hco_neg_s_3 - dsp_hco_neg)) - fast_tanh((V0 - dsp_hco_neg))));
		double neuron_hco_neg_i_3 = (gsm_hco_neg * (fast_tanh((neuron_hco_neg_s_3 - dsm_hco_neg)) - fast_tanh((V0 - dsm_hco_neg))));
		double neuron_hco_neg_i_4 = (gup_hco_neg * (fast_tanh((neuron_hco_neg_s_4 - dup_hco_neg)) - fast_tanh((V0 - dup_hco_neg))));
		double neuron_hco_neg_i_5 = (gum_hco_neg * (fast_tanh((neuron_hco_neg_s_4 - dum_hco_neg)) - fast_tanh((V0 - dum_hco_neg))));
		double bistable_out_i_1 = (gsyn_bistable_out_pos * ((fast_tanh(2 * ((bistable_s_1 - dsyn_bistable_out_pos))) + 1) / 2));
		double freq_pos_mod_i_1 = (bistable_out_i_1 * ((fast_tanh(2 * ((events_i_2 - dsyn_freq_pos_dec))) + 1) / 2));
		double bistable_out_i_2 = (gsyn_bistable_out_neg * ((fast_tanh(2 * ((-(bistable_s_1)-dsyn_bistable_out_neg))) + 1) / 2));
		double freq_pos_mod_i_2 = (bistable_out_i_2 * ((fast_tanh(2 * ((events_i_2 - dsyn_freq_pos_inc))) + 1) / 2));
		double synapse_hco_pos_i_1 = (gsyn_hco_pos_mod_s_1 * ((fast_tanh(2 * ((synapse_hco_pos_s_1 - dsyn_hco_pos))) + 1) / 2));
		double hco_out_i_1 = (gsyn_hco_out_pos * ((fast_tanh(2 * ((neuron_hco_pos_s_1 - dsyn_hco_out_pos))) + 1) / 2));
		double bistable_in_i_2 = (gsyn_bistable_in_pos * ((fast_tanh(2 * ((hco_out_i_1 - dsyn_bistable_in_pos))) + 1) / 2));
		double hco_out_i_2 = (gsyn_hco_out_neg * ((fast_tanh(2 * ((neuron_hco_neg_s_1 - dsyn_hco_out_neg))) + 1) / 2));
		double bistable_in_i_3 = (gsyn_bistable_in_neg * ((fast_tanh(2 * ((hco_out_i_2 - dsyn_bistable_in_neg))) + 1) / 2));
		double bistable_in_i_1 = (bistable_in_i_2 - bistable_in_i_3);
		double bistable_i_1 = (bistable_i_2 + bistable_in_i_1);
		double synapse_hco_neg_i_1 = (gsyn_hco_neg_mod_s_1 * ((fast_tanh(2 * ((synapse_hco_neg_s_1 - dsyn_hco_neg))) + 1) / 2));
		double amp_neg_mod_i_1 = (sin_dcos_comparator_i_3 * ((fast_tanh(2 * ((neuron_rebound_out_s_1 - dsyn_amp_neg_dec))) + 1) / 2));
		double amp_neg_mod_i_2 = (sin_dcos_comparator_i_4 * ((fast_tanh(2 * ((neuron_rebound_out_s_1 - dsyn_amp_neg_inc))) + 1) / 2));
		double hco_in_i_1 = (gsyn_hco_in_pos * ((fast_tanh(2 * ((events_i_1 - dsyn_hco_in_pos))) + 1) / 2));
		double hco_in_i_2 = (gsyn_hco_in_neg * ((fast_tanh(2 * ((events_i_1 - dsyn_hco_in_neg))) + 1) / 2));
		double neuron_hco_pos_in_i_1 = ((hco_in_i_1 - hco_in_i_2) + synapse_hco_pos_i_1);
		double neuron_hco_neg_in_i_1 = ((hco_in_i_1 - hco_in_i_2) + synapse_hco_neg_i_1);
		double neuron_rebound_inhib_i_1 = (gfm_rebound_inhib * (fast_tanh((neuron_rebound_inhib_s_2 - dfm_rebound_inhib)) - fast_tanh((V0 - dfm_rebound_inhib))));
		double neuron_rebound_inhib_i_2 = (gsp_rebound_inhib * (fast_tanh((neuron_rebound_inhib_s_3 - dsp_rebound_inhib)) - fast_tanh((V0 - dsp_rebound_inhib))));
		double synapse_rebound_inhib_i_1 = (gsyn_rebound_inhib * ((fast_tanh(2 * ((synapse_rebound_inhib_s_1 - dsyn_rebound_inhib))) + 1) / 2));
		double neuron_mot_neg_i_1 = (gfm_mot_neg * (fast_tanh((neuron_mot_neg_s_2 - dfm_mot_neg)) - fast_tanh((V0 - dfm_mot_neg))));
		double neuron_mot_neg_i_2 = (gsp_mot_neg * (fast_tanh((neuron_mot_neg_s_3 - dsp_mot_neg)) - fast_tanh((V0 - dsp_mot_neg))));
		double neuron_mot_neg_i_3 = (gsm_mot_neg_mod_s_1 * (fast_tanh((neuron_mot_neg_s_3 - dsm_mot_neg)) - fast_tanh((V0 - dsm_mot_neg))));
		double neuron_mot_neg_i_4 = (gup_mot_neg * (fast_tanh((neuron_mot_neg_s_4 - dup_mot_neg)) - fast_tanh((V0 - dup_mot_neg))));
		double neuron_mot_neg_i_5 = (gum_mot_neg * (fast_tanh((neuron_mot_neg_s_4 - dum_mot_neg)) - fast_tanh((V0 - dum_mot_neg))));
		double freq_neg_mod_i_1 = (bistable_out_i_2 * ((fast_tanh(2 * ((events_i_1 - dsyn_freq_neg_dec))) + 1) / 2));
		double gsyn_hco_neg_mod_i_1 = (gsyn_hco_neg_mod_inc * ((fast_tanh(2 * ((freq_neg_mod_i_1 - dsyn_hco_neg_mod_inc))) + 1) / 2));
		double freq_neg_mod_i_2 = (bistable_out_i_1 * ((fast_tanh(2 * ((events_i_1 - dsyn_freq_neg_inc))) + 1) / 2));
		double gsyn_hco_neg_mod_i_2 = (gsyn_hco_neg_mod_dec * ((fast_tanh(2 * ((freq_neg_mod_i_2 - dsyn_hco_neg_mod_dec))) + 1) / 2));
		double gsyn_hco_neg_mod_i_3 = (gsyn_hco_neg_mod_i_1 - gsyn_hco_neg_mod_i_2);
		double gsyn_hco_pos_mod_i_1 = (gsyn_hco_pos_mod_inc * ((fast_tanh(2 * ((freq_pos_mod_i_1 - dsyn_hco_pos_mod_inc))) + 1) / 2));
		double gsyn_hco_pos_mod_i_2 = (gsyn_hco_pos_mod_dec * ((fast_tanh(2 * ((freq_pos_mod_i_2 - dsyn_hco_pos_mod_dec))) + 1) / 2));
		double gsyn_hco_pos_mod_i_3 = (gsyn_hco_pos_mod_i_1 - gsyn_hco_pos_mod_i_2);
		double rebound_input_i_1 = (gsyn_dcos_neg * ((fast_tanh(2 * ((gain_dcos_neg * (-(lowpass_diff_i_1)-dsyn_dcos_neg)))) + 1) / 2));
		double output_i_1 = (gsyn_out_inv * (mot_out_i_1 - mot_out_i_2));
		double gsm_mot_neg_mod_i_1 = (gsyn_gsm_mot_neg_mod_inc * ((fast_tanh(2 * ((amp_neg_mod_i_1 - dsyn_gsm_mot_neg_mod_inc))) + 1) / 2));
		double gsm_mot_neg_mod_i_2 = (gsyn_gsm_mot_neg_mod_dec * ((fast_tanh(2 * ((amp_neg_mod_i_2 - dsyn_gsm_mot_neg_mod_dec))) + 1) / 2));
		double gsm_mot_neg_mod_i_3 = (gsm_mot_neg_mod_i_1 - gsm_mot_neg_mod_i_2);

		// Derivative Computing
		dstate[26] = (tau_vf_mot_pos_inv * (neuron_mot_pos_s_1 - neuron_mot_pos_s_2));
		dstate[27] = (tau_vs_mot_pos_inv * (neuron_mot_pos_s_1 - neuron_mot_pos_s_3));
		dstate[28] = (tau_vu_mot_pos_inv * (neuron_mot_pos_s_1 - neuron_mot_pos_s_4));
		dstate[25] = (tau_V_mot_pos_inv * ((((((((V0 + Iapp_mot_pos) + synapse_mot_pos_i_3) - neuron_mot_pos_i_1) - neuron_mot_pos_i_2) - neuron_mot_pos_i_3) - neuron_mot_pos_i_4) - neuron_mot_pos_i_5) - neuron_mot_pos_s_1));
		dstate[12] = (tau_vf_hco_pos_inv * (neuron_hco_pos_s_1 - neuron_hco_pos_s_2));
		dstate[13] = (tau_vs_hco_pos_inv * (neuron_hco_pos_s_1 - neuron_hco_pos_s_3));
		dstate[11] = (tau_V_hco_pos_inv * ((((((((V0 + Iapp_hco_pos) + neuron_hco_pos_in_i_1) - neuron_hco_pos_i_1) - neuron_hco_pos_i_2) - neuron_hco_pos_i_3) - neuron_hco_pos_i_4) - neuron_hco_pos_i_5) - neuron_hco_pos_s_1));
		dstate[14] = (tau_vu_hco_pos_inv * (neuron_hco_pos_s_1 - neuron_hco_pos_s_4));
		dstate[0] = (tau_angle_inv * (angle - preprocessing_s_1));
		dstate[1] = (tau_diff_fast_inv * ((gain_diff_fast * preprocessing_i_2) - lowpass_diff_s_1));
		dstate[2] = (tau_diff_slow_inv * ((gain_diff_slow * preprocessing_i_2) - lowpass_diff_s_1));
		dstate[7] = (tau_V_rebound_out_inv * (((((V0 + Iapp_rebound_out) + (rebound_input_i_1 + synapse_rebound_inhib_i_1)) - neuron_rebound_out_i_1) - neuron_rebound_out_i_2) - neuron_rebound_out_s_1));
		dstate[8] = (tau_vf_rebound_out_inv * (neuron_rebound_out_s_1 - neuron_rebound_out_s_2));
		dstate[9] = (tau_vs_rebound_out_inv * (neuron_rebound_out_s_1 - neuron_rebound_out_s_3));
		dstate[33] = ((tau_mot_pos_mod_inv * (gsm_mot_pos_mod_i_3 - gsm_mot_pos_mod_s_1)) + gsm_mot_pos);
		dstate[24] = (tau_syn_mot_neg_inv * (synapse_mot_neg_i_1 - synapse_mot_neg_s_1));
		dstate[23] = (tau_syn_mot_pos_inv * (synapse_mot_pos_i_1 - synapse_mot_pos_s_1));
		dstate[10] = (tau_bistable_inv * (bistable_i_1 - bistable_s_1));
		dstate[16] = (tau_vf_hco_neg_inv * (neuron_hco_neg_s_1 - neuron_hco_neg_s_2));
		dstate[15] = (tau_V_hco_neg_inv * ((((((((V0 + Iapp_hco_neg) + neuron_hco_neg_in_i_1) - neuron_hco_neg_i_1) - neuron_hco_neg_i_2) - neuron_hco_neg_i_3) - neuron_hco_neg_i_4) - neuron_hco_neg_i_5) - neuron_hco_neg_s_1));
		dstate[18] = (tau_vu_hco_neg_inv * (neuron_hco_neg_s_1 - neuron_hco_neg_s_4));
		dstate[17] = (tau_vs_hco_neg_inv * (neuron_hco_neg_s_1 - neuron_hco_neg_s_3));
		dstate[19] = (tau_syn_hco_pos_inv * (neuron_hco_neg_s_1 - synapse_hco_pos_s_1));
		dstate[20] = (tau_syn_hco_neg_inv * (neuron_hco_pos_s_1 - synapse_hco_neg_s_1));
		dstate[5] = (tau_vs_rebound_inhib_inv * (neuron_rebound_inhib_s_1 - neuron_rebound_inhib_s_3));
		dstate[4] = (tau_vf_rebound_inhib_inv * (neuron_rebound_inhib_s_1 - neuron_rebound_inhib_s_2));
		dstate[3] = (tau_V_rebound_inhib_inv * (((((V0 + Iapp_rebound_inhib) + rebound_input_i_1) - neuron_rebound_inhib_i_1) - neuron_rebound_inhib_i_2) - neuron_rebound_inhib_s_1));
		dstate[6] = (tau_syn_rebound_inhib_inv * (neuron_rebound_inhib_s_1 - synapse_rebound_inhib_s_1));
		dstate[22] = ((tau_hco_neg_mod_inv * (gsyn_hco_neg_mod_i_3 - gsyn_hco_neg_mod_s_1)) + gsyn_hco_neg);
		dstate[30] = (tau_vf_mot_neg_inv * (neuron_mot_neg_s_1 - neuron_mot_neg_s_2));
		dstate[31] = (tau_vs_mot_neg_inv * (neuron_mot_neg_s_1 - neuron_mot_neg_s_3));
		dstate[32] = (tau_vu_mot_neg_inv * (neuron_mot_neg_s_1 - neuron_mot_neg_s_4));
		dstate[29] = (tau_V_mot_neg_inv * ((((((((V0 + Iapp_mot_neg) + synapse_mot_neg_i_3) - neuron_mot_neg_i_1) - neuron_mot_neg_i_2) - neuron_mot_neg_i_3) - neuron_mot_neg_i_4) - neuron_mot_neg_i_5) - neuron_mot_neg_s_1));
		dstate[21] = ((tau_hco_pos_mod_inv * (gsyn_hco_pos_mod_i_3 - gsyn_hco_pos_mod_s_1)) + gsyn_hco_pos);
		dstate[34] = ((tau_mot_neg_mod_inv * (gsm_mot_neg_mod_i_3 - gsm_mot_neg_mod_s_1)) + gsm_mot_neg);

		// Output Setting
		output[0] = output_i_1;
		output[1] = neuron_hco_neg_s_1;
	}

	void stop_model() override
	{
		// Let empty, try to only use the destructor
	}

	~FULLModel()
	{
		std::cout << "Simulation Terminated" << std::endl;
	}

private:
	rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscription_;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;

	void freq_callback([[maybe_unused]] const std_msgs::msg::Empty msg)
	{
		this->count++;
	}

	long count;
};

int main(int argc, char *argv[])
{
	Simulator::launch(argc, argv, std::make_shared<FULLModel>());
}