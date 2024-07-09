#include "rclcpp/rclcpp.hpp"

#include "vex_message/msg/vexrotationsensor.hpp"
#include "std_msgs/msg/float32.hpp"

#include "template.cpp"

class Hco_adapt: public Template{
public:
	Hco_adapt():Template(){
		setup_pub_sub();
	}

	void setup(int *nb_state, int *nb_input, int *nb_output) override{
		*nb_state = 24;
		*nb_input = 7;
		*nb_output = 1;
	}
	
	void initial_value_setup(double **state, double **input) override{
		// Feedback
		(*state)[0] = 0;
		(*state)[1] = 0;
		// Neuron1
		(*state)[2] = 0;
		(*state)[3] = 0;
		(*state)[4] = 0;
		(*state)[5] = 0;
		// Neuron2
		(*state)[6] = -0.5;
		(*state)[7] = -0.5;
		(*state)[8] = -0.5;
		(*state)[9] = -0.5;
		// Synapses
		(*state)[10] = 0;
		(*state)[11] = 0;
		(*state)[12] = 0;
		(*state)[13] = 0;
		// Neuron spike 1
		(*state)[14] = 0;
		(*state)[15] = 0;
		(*state)[16] = 0;
		(*state)[17] = 0;
		// Neuron spike 2
		(*state)[18] = 0;	
		(*state)[19] = 0;
		(*state)[20] = 0;
		(*state)[21] = 0;
		// Neuromod Block
		(*state)[22] = -3.8; // gsm initial val
		(*state)[23] = -3.8; // gsm initial val
	
		(*input)[0] = 0; // angle
		(*input)[1] = 0; // speed
	
		(*input)[2] = (M_PI / 2) - 0.1; // angle ref
	
		(*input)[3] = -2; // Iapp
		(*input)[4] = -0;	// Iapp mod
	
		(*input)[5] = 12000.0; // gsyn
		(*input)[6] = 0.0;	   // dsyn
	}
		
	void system_update() override{
		double tmp_angle = (msg_rotation_sensor.angle * M_PI / 180.0);
		double tmp_speed = msg_rotation_sensor.velocity * M_PI / 30.0;
	
		filter_sum -= smoothing_filter[filter_ind];
		smoothing_filter[filter_ind] = tmp_speed;
		filter_sum += tmp_speed;
	
		filter_ind = (filter_ind + 1) % SMOOTHING_PERIOD;

		double smooth_speed = filter_sum / SMOOTHING_PERIOD;

		double tmp_voltage = voltage;
		angle = tmp_angle;
		speed = smooth_speed;
		
		auto message = std_msgs::msg::Float32();
		message.data = voltage;
		publisher_motor->publish(message);
	}
	
	void sim_com(double *input, double *output) override{
		input[0] = angle;
		input[1] = speed;

		voltage = output[0];
	}
	
	void dyn_system(double *input, double *state, double *dstate, double *output) override{
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
		double gsyn = -0.2;
		double dsyn = 0.0;
		double tau_vsyn_inv = 25.0;
		double gspeed = 10.0;
		double dspeed = 7;
		double gpos = 10.0;
		double dpos = 0.1;
		double Kfeed = 2;
		double gfm_mod = -2.0;
		double gsp_mod = 4.0;
		double gsm_mod = -1.0;
		double gup_mod = 1.0;
		double Kfeed_mod = 1.0;
		double gspeed_mod = 20.0;
		double dspeed_mod = 1.0;
		double gpos_mod = 20.0;
		double dpos_mod = 0.15;
		double d_gsm = 0.4;
		double tau_mod_inv = 5.0;
		double tau_feed_inv = 50.0;

		// Input reading
		double pos = input[0];
		double speed = input[1];
		double angle_ref = input[2];
		double Iapp = input[3];
		double Iapp_mod = input[4];
		double gsyn_out = input[5];
		double dsyn_out = input[6];

		// State reading
		double neuron1_s_1 = state[2];
		double neuron1_s_2 = state[3];
		double neuron1_s_3 = state[4];
		double neuron1_s_4 = state[5];
		double feedback_s_1 = state[0];
		double feedback_s_2 = state[1];
		double neuron2_s_1 = state[6];
		double neuron2_s_2 = state[7];
		double neuron2_s_3 = state[8];
		double neuron2_s_4 = state[9];
		double neuron_mod1_s_1 = state[14];
		double neuron_mod1_s_2 = state[15];
		double neuron_mod1_s_3 = state[16];
		double neuron_mod1_s_4 = state[17];
		double synapse1_s_1 = state[10];
		double neuron_mod2_s_1 = state[18];
		double neuron_mod2_s_2 = state[19];
		double neuron_mod2_s_3 = state[20];
		double neuron_mod2_s_4 = state[21];
		double synapse2_s_1 = state[11];
		double neuromod_s_1 = state[22];
		double neuromod_s_2 = state[23];
	
		// Output Computing
		double preprocessing_i_1 = sin(pos);
		double feedback_i_1 = fast_tanh((gpos * preprocessing_i_1));
		double feedback_i_2 = ((fast_tanh((gspeed * (speed + dspeed))) - fast_tanh((gspeed * (speed - dspeed)))) * (0.5));
		double feedback_i_3 = Kfeed * (feedback_i_2 * (((1.0) + feedback_i_1) * (0.5)));
		double feedback_i_4 = Kfeed * (feedback_i_2 * (((1.0) - feedback_i_1) * (0.5)));
		double neuron1_i_1 = (gfm * (fast_tanh((neuron1_s_2 - dfm)) - fast_tanh((V0 - dfm))));
		double neuron1_i_2 = (gsp * (fast_tanh((neuron1_s_3 - dsp)) - fast_tanh((V0 - dsp))));
		double neuron1_i_3 = (neuromod_s_2 * (fast_tanh((neuron1_s_3 - dsm)) - fast_tanh((V0 - dsm))));
		double neuron1_i_4 = (gup * (fast_tanh((neuron1_s_4 - dup)) - fast_tanh((V0 - dup))));
		double neuron2_i_1 = (gfm * (fast_tanh((neuron2_s_2 - dfm)) - fast_tanh((V0 - dfm))));
		double neuron2_i_2 = (gsp * (fast_tanh((neuron2_s_3 - dsp)) - fast_tanh((V0 - dsp))));
		double neuron2_i_3 = (neuromod_s_2 * (fast_tanh((neuron2_s_3 - dsm)) - fast_tanh((V0 - dsm))));
		double neuron2_i_4 = (gup * (fast_tanh((neuron2_s_4 - dup)) - fast_tanh((V0 - dup))));
		double feedback_mod_i_1 = ((fast_tanh((gspeed_mod * (speed + dspeed_mod))) - fast_tanh((gspeed_mod * (speed - dspeed_mod)))) * (0.5));
		// double adder_i_3 = (synapse_out_1_i_1-synapse_out_2_i_1);
		double adder_i_3 = gsyn_out * (fmin(fmax(neuron1_s_1, 0.0), 1.0) - fmin(fmax(neuron2_s_1, 0.0), 1.0));
		double neuron_mod1_i_1 = (gfm_mod * (fast_tanh((neuron_mod1_s_2 - dfm)) - fast_tanh((V0 - dfm))));
		double neuron_mod1_i_2 = (gsp_mod * (fast_tanh((neuron_mod1_s_3 - dsp)) - fast_tanh((V0 - dsp))));
		double neuron_mod1_i_3 = (gsm_mod * (fast_tanh((neuron_mod1_s_3 - dsm)) - fast_tanh((V0 - dsm))));
		double neuron_mod1_i_4 = (gup_mod * (fast_tanh((neuron_mod1_s_4 - dup)) - fast_tanh((V0 - dup))));
		double synapse1_i_1 = (gsyn * ((fast_tanh(((synapse1_s_1 - dsyn) * (2.0))) + (1.0)) * (0.5)));
		double adder_i_1 = (Iapp + (synapse1_i_1 + feedback_i_3));
		double preprocessing_i_2 = fabs((pos - round((pos * M_1_PI * 0.5)) * 2 * M_PI));
		double feedback_mod_i_2 = ((fast_tanh((gpos_mod * ((preprocessing_i_2 - angle_ref) - dpos_mod))) * (0.5)) - (1.0));
		double feedback_mod_i_3 = ((fast_tanh((gpos_mod * ((angle_ref - preprocessing_i_2) - dpos_mod))) * (0.5)) - (1.0));
		double feedback_mod_i_4 = (Kfeed_mod * (feedback_mod_i_1 + feedback_mod_i_2));
		double feedback_mod_i_5 = (Kfeed_mod * (feedback_mod_i_1 + feedback_mod_i_3));
		double adder_i_4 = (Iapp_mod + feedback_mod_i_4);	
		double adder_i_5 = (Iapp_mod + feedback_mod_i_5);
		double neuron_mod2_i_1 = (gfm_mod * (fast_tanh((neuron_mod2_s_2 - dfm)) - fast_tanh((V0 - dfm))));
		double neuron_mod2_i_2 = (gsp_mod * (fast_tanh((neuron_mod2_s_3 - dsp)) - fast_tanh((V0 - dsp))));
		double neuron_mod2_i_3 = (gsm_mod * (fast_tanh((neuron_mod2_s_3 - dsm)) - fast_tanh((V0 - dsm))));
		double neuron_mod2_i_4 = (gup_mod * (fast_tanh((neuron_mod2_s_4 - dup)) - fast_tanh((V0 - dup))));
		double synapse2_i_1 = (gsyn * ((fast_tanh(((synapse2_s_1 - dsyn) * (2.0))) + (1.0)) * (0.5)));
		double adder_i_2 = (Iapp + (synapse2_i_1 + feedback_i_4));
		double neuromod_i_1 = (fmax((0.0), fmin((1.0), neuron_mod1_s_1)) - fmax((0.0), fmin((1.0), neuron_mod2_s_1)));

		// Derivative Computing
		dstate[2] = (tau_V_inv * ((((((V0 + adder_i_2) - neuron1_i_1) - neuron1_i_2) - neuron1_i_3) - neuron1_i_4) - neuron1_s_1));
		dstate[4] = (tau_vs_inv * (neuron1_s_1 - neuron1_s_3));
		dstate[3] = (tau_vf_inv * (neuron1_s_1 - neuron1_s_2));
		dstate[5] = (tau_vu_inv * (neuron1_s_1 - neuron1_s_4));
		dstate[1] = (tau_feed_inv * ((Kfeed * (feedback_i_2 * (((1.0) - feedback_i_1) * (0.5)))) - feedback_s_2));
		dstate[0] = (tau_feed_inv * ((Kfeed * (feedback_i_2 * (((1.0) + feedback_i_1) * (0.5)))) - feedback_s_1));
		dstate[7] = (tau_vf_inv * (neuron2_s_1 - neuron2_s_2));
		dstate[6] = (tau_V_inv * ((((((V0 + adder_i_1) - neuron2_i_1) - neuron2_i_2) - neuron2_i_3) - neuron2_i_4) - neuron2_s_1));
		dstate[8] = (tau_vs_inv * (neuron2_s_1 - neuron2_s_3));
		dstate[9] = (tau_vu_inv * (neuron2_s_1 - neuron2_s_4));
		dstate[17] = (tau_vu_inv * (neuron_mod1_s_1 - neuron_mod1_s_4));
		dstate[14] = (tau_V_inv * ((((((V0 + adder_i_4) - neuron_mod1_i_1) - neuron_mod1_i_2) - neuron_mod1_i_3) - neuron_mod1_i_4) - neuron_mod1_s_1));
		dstate[16] = (tau_vs_inv * (neuron_mod1_s_1 - neuron_mod1_s_3));
		dstate[15] = (tau_vf_inv * (neuron_mod1_s_1 - neuron_mod1_s_2));
		dstate[10] = (tau_vsyn_inv * (neuron1_s_1 - synapse1_s_1));
		dstate[20] = (tau_vs_inv * (neuron_mod2_s_1 - neuron_mod2_s_3));
		dstate[19] = (tau_vf_inv * (neuron_mod2_s_1 - neuron_mod2_s_2));
		dstate[18] = (tau_V_inv * ((((((V0 + adder_i_5) - neuron_mod2_i_1) - neuron_mod2_i_2) - neuron_mod2_i_3) - neuron_mod2_i_4) - neuron_mod2_s_1));
		dstate[21] = (tau_vu_inv * (neuron_mod2_s_1 - neuron_mod2_s_4));
		dstate[11] = (tau_vsyn_inv * (neuron2_s_1 - synapse2_s_1));
		dstate[23] = (tau_mod_inv * (neuromod_s_1 - neuromod_s_2));
		dstate[22] = (d_gsm * neuromod_i_1);

		// Output Setting
		output[0] = adder_i_3;
	}
	
	void getTempTimer(double *timer) override{
		*timer = 100;
	}
	
	void setup_pub_sub() override{
		publisher_motor = this->create_publisher<std_msgs::msg::Float32>("SpinVolt_motor_1",10);
		
		subscription_rotation = this->create_subscription<vex_message::msg::Vexrotationsensor>("out_rotationsensor_2",10,std::bind(&Hco_adapt::rotation_callback,this,_1));
	}

	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_motor;

	rclcpp::Subscription<vex_message::msg::Vexrotationsensor>::SharedPtr subscription_rotation;
	
	vex_message::msg::Vexrotationsensor msg_rotation_sensor;
	
	void rotation_callback(const vex_message::msg::Vexrotationsensor msg){
		this->msg_rotation_sensor = msg;
	}

private:
	double angle=0;
	double speed=0;
	double voltage=0;
};

