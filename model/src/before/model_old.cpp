#include "rclcpp/rclcpp.hpp"

#include "hco.cpp"

#include <string>
#include <chrono>
#include <thread>
#include <iostream>
#include <cstdint>

#include "std_msgs/msg/float32.hpp"
#include "vex_message/msg/vexrotationsensor.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;
using std::placeholders::_1;

class Model: public rclcpp::Node{
public:
	Model():Node("model"){
		
		publisher_ = this->create_publisher<std_msgs::msg::Float32>("SpinVolt_motor_1",10);
		
		subscription_ = this->create_subscription<vex_message::msg::Vexrotationsensor>("out_rotationsensor_2",10,std::bind(&Model::topic_callback, this, _1));
		
		hco_setup(&this->nb_state,&this->nb_input,&this->nb_output,&this->input,&this->state,&this->state2,&this->dstate,&this->dstate2,&this->output);
		
		loop_rate = this->create_wall_timer(100us,std::bind(&Model::sim_function,this));
	}	
	
	
private:
	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
	rclcpp::Subscription<vex_message::msg::Vexrotationsensor>::SharedPtr subscription_;	
	rclcpp::TimerBase::SharedPtr loop_rate;

	double step = SIM_MICRO_STEP * 0.000001;

  	double *input, *state, *state2, *dstate, *dstate2, *output;
  	int nb_state, nb_input, nb_output;
  	
  	uint64_t start = timeSinceEpochMillisec();
  	uint64_t end;

	void topic_callback(const vex_message::msg::Vexrotationsensor msg){
		float motor_voltage = 0;
		float angle = msg.angle;
		float velocity = msg.velocity;
		
		std::cout << "rotation angle : " << angle << std::endl;
		std::cout << "rotation velocity: " << velocity << std::endl;
		
		motor_voltage = hco_system_update(angle,velocity);
		
		auto message = std_msgs::msg::Float32();
		message.data = motor_voltage;
		std::cout << "motor voltage " <<motor_voltage << std::endl;
		publisher_->publish(message);
	}
	
	void sim_function(){
		std::cout << "timer entry " << std::endl;
  		this->end = this->start + SIM_MICRO_STEP;
  		
  		hco_sim_com(this->input,this->output);
  		
  		dyn_system_hco(this->input,this->state, this->dstate, this->output);

    		for (int i = 0 ; i < this->nb_state ; i++) {
      			this->state2[i] = this->state[i] + this->step*this->dstate[i];
    		}


    		dyn_system_hco(this->input,this->state2, this->dstate2, this->output);

    		for (int i = 0 ; i < this->nb_state ; i++) {
      			this->state[i] = this->state[i] + this->step*(this->dstate[i] + this->dstate2[i])/2;
    		}
      }

};

int main(int argc, char* argv[]){
	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<Model>());
	rclcpp::shutdown();
	return 0;
}
