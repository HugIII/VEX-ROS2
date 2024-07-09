#include "rclcpp/rclcpp.hpp"

#include "hco.cpp"

#include <string>
#include <chrono>
#include <thread>
#include <iostream>
#include <cstdint>

#include "vex_message/msg/vexmsg.hpp"

#include "template_manager.cpp"

using namespace std::chrono;
using namespace std::chrono_literals;
using std::placeholders::_1;

class Model{
public:
	Template t;

	Model(){
		t = getTemplate();
		
		t->setup(nb_state,nb_input,nb_output);
		
		input = (double*)malloc(*nb_input * sizeof(double));
		state = (double*)malloc(*nb_state * sizeof(double));
		state2 = (double*)malloc(*nb_state * sizeof(double));
		dstate = (double*)malloc(*nb_state * sizeof(double));
		dstate2 = (double*)malloc(*nb_state * sizeof(double));
		output = (double*)malloc(*nb_output * sizeof(double));
		
		t->initial_value_setup(&state,&input);
		t->getTempTimer(timer);
		
		t->create_subscription<vex_message::msg::Vexmsg>("out_rotationsensor_2",10,std::bind(&Model::topic_callback, this, _1));
		
		auto time = *timer * 1us;
		loop_rate = t->create_wall_timer(time,std::bind(&Model::sim_function,this));
	}
	
private:
	rclcpp::TimerBase::SharedPtr loop_rate;

	int* nb_state;
	int* nb_input;
	int* nb_output;
	
	double *input, *state, *state2, *dstate, *dstate2, *output;
	
	double* timer;
	
	void topic_callback(const vex_message::msg::Vexmsg msg){
		t->system_update();
	}
	
	void sim_function(){
		std::cout << "timer entry " << std::endl;
  		
  		t->sim_com(input,output);
  		t->dyn_system(input,state,dstate,output);
			
		auto step = *timer * 0.000001;	
			
    		for (int i = 0 ; i < *nb_state ; i++) {
      			state2[i] = state[i] + step*dstate[i];
    		}


    		t->dyn_system(input,state2,dstate2,output);

    		for (int i = 0 ; i < *nb_state ; i++) {
      			state[i] = state[i] + step*(dstate[i] + dstate2[i])/2;
    		}
      }

};

int main(int argc, char* argv[]){
	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<Model>());
	rclcpp::shutdown();
	
	Model();
	
	return 0;
}
