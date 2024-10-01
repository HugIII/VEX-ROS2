//####################################################
//#                                                  #
//#Autor: BLAYES Hugo                                #
//#Created: 8/7                                      #
//#Version: 1.0.0                                    #
//#Description: Model manager for neuromorphic model #
//#                                                  #
//####################################################

#include "rclcpp/rclcpp.hpp"

#include "hco.cpp"

#include <string>
#include <chrono>
#include <thread>
#include <iostream>
#include <cstdint>

#include "vex_message/msg/vexmsg.hpp"

//change the include file to the model we want to implement
#include "hco_process.cpp"

using namespace std::chrono;
using namespace std::chrono_literals;
using std::placeholders::_1;

//change to the model we want to use
class Model: public Hco_process{
public:
	//change to the model we want to use
	Model():Hco_process(){
		nb_state = (int*)malloc(sizeof(int));
		nb_input = (int*)malloc(sizeof(int));
		nb_output = (int*)malloc(sizeof(int));
		timer = (double*)malloc(sizeof(double));

		this->setup(nb_state,nb_input,nb_output);
		
		input = (double*)malloc(*nb_input * sizeof(double));
		state = (double*)malloc(*nb_state * sizeof(double));
		state2 = (double*)malloc(*nb_state * sizeof(double));
		dstate = (double*)malloc(*nb_state * sizeof(double));
		dstate2 = (double*)malloc(*nb_state * sizeof(double));
		output = (double*)malloc(*nb_output * sizeof(double));
		
		
		this->initial_value_setup(&state,&input);
		
		//std::cout << "error before"  << std::endl;
		
		this->getTempTimer(timer);
		
		subscription_ = this->create_subscription<vex_message::msg::Vexmsg>("out_serial",10,std::bind(&Model::topic_callback, this, _1));
		
		auto time = *timer * 1us;
		loop_rate = this->create_wall_timer(time,std::bind(&Model::sim_function,this));
	}
	
	~Model(){
		free(input);
		free(state);
		free(state2);
		free(dstate);
		free(dstate2);
		free(output);
	}
	
private:
	rclcpp::TimerBase::SharedPtr loop_rate;

	rclcpp::Subscription<vex_message::msg::Vexmsg>::SharedPtr subscription_;

	int* nb_state;
	int* nb_input;
	int* nb_output;
	
	double *input, *state, *state2, *dstate, *dstate2, *output;
	
	double* timer;

	void topic_callback(const vex_message::msg::Vexmsg msg){
		this->system_update();
	}
	
	void sim_function(){
		//std::cout << "timer entry " << std::endl;
  		
  		this->sim_com(input,output);
  		this->dyn_system(input,state,dstate,output);
			
		auto step = *timer * 0.000001;	
			
    		for (int i = 0 ; i < *nb_state ; i++) {
      			state2[i] = state[i] + step*dstate[i];
    		}


    		this->dyn_system(input,state2,dstate2,output);

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
