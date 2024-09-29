#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

class Template: public rclcpp::Node{
public:
	Template():Node("model"){
	
	}

	virtual void setup(int *nb_state, int *nb_input, int *nb_output)=0;
	virtual void initial_value_setup(double **state, double **input)=0;
	virtual void system_update()=0;
	virtual void sim_com(double *input, double *output)=0;
	virtual void dyn_system(double *input, double *state, double *dstate, double *output)=0;
	virtual void getTempTimer(double *timer)=0;
	virtual void setup_pub_sub()=0;
private:
	
};
