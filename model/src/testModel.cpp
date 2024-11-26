
#include "simulator.hpp" 
#include "model.hpp"

#include <chrono>

using std::placeholders::_1;
using namespace std::chrono_literals;

#include <iostream>

class TestModel : public Model
{
public:
    TestModel():Model() {}

    void setup(int *nb_state, int *nb_input, int *nb_output) override
    {
        *nb_state = 1;
        *nb_input = 1;
        *nb_output = 1;
    }

    void initial_value_setup(double **state, double **input) override
    {
        **state = 0;
        **input = 0;
    }

    void system_update() override
    {
        std::cout << "system_update" << std::endl;
    }

    void sim_com(double *input, double *output) override
    {
        std::cout << "sim_com" << std::endl;
    }

    void dyn_system(double *input, double *state, double *dstate, double *output) override
    {
        std::cout << "dyn_system" << std::endl;
    }

    void get_temp_timer(double *timer, int *sim_nb_steps) override
    {
        *timer = 10000000;
        *sim_nb_steps=100;
    }

    void cool_callback(const std_msgs::msg::Empty msg)
    {
        std::cout << "cool_callback" << std::endl;
    }

    void setup_pub_sub(rclcpp::Node *simulator) override
    {
        subscription_ = simulator->create_subscription<std_msgs::msg::Empty>("cool_topic", 10, std::bind(&TestModel::cool_callback, this, _1));
    }

    void stop_model() override
    {
        std::cout << "delete_pub_sub" << std::endl;
    }

private:
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscription_;
};


int main(int argc, char *argv[])
{
    Simulator::launch(argc, argv, std::make_shared<TestModel>());
    std::cout << "ended" << std::endl;
}