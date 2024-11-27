
#include "simulator.hpp" 
#include "model.hpp"

#include <chrono>

using std::placeholders::_1;
using namespace std::chrono_literals;

#include <iostream>

class TestModel : public Model
{
public:
    TestModel():Model() {
        this->count = 0;
    }

    void setup(int &nb_state, int &nb_input, int &nb_output) override
    {
        nb_state = 1;
        nb_input = 1;
        nb_output = 1;
    }

    void initial_value_setup(double *state, double *input) override
    {
        state[0] = 0;
        input[0] = 0;
    }

    void get_temp_timer(double &timer, int &sim_nb_steps) override
    {
        timer = 100;
        sim_nb_steps = 1;
    }

    void setup_pub_sub(rclcpp::Node *simulator) override
    {
        subscription_ = simulator->create_subscription<std_msgs::msg::Empty>("cool_topic", 1, std::bind(&TestModel::cool_callback, this, _1));
        publisher_ = simulator->create_publisher<std_msgs::msg::Empty>("cool_topic", 10);
    }

    void sim_com([[maybe_unused]] double *input, [[maybe_unused]] double *output) override
    {
        publisher_->publish(std_msgs::msg::Empty());
        //std::cout << "sim_com" << std::endl;
    }

    void dyn_system([[maybe_unused]] double *input, [[maybe_unused]] double *state, [[maybe_unused]] double *dstate, [[maybe_unused]] double *output) override
    {
        //std::cout << "dyn_system" << std::endl;
    }

    void stop_model() override
    {
        // Let empty, try to only use the destructor
    }

    ~TestModel()
    {
        std::cout << "testModel Destroyed" << std::endl;
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
    Simulator::launch(argc, argv, std::make_shared<TestModel>());
    std::cout << "ended" << std::endl;
}