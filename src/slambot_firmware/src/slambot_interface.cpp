#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "slambot_firmware/diffdrive_stm.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

using std::placeholders::_1;

namespace slambot_firmware
{
    slambotInterface::slambotInterface()
    {
    }

    slambotInterface::~slambotInterface()
    {
        if (stm_.IsOpen())
        {
            RCLCPP_INFO(rclcpp::get_logger("slambotInterface"), "PORT IS OPEN");
        }
    }

    CallbackReturn slambotInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
    {
        CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
        if (result != CallbackReturn::SUCCESS)
        {
            return result;
        }

        try
        {
            port_ = info_.hardware_parameters.at("port");
        }
        catch (const std::out_of_range &e)
        {
            RCLCPP_FATAL(rclcpp::get_logger("slambotInterface"), "No Serial Port provided! Aborting");
            return CallbackReturn::FAILURE;
        }

        velocity_commands_.reserve(info_.joints.size());
        position_states_.reserve(info_.joints.size());
        velocity_states_.reserve(info_.joints.size());
        last_run_ = rclcpp::Clock().now();

        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> slambotInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        // Provide only a position Interafce
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> slambotInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        // Provide only a velocity Interafce
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
        }

        return command_interfaces;
    }

    CallbackReturn slambotInterface::on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("slambotInterface"), "Starting robot hardware ...");

        // Reset commands and states
        velocity_commands_ = {0.0, 0.0};
        position_states_ = {0.0, 0.0};
        velocity_states_ = {0.0, 0.0};

        try
        {
            stm_.Open(port_);
            stm_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        }
        catch (...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("slambotInterface"),
                                "Something went wrong while interacting with port " << port_);
            return CallbackReturn::FAILURE;
        }

        RCLCPP_INFO(rclcpp::get_logger("slambotInterface"),
                    "Hardware started, ready to take commands");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn slambotInterface::on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("slambotInterface"), "Stopping robot hardware ...");

        if (stm_.IsOpen())
        {
            try
            {
                stm_.Close();
            }
            catch (...)
            {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("slambotInterface"),
                                    "Something went wrong while closing connection with port " << port_);
            }
        }

        RCLCPP_INFO(rclcpp::get_logger("slambotInterface"), "Hardware stopped");
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type slambotInterface::read(const rclcpp::Time &,
                                                           const rclcpp::Duration &)
    {
        // Interpret the string
        if (stm_.IsDataAvailable())
        {
            auto dt = (rclcpp::Clock().now() - last_run_).seconds();
            std::string message;
            stm_.ReadLine(message);
            std::stringstream ss(message);
            std::string res;
            int multiplier = 1;
            while (std::getline(ss, res, ','))
            {
                multiplier = res.at(1) == 'p' ? 1 : -1;

                if (res.at(0) == 'r')
                {
                    velocity_states_.at(0) = (multiplier * std::stod(res.substr(2, res.size())))/1000.00;
                    position_states_.at(0) += velocity_states_.at(0) * dt;
                }
                else if (res.at(0) == 'l')
                {
                    velocity_states_.at(1) = (multiplier * std::stod(res.substr(2, res.size())))/1000.00;
                    position_states_.at(1) += velocity_states_.at(1) * dt;
                }
            }
            last_run_ = rclcpp::Clock().now();
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type slambotInterface::write(const rclcpp::Time &,
                                                            const rclcpp::Duration &)
    {
        // Implement communication protocol with the STM32
        std::stringstream message_stream;
        char right_wheel_sign = velocity_commands_.at(0) >= 0 ? '1' : '2';
        char left_wheel_sign = velocity_commands_.at(1) >= 0 ? '1' : '2';
        std::string compensate_zeros_right = "";
        std::string compensate_zeros_left = "";

        if (std::abs(velocity_commands_.at(0)) < 10)
        {
            compensate_zeros_right = "0";
        }
        else
        {
            compensate_zeros_right = "";
        }
        if (std::abs(velocity_commands_.at(1)) < 10)
        {
            compensate_zeros_left = "0";
        }
        else
        {
            compensate_zeros_left = "";
        }

        // Cast the velocities to integers
        int right_wheel_velocity = static_cast<int>(std::abs(velocity_commands_.at(0)));
        int left_wheel_velocity = static_cast<int>(std::abs(velocity_commands_.at(1)));

        message_stream << right_wheel_sign << compensate_zeros_right << right_wheel_velocity << left_wheel_sign << compensate_zeros_left << left_wheel_velocity;

        // // Implement communication protocol with the Arduino
        // std::stringstream message_stream;
        // char right_wheel_sign = velocity_commands_.at(0) >= 0 ? 'p' : 'n';
        // char left_wheel_sign = velocity_commands_.at(1) >= 0 ? 'n' : 'n';
        // std::string compensate_zeros_right = "";
        // std::string compensate_zeros_left = "";
        // if (std::abs(velocity_commands_.at(0)) < 10.0)
        // {
        //     compensate_zeros_right = "0";
        // }
        // else
        // {
        //     compensate_zeros_right = "";
        // }
        // if (std::abs(velocity_commands_.at(1)) < 10.0)
        // {
        //     compensate_zeros_left = "0";
        // }
        // else
        // {
        //     compensate_zeros_left = "";
        // }

        // message_stream << std::fixed << std::setprecision(2) << "r" << right_wheel_sign << compensate_zeros_right << std::abs(velocity_commands_.at(0)) << ",l" << left_wheel_sign << compensate_zeros_left << std::abs(velocity_commands_.at(1)) << ",";

        try
        {
            stm_.Write(message_stream.str());
        }
        catch (...)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("slambotInterface"),
                                "Something went wrong while sending the message "
                                    << message_stream.str() << " to the port " << port_);
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }

} // name slambot_firmware

PLUGINLIB_EXPORT_CLASS(slambot_firmware::slambotInterface, hardware_interface::SystemInterface)

// class Serial_Class : public rclcpp::Node
// {
// public:
//     Serial_Class() : Node("SerialNode1")
//     {
//         declare_parameter<std::string>("port", "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0");

//         port_ = get_parameter("port").as_string();

//         stm_.Open(port_);

//         if (stm_.IsOpen())
//         {
//             RCLCPP_INFO(this->get_logger(), "PORT IS OPEN");
//         }

//         subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
//             "/cmd_vel", 1, std::bind(&Serial_Class::send_msg_callback, this, std::placeholders::_1));

//         stm_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
//         wheel_odom_publisher_ = this->create_publisher<std_msgs::msg::String>("/distance_finder", 1);
//         timer_ = this->create_wall_timer(std::chrono::microseconds(500), std::bind(&Serial_Class::msgCallback, this));

//         // subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
//         //     "/cmd_vel", 1, std::bind(&Serial_Class::msgCallback, this, std::placeholders::_1));

//         // msgCallback();
//     }

//     ~Serial_Class()
//     {
//         stm_.Close();
//     }

// private:
//     rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
//     std::string port_;
//     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr wheel_odom_publisher_;
//     rclcpp::TimerBase::SharedPtr timer_;
//     LibSerial::SerialPort stm_;

//     void send_msg_callback(const geometry_msgs::msg::Twist::SharedPtr data)
//     {
//         // RCLCPP_INFO_STREAM(this->get_logger(), "Sending Message ");
//         RCLCPP_INFO(this->get_logger(), "Callback triggered.");
//         RCLCPP_INFO(this->get_logger(), "Linear:[%f,%f,%f]",
//                     data->linear.x, data->linear.y, data->linear.z);

//         if ((data->linear.x) > 0.0) // Moving Forward
//         {
//             /* 1st 3 ->  speed for Motor A
//                2nd 3 ->  speed for Motor B
//                7th bit  -> Motor A direction
//                8th bit  -> Motor B direction
//                */
//             int speed = 135135;
//             // std::string vel = "120120";
//             std::string vel = std::to_string(speed);
//             std::string data_to_send = vel;
//             RCLCPP_INFO(this->get_logger(), "Speed Sent to STM: %s Moving Forward", data_to_send.c_str());
//             stm_.Write(data_to_send.c_str());
//         }

//         else if ((data->linear.x) < 0.0) // Moving Backward
//         {
//             int speed = 235235;
//             // std::string vel = "12012011";
//             std::string vel = std::to_string(speed);
//             std::string data_to_send = vel;
//             RCLCPP_INFO(this->get_logger(), "Speed Sent to STM: %s Moving Backward ", data_to_send.c_str());
//             stm_.Write(data_to_send.c_str());
//         }
//         else if ((data->angular.z) < 0.0) // Moving to Right
//         {
//             int speed = 130115;
//             // std::string vel = "12012001";
//             std::string vel = std::to_string(speed);
//             std::string data_to_send = vel;
//             RCLCPP_INFO(this->get_logger(), "Speed Sent to STM: %s Moving Right ", data_to_send.c_str());
//             stm_.Write(data_to_send.c_str());
//         }
//         else if ((data->angular.z) > 0.0) // Moving to Left
//         {
//             int speed = 115130;
//             // std::string vel = "12012010";
//             std::string vel = std::to_string(speed);
//             std::string data_to_send = vel;
//             RCLCPP_INFO(this->get_logger(), "Speed Sent to STM: %s Moving Left", data_to_send.c_str());
//             stm_.Write(data_to_send.c_str());
//         }

//         else if ((data->linear.x) == 0.0 && (data->angular.z) == 0.0)
//         {
//             // int speed = 000000;
//             std::string vel = "000000";
//             std::string data_to_send = vel;
//             RCLCPP_INFO(this->get_logger(), "Speed Sent to STM: %s", data_to_send.c_str());
//             stm_.Write(data_to_send);
//         }
//     }

//     void publish_Odom(const std_msgs::msg::String Data_)
//     {
//         wheel_odom_publisher_->publish(Data_);
//     }

//     void msgCallback()
//     {

//         auto message = std_msgs::msg::String();
//         if (rclcpp::ok() && stm_.IsDataAvailable())
//         {
//             stm_.ReadLine(message.data);
//             // RCLCPP_INFO_STREAM(this->get_logger(), "MSG received from Stm:" << message.data);
//             publish_Odom(message);
//         }
//     }
// };

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<Serial_Class>());
//     rclcpp::shutdown();
//     return 0;
// }