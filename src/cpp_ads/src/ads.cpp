#include <iostream>
#include <string>
#include "AdsLib.h"
#include "AdsVariable.h"
//#include "ads_library_pkg/ADS/AdsLib"

#include <thread>
#include <chrono>
#include <functional>
#include <memory>


#include "crane_interface/msg/craneparameters.hpp"
#include "crane_interface/msg/cranecontrol.hpp"
#include "rclcpp/rclcpp.hpp"
//#include "ads_library_pkg/ADS/AdsLib"

namespace crane 
{
   namespace config 
   {
      using namespace std::chrono_literals;
      const auto dt = 500ms;
   }
}
//
//namespace craneads {
//    class AdsHandler;
//}
//


namespace craneads 
{

    struct AdsVariables
    {
        AdsVariables() = delete;

        explicit AdsVariables(AdsDevice& route)
            : activateMotion{route, "G_ControllerGreenCrane.bStartAutoMode"}
            , positionReference{route, "G_ControllerGreenCrane.fPosSP"}
            , positionMeasurement{route, "G_CylinderState.thetaBoom"}
            , ppBar{route, "G_CylinderState.ppbar"}
            , prBar{route, "G_CylinderState.prbar"}
        {
            // Do nothing.
        }

        AdsVariable<bool> activateMotion;
        AdsVariable<double> positionReference;
        AdsVariable<double> positionMeasurement;
        AdsVariable<double> ppBar;
        AdsVariable<double> prBar;
    };


    
    class AdsHandler
    {
    public:
        explicit AdsHandler(const AmsNetId remoteNetId, const std::string remoteIpV4)
            : remoteNetId_(remoteNetId)
            , remoteIpV4_(remoteIpV4)
            , route_{remoteIpV4_, remoteNetId_, AMSPORT_R0_PLC_TC3}
            , ads_(route_) { }

        AdsHandler() : AdsHandler({127, 0, 0, 1,  1, 1}, "127.0.0.1") { }


        void startStopMotion(bool bAutoStart)
        {
            ads_.activateMotion = bAutoStart;
        }

        bool getActivateMotion()
        {
            return ads_.activateMotion;
        }

        void deactivateMotion()
        {
            ads_.activateMotion = false;
        }

        void setPositionReference(double value)
        {
            ads_.positionReference = value;
        }

        double getPositionMeasurement()
        {
            return ads_.positionMeasurement;
        }

        double getppBar()
        {
            return ads_.ppBar;
        }

        double getprBar()
        {
            return ads_.prBar;
        }


        void printState()
        {
            const auto state = route_.GetState();
            std::cout << "ADS state: "
                      << std::dec << static_cast<uint16_t>(state.ads)
                      << " devState: "
                      << std::dec << static_cast<uint16_t>(state.device);
        }

        ~AdsHandler() { }

    private:
        const AmsNetId remoteNetId_;
        const std::string remoteIpV4_;
        AdsDevice route_;
        AdsVariables ads_;
    };

    class JointStatePublisher : public rclcpp::Node
{
    public:
        JointStatePublisher()
        : Node("ads_node")
        , step_(0)
        , adsHandler_{{192,168,0,15,1,1}, "192.168.0.15"}
        , thetaBoomValue_(0.0)
        
        //, AdsHandler testads;
        {
            using namespace std::chrono_literals;
            this->declare_parameter("bstartstop", false);
            this->declare_parameter("posSet", 0.0);
            
            //posSetdenne = this->get_parameter("posSet").as_double();
            //posSetdenne = this->get_parameter("posSet");

            publisher_ = this->create_publisher<crane_interface::msg::Craneparameters>("thetaBoomValue", 5);
            timer_ = this->create_wall_timer(crane::config::dt, std::bind(&JointStatePublisher::update_theta_Boom, this));

            //publisherAutoMode_ = this->create_publisher<crane_interface::msg::Cranecontrol>("autoModeParam", 5);
            //timerAutoMode_  = this->create_wall_timer(crane::config::dt, std::bind(&JointStatePublisher::update_auto_param, this));
            
        }
    private:
        void update_theta_Boom()
        {
            auto message = crane_interface::msg::Craneparameters();
            message.theta_boom = -adsHandler_.getPositionMeasurement();
            prBar1 = adsHandler_.getprBar();
            ppBar2 = adsHandler_.getppBar();
            RCLCPP_INFO(this->get_logger(), "\n Theta boom: '%f'\n Pressure rod side: '%f'\n Pressure piston side: '%f'", message.theta_boom, ppBar2, prBar1);
            publisher_->publish(message);  
            adsHandler_.setPositionReference(this->get_parameter("posSet").as_double());
            adsHandler_.startStopMotion(this->get_parameter("bstartstop").as_bool());     
        }
        /*
        void update_auto_param()
        {
            auto message = crane_interface::msg::Cranecontrol();
            message.start = adsHandler_.getActivateMotion();
            //RCLCPP_INFO(this->get_logger(), "Theta boom: '%f'", message.theta_boom);
            publisherAutoMode_->publish(message);       
        }
        */
        rclcpp::TimerBase::SharedPtr timer_;
        //rclcpp::TimerBase::SharedPtr timerAutoMode_;
        rclcpp::Publisher<crane_interface::msg::Craneparameters>::SharedPtr publisher_;
        //rclcpp::Publisher<crane_interface::msg::Cranecontrol>::SharedPtr publisherAutoMode_;
        size_t step_;
        craneads::AdsHandler adsHandler_;
        
        //craneads::AdsHandler blablabal(remoteNetId, remoteIpV4);
        double thetaBoomValue_;
        double posSetdenne;
        double prBar1;
        double ppBar2;

};
}


int main(int argc, char * argv[])
{
    const AmsNetId remoteNetId {192,168,0,15,1,1};
    const std::string remoteIpV4 = "192.168.0.15";
    //craneads::AdsHandler blabla(remoteNetId, remoteIpV4);
    rclcpp::init(argc, argv);

    //rclcpp::spin(jointStatePublisher);
    rclcpp::spin(std::make_shared<craneads::JointStatePublisher>());
    rclcpp::shutdown();

    return 0;
}



/*
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStatePublisher>());
    rclcpp::shutdown();

    return 0;
}
*/


/**
#include <iostream>
#include <string>
#include "AdsLib.h"
#include "AdsVariable.h"

#include <thread>
#include <chrono>

#include "rclcpp/rcl.hpp"
#include "std_msgs/Float32.msg"

using namespace std::chrono_literals;

namespace craneads {

    struct AdsVariables
    {
        AdsVariables() = delete;

        explicit AdsVariables(AdsDevice& route)
            : activateMotion{route, "MAIN.bActivateMotion"}
            , velocityReference{route, "MAIN.fVelRef"}
            , positionReference{route, "MAIN.fPosRef"}
            , positionMeasurement{route, "G_test.fPosMeas"}
        {
            // Do nothing.
        }

        AdsVariable<bool> activateMotion;
        AdsVariable<double> velocityReference;
        AdsVariable<double> positionReference;
        AdsVariable<double> positionMeasurement;
    };

    class AdsHandler : public rclcpp::Node
    {
    public:
        AdsHandler()
        : Node("ads_handler")
        , cout_(0)
        {
            publisher_ this->create_publisher<std_msgs::msg::Float32>("thetaboom", 1);
            timer_ = this->create_wall_timer(
            500ms, std::bind(&AdsHandler::timer_callback, this));
        }
        

    private:
        const AmsNetId remoteNetId_;
        const std::string remoteIpV4_;
        AdsDevice route_;
        AdsVariables ads_;

        explicit AdsHandler(const AmsNetId remoteNetId, const std::string remoteIpV4)
            : remoteNetId_(remoteNetId)
            , remoteIpV4_(remoteIpV4)
            , route_{remoteIpV4_, remoteNetId_, AMSPORT_R0_PLC_TC3}
            , ads_(route_) { }

        AdsHandler() : AdsHandler({127, 0, 0, 1,  1, 1}, "127.0.0.1") { }

        void timer_callback()
        {
            auto message = std_msgs
        }

        void activateMotion()
        {
            ads_.activateMotion = true;
        }

        void deactivateMotion()
        {
            ads_.activateMotion = false;
        }

        void setVelocityReference(double value)
        {
            ads_.velocityReference = value;
        }

        void setPositionReference(double value)
        {
            ads_.velocityReference = value;
        }

        double getPositionMeasurement()
        {
            return ads_.positionMeasurement;
        }


        void printState()
        {
            const auto state = route_.GetState();
            std::cout << "ADS state: "
                      << std::dec << static_cast<uint16_t>(state.ads)
                      << " devState: "
                      << std::dec << static_cast<uint16_t>(state.device);
        }

        ~AdsHandler() { }
    };

}



int main(int  argc , char** argv)
{

  std::cout << "Example ROS2 ADS node starting up.." << std::endl;

  // Real lab PLC IP.
  const AmsNetId remoteNetId {192,168,0,15,1,1};
  const std::string remoteIpV4 = "192.168.0.15";

  // Connecting to testbed computer.
  //const AmsNetId remoteNetId { 192, 168, 56, 1, 1, 1 };
  //const std::string remoteIpV4 = "192.168.56.1";

  std::cout << "  Create AdsHandler.. ";
  craneads::AdsHandler adsHandler(remoteNetId, remoteIpV4);
  std::cout << "  OK" << std::endl;

  adsHandler.deactivateMotion();

  adsHandler.printState();

  adsHandler.setVelocityReference(3.2);
  std::this_thread::sleep_for (std::chrono::seconds(5));
  adsHandler.setPositionReference(3.14);

  adsHandler.activateMotion();
  for(uint8_t n = 0; ; ++n)
  {
    adsHandler.setPositionReference(static_cast<double>(n) / 255.0);
    std::cout << "Position measurement from ADS: " << adsHandler.getPositionMeasurement() << std::endl;
    std::this_thread::sleep_for (std::chrono::seconds(2));
  }


  return 0;
}
*/


