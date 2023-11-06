#include <rclcpp/rclcpp.hpp>
#include <can_plugins2/msg/frame.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <can_utils.hpp>
#include <auto_move/auto_move4.hpp>

#include <cmath>
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;

class AutoMove : public rclcpp::Node
{
    private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void can_callback(const can_plugins2::msg::Frame::SharedPtr msg);
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<can_plugins2::msg::Frame>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriber_;
    rclcpp::Subscription<can_plugins2::msg::Frame>::SharedPtr can_subscriber_;
    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_[7];
    size_t count_;
    ShirasuLegID shirasuID_;//シラスのID
    Location location_;
    Location initLocation_;
    Velocity velocity_;
    Location target_;
    uint8_t automode = 0;
    public:
    AutoMove(/* args */);
    void timer_callback();
    void shirasuValuePublish(float upperRight,float upperLeft,float lowerLeft,float lowerRight);
    void shirasuModePublish(uint8_t upperRight,uint8_t upperLeft,uint8_t lowerLeft,uint8_t lowerRight);
    void reset();
    uint8_t count = 0;
    uint16_t autocount = 0;
    float maxSpeed = 0.0f;//厳密にはちょっと違う。
};

AutoMove::AutoMove(/* args */) : Node("auto_move4_node"), count_(0)
{       
    publisher_ = this->create_publisher<can_plugins2::msg::Frame>("can_tx", 10);
    subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&AutoMove::joy_callback, this, _1));
    can_subscriber_ = this->create_subscription<can_plugins2::msg::Frame>("can_rx", 10, std::bind(&AutoMove::can_callback, this, _1));
    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    timer_ = this->create_wall_timer(1ms, std::bind(&AutoMove::timer_callback, this));
    this->declare_parameter("velButton", 6);
    this->declare_parameter("disButton", 1);
    this->declare_parameter("maxSpeed", 6.28f);
    this->declare_parameter("upperRight", 0x160);
    this->declare_parameter("upperLeft", 0x16c);
    this->declare_parameter("lowerLeft", 0x164);
    this->declare_parameter("lowerRight", 0x168);
    this->declare_parameter("syoukouID", 0x154);
    this->declare_parameter("targetX", 20.0f);
    this->declare_parameter("targetY", 20.0f);
    auto param_callback = [this](const rclcpp::Parameter & p) {
        
        switch(p.get_type())
        {
        case rclcpp::ParameterType::PARAMETER_DOUBLE:
            RCLCPP_INFO(
            this->get_logger(), "param callback: Received an update to parameter \"%s\" of type %s: \"%f\"",
            p.get_name().c_str(),
            p.get_type_name().c_str(),
            p.as_double());
            if(p.get_name() == "maxSpeed") maxSpeed = p.as_double();
            if(p.get_name() == "targetX") target_.x = p.as_double();
            if(p.get_name() == "targetY") target_.y = p.as_double();
            break;
        case rclcpp::ParameterType::PARAMETER_INTEGER:
            RCLCPP_INFO(
            this->get_logger(), "param callback: Received an update to parameter \"%s\" of type %s: \"%ld\"",
            p.get_name().c_str(),
            p.get_type_name().c_str(),
            p.as_int());
            if(p.get_name() == "upperRight") shirasuID_.upperRightID = p.as_int();
            if(p.get_name() == "upperLeft") shirasuID_.upperLeftID = p.as_int();
            if(p.get_name() == "lowerLeft") shirasuID_.lowerLeftID = p.as_int();
            if(p.get_name() == "lowerRight") shirasuID_.lowerRightID = p.as_int();
            if(p.get_name() == "syoukouID") shirasuID_.syoukouID = p.as_int();
            break;
        
        default:
            break;
        }
        
    };

    cb_handle_[0] = param_subscriber_->add_parameter_callback("maxSpeed", param_callback);
    cb_handle_[1] = param_subscriber_->add_parameter_callback("upperRight", param_callback);
    cb_handle_[2] = param_subscriber_->add_parameter_callback("upperLeft", param_callback);
    cb_handle_[3] = param_subscriber_->add_parameter_callback("lowerLeft", param_callback);
    cb_handle_[4] = param_subscriber_->add_parameter_callback("lowerRight", param_callback);
    cb_handle_[5] = param_subscriber_->add_parameter_callback("targetX", param_callback);
    cb_handle_[6] = param_subscriber_->add_parameter_callback("targetY", param_callback);
    timer_ = this->create_wall_timer(1ms, std::bind(&AutoMove::timer_callback, this));
    timer_callback();
}

void AutoMove::timer_callback()
{
    location_.x = -(-velocity_.upperRight - velocity_.upperLeft + velocity_.lowerLeft + velocity_.lowerRight);
    location_.y = -(velocity_.upperRight - velocity_.upperLeft + velocity_.lowerLeft - velocity_.lowerRight);

    RCLCPP_INFO(this->get_logger(), "loc x: %f, y: %f", location_.x, location_.y);
}

void AutoMove::can_callback(const can_plugins2::msg::Frame::SharedPtr msg){
    uint8_t data[4] = {msg->data[3],msg->data[2],msg->data[1],msg->data[0]};
    if((this->get_parameter("upperRight").as_int()+2) == msg->id){
        std::memcpy(&velocity_.upperRight,&data[0],sizeof(float));
        //RCLCPP_INFO(this->get_logger(),"%f",velocity_.upperRight);
    }else if((this->get_parameter("upperLeft").as_int()+2) == msg->id){
        std::memcpy(&velocity_.upperLeft,&data[0],sizeof(float));
    }else if((this->get_parameter("lowerLeft").as_int()+2) == msg->id){
        std::memcpy(&velocity_.lowerLeft,&data[0],sizeof(float));
    }else if((this->get_parameter("lowerRight").as_int()+2) == msg->id){
        std::memcpy(&velocity_.lowerRight,&data[0],sizeof(float));
    }
}

void AutoMove::shirasuValuePublish(float upperRight,float upperLeft,float lowerLeft,float lowerRight){
  //RCLCPP_INFO(this->get_logger(), "-upperRight: %f", -upperRight);
  publisher_->publish(shirasu_frame(shirasuID_.upperRightID+1, -upperRight));
  publisher_->publish(shirasu_frame(shirasuID_.upperLeftID+1, -upperLeft));
  publisher_->publish(shirasu_frame(shirasuID_.lowerLeftID+1, -lowerLeft));
  publisher_->publish(shirasu_frame(shirasuID_.lowerRightID+1, -lowerRight));
  //100右上、110左上、120左下、130右下
}

void AutoMove::shirasuModePublish(uint8_t upperRight,uint8_t upperLeft,uint8_t lowerLeft,uint8_t lowerRight){
  publisher_->publish(get_frame(shirasuID_.upperRightID, static_cast<uint8_t>(upperRight)));
  publisher_->publish(get_frame(shirasuID_.upperLeftID, static_cast<uint8_t>(upperLeft)));
  publisher_->publish(get_frame(shirasuID_.lowerLeftID, static_cast<uint8_t>(lowerLeft)));
  publisher_->publish(get_frame(shirasuID_.lowerRightID, static_cast<uint8_t>(lowerRight)));
  //100右上、110左上、120左下、130右下
}

void AutoMove::reset(){
    initLocation_.x = location_.x;
    initLocation_.y = location_.y;
    velocity_.upperRight = 0.0f;
    velocity_.upperLeft = 0.0f;
    velocity_.lowerLeft = 0.0f;
    velocity_.lowerRight = 0.0f;
}

void AutoMove::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    if(msg->buttons[this->get_parameter("velButton").as_int()]==1){
      shirasuModePublish(5,5,5,5);
    //   publisher_->publish(get_frame(0x154,static_cast<uint8_t>(5)));
    }

    if(msg->buttons[this->get_parameter("disButton").as_int()]==1){
      shirasuModePublish(0,0,0,0);
      reset();
      publisher_->publish(get_frame(0x154,static_cast<uint8_t>(0)));
    }

    // if(msg->buttons[3]==1){
    //     publisher_->publish(shirasu_frame(0x155,static_cast<float>(-1.0f)));
    // }else{
    //     publisher_->publish(shirasu_frame(0x155, static_cast<float>(0.0f)));
    // }

    float x= -(msg->axes[0]);
    float y=  (msg->axes[1]);

    if(msg->buttons[3]==1){
        reset();
        automode = 1;
        autocount = 0;
    }

    if(automode == 1){
        if(std::isnan(maxSpeed)) RCLCPP_ERROR(this->get_logger(), "maxSpeed is NaN!");
        x = (target_.x-(location_.x-initLocation_.x));
        y = (target_.y-(location_.y-initLocation_.y));
        float r1 = sqrt(x*x+y*y);
        float xt = x/r1;
        float yt = y/r1;
        if(autocount < 100) {
            autocount++;
            xt = xt*(autocount/100.0f);
            yt = yt*(autocount/100.0f);
        }
        if(autocount == 100) autocount = 0;
        if(r1 < 300.0f){
            xt = xt*(r1/300.0f);
            yt = yt*(r1/300.0f);
        }
        shirasuValuePublish(maxSpeed*(yt-xt)/4,maxSpeed*(-xt-yt)/4,maxSpeed*(xt+yt)/4,maxSpeed*(xt-yt)/4);
        if(((target_.x-(location_.x-initLocation_.x))*(target_.x-(location_.x-initLocation_.x)) < 20.0)&&((target_.y-(location_.y-initLocation_.y)*(target_.y-(location_.y-initLocation_.y)) < 20.0))){
            shirasuValuePublish(0.0f,0.0f,0.0f,0.0f);
            automode = 2;
        }
    }
    if(automode==2){
        shirasuValuePublish(0.0f,0.0f,0.0f,0.0f);
        automode = 3;
    }
    if(automode==3){
        shirasuValuePublish(0.0f,0.0f,0.0f,0.0f);
        automode = 4;
    }

    if(automode == 4){
        RCLCPP_INFO(this->get_logger(),"AutoMove Finished");
        automode = 0;
    }

    if(automode == 0){
        float r= 0;
        if(msg->buttons[4]==1){
        r =1.0f;//↑左回転
        }
        else if(msg->buttons[5]==1){
        r =-1.0f;//右回転
        }
        else if(msg->buttons[4] == msg->buttons[5]){
        r =0.0f;
        }
        if(((x != 0) || (y != 0))||(r != 0)){
        shirasuValuePublish(maxSpeed*(y-x+r),maxSpeed*(-x-y+r),maxSpeed*(x+y+r),maxSpeed*(x-y+r));
        //chatter.publish(get_frame(0x101, x/static_cast<float>(sqrt(2))-y/static_cast<float>(sqrt(2))));
        count = 0;

        // RCLCPP_INFO(this->get_logger(), "Publishing:bokuha warukunai!");
        // std::string str = std::to_string(r);
        // const char* cstr = str.c_str();
        // RCLCPP_INFO(this->get_logger(), cstr);
        }else{
        if(count == 0){
            shirasuValuePublish(0.0f,0.0f,0.0f,0.0f);
            count = 1;
        }
        }
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoMove>());
    rclcpp::shutdown();
    return 0;
}