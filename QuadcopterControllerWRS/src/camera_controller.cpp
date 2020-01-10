#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cnoid/Camera>
#include <cnoid/SimpleController>

using namespace cnoid;

namespace {

class CameraController : public SimpleController
{

    BodyPtr ioBody;
    Camera* camera2;
    double timeCounter;
    double timeStep;
    ros::Publisher camera_image_pub_;

public:

    virtual bool initialize(SimpleControllerIO* io) override
    {
        ioBody = io->body();
        camera2 = ioBody->findDevice<Camera>("Camera2");  //取得
        io->enableInput(camera2);

        ros::NodeHandle n_;
        camera_image_pub_ = n_.advertise<sensor_msgs::Image>("/quadcopter/camera_image", 1);

        return true;
    }

    virtual bool control() override
    {
        sensor_msgs::Image image;
        image.height = (uint32_t)camera2->image().height();
        image.width = (uint32_t)camera2->image().width();
        image.step = image.width * 3;
        int size = image.height*image.step;
        std::vector<unsigned char> vec(size);
        vec.assign(camera2->image().pixels(), camera2->image().pixels()+size);
        image.data = vec;
        image.encoding = "rgb8";
        camera_image_pub_.publish(image);

        return true;
    }

};

}

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(CameraController)