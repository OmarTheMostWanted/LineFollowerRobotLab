#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "main.cpp"
#include <geometry_msgs/Twist.h>

#include "std_msgs/String.h"

#include <sstream>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;


    ros::Publisher chatter_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

public:
    ImageConverter()
            : it_(nh_) {
                
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/camera/image", 1,
                                   &ImageConverter::imageCb, this);
    }

    ~ImageConverter() {
        cv::destroyWindow(OPENCV_WINDOW);
    }


    void imageCb(const sensor_msgs::ImageConstPtr &msg) {

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

            std::vector<int> res = convert(cv_ptr->image);



            geometry_msgs::Twist t;

            //left
            if (res[0] == 0) {

                if(res[1] > 160){
                    t.linear.x = 0;
                    t.angular.z = -1;
                }

                if(res[1] > 100 && res[1] < 160){
                    t.linear.x = 3;
                    t.angular.z = -1;
                }

                if(res[1] < 100){
                    t.linear.x = 0.6;
                    t.angular.z = 0;
                }
            }

            //right
            if (res[0] == 1) {
                if(res[1] > 160){
                    t.linear.x = 0;
                    t.angular.z = 1;
                }

                if(res[1] > 100 && res[1] < 160){
                    t.linear.x = 3;
                    t.angular.z = 1;
                }

                if(res[1] < 100){
                    t.linear.x = 0.6;
                    t.angular.z = 0;
                }
            }

            if(res[0] == 2){
                t.linear.x = 0;
                t.angular.z = 0;
            }

            chatter_pub.publish(t);

        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());

            return;
        }

        // Draw an example circle on the video stream
        if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
            cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255, 0, 0));

        cv::waitKey(3);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_converter");

    ImageConverter ic;

    ros::spin();

    return 0;
}
