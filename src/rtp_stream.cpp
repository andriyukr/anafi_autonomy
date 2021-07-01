#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio.hpp>

class RtpStream {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

    cv_bridge::CvImagePtr latest_image_;
    std::unique_ptr<cv::VideoWriter> video_writer_;

public:
    RtpStream(ros::NodeHandle n)
        : it_(n)
    {
        std::string image_topic = "/anafi/image";
        image_sub_ = it_.subscribe(image_topic, 1, &RtpStream::image_cb, this);
        cv::Size frame_size(1280, 720);
        video_writer_ = std::make_unique<cv::VideoWriter>("appsrc ! videoconvert ! video/x-raw,format=I420 ! queue ! vp8enc threads=4 target-bitrate=3000000 overshoot=25 undershoot=100 deadline=1 keyframe-max-dist=5 ! queue ! rtpvp8pay ! udpsink host=127.0.0.1 port=5004", 0, 30, frame_size, true);
        // std::cout << "is opened: " << video_writer_->isOpened() << std::endl;
    }

    ~RtpStream() {}

    void image_cb(const sensor_msgs::ImageConstPtr& msg)
    {
        try {
            latest_image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            stream_frame();
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

    void stream_frame()
    {
        video_writer_->write(latest_image_->image);
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rtp_stream");
    ros::NodeHandle n(ros::this_node::getName());
    std::string n_namespace = n.getNamespace();
    RtpStream rs(n);
    ros::spin();
    return 0;
}

