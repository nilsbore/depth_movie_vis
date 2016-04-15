#include "ros/ros.h"
#include <boost/algorithm/string.hpp>
#include "sensor_msgs/Image.h"
#include <cv.h>
#include <highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <thread>

using namespace std;

class depth_movie_vis {
public:

    ros::NodeHandle n;

    ros::Subscriber image_sub;
    ros::Publisher image_pub;
    string image_input;
    string image_output;

    vector<string> video_paths;

    cv::VideoCapture cap;
    cv::VideoCapture next_cap;
    size_t current_video;
    cv::Mat frame;
    cv::Mat last_frame;

    cv::Mat static_image;
    std::thread worker_thread;

    int counter;

    depth_movie_vis(const string& name)
    {
        ros::NodeHandle pn("~");
        pn.param<string>("image_input", image_input, string("/camera/depth/image_raw"));
        pn.param<string>("image_output", image_output, string("/psychedelic_image"));
        string video_input;
        pn.param<string>("video_input", video_input, string(""));
        boost::split(video_paths, video_input, boost::is_any_of(" \t"));

        image_sub = n.subscribe(image_input, 1, &depth_movie_vis::image_callback, this);
        image_pub = n.advertise<sensor_msgs::Image>(image_output, 1);

        current_video = 0;
        counter = 3;
        cap = cv::VideoCapture(video_paths[current_video]);
        load_next_cap();
        read_next_frame();

        //play_video();
    }

    void load_next_cap()
    {
        worker_thread = std::thread([this]() {
            cout << "Loading video" << endl;
            next_cap = cv::VideoCapture(video_paths[(current_video + 1) % video_paths.size()]);
            cv::Mat temp;
            next_cap.read(temp);
            cout << "Finished loading video" << endl;
        });
        //worker_thread.detach();
    }

    void read_next_frame()
    {
        if (counter < 3) {
            last_frame.copyTo(frame);
            counter++;
            return;
        }
        counter = 0;
        if (!cap.read(frame) || frame.empty()) {
            current_video = (current_video + 1) % video_paths.size();
            //cap = cv::VideoCapture(video_paths[current_video
            worker_thread.join();
            cap = next_cap;
            load_next_cap();
            read_next_frame();
            //cap.read(frame);
        }
        frame.copyTo(last_frame);

    }

    void image_callback(const sensor_msgs::Image& image_msg)
    {
        cout << "Got image!" << endl;

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::TYPE_16UC1);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        //cv::Mat gray_image;
        //cvtColor(cv_ptr->image, gray_image, CV_BGR2GRAY);

        cv::Mat resized_image;
        cv::resize(cv_ptr->image, resized_image, cv::Size(frame.cols, frame.rows), 0, 0, cv::INTER_CUBIC);
        resized_image.convertTo(resized_image, CV_32FC1);

        if (static_image.empty()) {
            static_image = resized_image.clone();
        }

        float threshold = 300;
        cv::Mat mask = cv::abs(resized_image - static_image) < threshold;
        cv::dilate(mask, mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(13,13)));

        //cv::Mat inverted_mask;
        //cv::bitwise_not(mask, inverted_mask);
        cv::Mat bland;// = cv::Mat::zeros(frame.rows, frame.cols, CV_8UC3);
        frame.copyTo(bland);
        bland = bland/3;
        //frame.setTo(cv::Scalar(255, 255, 255), mask);
        bland.copyTo(frame, mask);

        cv::copyMakeBorder(frame, frame, 0, 0, 73, 74, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

        //cv::Mat color_image;
        //cvtColor(resized_image, color_image, CV_GRAY2RGB);

        /*cv::Mat gray_frame;
        cvtColor(frame, gray_frame, CV_BGR2GRAY);
        gray_frame.convertTo(gray_frame, CV_32FC1);

        cout << "frame size: " << gray_frame.cols << ", " << gray_frame.rows << ", " << gray_frame.channels() << endl;
        cout << "image size: " << resized_image.cols << ", " << resized_image.rows << ", " << resized_image.channels() << endl;

        gray_frame = resized_image * gray_frame;*/

        cv::namedWindow("window", CV_WINDOW_NORMAL);
        cvSetWindowProperty("window", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
        cv::imshow("window", frame);
        cv::waitKey(5);

        read_next_frame();

        /*cv_bridge::CvImagePtr cv_pub_ptr(new cv_bridge::CvImage);
        frame.copyTo(cv_pub_ptr->image);
        cv_pub_ptr->encoding = "bgr8";

        image_pub.publish(cv_pub_ptr->toImageMsg());*/
    }

    void play_video()
    {
        while (true)
        {
            cv::imshow("window", frame);

            char key = cvWaitKey(10);
            if (key == 27) {
                break;
            }

            read_next_frame();
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "depth_movie_node");

    depth_movie_vis dv(ros::this_node::getName());

    ros::spin();

    return 0;
}
