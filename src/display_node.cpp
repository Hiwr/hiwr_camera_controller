/*********************************************************************
*
*
* Copyright 2014 Worldline
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
* 
*     http://www.apache.org/licenses/LICENSE-2.0
* 
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
* 
***********************************************************************/
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "std_msgs/String.h"
#include <stdio.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <math.h> //fabs

#include <mutex>
#include <thread>


using namespace cv;

namespace display_nodelet
{

class Display_node : public nodelet::Nodelet
{
public:
    Display_node()
        : value_(0)
    {}

private:
    image_transport::Subscriber image_sub_;
    image_transport::ImageTransport * it_;
    std::string name;
    std::mutex mtx;
    bool receivingImage;
    ros::Publisher pub;

    //Will subscribe to WebcamNodelet
    double value_;
    IplImage * imageGrayscale;

    //FIXME Implements a mutex for cv_ptr
    cv_bridge::CvImagePtr cv_ptr;

    std::thread spin_thread;
    virtual void onInit()
    {
        ros::NodeHandle& public_nh = getNodeHandle();
        ros::NodeHandle& private_nh = getPrivateNodeHandle();
        it_ = new image_transport::ImageTransport(public_nh);
        // public_nh.getParam("value", value_);
        std::string video_stream;
        private_nh.getParam("video_stream", video_stream);
        image_sub_ = it_->subscribe(video_stream.c_str(), 1,&Display_node::imageCb, this);

        cv::startWindowThread();
        //Main loop thread
        spin_thread = std::thread( &Display_node::loop  , this);
        receivingImage = false;
        NODELET_INFO("Initialize Nodelet Webcam Grayscale with video_stream value = %s", video_stream.c_str());

    }

    void loop(){
        name = "Display_";
        name+=('A'+(random() % 26));
        cv::namedWindow(name,CV_WINDOW_AUTOSIZE);
        while(ros::ok()){
            if(cv_ptr != NULL && !receivingImage){

                //cv::imshow(name,Mat(500,500,CV_8UC3, Scalar(rand() % 255,rand() % 255, rand() % 255)));
                //Duplicated
                try{
                    cv::imshow(name,cv_ptr->image);
                    //cv::imshow(name, cv_ptr->image);
                    cv::waitKey(40);
                }catch(...){
                    //Skipping if corrupted image or whatever
                }

            }
        }
    }
    //Callback for receiving images
    void imageCb(const sensor_msgs::ImageConstPtr& msg){
        receivingImage = true;
        // std::unique_lock<std::mutex> lck (mtx);
        //  if(mtx.try_lock()== true){
        // cv_bridge::CvImagePtr cv_ptr;
        //printf("[Grayscale Nodelet %s] Receiving image data...\n", name.c_str());
        try
        {
            //FIXME check encoding first
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        }
        catch (cv_bridge::Exception& e)
        {
            NODELET_INFO("cv_bridge exception: %s", e.what());
            return;
        }
        receivingImage = false;
    }
};

PLUGINLIB_DECLARE_CLASS(display_nodelet, Display_node, display_nodelet::Display_node, nodelet::Nodelet)
}
