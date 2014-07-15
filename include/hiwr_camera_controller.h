#include <opencv/cv.h>

#include <cv_bridge/cv_bridge.h>
#include <nodelet/nodelet.h>
#include "std_msgs/String.h"
#include <uvc_cam.h>

#include <hyve_msg/SetState.h>
#include <hyve_msg/GetState.h>

#include <stdexcept> //runtime_error exception
#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <signal.h>
#include <cstdio>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/console.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>

#include <camera_info_manager/camera_info_manager.h>

#include <dynamic_reconfigure/server.h>

#include <driver_base/SensorLevels.h>
#include <driver_base/driver.h>

#include <image_transport/image_transport.h>
#include "hyve_camera_common/UVCCamConfig.h"

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rospack/rospack.h>

#include <tf/transform_listener.h>

#include <condition_variable>

//#include "HiwrCameraControllerNodelet.h"
#include "../src/selectCam.c"

//Nodelet include
#include <pluginlib/class_list_macros.h>

namespace hiwr_camera_controller
{



class HiwrCameraControllerNodelet : public nodelet::Nodelet{
public:
    typedef hyve_camera_common::UVCCamConfig Config;
    Config config_;
    void reconfig(HiwrCameraControllerNodelet::Config&, uint32_t);
    void lockedReconfig(HiwrCameraControllerNodelet::Config &, uint32_t );

    HiwrCameraControllerNodelet();
    virtual void onInit();
    void callback(const std_msgs::String::ConstPtr& msg);

    ~HiwrCameraControllerNodelet() {
        if (running_)
        {
            NODELET_INFO("shutting down driver thread");
            running_ = false;
            device_thread_->join();
            spinning_thread_->join();
            NODELET_INFO("driver thread stopped");
        }
        if(state_ != Driver::CLOSED) {
            closeCamera();
        }

    }
    void spin();
    std::unique_lock<std::mutex> getLock();

    void changeSize(int width, int height);

    void setSpinningState(bool pSpinningState);
    bool getSpinningState();

    void configurePublishing(ros::NodeHandle&);
    void configureSpinning(ros::NodeHandle&);

    bool serviceSetSpinningState( hyve_msg::SetState::Request&, hyve_msg::SetState::Response&);
    bool serviceGetSpinningState( hyve_msg::GetState::Request&, hyve_msg::GetState::Response&);


protected:
    sensor_msgs::CameraInfo cam_info_;
    /** dynamic parameter configuration */
    bool config_updated_;

    volatile int config_width_;
    volatile int config_height_;

    volatile bool running_;               ///< device is running
    boost::shared_ptr<boost::thread> device_thread_;
    boost::shared_ptr<boost::thread> spinning_thread_;

private:

    typedef driver_base::Driver Driver;
    typedef driver_base::SensorLevels Levels;
    Driver::state_t state_;               // current driver state

    ros::NodeHandle private_nh_;              // private node handle
    ros::NodeHandle public_nh_;           // camera name space handle

    sensor_msgs::Image image_;
    std::string device_;
    std::string camera_name_;

    uvc_cam::Cam *cam_;
    unsigned char *frame_;
    bool new_frame_;

    unsigned char *next_frame_;

    std::mutex mutex_;
    std::condition_variable waiter_;

    hyve_camera_common::UVCCamConfig next_config_;
    uint32_t next_config_level_;
    bool next_config_update_;
    int next_config_count_;

    volatile uint32_t bytes_used_;

    int reconfig_number_;
    int published_frame_;

    void applyNewConfig(Config &, uint32_t );
    void loopGrabImage();

    /** camera calibration information */
    //camera_info_manager::CameraInfoManager cinfo_;
    bool calibration_matches_;            // cam_info_ matches video mode

    /** image transport interfaces */
    image_transport::ImageTransport * it_;
    IplImage  *image_ipl_ ;

    //States
    bool spining_state_;
    bool publishing_state_;
    bool processing_state_;

    unsigned char *final_;

    void closeCamera();
    bool openCamera(HiwrCameraControllerNodelet::Config&);
    void dealMemory();
    bool copyRead();
    bool read();

    //Publishers
    image_transport::Publisher image_publisher_;
    image_transport::Publisher image_pub_;

    //Services
    ros::ServiceServer service_spinning_state_setter_;
    ros::ServiceServer service_spinning_state_getter_;

    void publishFrame(cv::Mat);

    std::thread spin_thread_;
    std::thread loop_grab_image_thread_;


};
}
