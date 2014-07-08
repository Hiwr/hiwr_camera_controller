#include <opencv/cv.h>


#include <driver_base/SensorLevels.h>
#include <driver_base/driver.h>

#include "hyve_camera_common/UVCCamConfig.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <nodelet/nodelet.h>

#include "std_msgs/String.h"

#include <uvc_cam.h>

#include <hyve_msg/SetState.h>
#include <hyve_msg/GetState.h>

#include <stdexcept> //runtime_error exception
namespace cam_nodelet
{

//typedef driver_base::Driver Driver;
//typedef driver_base::SensorLevels Levels;

class Uvc_cam_node : public nodelet::Nodelet{
protected:
    sensor_msgs::CameraInfo cam_info_;
    /** dynamic parameter configuration */
    typedef hyve_camera_common::UVCCamConfig Config;
    Config config_;
    void reconfig(Uvc_cam_node::Config&, uint32_t);
    void locked_reconfig(Uvc_cam_node::Config &, uint32_t );
    bool configUpdated;


    volatile int config_width;
    volatile int config_height;


    volatile bool running_;               ///< device is running
    boost::shared_ptr<boost::thread> deviceThread_;
    boost::shared_ptr<boost::thread> spinThread_;
private:
    typedef driver_base::Driver Driver;
    typedef driver_base::SensorLevels Levels;
    Driver::state_t state_;               // current driver state

    ros::NodeHandle private_nh;              // private node handle
    ros::NodeHandle public_nh;           // camera name space handle

    sensor_msgs::Image image_;
    std::string device_;
    std::string camera_name_;

    uvc_cam::Cam *cam_;
    unsigned char *frame;
    bool new_frame;

    unsigned char *next_frame ;

    std::mutex mutex;
    std::condition_variable waiter;

    hyve_camera_common::UVCCamConfig nextconfig_;
    uint32_t nextconfigLevel_;
    bool nextconfigUpdate;
    int nextconfigCount;

    volatile uint32_t bytes_used;

    int reconfigNumber;
    int publishedFrame;

    void apply_newconfig(Uvc_cam_node::Config &, uint32_t );
    void loop_grab_image();

    /** camera calibration information */
    //camera_info_manager::CameraInfoManager cinfo_;
    bool calibration_matches_;            // cam_info_ matches video mode

    /** image transport interfaces */
    image_transport::ImageTransport * it_;
    //image_transport::CameraPublisher image_pub_;
    IplImage  *imageIpl  ;

    //States
    bool spiningState;
    bool publishingState;
    bool processingState;

    unsigned char *final ;

    void closeCamera();
    bool openCamera(Uvc_cam_node::Config&);
    void deal_memory();
    bool copy_read();
    bool read();

    //Publishers
    image_transport::Publisher imagePublisher;
    image_transport::Publisher image_pub_;

    //Services
    ros::ServiceServer serviceSpinningStateSetter;
    ros::ServiceServer serviceSpinningStateGetter;

    void publish_frame(cv::Mat);

    std::thread spin_thread;
    std::thread loop_grab_image_thread;

public:
    Uvc_cam_node();
    virtual void onInit();
    void callback(const std_msgs::String::ConstPtr& msg);

    /* Uvc_cam_node():
        privNH_("~"),
        camera_nh_("camera"),
        cinfo_(camera_nh_)//,
      //  it_(camera_nh_)
    {
        state_ = Driver::CLOSED;
        calibration_matches_ = true;
        device_ = "Microsoft® LifeCam Cinema(TM)";
        camera_name_ = "camera";
    };*/

    ~Uvc_cam_node() {
        if (running_)
        {
            NODELET_INFO("shutting down driver thread");
            running_ = false;
            deviceThread_->join();
            spinThread_->join();
            NODELET_INFO("driver thread stopped");
        }
        if(state_ != Driver::CLOSED) {
            closeCamera();
        }

    }
    void spin();
    std::unique_lock<std::mutex> getLock();

    //  virtual void configure( ){} ;
    //  virtual void process_frame(cv::Mat frame ){} ;
    void change_size(int width, int height);

    void setSpinningState(bool pSpinningState);
    bool getSpinningState();

    //   void setProcessingState(bool pProcessingState);
    //   bool getProcessingState();

    //   void setPublishingState(bool pPublishingState);
    //  bool getPublishingState();

    void configurePublishing(ros::NodeHandle&);
    void configureSpinning(ros::NodeHandle&);

    bool service_SetSpinningState( hyve_msg::SetState::Request&, hyve_msg::SetState::Response&);
    bool service_GetSpinningState( hyve_msg::GetState::Request&, hyve_msg::GetState::Response&);

};
}
