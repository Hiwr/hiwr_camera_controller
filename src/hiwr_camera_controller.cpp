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

#include "hiwr_camera_controller.h"

using namespace cv;

namespace hiwr_camera_controller
{

/** Segfault signal handler */
void sigsegvHandler(int sig)
{
    signal(SIGSEGV, SIG_DFL);
    fprintf(stderr, "Segmentation fault, catched by sigsegvHandler.\n");
    ROS_ERROR("Segmentation fault, catched by sigsegvHandler");
    ros::shutdown();                      // stop the main loop
}


void HiwrCameraControllerNodelet::onInit(){
    public_nh_ = getNodeHandle(); //Public namespace

    private_nh_ = getMTPrivateNodeHandle();

    //Retrieveing params from param/config__xx.yaml
    //Can be better with have_param() before getting ...
    private_nh_.getParam("absolute_exposure", config_.absolute_exposure);
    private_nh_.getParam("camera_info_url", config_.camera_info_url);
    private_nh_.getParam("camera_name", config_.camera_name);
    private_nh_.getParam("brightness", config_.brightness);
    private_nh_.getParam("contrast", config_.contrast);
    private_nh_.getParam("device_name", config_.device);
    private_nh_.getParam("exposure", config_.exposure);
    private_nh_.getParam("focus_absolute", config_.focus_absolute);
    private_nh_.getParam("focus_auto", config_.focus_auto);
    private_nh_.getParam("format_mode", config_.format_mode);
    private_nh_.getParam("frame_id", config_.frame_id);
    private_nh_.getParam("frame_rate", config_.frame_rate);
    private_nh_.getParam("gain", config_.gain);
    private_nh_.getParam("height", config_.height);
    private_nh_.getParam("power_line_frequency", config_.power_line_frequency);
    private_nh_.getParam("saturation", config_.saturation);
    private_nh_.getParam("sharpness", config_.sharpness);
    private_nh_.getParam("white_balance_temperature", config_.white_balance_temperature);
    private_nh_.getParam("width", config_.width);

    configureSpinning(private_nh_);

    it_ = new image_transport::ImageTransport(private_nh_);
    image_pub_ = it_->advertise("output_video", 1);

    running_ = true;
    device_thread_ = boost::shared_ptr< boost::thread >
       (new boost::thread(boost::bind(&HiwrCameraControllerNodelet::spin, this)));

    NODELET_INFO("[UVC Cam Nodelet] Loading OK");
}

HiwrCameraControllerNodelet::HiwrCameraControllerNodelet():
    private_nh_("~"),
    public_nh_(config_.camera_name)
{
    next_config_level_=0;
    next_config_update_ = false;
    next_config_count_=0;

    reconfig_number_=0;
    published_frame_=0;

    state_ = Driver::CLOSED;
    spining_state_ = true;
    calibration_matches_ = true;
    frame_= NULL;
}

void HiwrCameraControllerNodelet::loopGrabImage(){

    std::unique_lock<std::mutex> lk(mutex_);

    if(lk.owns_lock()){
        lk.unlock();
    }

    while(ros::ok()){
        int buf_idx;

        if(next_config_update_){
            printf("[%s] will apply nextconfig__ \n", config_.camera_name.c_str());
            applyNewConfig(next_config_ , next_config_level_);
            printf("[%s] Did apply nextconfig__ \n", config_.camera_name.c_str());
        }

        if(!spining_state_){
            usleep(1000);
            continue;
        }

        uint32_t tmp_bytes_used;
        try{
            buf_idx = cam_->grab(&next_frame_, &tmp_bytes_used);
        }catch (std::exception e) {
            printf("exception calling grab\n");
            usleep(1000);
            continue;
        }

        if (buf_idx < 0) {
            printf("Could not grab image\n");
            usleep(1000);
            continue;
        }

        if(!lk.try_lock()){
            cam_->release(buf_idx);
            continue;
        }
        if(frame_!=NULL){
            free(frame_);
        }
        frame_ = (unsigned char*)malloc(tmp_bytes_used);
        memcpy(frame_, next_frame_, tmp_bytes_used);
        bytes_used_ = tmp_bytes_used;
        new_frame_=true;
        lk.unlock();
        waiter_.notify_all();

        cam_->release(buf_idx);

    }

}

void HiwrCameraControllerNodelet::closeCamera(){
    if (state_ != Driver::CLOSED){

        ROS_INFO_STREAM("[" << camera_name_ << "] closing device");
        if(cam_) {
            delete cam_;
        }
        state_ = Driver::CLOSED;
    }
}


/** Open the camera device.
 *
 * @param newconfig_ config_uration parameters
 * @return true, if successful
 *
 * if successful:
 *   state_ is Driver::OPENED
 *   camera_name_ set to camera_name string
 */
bool HiwrCameraControllerNodelet::openCamera(Config &new_config){

    new_config.format_mode = uvc_cam::Cam::MODE_YUYV;
    uvc_cam::Cam::mode_t mode = uvc_cam::Cam::MODE_YUYV;
    bool success = true;
    final_=NULL;
    try
    {
        ROS_INFO("opening uvc_cam at %dx%d, %f fps - %s : %s", new_config.width, new_config.height, new_config.frame_rate , new_config.device.c_str() , findVideoDevice(new_config.device.c_str()));
        printf("before cam creating, cam is at %p\n", cam_);
        ROS_INFO("[Uvc Cam Nodelet] video device is {%s}", new_config.device.c_str() );
        char * video_device = findVideoDevice(new_config.device.c_str());
        NODELET_INFO("Video DEVICE IS %s",video_device);
        cam_ = new uvc_cam::Cam(video_device, mode, new_config.width, new_config.height, new_config.frame_rate);
        printf("after cam creating, cam is at %p\n", cam_);

        camera_name_ = config_.camera_name ;
        NODELET_INFO("[%s] camera_name_=%s",config_.camera_name.c_str(), camera_name_.c_str()  );
        if (camera_name_ != camera_name_)
        {
            camera_name_ = camera_name_;
        }
        state_ = Driver::OPENED;
        calibration_matches_ = true;

    }
    catch (uvc_cam::Exception& e)
    {
        ROS_FATAL_STREAM("[" << camera_name_
                         << "] exception opening device: " << e.what());
        success = false;
    }
    catch(std::runtime_error& ex){
        success = false;
        NODELET_WARN("[UVC Cam Node] runtime_error ex : %s",ex.what());
    }

    return success;
}


void HiwrCameraControllerNodelet::dealMemory(){
    if(final_!=NULL){
        free(final_);
    };
    final_ = (unsigned char *) malloc( sizeof( char) * config_width_ * config_height_ );
    if(final_ == NULL){
        printf("############# MALLOC FAILED !! ##########\n");
    }
}

bool HiwrCameraControllerNodelet::copyRead(){
    std::unique_lock<std::mutex> lk(mutex_);

    if(!lk.owns_lock()){
        try {
            lk.lock();
        } catch(const std::system_error& e) {
            std::cout << "coin2 Caught system_error with code " << e.code()
                      << " meaning " << e.what() << '\n';
        }
    }

    if(!new_frame_){
        waiter_.wait(lk);
    }
    image_ipl_ = cvCreateImageHeader(cvSize(config_width_ ,config_height_), 8, 1);
    dealMemory();
    const int total = config_width_*config_height_*2;

    if(total!=bytes_used_)
        return false;

    int j=0;
    for(int i=0; i< total; j++){
        final_[j]=frame_[i];
        i+=2;
    };
    new_frame_=false;
    lk.unlock();

    return true;
}

std::unique_lock<std::mutex> HiwrCameraControllerNodelet::getLock(){
    std::unique_lock<std::mutex> lk(mutex_);
    if(!lk.owns_lock()){
        printf(" dont get the lock \n");
        try {
            lk.lock();
        } catch(const std::system_error& e) {
            std::cout << "coin2 Caught system_error with code " << e.code()
                      << " meaning " << e.what() << '\n';
        }
    }
    printf(" do get the lock \n");
    return lk;
}

/** Read camera data.
 *
 * @return true if successful
 */
bool HiwrCameraControllerNodelet::read() {
    bool success = true;
    if(config_width_ < 1 || config_height_ < 1 )
        return false;

    try
    {
        bool copy_worked = copyRead();
        if(!copy_worked)
            return false;

        image_ipl_->imageData = (char *)final_;
    }
    catch (uvc_cam::Exception& e)
    {
        printf("exception readind data %s", e.what());
        success = false;
    }
    return success;
}

/** Dynamic reconfig_ure callback
 *
 *  Called immediately when callback first defined. Called again
 *  when dynamic reconfig_ure starts or changes a parameter value.
 *
 *  @param newconfig_ new Config values
 *  @param level bit-wise OR of reconfig_uration levels for all
 *               changed parameters (0xffffffff on initial call)
 **/

void HiwrCameraControllerNodelet::reconfig(Config &new_config, uint32_t level)
{

    next_config_= new_config;
    next_config_.device = config_.device;
    next_config_level_ = level;
    printf("################################## reconfig_ call with level %u \n" , level);
    next_config_update_ = true;
    next_config_count_ ++;

}


void HiwrCameraControllerNodelet::changeSize(int width, int height){

    printf("############## changing size called %d %d %d \n", width, height, next_config_update_);
    if(next_config_update_ == true){
        printf("change sizze called, but already called before \n");
        return;
    }
    next_config_ = Config(config_);
    next_config_.width = width;
    next_config_.height = height;
    next_config_update_ = true;

}

void HiwrCameraControllerNodelet::applyNewConfig(Config &new_config, uint32_t level)
{
    printf("### dynamic reconfig_ure level 0x%x \n", level);
    ROS_DEBUG("dynamic reconfig_ure level 0x%x", level);
    reconfig_number_++;
    // resolve frame ID using tf_prefix parameter
    if (new_config.frame_id == "")
        new_config.frame_id = "camera";

    ROS_DEBUG("dynamic reconfig_ure level 0x%x", level);
    std::string tf_prefix = tf::getPrefixParam(private_nh_);
    ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
    new_config.frame_id = tf::resolve(tf_prefix, new_config.frame_id);

    if (state_ != Driver::CLOSED && (level & Levels::RECONFIGURE_CLOSE))
    {
        // must close the device before updating these parameters
        closeCamera();                  // state_ --> CLOSED
    }
    if (state_ == Driver::CLOSED)
    {
        // open with new values
        if (openCamera(new_config))
        {
            // update camera name string
            new_config.camera_name = camera_name_;
        }
    }


    if(reconfig_number_ == 1 || config_.focus_auto != new_config.focus_auto){
        try {
            NODELET_INFO("will try to set autofocus to %d \n " , new_config.focus_auto);
            cam_->set_control( 0x009A090C , new_config.focus_auto);
        } catch (uvc_cam::Exception& e) {
            ROS_ERROR_STREAM("Problem setting focus_auto. Exception was " << e.what());
        }
    }


    if(reconfig_number_ == 1 || config_.focus_absolute != new_config.focus_absolute){
        try {
            NODELET_INFO("will try to set focus_absolute to %d \n " , new_config.focus_absolute);
            cam_->set_control( 0x009A090A  , new_config.focus_absolute);
        } catch (uvc_cam::Exception& e) {
            ROS_ERROR_STREAM("Problem setting focus_absolute. Exception was " << e.what() << "value was" << new_config.focus_absolute ) ;
        }
    }

    config_ = new_config;

    config_width_ = new_config.width;
    config_height_ = new_config.height;

    next_config_update_ = false;
    printf("### dynamic reconfig_ure will unlock\n");

}


void HiwrCameraControllerNodelet::setSpinningState(bool p_spinning_state){
    spining_state_ = p_spinning_state;
}

bool HiwrCameraControllerNodelet::getSpinningState(){
    return spining_state_;
}

cv_bridge::CvImage out_msg_;

void HiwrCameraControllerNodelet::publishFrame(Mat frame ) {
    out_msg_.encoding = sensor_msgs::image_encodings::MONO8;
    out_msg_.image    = frame;
    out_msg_.header.seq = published_frame_;


    char frame_id[20];
    sprintf(frame_id , "%d" , published_frame_);
    out_msg_.header.frame_id = frame_id;
    out_msg_.header.stamp = ros::Time::now();

    published_frame_++;
    image_publisher_.publish(out_msg_.toImageMsg() );
}


/** driver main spin loop */\
void HiwrCameraControllerNodelet::spin() {
    NODELET_INFO("[UVC Cam Nodelet] Main Loop...");
    printf("inside spinOnce loop \n");
    dynamic_reconfigure::Server<Config> srv;
    dynamic_reconfigure::Server<Config>::CallbackType f
            = boost::bind(&HiwrCameraControllerNodelet::reconfig, this, _1, _2);
    srv.setCallback(f);

    reconfig(config_, 0xffffffff);
    applyNewConfig(config_ , 0xffffffff);




   spinning_thread_ = boost::shared_ptr< boost::thread >
      (new boost::thread(boost::bind(&HiwrCameraControllerNodelet::loopGrabImage, this)));

    while (ros::ok())
    {
        if (state_ != Driver::CLOSED)
        {
            //Check nb subscribers
            if(image_pub_.getNumSubscribers() > 0 && !next_config_update_)
            {
                if (spining_state_ == true  && next_config_count_ > 0 && read() )
                {

                    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
                    cv_ptr->header.stamp = ros::Time::now();
                    cv_ptr->header.frame_id = config_.frame_id;
                    cv_ptr->encoding = sensor_msgs::image_encodings::MONO8; //"mono8";//"bgr8";
                    cv_ptr->image = Mat( image_ipl_ );


                    image_pub_.publish(cv_ptr->toImageMsg());

                }else{
                    usleep(1000);//1ms sleep
                }
            }else{
                usleep(1000);
            }
        }
        ros::spinOnce();
    }
}

bool HiwrCameraControllerNodelet::serviceSetSpinningState( hyve_msg::SetState::Request &req ,hyve_msg::SetState::Response  &res  ){
    res.state = req.state;
    setSpinningState(req.state);
    return true;
}

bool HiwrCameraControllerNodelet::serviceGetSpinningState( hyve_msg::GetState::Request &req ,hyve_msg::GetState::Response  &res  ){
    res.state =getSpinningState();
    return true;
}

void HiwrCameraControllerNodelet::configureSpinning(ros::NodeHandle& nh){
    NODELET_INFO("[UVCCam Nodelet] START config_uring spinning state");
    ros::NodeHandle& ph = getMTPrivateNodeHandle();
    service_spinning_state_setter_ = ph.advertiseService("setSpinningState", &HiwrCameraControllerNodelet::serviceSetSpinningState, this);
    service_spinning_state_getter_ = ph.advertiseService("getSpinningState", &HiwrCameraControllerNodelet::serviceGetSpinningState, this);

    NODELET_INFO("[UVCCam Nodelet] DONE");
}

PLUGINLIB_DECLARE_CLASS(hiwr_camera_controller,HiwrCameraControllerNodelet, hiwr_camera_controller::HiwrCameraControllerNodelet, nodelet::Nodelet);
}
