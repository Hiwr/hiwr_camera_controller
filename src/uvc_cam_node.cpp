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

// Modified Apr 6, 2010 by Adam Leeper - changed to use "image_transport"

/* Much of this code is either heavily inspired by or taken directly from the
 * camera1394 ROS driver by Jack O'Quin
 */
#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <signal.h>
#include <cstdio>
#include <thread>
#include <condition_variable>
#include <mutex>
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/console.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <tf/transform_listener.h>
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

#include "uvc_cam_node.h"
//#include <uvc_cam.h>
#include "./selectCam.c"


//Nodelet include
#include <pluginlib/class_list_macros.h>

#include "std_msgs/String.h"

using namespace cv;



namespace cam_nodelet
{


/** Segfault signal handler */
void sigsegv_handler(int sig)
{
    signal(SIGSEGV, SIG_DFL);
    fprintf(stderr, "Segmentation fault, catched by sigsegv_handler.\n");
    ROS_ERROR("Segmentation fault, catched by sigsegv_handler");
    ros::shutdown();                      // stop the main loop
}



void Uvc_cam_node::onInit(){
    public_nh = getNodeHandle(); //Public namespace

    private_nh = getMTPrivateNodeHandle();
    // private_nh = getPrivateNodeHandle(); //private namespace

    //Retrieveing params from param/config_xx.yaml
    //Can be better with have_param() before getting ...
    private_nh.getParam("absolute_exposure", config_.absolute_exposure);
    private_nh.getParam("camera_info_url", config_.camera_info_url);
    private_nh.getParam("camera_name", config_.camera_name);
    private_nh.getParam("brightness", config_.brightness);
    private_nh.getParam("contrast", config_.contrast);
    private_nh.getParam("device_name", config_.device);
    private_nh.getParam("exposure", config_.exposure);
    private_nh.getParam("focus_absolute", config_.focus_absolute);
    private_nh.getParam("focus_auto", config_.focus_auto);
    private_nh.getParam("format_mode", config_.format_mode);
    private_nh.getParam("frame_id", config_.frame_id);
    private_nh.getParam("frame_rate", config_.frame_rate);
    private_nh.getParam("gain", config_.gain);
    private_nh.getParam("height", config_.height);
    private_nh.getParam("power_line_frequency", config_.power_line_frequency);
    private_nh.getParam("saturation", config_.saturation);
    private_nh.getParam("sharpness", config_.sharpness);
    private_nh.getParam("white_balance_temperature", config_.white_balance_temperature);
    private_nh.getParam("width", config_.width);

    configureSpinning(private_nh);

    it_ = new image_transport::ImageTransport(private_nh);
    image_pub_ = it_->advertise("output_video", 1);

    running_ = true;
    deviceThread_ = boost::shared_ptr< boost::thread >
       (new boost::thread(boost::bind(&Uvc_cam_node::spin, this)));

   // spin_thread = std::thread( &Uvc_cam_node::spin  , this);

    NODELET_INFO("[UVC Cam Nodelet] Loading OK");
}

Uvc_cam_node::Uvc_cam_node():
    private_nh("~"),
    public_nh(config_.camera_name)
  //  cinfo_(private_nh)
{
    // cinfo_private_nh;
    nextconfigLevel_=0;
    nextconfigUpdate = false;
    nextconfigCount=0;

    reconfigNumber=0;
    publishedFrame=0;

    state_ = Driver::CLOSED;
    spiningState = true;
    calibration_matches_ = true;
    frame= NULL;
}

void Uvc_cam_node::loop_grab_image(){

    std::unique_lock<std::mutex> lk(mutex);

    if(lk.owns_lock()){
        lk.unlock();
    }

    while(ros::ok()){
        //printf("I'm looping with spinningState=%s\n",spiningState ? "true" : "false");
        int buf_idx;

        if(nextconfigUpdate){
            printf("[%s] will apply nextconfig_ \n", config_.camera_name.c_str());
            apply_newconfig(nextconfig_ , nextconfigLevel_);
            printf("[%s] Did apply nextconfig_ \n", config_.camera_name.c_str());
        }

        if(!spiningState){
            //printf("Going to sleep\n");
            usleep(1000);
            continue;
        }

        uint32_t tmp_bytes_used;
        try{
            buf_idx = cam_->grab(&next_frame, &tmp_bytes_used);
        }catch (std::exception e) {
            printf("exception calling grab\n");
            //std::cout << "An exception occurred. Exception Nr. "<< e.what() << std::endl;
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
        if(frame!=NULL){
            free(frame);
        }
        frame = (unsigned char*)malloc( tmp_bytes_used);
        memcpy(frame, next_frame  , tmp_bytes_used);
        bytes_used = tmp_bytes_used;
        /*if(frame!=NULL){
            free(frame);
            }
            frame = (unsigned char*) malloc( bytes_used);
            memcpy(frame, next_frame,bytes_used) ;*/
        new_frame=true;
        /*if(frame!=NULL)
            printf("Frame[0]=%c\n",frame[0]);
        else
            printf("Frame[0] is NULL\n");*/

        lk.unlock();
        waiter.notify_all();

        cam_->release(buf_idx);

    }

}

void Uvc_cam_node::closeCamera(){
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
 * @param newconfig configuration parameters
 * @return true, if successful
 *
 * if successful:
 *   state_ is Driver::OPENED
 *   camera_name_ set to camera_name string
 */
bool Uvc_cam_node::openCamera(Config &newconfig){

    newconfig.format_mode = uvc_cam::Cam::MODE_YUYV;
    uvc_cam::Cam::mode_t mode = uvc_cam::Cam::MODE_YUYV;
    bool success = true;
    final=NULL;
    try
    {
        ROS_INFO("opening uvc_cam at %dx%d, %f fps - %s : %s", newconfig.width, newconfig.height, newconfig.frame_rate , newconfig.device.c_str() , findVideoDevice(newconfig.device.c_str()));
        printf("before cam creating, cam is at %p\n", cam_);
        ROS_INFO("[Uvc Cam Nodelet] video device is {%s}", newconfig.device.c_str() );
        //newconfig.device = "MicrosoftÂ® LifeCam Cinema(TM)";
        char * videoDevice = findVideoDevice(newconfig.device.c_str());
        NODELET_INFO("Video DEVICE IS %s",videoDevice);
        cam_ = new uvc_cam::Cam(videoDevice, mode, newconfig.width, newconfig.height, newconfig.frame_rate);
        printf("after cam creating, cam is at %p\n", cam_);

        camera_name_ = config_.camera_name ;
        NODELET_INFO("[%s] camera_name_=%s",config_.camera_name.c_str(), camera_name_.c_str()  );
        if (camera_name_ != camera_name_)
        {
            camera_name_ = camera_name_;
            /*  if (!cinfo_.setCameraName(camera_name_))
            {
                // GUID is 16 hex digits, which should be valid.
                // If not, use it for log messages anyway.
                ROS_WARN_STREAM("[" << camera_name_ << "] name not valid"
                                << " for camera_info_manger");
            }*/
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


void Uvc_cam_node::deal_memory(){
    if(final!=NULL){
        free(final);
    };
    final = (unsigned char *) malloc( sizeof( char) * config_width * config_height );
    if(final == NULL){
        printf("############# MALLOC FAILED !! ##########\n");
    }
}

bool Uvc_cam_node::copy_read(){
    std::unique_lock<std::mutex> lk(mutex);

    if(!lk.owns_lock()){
        try {
            lk.lock();
        } catch(const std::system_error& e) {
            std::cout << "coin2 Caught system_error with code " << e.code()
                      << " meaning " << e.what() << '\n';
        }
    }

    if(!new_frame){
        //printf("print until next frame - will wait\n");
        waiter.wait(lk);
        //printf("print until next frame - did wait\n");
    }
    //printf("before header creation , config width %d , config height %d,  bytes_used%d \n", config_width, config_height, bytes_used);
    imageIpl = cvCreateImageHeader(cvSize(config_width ,config_height), 8, 1);
    deal_memory();
    const int total = config_width*config_height*2;

    if(total!=bytes_used)
        return false;

    int j=0;
    //printf("copy read , config width %d , config height %d, total %d , bytes_used%d \n", config_width, config_height, total, bytes_used);
    for(int i=0; i< total; j++){
        final[j]=frame[i];
        i+=2;
    };
    new_frame=false;
    lk.unlock();

    return true;
}

std::unique_lock<std::mutex> Uvc_cam_node::getLock(){
    std::unique_lock<std::mutex> lk(mutex);
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
bool Uvc_cam_node::read() {
    bool success = true;
    if(config_width < 1 || config_height < 1 )
        return false;

    try
    {
        // Read data from the Camera
        //ROS_DEBUG_STREAM("[" << camera_name_ << "] reading data");
        //printf("Read data from the camera\n");
        //deal_memory();
        bool copy_worked = copy_read();
        if(!copy_worked)
            return false;

        //printf("deal with the frame\n");
        imageIpl->imageData = (char *)final;

        //ROS_DEBUG_STREAM("[" << camera_name_ << "] read returned");
    }
    catch (uvc_cam::Exception& e)
    {
        //ROS_WARN_STREAM("[" << camera_name_ << "] Exception reading data: " << e.what());
        printf("exception readind data %s", e.what());
        success = false;
    }
    return success;
}

/** Dynamic reconfigure callback
 *
 *  Called immediately when callback first defined. Called again
 *  when dynamic reconfigure starts or changes a parameter value.
 *
 *  @param newconfig new Config values
 *  @param level bit-wise OR of reconfiguration levels for all
 *               changed parameters (0xffffffff on initial call)
 **/

void Uvc_cam_node::reconfig(Config &newconfig, uint32_t level)
{

    nextconfig_ = newconfig;
    nextconfig_.device = config_.device;
    nextconfigLevel_ = level;
    printf("################################## reconfig call with level %u \n" , level);
    nextconfigUpdate = true;
    nextconfigCount ++;

}


void Uvc_cam_node::change_size(int width, int height){

    printf("############## changing size called %d %d %d \n", width, height, nextconfigUpdate);
    if(nextconfigUpdate == true){
        printf("change sizze called, but already called before \n");
        return;
    }
    nextconfig_ = Config(config_);
    nextconfig_.width = width;
    nextconfig_.height = height;
    nextconfigUpdate = true;

}

void Uvc_cam_node::apply_newconfig(Config &newconfig, uint32_t level)
{


    printf("### dynamic reconfigure level 0x%x \n", level);
    ROS_DEBUG("dynamic reconfigure level 0x%x", level);
    reconfigNumber++;
    // resolve frame ID using tf_prefix parameter
    if (newconfig.frame_id == "")
        newconfig.frame_id = "camera";

    ROS_DEBUG("dynamic reconfigure level 0x%x", level);
    std::string tf_prefix = tf::getPrefixParam(private_nh);
    ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
    newconfig.frame_id = tf::resolve(tf_prefix, newconfig.frame_id);

    if (state_ != Driver::CLOSED && (level & Levels::RECONFIGURE_CLOSE))
    {
        // must close the device before updating these parameters
        closeCamera();                  // state_ --> CLOSED
    }
    if (state_ == Driver::CLOSED)
    {
        // open with new values
        if (openCamera(newconfig))
        {
            // update camera name string
            newconfig.camera_name = camera_name_;
        }
    }


    if(reconfigNumber == 1 || config_.focus_auto != newconfig.focus_auto){
        try {
            NODELET_INFO("will try to set autofocus to %d \n " , newconfig.focus_auto);
            cam_->set_control( 0x009A090C , newconfig.focus_auto);
        } catch (uvc_cam::Exception& e) {
            ROS_ERROR_STREAM("Problem setting focus_auto. Exception was " << e.what());
        }
    }


    if(reconfigNumber == 1 || config_.focus_absolute != newconfig.focus_absolute){
        try {
            NODELET_INFO("will try to set focus_absolute to %d \n " , newconfig.focus_absolute);
            cam_->set_control( 0x009A090A  , newconfig.focus_absolute);
        } catch (uvc_cam::Exception& e) {
            ROS_ERROR_STREAM("Problem setting focus_absolute. Exception was " << e.what() << "value was" << newconfig.focus_absolute ) ;
        }
    }

    ///THIS IS A QUICK HACK TO GET EXPOSURE ON OUR CAMERA'S THIS DOES NOT WORK FOR ALL CAMERAS
    /*
       if(config_.exposure != newconfig.exposure){
       try {
       cam_->set_control(0x9a0901, newconfig.exposure);
       } catch (uvc_cam::Exception& e) {
       ROS_ERROR_STREAM("Problem setting exposure. Exception was " << e.what());
       }
       }
       if(config_.absolute_exposure != newconfig.absolute_exposure){
       try {
       cam_->set_control(0x9a0902, newconfig.absolute_exposure);
       } catch (uvc_cam::Exception& e) {
       ROS_ERROR_STREAM("Problem setting absolute exposure. Exception was " << e.what());
       }
       }
       if(config_.sharpness != newconfig.sharpness){
       try {
       cam_->set_control(0x98091b, newconfig.sharpness);
       } catch (uvc_cam::Exception& e) {
       ROS_ERROR_STREAM("Problem setting sharpness. Exception was " << e.what());
       }
       }
       if(config_.power_line_frequency != newconfig.power_line_frequency){
       try {
       cam_->set_control(0x980918, newconfig.power_line_frequency);
       } catch (uvc_cam::Exception& e) {
       ROS_ERROR_STREAM("Problem setting powerline frequency. Exception was " << e.what());
       }
       }
       if(config_.white_balance_temperature != newconfig.white_balance_temperature){
       try {
       cam_->set_control(0x98090c, newconfig.white_balance_temperature);
       } catch (uvc_cam::Exception& e) {
       ROS_ERROR_STREAM("Problem setting white balance temperature. Exception was " << e.what());
       }
       }
       if(config_.gain != newconfig.gain){
       try {
       cam_->set_control(0x980913, newconfig.gain);
       } catch (uvc_cam::Exception& e) {
       ROS_ERROR_STREAM("Problem setting gain. Exception was " << e.what());
       }
       }
       if(config_.saturation != newconfig.saturation){
       try {
       cam_->set_control(0x980902, newconfig.saturation);
       } catch (uvc_cam::Exception& e) {
       ROS_ERROR_STREAM("Problem setting saturation. Exception was " << e.what());
       }
       }
       if(config_.contrast != newconfig.contrast){
       try {
       cam_->set_control(0x980901, newconfig.contrast);
       } catch (uvc_cam::Exception& e) {
       ROS_ERROR_STREAM("Problem setting contrast. Exception was " << e.what());
       }
       }
       if(config_.brightness != newconfig.brightness){
       try {
       cam_->set_control(0x980900, newconfig.brightness);
       } catch (uvc_cam::Exception& e) {
       ROS_ERROR_STREAM("Problem setting brightness. Exception was " << e.what());
       }
       }
     */
    /*  if (config_.camera_info_url != newconfig.camera_info_url){
    // set the new URL and load CameraInfo (if any) from it
    if (cinfo_.validateURL(newconfig.camera_info_url)){
    cinfo_.loadCameraInfo(newconfig.camera_info_url);
    }else{
    // new URL not valid, use the old one
    newconfig.camera_info_url = config_.camera_info_url;
    }
    }*/

    config_ = newconfig;

    config_width = newconfig.width;
    config_height = newconfig.height;

    nextconfigUpdate = false;
    printf("### dynamic reconfigure will unlock\n");

}


/*void Uvc_cam_node::setPublishingState(bool pPublishingState){
    publishingState = pPublishingState;

    spiningState = processingState or publishingState;
}

bool Uvc_cam_node::getPublishingState(){
    return publishingState;
}

void Uvc_cam_node::setProcessingState(bool pProcessingState){
    processingState = pProcessingState;

    spiningState = processingState or publishingState;
}

bool Uvc_cam_node::getProcessingState(){
    return processingState;
}*/

void Uvc_cam_node::setSpinningState(bool pSpinningState){
    spiningState = pSpinningState;
}

bool Uvc_cam_node::getSpinningState(){
    return spiningState;
}

cv_bridge::CvImage out_msg;

void Uvc_cam_node::publish_frame(Mat frame ) {



    out_msg.encoding = sensor_msgs::image_encodings::MONO8;
    out_msg.image    = frame;
    out_msg.header.seq = publishedFrame;

    /*if (out_msg.header.frame_id != 0)
                {delete[] out_msg.header.frame_id;}
*/

    char frame_id[20];
    sprintf(frame_id , "%d" , publishedFrame);
    out_msg.header.frame_id = frame_id;
    out_msg.header.stamp = ros::Time::now();
    /*
            ci->header.seq = publish_frame;
            ci->header.frame_id = publish_frame;
            ci->header.stamp = out_msg.header.stamp;
*/
    publishedFrame++;
    //pub.publish(out_msg.toImageMsg(), ci);
    imagePublisher.publish(out_msg.toImageMsg() );
}


/** driver main spin loop */\
void Uvc_cam_node::spin() {
    NODELET_INFO("[UVC Cam Nodelet] Main Loop...");
    printf("inside spinOnce loop \n");
    // define segmentation fault handler in case the underlying uvc driver craps out
    //signal(SIGSEGV, &sigsegv_handler);

    // Define dynamic reconfigure callback, which gets called
    // immediately with level 0xffffffff.  The reconfig() method will
    // set initial parameter values, then open the device if it can.
    dynamic_reconfigure::Server<Config> srv;
    dynamic_reconfigure::Server<Config>::CallbackType f
            = boost::bind(&Uvc_cam_node::reconfig, this, _1, _2);
    srv.setCallback(f);

    reconfig(config_, 0xffffffff);
    apply_newconfig(config_ , 0xffffffff);




   // boost::thread    m_Thread;
  //  m_Thread = boost::thread(&Uvc_cam_node::loop_grab_image, this);
    spinThread_ = boost::shared_ptr< boost::thread >
      (new boost::thread(boost::bind(&Uvc_cam_node::loop_grab_image, this)));
    //loop_grab_image_thread = std::thread( &Uvc_cam_node::loop_grab_image  , this);

    while (ros::ok())
    {
        if (state_ != Driver::CLOSED)
        {
            //Check nb subscribers
            if(image_pub_.getNumSubscribers() > 0 && !nextconfigUpdate)
            {
                // printf("uvc cam looping spining %d , processing %d , publishing %d " , spiningState , processingState, publishingState);
                if (spiningState == true  && nextconfigCount > 0 && read() )
                {

                    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
                    cv_ptr->header.stamp = ros::Time::now();
                    cv_ptr->header.frame_id = config_.frame_id;
                    cv_ptr->encoding = sensor_msgs::image_encodings::MONO8; //"mono8";//"bgr8";
                    //config_.format_mode
                    cv_ptr->image = Mat( imageIpl );


                    image_pub_.publish(cv_ptr->toImageMsg());
                    //Vec3b bgrPixel = frame.at<Vec3b>(0,0);
                    //printf("Frame is available, pixel(0,0)=%ui\n",bgrPixel[0]);
                    // imshow("Window",frame);
                    // waitKey(1);
                    // if (processingState) { process_frame(frame ) ; }
                    //if (publishingState ) { publish_frame(frame) ; }


                }else{
                    usleep(1000);//1ms sleep
                }
            }else{
                usleep(1000);
            }
        }
        //printf("Looping ONCE");
        //printf("Looping here too with state=%d",state_);
        ros::spinOnce();
    }

    //Camera is closed within destructor
    //closeCamera();
}

//Must die
/*Uvc_cam_node* bean;
ros::ServiceServer servicePublishingStateSetter;
ros::ServiceServer servicePublishingStateGetter;

bool SetPublishingState( hyve_msg::SetState::Request &req ,hyve_msg::SetState::Response  &res  ){
    res.state = req.state;
    bean->setPublishingState( req.state);
    return true;
}

bool GetPublishingState( hyve_msg::GetState::Request &req ,hyve_msg::GetState::Response  &res  ){

    res.state = bean->getPublishingState();
    return true;
}*/

bool Uvc_cam_node::service_SetSpinningState( hyve_msg::SetState::Request &req ,hyve_msg::SetState::Response  &res  ){
    res.state = req.state;
    setSpinningState(req.state);
    return true;
}

bool Uvc_cam_node::service_GetSpinningState( hyve_msg::GetState::Request &req ,hyve_msg::GetState::Response  &res  ){
    res.state =getSpinningState();
    return true;
}

void Uvc_cam_node::configureSpinning(ros::NodeHandle& nh){
    NODELET_INFO("[UVCCam Nodelet] START configuring spinning state");
    ros::NodeHandle& ph = getMTPrivateNodeHandle();
    serviceSpinningStateSetter = ph.advertiseService("setSpinningState", &Uvc_cam_node::service_SetSpinningState, this);
    serviceSpinningStateGetter = ph.advertiseService("getSpinningState", &Uvc_cam_node::service_GetSpinningState, this);

    NODELET_INFO("[UVCCam Nodelet] DONE");
}

/*void Uvc_cam_node::configurePublishing( ros::NodeHandle &nh){
    printf("configurePublishing called \n");
    image_transport::ImageTransport it(nh);
    imagePublisher = it.advertise("image", 1);

    servicePublishingStateSetter = nh.advertiseService("setPublishingState", SetPublishingState);
    servicePublishingStateGetter = nh.advertiseService("getPublishingState", GetPublishingState);
    bean = this;

    printf("configurePublishing called - end \n");
}*/

PLUGINLIB_DECLARE_CLASS(cam_nodelet,Uvc_cam_node, cam_nodelet::Uvc_cam_node, nodelet::Nodelet);
}
