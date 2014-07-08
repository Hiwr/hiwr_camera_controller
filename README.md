hiwr\_camera\_controller
=============================================== 
The hiwr\_camera\_common package aims to retrieve video stream from any connected camera.

The behavior may be undefined when using two or more identical cameras.

Contributing
----------------------

Contributions via pull request are welcome and may be included under the
same license as below.

Copyright
----------------------

hiwr\_camera\_controller, except where otherwise noted, is released under the
[Apache License 2.0](http://www.apache.org/licenses/LICENSE-2.0.html).
See the LICENSE file located in the root directory.

Build
----------------------
It requires hyve\_msg.

Execution
----------------------

To start hiwr\_camera\_controller, do the following (assuming, you
have a working ROS core running):

   Launch using roslaunch:

   > roslaunch hyve\_camera_common camera.launch


Node
----------------------

### Subscribed Topics


### Published Topics

- `/uvc_cam_node/output_video`
      *  video stream (sensor_msgs/Image)


### Parameters

- `device_name ` (String)
 * Name of the camera (ex: HD Pro Webcam C920)

- `absolute_exposure` (Int, default= 2089)
 
- `camera_info_url (String)`

- `camera_name` (String)
 * Name for dynamic reconfigure

- `brightness` (Int, default=66 )
 
- `contrast` (Int, default=50)
 
- `exposure` (Int, default=1)
 
- `focus_absolute: 0` (Int, default=0)
 
- `focus_auto: 1` (Int, default=1)
 
- `format_mode: 0`(Int, default=0)
 
- `frame_id` (String)
 
- `frame_rate` (Int, default=12)
 
- `gain`(Int, default=82)
 
- `height`(Int, default=480)
 
- `power_line_frequency`(Int, default=0)
 
- `saturation`(Int, default=50)
 
- `sharpness` (Int, default=142)
 
- `white_balance_temperature` (Boolean, default= true)
 
- `width` (int, default =640)