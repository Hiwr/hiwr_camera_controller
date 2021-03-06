## *********************************************************
## 
## File autogenerated for the hiwr_camera_controller package 
## by the dynamic_reconfigure package.
## Please do not edit.
## 
## ********************************************************/

##**********************************************************
## Software License Agreement (BSD License)
##
##  Copyright (c) 2008, Willow Garage, Inc.
##  All rights reserved.
##
##  Redistribution and use in source and binary forms, with or without
##  modification, are permitted provided that the following conditions
##  are met:
##
##   * Redistributions of source code must retain the above copyright
##     notice, this list of conditions and the following disclaimer.
##   * Redistributions in binary form must reproduce the above
##     copyright notice, this list of conditions and the following
##     disclaimer in the documentation and/or other materials provided
##     with the distribution.
##   * Neither the name of the Willow Garage nor the names of its
##     contributors may be used to endorse or promote products derived
##     from this software without specific prior written permission.
##
##  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
##  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
##  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
##  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
##  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
##  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
##  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
##  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
##  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
##  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
##  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
##  POSSIBILITY OF SUCH DAMAGE.
##**********************************************************/

from dynamic_reconfigure.encoding import extract_params

inf = float('inf')

config_description = {'upper': 'DEFAULT', 'lower': 'groups', 'srcline': 233, 'name': 'Default', 'parent': 0, 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT', 'field': 'default', 'state': True, 'parentclass': '', 'groups': [], 'parameters': [{'srcline': 259, 'description': 'The name of the camera (serial number or otherwise)', 'max': '', 'cconsttype': 'const char * const', 'ctype': 'std::string', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'camera_name', 'edit_method': '', 'default': 'camera', 'level': 3, 'min': '', 'type': 'str'}, {'srcline': 259, 'description': 'The v4l2 device path', 'max': '', 'cconsttype': 'const char * const', 'ctype': 'std::string', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'device', 'edit_method': '', 'default': '/dev/video0', 'level': 3, 'min': '', 'type': 'str'}, {'srcline': 259, 'description': 'The format the camera should be captured in', 'max': 3, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'format_mode', 'edit_method': "{'enum_description': 'The camera capture format', 'enum': [{'srcline': 44, 'description': 'Capture in RGB mode', 'srcfile': '/home/jules/git_hiwr/src/hiwr-camera-controller/cfg/UVCCam.cfg', 'cconsttype': 'const int', 'value': 1, 'ctype': 'int', 'type': 'int', 'name': 'MODE_RGB'}, {'srcline': 45, 'description': 'Capture in YUYV mode', 'srcfile': '/home/jules/git_hiwr/src/hiwr-camera-controller/cfg/UVCCam.cfg', 'cconsttype': 'const int', 'value': 2, 'ctype': 'int', 'type': 'int', 'name': 'MODE_YUYV'}, {'srcline': 46, 'description': 'Capture in MJPG mode', 'srcfile': '/home/jules/git_hiwr/src/hiwr-camera-controller/cfg/UVCCam.cfg', 'cconsttype': 'const int', 'value': 3, 'ctype': 'int', 'type': 'int', 'name': 'MODE_MJPEG'}]}", 'default': 1, 'level': 3, 'min': 1, 'type': 'int'}, {'srcline': 259, 'description': 'Width of image in pixels.', 'max': 4000, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'width', 'edit_method': '', 'default': 640, 'level': 3, 'min': 0, 'type': 'int'}, {'srcline': 259, 'description': 'Height of image in pixels.', 'max': 4000, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'height', 'edit_method': '', 'default': 480, 'level': 3, 'min': 0, 'type': 'int'}, {'srcline': 259, 'description': 'ROS tf frame of reference, resolved with tf_prefix unless absolute.', 'max': '', 'cconsttype': 'const char * const', 'ctype': 'std::string', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'frame_id', 'edit_method': '', 'default': 'camera', 'level': 3, 'min': '', 'type': 'str'}, {'srcline': 259, 'description': 'Camera speed (frames per second).', 'max': 240.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'frame_rate', 'edit_method': '', 'default': 15.0, 'level': 3, 'min': 1.875, 'type': 'double'}, {'srcline': 259, 'description': 'Camera calibration URL for this video_mode (uncalibrated if null).', 'max': '', 'cconsttype': 'const char * const', 'ctype': 'std::string', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'camera_info_url', 'edit_method': '', 'default': '', 'level': 0, 'min': '', 'type': 'str'}, {'srcline': 259, 'description': 'absolute exposure of the camera', 'max': 2147483647, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'absolute_exposure', 'edit_method': '', 'default': 2089, 'level': 0, 'min': -2147483648, 'type': 'int'}, {'srcline': 259, 'description': 'menu selection for the exposure mode of the camera', 'max': 2147483647, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'exposure', 'edit_method': '', 'default': 1, 'level': 0, 'min': -2147483648, 'type': 'int'}, {'srcline': 259, 'description': 'sharpness of the camera image', 'max': 2147483647, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'sharpness', 'edit_method': '', 'default': 142, 'level': 0, 'min': -2147483648, 'type': 'int'}, {'srcline': 259, 'description': 'menu selection for the power line frequency', 'max': 2147483647, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'power_line_frequency', 'edit_method': '', 'default': 0, 'level': 0, 'min': -2147483648, 'type': 'int'}, {'srcline': 259, 'description': 'white balance of the camera', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'white_balance_temperature', 'edit_method': '', 'default': True, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 259, 'description': 'gain of the camera', 'max': 2147483647, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'gain', 'edit_method': '', 'default': 82, 'level': 0, 'min': -2147483648, 'type': 'int'}, {'srcline': 259, 'description': 'saturation of the camera', 'max': 2147483647, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'saturation', 'edit_method': '', 'default': 50, 'level': 0, 'min': -2147483648, 'type': 'int'}, {'srcline': 259, 'description': 'contrast of the camera', 'max': 2147483647, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'contrast', 'edit_method': '', 'default': 50, 'level': 0, 'min': -2147483648, 'type': 'int'}, {'srcline': 259, 'description': 'brightness of the camera', 'max': 2147483647, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'brightness', 'edit_method': '', 'default': 66, 'level': 0, 'min': -2147483648, 'type': 'int'}, {'srcline': 259, 'description': 'flag - focus auto or not', 'max': 2147483647, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'focus_auto', 'edit_method': '', 'default': 1, 'level': 0, 'min': -2147483648, 'type': 'int'}, {'srcline': 259, 'description': 'focus absolute', 'max': 2147483647, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'focus_absolute', 'edit_method': '', 'default': 0, 'level': 0, 'min': -2147483648, 'type': 'int'}], 'type': '', 'id': 0}

min = {}
max = {}
defaults = {}
level = {}
type = {}
all_level = 0

#def extract_params(config):
#    params = []
#    params.extend(config['parameters'])    
#    for group in config['groups']:
#        params.extend(extract_params(group))
#    return params

for param in extract_params(config_description):
    min[param['name']] = param['min']
    max[param['name']] = param['max']
    defaults[param['name']] = param['default']
    level[param['name']] = param['level']
    type[param['name']] = param['type']
    all_level = all_level | param['level']

UVCCam_MODE_RGB = 1
UVCCam_MODE_YUYV = 2
UVCCam_MODE_MJPEG = 3
