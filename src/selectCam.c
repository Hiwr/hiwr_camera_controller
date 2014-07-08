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
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <dirent.h>
#include <string.h>
#include <errno.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <fcntl.h>



char * findVideoDevice(const char *name)
{
    if(name == NULL) return NULL;
    DIR * d;
    char * video_pattern = "video";
    size_t video_pattern_len = strlen(video_pattern);
    size_t name_len = strlen(name);
    char * dir_name = "/dev/";
    char * result = NULL;

    printf("[UVC Cam ] Start finding video device\n");

    d = opendir (dir_name);
    if (! d) {
        fprintf (stderr, "Cannot open directory '%s': %s\n",
                 dir_name, strerror (errno));
        exit (EXIT_FAILURE);
    }
    while (1) {
        try{
            struct dirent * entry;
            entry = readdir (d);
            if (! entry) {
                break;
            }
            // if it is not a video device continue
            if(strncmp(video_pattern,entry->d_name,video_pattern_len)!=0) {
                continue;
            }
            // if video device name is not correct, coninue
            char * device = (char*) malloc(strlen(dir_name)+strlen(entry->d_name)+1);
            strcpy(device,dir_name);
            strcat(device, entry->d_name);
            printf("device=%s\n",device);
            printf("entry->d_name=%s\n",entry->d_name);
            int fd = open(device, O_RDWR|O_NONBLOCK);
            printf("FIND VIDEO DEVIC, FD is %d\n",fcntl(fd, F_GETFD));
            if (fd < 0) {
                printf("fd = -1\n");
                free(device);
                continue;
            }
            struct v4l2_capability info;
            memset(&info, 0x0, sizeof(info));
            int res = ioctl(fd, VIDIOC_QUERYCAP, &info);
            close(fd);
            if (res < 0 || strncmp(name,(char*)info.card,name_len)!=0)
            {
                printf("name=%s\n",name);
                printf("info.card=%s",(char*)info.card);
                printf("Error res is %d",res);
                free(device);
                continue;
            }
            result = device;
        }catch(...){
            continue;
        }
    }
    if (closedir (d)) {
        fprintf (stderr, "Could not close '%s': %s\n",
                 dir_name, strerror (errno));
        exit (EXIT_FAILURE);
    }
    printf("[UVC Cam] find video device OK result is %s\n", result);
    return result;
}
/*
int main(int argc, char *argv[] )
{

  char * device = findVideoDevice("Microsoft® LifeCam Cinema(TM)");

  if(device == NULL) return -1;
  fprintf (stdout, "%s\n", device);
  free(device);
  return 0;
}*/
