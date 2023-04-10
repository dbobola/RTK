// Include RTKLIB library headers
#include "rtklib.h"

// Initialize RTKLIB processing options
rtk_t options = {0};
options.rtk_mode = RTK_MODE_STATIC;
options.soltype = SOLF_LLH;
options.navsys = SYS_GPS;

// Parse RTCM message using RTKLIB library
obs_t obs = {0};
nav_t nav = {0};
rtcm_t rtcm = {0};
rtcm.rtcmtype = 1004;
rtcm.obs = &obs;
rtcm.nav = &nav;
rtcm.outtype = SOLF_LLH;
input_rtcm3f(&rtcm, message, length, &options);

// Use RTKLIB library to process RTCM data and obtain corrected GPS positions
double pos[3] = {0};
int status = rtkpos(&options, &obs, &nav, NULL, pos, NULL, NULL);
if (status > 0) {
    // Publish corrected GPS position in ROS message
    geometry_msgs::PointStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.point.x = pos[0];
    msg.point.y = pos[1];
    msg.point.z = pos[2];
    pub.publish(msg);
}
