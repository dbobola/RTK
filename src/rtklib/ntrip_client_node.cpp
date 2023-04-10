#include "rtklib.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PointStamped.h"

#include "ros/ros.h"
#include "rtcm3_decoder.h"

void process_rtcm3_data(std::vector<uint8_t> data) {
    // create an RTCM3 decoder object
    rtcm3_decoder decoder;
    
    // decode the RTCM 3.x data
    if (!decoder.decode(data)) {
        ROS_WARN_STREAM("Failed to decode RTCM3 data");
        return;
    }
    
    // get the decoded RTCM 3.x message type and data payload
    uint32_t msg_type = decoder.get_message_type();
    std::vector<uint8_t> payload = decoder.get_message_payload();
    
    // process the decoded message based on its type
    switch (msg_type) {
        case 1005: // station coordinates
            // parse the station coordinates from the message payload
            double latitude = rtcm3_message_1005::get_latitude(payload);
            double longitude = rtcm3_message_1005::get_longitude(payload);
            double height = rtcm3_message_1005::get_height(payload);
            
            // use the station coordinates to calculate the correction base position
            calculate_correction_base_position(latitude, longitude, height);
            break;
            
        case 1019: // GPS ephemeris
            // parse the GPS ephemeris from the message payload
            gps_ephemeris eph = rtcm3_message_1019::get_gps_ephemeris(payload);
            
            // use the GPS ephemeris to calculate the satellite positions
            calculate_satellite_positions(eph);
            break;
            
        case 1074: // GPS MSM4
            // parse the GPS MSM4 (multi-signal message) from the message payload
            std::vector<gps_msm4> msm4 = rtcm3_message_1074::get_gps_msm4(payload);
            
            // use the MSM4 to calculate the carrier-phase and code-phase measurements
            calculate_carrier_phase_measurements(msm4);
            break;
            
        // add cases for other message types as needed
            
        default:
            ROS_WARN_STREAM("Unknown RTCM3 message type: " << msg_type);
            break;
    }
}

// Initialize RTKLIB processing options
rtk_t options = {0};

// Set up the publisher for the corrected GPS position
ros::Publisher pub;

// RTKLIB RTCM callback function
void rtcmCallback(const std_msgs::String::ConstPtr& msg);

int main(int argc, char **argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "rtklib_node");
    ros::NodeHandle nh;

    // Set up RTKLIB processing options
    set_default_opts(&options);

    // Configure the processing options
    options.mode = PMODE_SINGLE;
    options.navsys = SYS_GPS;
    options.nf = 1;
    options.posopt[0] = 1;

    // Subscribe to the RTCM data
    ros::Subscriber sub = nh.subscribe("/ntrip/rtcm", 1, rtcmCallback);

    // Set up the publisher for the corrected GPS position
    pub = nh.advertise<geometry_msgs::PointStamped>("/gps/rtk", 1);

    // Spin until shutdown
    ros::spin();

    return 0;
}

void rtcmCallback(const std_msgs::String::ConstPtr& msg)
{
    // Use RTKLIB library to process RTCM data and obtain corrected GPS positions
    obs_t obs = {0};
    nav_t nav = {0};
    rtcm_t rtcm = {0};
    rtcm.rtcmtype = 1004;
    rtcm.obs = &obs;
    rtcm.nav = &nav;
    rtcm.outtype = SOLF_LLH;
    input_rtcm3f(&rtcm, msg->data.c_str(), msg->data.size());
    int length = msg->data.length();
    double pos[3] = {0};
    int status = rtkpos(&options, &obs, &nav, NULL, pos, NULL, NULL);
    if (status > 0) {
        // Publish corrected GPS position in ROS message
        geometry_msgs::PointStamped msg_out;
        msg_out.header.stamp = ros::Time::now();
        msg_out.point.x = pos[0];
        msg_out.point.y = pos[1];
        msg_out.point.z = pos[2];
        pub.publish(msg_out);

        ROS_INFO("RTKLIB: Solution obtained. Lat: %.8f, Lon: %.8f, Height: %.4f",
            pos[0]*R2D, pos[1]*R2D, pos[2]);
    } else {
        ROS_WARN("RTKLIB: Failed to obtain a solution");
    }
}
