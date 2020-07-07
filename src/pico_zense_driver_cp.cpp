#include "Vzense_enums.h"
#include "pico_zense_driver.h"
#include "sensor_msgs/PointCloud.h"

namespace autolabor_driver {
    PicoZenseDriver::PicoZenseDriver() {

    }

    PicoZenseDriver::~PicoZenseDriver() {
        if (device_handle_) {
            Ps2_StopStream(device_handle_, session_index_);
            Ps2_CloseDevice(device_handle_);
            Ps2_Shutdown();
        }
    }

    bool PicoZenseDriver::init() {
        // Init SDK
        status_ = Ps2_Initialize();
        if (status_ != PsRetOK) {
            ROS_ERROR("PsInitialize error!");
            return false;
        }
        // Get Count
        uint32_t device_count = 0;
        status_ = Ps2_GetDeviceCount(&device_count);
        if (status_ != PsRetOK) {
            ROS_ERROR("PsGetDeviceCount error!");
            return false;
        }
        // Open Device
        if (device_count > 0 && Ps2_GetDeviceInfo(&device_info_, device_index_) == PsRetOK) { ;
            status_ = Ps2_OpenDevice(device_info_.uri, &device_handle_);
            if (status_ != PsRetOK) {
                ROS_ERROR("PsOpenDevice error!");
                return false;
            }
        } else {
            ROS_ERROR("No device!");
            return false;
        }
        // Set Properties
        Ps2_StartStream(device_handle_, session_index_);
        // Set Data Mode
        if ((status_ = Ps2_SetDataMode(device_handle_, session_index_, PsDepthAndIR_30)) != PsRetOK) {
            ROS_ERROR("Ps2_SetDataMode error %d", status_);
            return false;
        }

        Ps2_SetDepthRange(device_handle_, session_index_, PsMidRange);
        Ps2_SetSpatialFilterEnabled(device_handle_, session_index_, true);
        Ps2_SetTimeFilterEnabled(device_handle_, session_index_, true);
        Ps2_SetComputeRealDepthCorrectionEnabled(device_handle_, session_index_, true);
        Ps2_SetSynchronizeEnabled(device_handle_, session_index_, false);
        Ps2_SetDepthDistortionCorrectionEnabled(device_handle_, session_index_, true);
        Ps2_SetThreshold(device_handle_, session_index_, 50);

        return true;
    }

    void PicoZenseDriver::run() {
        ros::NodeHandle n;
        ros::Publisher pointcloud_pub = n.advertise<sensor_msgs::PointCloud>("point_cloud", 10);
        if (init()) {
            PsFrame depth_frame = {0};
            size_t len = 640 * 480;
            PsVector3f point_cloud[len];

            sensor_msgs::PointCloud point_msg;
            point_msg.header.frame_id = "map";

            while (true) {
                if (Ps2_ReadNextFrame(device_handle_, session_index_, &ready_) == PsRetOK) {
                    if ((status_ = Ps2_GetFrame(device_handle_, session_index_, PsDepthFrame, &depth_frame)) ==
                        PsRetOK) {
                        if (Ps2_ConvertDepthFrameToWorldVector(device_handle_, session_index_, depth_frame,
                                                               point_cloud) == PsRetOK) {
                            point_msg.header.stamp = ros::Time::now();
                            point_msg.points.resize(len);
                            for (int i = 0; i < len; i++) {
                                point_msg.points[i].x = point_cloud[i].x / 1000.0;
                                point_msg.points[i].y = point_cloud[i].y / 1000.0;
                                point_msg.points[i].z = point_cloud[i].z / 1000.0;
                            }
                            pointcloud_pub.publish(point_msg);
//                            uint16_t threshold = 0;
//                            Ps2_GetThreshold(device_handle_, session_index_, &threshold);
//                            ROS_INFO("Ps2_GetThreshold : %d", threshold);
                        }
                    } else {
                        ROS_ERROR("get data error : %d !", status_);
                    }
                } else {
                    ROS_INFO("ERROR CODE %d", Ps2_ReadNextFrame(device_handle_, session_index_, &ready_));
                }
                usleep(100000);
            }
        } else {
            ROS_ERROR_STREAM("Init error!");
        }
    }
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "PicoZenseDriver");
    autolabor_driver::PicoZenseDriver picoZenseDriver;
    picoZenseDriver.run();
    return 0;
}