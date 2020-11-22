#include <ros/ros.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <dynamic_reconfigure/server.h>
#include <espros_tof_cam635/espros_tof_cam635Config.h>
#include "camera635_driver.h"
#include "tof_cam635_image.h"

using namespace ComLib;
using namespace std;

static ros::Publisher imagePublisher1;
static ros::Publisher imagePublisher2;
static ros::Publisher imageHeaderPublisher;
static ros::Publisher cameraInfoPublisher;
static ros::Publisher pointCloud2Publisher;
static ros::ServiceServer cameraInfoService;
static sensor_msgs::CameraInfo cameraInfo;
static Settings settings;

//===================================================================

void updateConfig(espros_tof_cam635::espros_tof_cam635Config &config, uint32_t level)
{
    settings.runVideo = false;
    (void) level; //unused parameter

    switch(config.image_type){
    case 0: settings.iType = TofCam635Image::TofCam635ImageType_e::TOFCAM635_IMAGE_GRAYSCALE;
        break;
    case 1: settings.iType = TofCam635Image::TofCam635ImageType_e::TOFCAM635IMAGE_DISTANCE;
       break;
    case 2: settings.iType = TofCam635Image::TofCam635ImageType_e::TOFCAM635_IMAGE_DISTANCE_AMPLITUDE;
       break;
    case 3: settings.iType = TofCam635Image::TofCam635ImageType_e::TOFCAM635_IMAGE_DISTANCE_GRAYSCALE;
       break;
    default: break;
    }

    settings.hdr = static_cast<uint>(config.hdr);
    settings.mode = static_cast<uint>(config.mode);
    settings.frameRate = config.frame_rate;
    settings.automaticIntegrationTime = config.automatic_integration_time;
    settings.integrationTimeATOF1  = static_cast<uint>(config.integration_time_0);
    settings.integrationTimeATOF2  = static_cast<uint>(config.integration_time_1);
    settings.integrationTimeATOF3  = static_cast<uint>(config.integration_time_2);
    settings.integrationTimeATOF4  = static_cast<uint>(config.integration_time_3);
    settings.integrationTimeBTOF1  = static_cast<uint>(config.integration_time_4);
    settings.integrationTimeBTOF2  = static_cast<uint>(config.integration_time_5);
    settings.integrationTimeGray   = static_cast<uint>(config.integration_time_gray);
    settings.kalmanThreshold = static_cast<uint>(config.temporal_filter_threshold_WFOV);
    settings.kalmanFactor  = config.temporal_filter_factor_WFOV;
    settings.kalmanThresholdSpot = static_cast<uint>(config.temporal_filter_threshold_NFOV);
    settings.kalmanFactorSpot  = config.temporal_filter_factor_NFOV;
    settings.averageFilter = config.spatial_average_filter;
    settings.minAmplitude1 = static_cast<uint>(config.min_amplitude_0);
    settings.minAmplitude2 = static_cast<uint>(config.min_amplitude_1);
    settings.minAmplitude3 = static_cast<uint>(config.min_amplitude_2);
    settings.minAmplitude4 = static_cast<uint>(config.min_amplitude_3);
    settings.minAmplitude5 = static_cast<uint>(config.min_amplitude_4);
    settings.offsetDistance = config.offset_distance;
    settings.lensCenterOffsetX = config.lensCenterOffsetX;
    settings.lensCenterOffsetY = config.lensCenterOffsetY;
    settings.enableCartesian = config.enable_cartesian;        
    settings.enableImages = config.enable_images;
    settings.enablePointCloud = config.enable_point_cloud;
    settings.enableImageHeader = config.enable_image_header;
    settings.roi_leftX   = static_cast<uint>(config.roi_left_x);
    settings.roi_topY    = static_cast<uint>(config.roi_top_y);
    settings.roi_rightX  = static_cast<uint>(config.roi_right_x);
    settings.roi_bottomY = static_cast<uint>(config.roi_bottom_y);
    settings.startStream = static_cast<uint>(config.start_stream);

    settings.roi_rightX  -= (settings.roi_rightX - settings.roi_leftX + 1) % 4;
    settings.roi_bottomY -= (settings.roi_bottomY - settings.roi_topY + 1) % 4;

    config.roi_right_x  = static_cast<int>(settings.roi_rightX);
    config.roi_bottom_y = static_cast<int>(settings.roi_bottomY);

    if(config.start_stream)
        settings.runVideo = true;

    settings.triggerSingleShot = config.trigger_single_shot;
    settings.updateParam = true;    
}

bool setCameraInfo(sensor_msgs::SetCameraInfo::Request& req, sensor_msgs::SetCameraInfo::Response& res)
{
    req.camera_info.width  = cameraInfo.width;
    req.camera_info.height = cameraInfo.height;
    req.camera_info.roi    = cameraInfo.roi;
    req.camera_info.K      = cameraInfo.K;
    req.camera_info.D      = cameraInfo.D;

    cameraInfoPublisher.publish(req.camera_info);

    res.success = true;
    res.status_message = "";
    return true;
}

void initialise()
{
    ros::NodeHandle nh("~");
    nh.param("port_name", settings.port_name, std::string("/dev/ttyACM0"));
    nh.param("frame_rate", settings.frameRate, 30.0);


    //advertise publishers    
    imagePublisher1 = nh.advertise<sensor_msgs::Image>("image_raw1", 1000);
    imagePublisher2 = nh.advertise<sensor_msgs::Image>("image_raw2", 1000);    
    pointCloud2Publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZI> > ("points", 100);
    cameraInfoPublisher  = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 1000);
    imageHeaderPublisher = nh.advertise<std_msgs::Int32MultiArray>("image_header", 1000);

    //advertise services
    cameraInfoService = nh.advertiseService("set_camera_info", setCameraInfo);

    settings.runVideo = false;
    settings.updateParam = false;

    ROS_INFO("Camera driver version 1.5.0");
}

//======================================================================

int main(int argc, char **argv)
{
    //if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) ros::console::notifyLoggerLevelsChanged();

    ros::init(argc, argv, "tof_cam635_node");

    dynamic_reconfigure::Server<espros_tof_cam635::espros_tof_cam635Config> server;
    dynamic_reconfigure::Server<espros_tof_cam635::espros_tof_cam635Config>::CallbackType f;
    f = boost::bind(&updateConfig, _1, _2);
    server.setCallback(f);

    initialise();

    Camera635Driver cameraDriver(imagePublisher1, imagePublisher2, imageHeaderPublisher, pointCloud2Publisher, cameraInfo, settings);

    while(ros::ok()){
        cameraDriver.update();        
        ros::spinOnce();       
    }
}






