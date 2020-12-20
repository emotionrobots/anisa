#ifndef CAMERA635_DRIVER_H
#define CAMERA635_DRIVER_H

#include "tof_cam635_image.h"
#include "communication_635.h"

#include <dynamic_reconfigure/server.h>
#include <espros_tof_cam635/espros_tof_cam635Config.h>

#include <vector>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/PointCloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Int32MultiArray.h>


struct Settings{

  std::string port_name;
  double frameRate;
  bool startStream;
  bool triggerSingleShot;
  bool runVideo;
  bool updateParam;
  uint  mode;
  uint  hdr;
  bool automaticIntegrationTime;
  uint integrationTimeATOF1;
  uint integrationTimeATOF2;
  uint integrationTimeATOF3;
  uint integrationTimeATOF4;
  uint integrationTimeBTOF1;
  uint integrationTimeBTOF2;
  uint integrationTimeGray;

  int modFrequency;
  double kalmanFactor;
  uint kalmanThreshold;
  double kalmanFactorSpot;
  uint kalmanThresholdSpot;
  bool averageFilter;
  uint minAmplitude1;
  uint minAmplitude2;
  uint minAmplitude3;
  uint minAmplitude4;
  uint minAmplitude5;
  int offsetDistance;
  int lensCenterOffsetX;
  int lensCenterOffsetY;
  bool enableCartesian;
  bool enableImages;
  bool enablePointCloud;
  bool enableImageHeader;

  uint roi_leftX;
  uint roi_topY;
  uint roi_rightX;
  uint roi_bottomY;

  ComLib::TofCam635Image::TofCam635ImageType_e iType;

};



class Camera635Driver
{

public:
    Camera635Driver(const ros::Publisher &imagePublisher1_, const ros::Publisher &imagePublisher2_, const ros::Publisher &imageHeaderPublisher_,
                    const ros::Publisher &pointCloud2Publisher_, sensor_msgs::CameraInfo &cameraInfo_, Settings &set_);

    ~Camera635Driver();

    void update();   
    void initCommunication();

private:

    uint hdr_last;
    uint leftX_last;
    uint topY_last;
    uint rightX_last;
    uint bootomY_last;

    double angle;
    bool lastSingleShot;
    bool lastStreaming;
    double framePeriod;
    ros::Time timeLast;
    uint frameSeq;
    static Settings *gSettings;
    const ros::Publisher &imagePublisher1;
    const ros::Publisher &imagePublisher2;
    const ros::Publisher &imageHeaderPublisher;
    const ros::Publisher &pointCloud2Publisher;    

    uint imageSize8;
    uint imageSize16_1;
    uint imageSize16_2;
    std::string strFrameID;
    sensor_msgs::Image img8;
    sensor_msgs::Image img16_1;
    sensor_msgs::Image img16_2;

    sensor_msgs::CameraInfo &cameraInfo;    
    ComLib::Communication635 communication;
    std_msgs::Int32MultiArray imageHeaderMsg;

    int distortionTableSize;
    int numCols;
    int numRows;
    double lensAngle[101];
    double rp[101];
    double xUA[160][60];
    double yUA[160][60];
    double zUA[160][60];
    int oldLensCenterOffsetX;
    int oldLensCenterOffsetY;

    void updateData();
    void setParameters();    
    void publishImageHeader(std::shared_ptr<ComLib::TofCam635Image> image);
    void updateCameraInfo(std::shared_ptr<ComLib::TofCam635Image> image);
    void updateGrayscaleFrame(std::shared_ptr<ComLib::TofCam635Image> image);
    void updateDistanceFrame(std::shared_ptr<ComLib::TofCam635Image> image);
    void updateDistanceAmplitudeFrame(std::shared_ptr<ComLib::TofCam635Image> image);
    void updateDistanceGrayscaleFrame(std::shared_ptr<ComLib::TofCam635Image> image);

    //Cartesian transformation functions
    void initLensDistortionTable();
    double interpolate(double x_in, double x0, double y0, double x1, double y1);
    double getAngle(double x, double y, double sensorPointSizeMM);
    void initLensTransform(double sensorPointSizeMM, int width, int height, int offsetX, int offsetY);
    void transformPixel(uint srcX, uint srcY, double srcZ, double &destX, double &destY, double &destZ);

};


#endif // CAMERA635_DRIVER_H
