#include <condition_variable>
#include <mutex>
#include <stdio.h>
#include <iostream>

#include <ST/CaptureSession.h>
#include <ST/Utilities.h>
#include "register.hpp"

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/Imu.h>


#define DEFAULT_FRAME_ID 	"imu"

bool Flag_Acc = false;
bool Flag_Gyro = false;
double timenow = 0; // initial time of the system
double dt = 0; // initial time of the sensor
double biasT = 0; // bias between sensor time and ros time
double const g2ms2 = -9.81;
double Framerate_double_sync = 10.0;
double Framerate_double_depth = 10.0;
double Framerate_double_vis = 10.0;
double Framerate_double_ir = 10.0;
double Framerate_double_depth_ir = 10.0;
double Framerate_double_ir_inv = 0.1;
double lastTime_ir = 0;
bool ir_depth_diff_rate_flag = false;
class SessionDelegate : public ST::CaptureSessionDelegate {
    private:
        std::mutex lock;
        std::condition_variable cond;
        std::string frame_id_;

        bool ready = false;
        bool done = false;

        ros::Publisher depth_image_pub_;
        ros::Publisher depth_info_pub_;

        ros::Publisher depth_color_aligned_pub_;
        ros::Publisher depth_color_aligned_info_pub_;

        ros::Publisher depth_ir_aligned_pub_;
        ros::Publisher depth_ir_aligned_info_pub_;

        ros::Publisher visible_image_pub_;
        ros::Publisher visible_info_pub_;

        ros::Publisher left_image_pub_;
        ros::Publisher left_info_pub_;

        ros::Publisher right_image_pub_;
        ros::Publisher right_info_pub_;

        ros::Publisher imu_pub_;
        sensor_msgs::Imu imu_;

        sensor_msgs::ImagePtr imageFromDepthFrame(const std::string& frame_id, const ST::DepthFrame& f)
        {
            sensor_msgs::ImagePtr msg(new sensor_msgs::Image);

            msg->header.frame_id = frame_id;
/*
            if (dt==0) {
              dt = f.timestamp();
	      timenow = ros::Time::now().toSec();
	      timenow2 = ST::getTimestampNow();	
              printf("====================>>>>>    %lf,%lf,%lf \n",timenow,timenow2,dt);

            }
*/

            msg->header.stamp.fromSec(f.timestamp()  + biasT);

            msg->encoding = "16UC1";
            msg->height = f.height();
            msg->width = f.width();
            msg->step = 2*f.width();
            msg->is_bigendian = 0;

            msg->data.resize(msg->height * msg->step);
            uint16_t* data_as_shorts = reinterpret_cast<uint16_t*>(msg->data.data());
            std::transform (&f.depthInMillimeters()[0], &f.depthInMillimeters()[0]+(f.height()*f.width()), &data_as_shorts[0],
                [](float f)->uint16_t{
                    if(std::isnormal(f))
                    {
                        return f;
                    }
                    return 0;
                }
            );

            return msg;
        }


        template <class frameType>
        sensor_msgs::CameraInfoPtr infoFromFrame(const std::string& frame_id, const frameType& f, int width_scale=1)
        {
          sensor_msgs::CameraInfoPtr info(new sensor_msgs::CameraInfo);
          info->header.frame_id = frame_id;
          info->header.stamp.fromSec(f.timestamp()  + biasT);
          info->height = f.intrinsics().height;
          info->width = f.intrinsics().width/width_scale;
          info->distortion_model = "plumb_bob";
          info->D = { f.intrinsics().k1, f.intrinsics().k2, f.intrinsics().p1, f.intrinsics().p2, f.intrinsics().k3 };
          info->K = { f.intrinsics().fx, 0, f.intrinsics().cx,
                      0, f.intrinsics().fy, f.intrinsics().cy,
                      0, 0, 1. };
          info->P = { f.intrinsics().fx, 0, f.intrinsics().cx, 0,
                      0, f.intrinsics().fy, f.intrinsics().cy, 0,
                      0, 0, 1., 0 };
          info->R = { 1, 0, 0,
                      0, 1, 0,
                      0, 0, 1 };

          return info;
        }

        sensor_msgs::ImagePtr imageFromVisibleFrame(const std::string& frame_id, const ST::ColorFrame& f)
        {
            sensor_msgs::ImagePtr msg(new sensor_msgs::Image);
            msg->header.frame_id = frame_id;
            msg->header.stamp.fromSec(f.timestamp()  + biasT);

            int num_channels = f.rgbSize()/(f.width()*f.height());

            if( num_channels == 3){
                msg->encoding = "rgb8";
            } else {
                msg->encoding = "mono8";
            }

            msg->height = f.height();
            msg->width = f.width();
            msg->step = f.rgbSize()/f.height();
            msg->is_bigendian = 0;

            msg->data.resize(msg->height * msg->step);

            std::copy(&f.rgbData()[0], &f.rgbData()[0]+f.rgbSize(), &msg->data[0]);

            return msg;
        }

        std::vector<sensor_msgs::ImagePtr> imagesFromInfraredFrame(const std::string& left_frame_id, const std::string& right_frame_id, const ST::InfraredFrame& f, bool as_8bit=false)
        {


            int single_frame_width = f.width()/2;

            sensor_msgs::ImagePtr right(new sensor_msgs::Image);
            right->header.frame_id = right_frame_id;
            right->header.stamp.fromSec(f.timestamp()  + biasT);

            right->encoding = "mono16";
            right->height = f.height();
            right->width = single_frame_width;
            right->step = 2*single_frame_width;
            right->is_bigendian = 0;
            right->data.resize(right->height * right->step);
            uint16_t* right_as_shorts = reinterpret_cast<uint16_t*>(right->data.data());

	    sensor_msgs::ImagePtr left(new sensor_msgs::Image);
            left->header.frame_id = left_frame_id;
            left->header.stamp.fromSec(f.timestamp()  + biasT);

            left->encoding = "mono16";
            left->height = f.height();
            left->width = single_frame_width;
            left->step = 2*single_frame_width;
            left->is_bigendian = 0;
            left->data.resize(left->height * left->step);
            uint16_t* left_as_shorts = reinterpret_cast<uint16_t*>(left->data.data());


            if(as_8bit){ // This is very expensive to use
                left->data.resize(left->height*left->width);
                right->data.resize(right->height*right->width);

                left->encoding = "mono8";
                right->encoding = "mono8";

                left->step = single_frame_width;
                right->step = single_frame_width;

                for(int j = 0; j < f.height(); j++){
                    for(int i = 0; i < f.width(); i++){
                        if(i < single_frame_width){
                            right->data[right->width*j + i] = f.data()[f.width()*j + i] >> 3; // Reduce from 11 bits to 8 bits
                        } else {
                            left->data[left->width*j + i - single_frame_width] = f.data()[f.width()*j + i] >> 3; // Reduce from 11 bits to 8 bits
                        }
                    }
                }
            }
            else // Copy as full 16 bit
            {
                for(int j = 0; j < f.height(); j++){
                    // Right is first and main imager
                    std::copy(&f.data()[f.width()*j], &f.data()[f.width()*j + 0] + single_frame_width, &right_as_shorts[right->width*j]);
                    std::copy(&f.data()[f.width()*j+single_frame_width], &f.data()[f.width()*j] + 2*single_frame_width, &left_as_shorts[left->width*j]);
                }
            }

            return {left, right};
        }

        void publishDepthFrame(const ST::DepthFrame& f)
        {
            if(not f.isValid() or depth_image_pub_.getNumSubscribers() == 0)
            {
                return;
            }
            std::string depth_frame_id = "camera_depth_optical_frame";
            depth_image_pub_.publish(imageFromDepthFrame(depth_frame_id, f));
            depth_info_pub_.publish(infoFromFrame(depth_frame_id, f));
        }

        void publishVisibleFrame(const ST::ColorFrame& f)
        {
            if(not f.isValid() or visible_image_pub_.getNumSubscribers() == 0)
            {
                return;
            }
            std::string visible_frame_id = "camera_visible_optical_frame";
            visible_image_pub_.publish(imageFromVisibleFrame(visible_frame_id, f));
            visible_info_pub_.publish(infoFromFrame(visible_frame_id, f));
        }

        void publishInfraredFrame(const ST::InfraredFrame& f, bool as_8bit=false)
        {

            if ((not f.isValid()) or (left_image_pub_.getNumSubscribers() == 0 and right_image_pub_.getNumSubscribers() == 0))
            {
                return;
            }

	    // if IR rate is diffrent than depth rate (Should be smaller) then we publish it only when needed
	    if (ir_depth_diff_rate_flag && f.timestamp() - lastTime_ir < (Framerate_double_ir_inv) ) {
		//ROS_INFO_STREAM("<---------- --------->");
		return;
		}
	    else {
		lastTime_ir = f.timestamp();
		}

            std::string left_frame_id = "camera_left_optical_frame";
            std::string right_frame_id = "camera_depth_optical_frame";

            std::vector<sensor_msgs::ImagePtr> frames = imagesFromInfraredFrame(left_frame_id, right_frame_id, f, as_8bit);
            left_image_pub_.publish(frames[0]);
            left_info_pub_.publish(infoFromFrame(left_frame_id, f, 2));

            right_image_pub_.publish(frames[1]);
            right_info_pub_.publish(infoFromFrame(right_frame_id, f, 2));
        }

        void publishDepthAligned(const ST::DepthFrame& depth, const ST::ColorFrame& visual)
        {
            if(not depth.isValid() or not visual.isValid() or depth_color_aligned_pub_.getNumSubscribers() == 0){
                return;
            }

            sensor_msgs::ImagePtr msg(new sensor_msgs::Image);
            msg->header.frame_id = "camera_visible_optical_frame";
            msg->header.stamp.fromSec(depth.timestamp()  + biasT);
            msg->encoding = "16UC1";
            msg->height = visual.height();
            msg->width = visual.width();
            msg->step = 2*visual.width();
            msg->is_bigendian = 0;
            register_convert(depth, visual, msg->data);

            // Camera info is same as visual, but with depth timestamp
            auto info = infoFromFrame(msg->header.frame_id, visual);
            info->header.stamp = msg->header.stamp;

            depth_color_aligned_pub_.publish(msg);
            depth_color_aligned_info_pub_.publish(info);
        }

        void publishDepthIRAligned(const ST::DepthFrame& depth, const ST::InfraredFrame& ir)
        {
            if(not depth.isValid() or not ir.isValid() or depth_ir_aligned_pub_.getNumSubscribers() == 0){
                return;
            }

            sensor_msgs::ImagePtr msg(new sensor_msgs::Image);
            msg->header.frame_id = "camera_depth_optical_frame";
            msg->header.stamp.fromSec(depth.timestamp()  + biasT);
            msg->encoding = "16UC1";
            msg->height = ir.height();
            msg->width = ir.width()/2;
            msg->step = 2*msg->width;
            msg->is_bigendian = 0;
            register_convert(depth, ir, msg->data);

            // Camera     std::string frame_id;

            auto info = infoFromFrame(msg->header.frame_id, ir, 2);
            info->header.stamp = msg->header.stamp;

            depth_ir_aligned_pub_.publish(msg);
            depth_ir_aligned_info_pub_.publish(info);
        }

        void publishIMU()
      	{
      	  imu_.header.frame_id = DEFAULT_FRAME_ID;
          if (Flag_Acc && Flag_Gyro) {
            imu_pub_.publish(imu_);
            Flag_Acc = false;
            Flag_Gyro = false;
          }

      	}

        void handleAccel(const ST::AccelerometerEvent &accelEvent)
        { // to avoid using TFs --> rotate to x-- camera forward, z-- up, y-- the rest
          ROS_DEBUG_STREAM_THROTTLE(1.0, "Structure_Core_Node" << ": handleAccel");
          Flag_Acc = true;
          imu_.linear_acceleration.x = -accelEvent.acceleration().z * g2ms2;  
          imu_.linear_acceleration.y = accelEvent.acceleration().y * g2ms2;
          imu_.linear_acceleration.z = accelEvent.acceleration().x * g2ms2;

          imu_.header.stamp.fromSec(accelEvent.timestamp()  + biasT);

          publishIMU();
        }

        void handleGyro(const ST::GyroscopeEvent &gyroEvent)
        { // to avoid using TFs --> rotate to x-- camera forward, z-- up, y-- the rest
          ROS_DEBUG_STREAM_THROTTLE(1.0, "Structure_Core_Node" << ": handleGyro");
          Flag_Gyro = true;
          imu_.angular_velocity.x = -gyroEvent.rotationRate().z;
          imu_.angular_velocity.y = gyroEvent.rotationRate().y;
          imu_.angular_velocity.z = gyroEvent.rotationRate().x;

          imu_.header.stamp.fromSec(gyroEvent.timestamp() + biasT);

          publishIMU();

        }
    public:

        SessionDelegate(ros::NodeHandle& n, ros::NodeHandle& pnh)
        {
            ros::NodeHandle dn(n, "depth");
            depth_image_pub_ = dn.advertise<sensor_msgs::Image>("image", 10);
            depth_info_pub_ = dn.advertise<sensor_msgs::CameraInfo>("camera_info", 10);

            ros::NodeHandle da(n, "depth_aligned");
            depth_color_aligned_pub_ = da.advertise<sensor_msgs::Image>("image", 10);
            depth_color_aligned_info_pub_ = da.advertise<sensor_msgs::CameraInfo>("camera_info", 10);

            ros::NodeHandle di(n, "depth_ir_aligned");
            depth_ir_aligned_pub_ = di.advertise<sensor_msgs::Image>("image", 10);
            depth_ir_aligned_info_pub_ = di.advertise<sensor_msgs::CameraInfo>("camera_info", 10);

            ros::NodeHandle vn(n, "visible");
            visible_image_pub_ = vn.advertise<sensor_msgs::Image>("image_raw", 10);
            visible_info_pub_ = vn.advertise<sensor_msgs::CameraInfo>("camera_info", 10);

            ros::NodeHandle ln(n, "left");
            left_image_pub_ = ln.advertise<sensor_msgs::Image>("image_raw", 10);
            left_info_pub_ = ln.advertise<sensor_msgs::CameraInfo>("camera_info", 10);

            ros::NodeHandle rn(n, "right");
            right_image_pub_ = rn.advertise<sensor_msgs::Image>("image_raw", 10);
            right_info_pub_ = rn.advertise<sensor_msgs::CameraInfo>("camera_info", 10);

            ros::NodeHandle im(n, "imu_node");
            imu_pub_ = im.advertise<sensor_msgs::Imu>("imu",10);

        }

        void captureSessionEventDidOccur(ST::CaptureSession *, ST::CaptureSessionEventId event) override {
            printf("Received capture session event %d (%s)\n", (int)event, ST::CaptureSessionSample::toString(event));
            switch (event) {
                case ST::CaptureSessionEventId::Ready: {
                    std::unique_lock<std::mutex> u(lock);
                    ready = true;
                    cond.notify_all();
                } break;
                case ST::CaptureSessionEventId::Disconnected:
                case ST::CaptureSessionEventId::EndOfFile:
                case ST::CaptureSessionEventId::Error: {
                    std::unique_lock<std::mutex> u(lock);
                    done = true;
                    cond.notify_all();
                } break;
		case ST::CaptureSessionEventId::Streaming: {
		    printf("Event %d ===> Streaming \n", (int)event);
		} break;
                default:
                    printf("Event %d unhandled\n", (int)event);
            }
        }

        void captureSessionDidOutputSample(ST::CaptureSession *, const ST::CaptureSessionSample& sample) {
            //printf("Received capture session sample of type %d (%s)\n", (int)sample.type, ST::CaptureSessionSample::toString(sample.type));
            switch (sample.type) {
                case ST::CaptureSessionSample::Type::DepthFrame:
                    //printf("Depth frame: size %dx%d\n", sample.depthFrame.width(), sample.depthFrame.height());
                    publishDepthFrame(sample.depthFrame);
                    break;
                case ST::CaptureSessionSample::Type::VisibleFrame:
                    //printf("Visible frame: size %dx%d\n", sample.visibleFrame.width(), sample.visibleFrame.height());
                    publishVisibleFrame(sample.visibleFrame);
                    break;
                case ST::CaptureSessionSample::Type::InfraredFrame:
                    //printf("Infrared frame: size %dx%d\n", sample.infraredFrame.width(), sample.infraredFrame.height());
                    publishInfraredFrame(sample.infraredFrame);
                    break;
                case ST::CaptureSessionSample::Type::SynchronizedFrames:
                    {
                        publishDepthFrame(sample.depthFrame);

                        publishVisibleFrame(sample.visibleFrame);

                        publishInfraredFrame(sample.infraredFrame, false);

                        //publishDepthAligned(sample.depthFrame, sample.visibleFrame);

                        //publishDepthIRAligned(sample.depthFrame, sample.infraredFrame);
                    }
                    break;
                case ST::CaptureSessionSample::Type::AccelerometerEvent:
                    handleAccel(sample.accelerometerEvent);
                    //printf("Accelerometer event: [% .5f % .5f % .5f]\n", sample.accelerometerEvent.acceleration().x, sample.accelerometerEvent.acceleration().y, sample.accelerometerEvent.acceleration().z);
                    break;
                case ST::CaptureSessionSample::Type::GyroscopeEvent:
                    //printf("Gyroscope event: [% .5f % .5f % .5f]\n", sample.gyroscopeEvent.rotationRate().x, sample.gyroscopeEvent.rotationRate().y, sample.gyroscopeEvent.rotationRate().z);
                    handleGyro(sample.gyroscopeEvent);
                    break;
                default:
                    printf("Sample type unhandled\n");
            }
        }

        void waitUntilReady() {
            std::unique_lock<std::mutex> u(lock);
            cond.wait(u, [this]() {
                return ready;
            });
        }

        void waitUntilDone() {
            std::unique_lock<std::mutex> u(lock);
            cond.wait(u, [this]() {
                return done;
            });
        }
};

int main(int argc, char **argv) {

    ros::init(argc, argv, "structure_driver");


    ros::NodeHandle n;

    ros::NodeHandle pnh("~");

    ST::CaptureSessionSettings settings;
    settings.source = ST::CaptureSessionSourceId::StructureCore;
    /** @brief Set to true to deliver IMU events on a separate, dedicated background thread. Only supported for Structure Core, currently. */
    settings.lowLatencyIMU = true;
    /** @brief Set to true to apply a correction filter to the depth before streaming. This may effect performance. */
    settings.applyExpensiveCorrection = true;//ST::DepthFrame::applyExpensiveCorrection::True;
    /** @brief Set to true to enable depth streaming. */
    ros::param::param<bool>("~depth_enable",settings.structureCore.depthEnabled,false);
//    ros::param::param<bool>("~ir_enable",settings.structureCore.infraredEnabled,false);
    ros::param::param<bool>("~vis_enable",settings.structureCore.visibleEnabled,false);
    ROS_INFO_STREAM("vis_enable :  " << settings.structureCore.visibleEnabled);
    ros::param::param<bool>("~acc_enable",settings.structureCore.accelerometerEnabled,false);
    ros::param::param<bool>("~gyro_enable",settings.structureCore.gyroscopeEnabled,false);



    int param_depthres = 1;
    ros::param::param<int>("~configuration_res_depth",param_depthres,1);
    switch(param_depthres) {
      case 1 :
        settings.structureCore.depthResolution = ST::StructureCoreDepthResolution::QVGA;
        break;
      case 2 :
        settings.structureCore.depthResolution = ST::StructureCoreDepthResolution::VGA;
        break;
      case 3 :
        settings.structureCore.depthResolution = ST::StructureCoreDepthResolution::SXGA;
        break;
      default : settings.structureCore.depthResolution = ST::StructureCoreDepthResolution::Default;
    }


    int depthRangeMode = 1;
    ros::param::param<int>("~configuration_mode_depth",depthRangeMode,1);
    switch(depthRangeMode) {
      case 1 :
        settings.structureCore.depthRangeMode = ST::StructureCoreDepthRangeMode::VeryShort;
        break;
      case 2 :
        settings.structureCore.depthRangeMode = ST::StructureCoreDepthRangeMode::Short;
        break;
      case 3 :
        settings.structureCore.depthRangeMode = ST::StructureCoreDepthRangeMode::Medium;
        break;
      case 4 :
        settings.structureCore.depthRangeMode = ST::StructureCoreDepthRangeMode::Long;
        break;
      case 5 :
        settings.structureCore.depthRangeMode = ST::StructureCoreDepthRangeMode::VeryLong;
        break;
      case 6 :
        settings.structureCore.depthRangeMode = ST::StructureCoreDepthRangeMode::Hybrid;
        break;
      default :
        settings.structureCore.depthRangeMode = ST::StructureCoreDepthRangeMode::Default;
        break;
    }


 
    int dynamic_calibration_param = 1;
    ros::param::param<int>("~dynamic_calibration",dynamic_calibration_param,1);
    switch(dynamic_calibration_param) {
      case 1 :
        settings.structureCore.dynamicCalibrationMode = ST::StructureCoreDynamicCalibrationMode::Off;
        break;
      case 2 :
        settings.structureCore.dynamicCalibrationMode = ST::StructureCoreDynamicCalibrationMode::OneShotPersistent;
        break;
      case 3 :
        settings.structureCore.dynamicCalibrationMode = ST::StructureCoreDynamicCalibrationMode::ContinuousNonPersistent;
        break;
      default :
        settings.structureCore.dynamicCalibrationMode = ST::StructureCoreDynamicCalibrationMode::Off;
        break;
    }
   
    ros::param::param<bool>("~IR_auto_exposure",settings.structureCore.infraredAutoExposureEnabled,false);

    int infra_red_mode = 0;
    ros::param::param<int>("~infra_red_mode",infra_red_mode,0);
    switch(infra_red_mode) {
      case 0 :
	settings.structureCore.infraredEnabled = false;
        break;
      case 1 :
	settings.structureCore.infraredEnabled = true;
        settings.structureCore.infraredResolution = ST::StructureCoreInfraredResolution::Default;
        settings.structureCore.infraredMode = ST::StructureCoreInfraredMode::BothCameras;
        break;
      case 2 :
	settings.structureCore.infraredEnabled = true;
        settings.structureCore.infraredResolution = ST::StructureCoreInfraredResolution::Default;
        settings.structureCore.infraredMode =  ST::StructureCoreInfraredMode::LeftCameraOnly;
        break;
      case 3 :
	settings.structureCore.infraredEnabled = true;
        settings.structureCore.infraredResolution = ST::StructureCoreInfraredResolution::Default;
        settings.structureCore.infraredMode =  ST::StructureCoreInfraredMode::RightCameraOnly;
        break;
      default :
	settings.structureCore.infraredEnabled = false;
        break;
    }


    int imu_update_rate = 100;
    ros::param::param<int>("~imu_update_rate",imu_update_rate,100);
    switch(imu_update_rate) {
      case 100 :
        settings.structureCore.imuUpdateRate = ST::StructureCoreIMUUpdateRate::AccelAndGyro_100Hz;
        break;
      case 200 :
        settings.structureCore.imuUpdateRate = ST::StructureCoreIMUUpdateRate::AccelAndGyro_200Hz;
        break;
      case 800 :
        settings.structureCore.imuUpdateRate = ST::StructureCoreIMUUpdateRate::AccelAndGyro_800Hz;
        break;
      case 1000 :
        settings.structureCore.imuUpdateRate = ST::StructureCoreIMUUpdateRate::AccelAndGyro_1000Hz;
        break;
      default :
        settings.structureCore.imuUpdateRate = ST::StructureCoreIMUUpdateRate::AccelAndGyro_100Hz;
        break;
    }



    settings.structureCore.visibleResolution = ST::StructureCoreVisibleResolution::_640x480;
    /** @brief Set to true to apply gamma correction to incoming visible frames. */
    settings.structureCore.visibleApplyGammaCorrection = true;


    /** @brief The target stream rate for IMU data. (gyro and accel) */
    /** @brief Serial number of sensor to stream. If null, the first connected sensor will be used. */
    settings.structureCore.sensorSerial = nullptr;
    /** @brief Maximum amount of time (in milliseconds) to wait for a sensor to connect before throwing a timeout error. */
    settings.structureCore.sensorInitializationTimeout = 6000;


    /** @brief Set to true to enable frame synchronization between visible or color and depth. */
    ros::param::param<bool>("~frameSyncEnabled",settings.frameSyncEnabled,true);




	// Framerate_double_sync is used unly id frameSync is enabled

    if (settings.frameSyncEnabled) {
        ros::param::param<double>("~Framerate_sync",Framerate_double_sync,10.0);
    	settings.structureCore.infraredFramerate = (float) Framerate_double_sync;
    	settings.structureCore.depthFramerate = (float) Framerate_double_sync;
    	settings.structureCore.visibleFramerate = (float) Framerate_double_sync;
	ROS_INFO_STREAM("Framerate (ALL SYNC) :  " << Framerate_double_sync);
    }
    else {
	// depth and IR must have the same frame rate. we can reduce the IR frame rate when publishing
	ros::param::param<double>("~Framerate_depth",Framerate_double_depth,10.0);
	ros::param::param<double>("~Framerate_vis",Framerate_double_vis,10.0);
	ros::param::param<double>("~Framerate_ir",Framerate_double_ir ,10.0);
	
	// TODO: take the maximum between ir and depth
	if (Framerate_double_depth != Framerate_double_ir) ir_depth_diff_rate_flag = true;
	Framerate_double_depth_ir = Framerate_double_depth;
	if (Framerate_double_depth < Framerate_double_ir) {
		Framerate_double_ir = Framerate_double_depth;
    		ROS_INFO_STREAM("!!!!!!!!! : IR frame rate should be smaller or eaual to depth frame rate ---- using depth frame rate for IR");
	}
    	settings.structureCore.infraredFramerate = (float) Framerate_double_depth_ir;
    	settings.structureCore.depthFramerate = (float) Framerate_double_depth_ir;
    	settings.structureCore.visibleFramerate = (float) Framerate_double_vis;
	ROS_INFO_STREAM("Framerate (DEPTH) :  " << Framerate_double_depth);
    	ROS_INFO_STREAM("Framerate (IR) :  " << Framerate_double_ir);
    	ROS_INFO_STREAM("Framerate (VIS) :  " << Framerate_double_vis);
    }
    Framerate_double_ir_inv = 1/Framerate_double_ir;



    double initialVisibleExposure_double = 0.015;
    double initialVisibleGain_double = 3.0;
    double initialInfraredExposure_double = 0.015;

    ros::param::param<double>("~initialVisibleExposure",initialVisibleExposure_double,0.015);
    settings.structureCore.initialVisibleExposure = (float) initialVisibleExposure_double;

    ros::param::param<double>("~initialVisibleGain",initialVisibleGain_double,3.0);
    settings.structureCore.initialVisibleGain = (float) initialVisibleGain_double;

    ros::param::param<double>("~initialInfraredExposure",initialInfraredExposure_double,0.015);
    settings.structureCore.initialInfraredExposure = (float) initialInfraredExposure_double;

    /** @brief The initial infrared gain to start streaming with. Can be 0, 1, 2, or 3. */
    //settings.structureCore.initialInfraredGain = 3.0f;
    ros::param::param<int>("~initialInfraredGain",settings.structureCore.initialInfraredGain,3);

    /** @brief Setting this to true will eliminate saturation issues, but might result in sparser depth. */
    settings.structureCore.disableInfraredIntensityBalance = true;
    /** @brief Setting this to true will reduce latency, but might drop more frame */
    settings.structureCore.latencyReducerEnabled = true;
    /** @brief Laser projector power setting from 0.0 to 1.0 inclusive. Projector will only activate if required by streaming configuration. */
    //settings.structureCore.initialProjectorPower = 1.0f;  // <--- CANCELED on 0.7.2
    //printf("%d,%d\n",time_begin.sec, time_begin.nsec);


    SessionDelegate delegate(n, pnh);
    ST::CaptureSession session;
    session.setDelegate(&delegate);
    if (!session.startMonitoring(settings)) {
        printf("Failed to initialize capture session\n");
        return 1;
    }

    printf("Waiting for session to become ready...\n");
    delegate.waitUntilReady();
    session.startStreaming();


    if (dt==0) {
      dt = ST::getTimestampNow();
      timenow = ros::Time::now().toSec();
      biasT = timenow - dt;      	
      printf("====================>>>>> \n rosTime: %lf \n structureTime: %lf \n biasT: %lf \n ====================>>>>> \n ",timenow,dt,biasT);

    }

    //while loop which waits 2 sec to reload exposure and gain
    while (ros::Time::now().toSec() - timenow < 0.5)  {
    };

    session.setInfraredCamerasExposureAndGain((float) settings.structureCore.initialInfraredExposure,(float) settings.structureCore.initialInfraredGain);

    ros::spin();

    session.stopStreaming();
    return 0;
}
