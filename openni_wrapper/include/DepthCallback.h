#ifndef __DEPTH_CALLBACK
#define __DEPTH_CALLBACK

#include <stdio.h>

#include "OpenNI.h"
#include "ros/ros.h"

#include <clams/discrete_depth_distortion_model.h>

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
#include "openni_wrapper/dynamic_parametersConfig.h"

class DepthCallback : public openni::VideoStream::NewFrameListener
{
public:
    DepthCallback(ros::NodeHandle aRosNode, std::string camNamespace="camera", bool publishRosMessage = true, bool createCVwin = false);
    void onNewFrame(openni::VideoStream& stream);
    void analyzeFrame(const openni::VideoFrameRef& frame);
    bool        saveOneFrame, saveFrameSequence, publishRosMessage, createCVWindow, undistortDepth;
    std::string m_CameraNamespace;

    void configCallback(openni_wrapper::dynamic_parametersConfig &config, uint32_t level);

private:
    openni::VideoFrameRef m_frame;
    ros::NodeHandle       m_RosNode;
    ros::Publisher        m_RosPublisher;
    ros::Publisher        m_RosCameraInfoPublisher;
    clams::DiscreteDepthDistortionModel*           m_Dddm;
    dynamic_reconfigure::Server<openni_wrapper::dynamic_parametersConfig> dr_srv;
    dynamic_reconfigure::Server<openni_wrapper::dynamic_parametersConfig>::CallbackType cb;

};


#endif
