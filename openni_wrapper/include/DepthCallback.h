#ifndef __DEPTH_CALLBACK
#define __DEPTH_CALLBACK

#include <stdio.h>

#include "OpenNI.h"
#include "ros/ros.h"

#include <clams/discrete_depth_distortion_model.h>


class DepthCallback : public openni::VideoStream::NewFrameListener
{
public:
    DepthCallback(ros::NodeHandle aRosNode, std::string camNamespace="camera", bool publishRosMessage = true, bool createCVwin = false);
    void onNewFrame(openni::VideoStream& stream);
    void analyzeFrame(const openni::VideoFrameRef& frame);
    bool        saveOneFrame, saveFrameSequence, publishRosMessage, createCVWindow, undistortDepth;
    std::string m_CameraNamespace;

    void setUndistortDepth(bool undistort_depth);

private:
    openni::VideoFrameRef m_frame;
    ros::NodeHandle       m_RosNode;
    ros::Publisher        m_RosPublisher;
    ros::Publisher        m_RosCameraInfoPublisher;
    clams::DiscreteDepthDistortionModel*           m_Dddm;

};


#endif
