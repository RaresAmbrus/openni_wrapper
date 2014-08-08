#include "OpenniWrapperNode.h"

#include <sstream>

using namespace openni;
using namespace std;
using namespace cv;

const int OpenniWrapperNode::openni_wrapper_max_devices = 1;

OpenniWrapperNode::OpenniWrapperNode(ros::NodeHandle nh, ros::NodeHandle private_nh)
//    m_ColorCallback(nh,"rgb/image_raw","rgb/camera_info", true, false),
//    m_DepthCallback(nh, true, false)
{
    m_nodeHandle = nh;
    m_privateNodeHandle = private_nh;

    // dynamic reconfigure
    // check how many cameras have been defined
    m_DevicesDefined = 0;
    m_bHwDepthRegister = true; // enabled by default

    for (size_t i=0; i<openni_wrapper_max_devices;i++)
    {
        std::string deviceName;
        ostringstream deviceNameStream;
        deviceNameStream<<"camera";
        if (i>0) {    deviceNameStream<<i; }

        bool found = m_privateNodeHandle.getParam(deviceNameStream.str(),deviceName);
        if (found)
        {
            if (deviceName != "false")
            {
                m_DevicesDefined++;
                m_vCameraNamespace.push_back(deviceName);
            }
        } else {
//            break;
        }
    }

    if (m_vCameraNamespace.size() == 0)
    {
        ROS_INFO_STREAM("---------------------------- No devices defined. Exitting.-------------------------------");
        exit(-1);
    }


    ROS_INFO_STREAM("Devices defined "<<m_DevicesDefined);
    for (size_t i=0; i<m_DevicesDefined;i++)
    {
//        ROS_INFO_STREAM(m_vCameraNamespace[i]);
        ColorCallback* colorCB = new ColorCallback(private_nh, m_vCameraNamespace[i], true, false);
        DepthCallback* depthCB = new DepthCallback(private_nh, m_vCameraNamespace[i], true, false);

        m_vDepthCallback.push_back(depthCB);
        m_vColorCallback.push_back(colorCB);
    }

    cb = boost::bind(&OpenniWrapperNode::configCallback, this, _1, _2);
    dr_srv.setCallback(cb);

}

OpenniWrapperNode::~OpenniWrapperNode()
{

}


void OpenniWrapperNode::initializeOpenni()
{
    Status rc = OpenNI::initialize();
    if (rc != STATUS_OK)
    {
        ROS_INFO_STREAM("Initialize failed "<<OpenNI::getExtendedError());
        return;
    } else {
        ROS_INFO_STREAM("Openni initialized");
    }



    OpenNI::addDeviceConnectedListener(&m_devicePrinter);
    OpenNI::addDeviceDisconnectedListener(&m_devicePrinter);
    OpenNI::addDeviceStateChangedListener(&m_devicePrinter);

    openni::Array<openni::DeviceInfo> deviceList;
    openni::OpenNI::enumerateDevices(&deviceList);
    for (int i = 0; i < deviceList.getSize(); ++i)
    {
        ROS_INFO_STREAM("Device \""<<deviceList[i].getUri()<<"\" already connected\n");
    }

    m_vDevice.resize(deviceList.getSize());
    m_vColor.resize(deviceList.getSize());
    m_vDepth.resize(deviceList.getSize());

    ROS_INFO_STREAM("Number of devices connected "<<deviceList.getSize());

    for (size_t i=0; i<m_DevicesDefined;i++)
    {
        if (i+1>deviceList.getSize())
        {
            ROS_INFO_STREAM("Name "<<m_vCameraNamespace[i]<<" defined but the device is not connected.");
            break;
        }

        m_vDevice[i] = new Device();
        rc = m_vDevice[i]->open(deviceList[i].getUri());
        if (rc != STATUS_OK)
        {
            ROS_INFO_STREAM("Couldn't open device "<<OpenNI::getExtendedError());
            return;
        } else {
            ROS_INFO_STREAM("Opened device "<<deviceList[0].getUri()<<"   namespace "<<m_vCameraNamespace[i]);
        }

        string devicesSupported="This device supports the following sensors ";
        bool sensor = m_vDevice[i]->hasSensor(SENSOR_IR); if (sensor) devicesSupported+="IR ";
        sensor = m_vDevice[i]->hasSensor(SENSOR_COLOR); if (sensor) devicesSupported+="COLOR ";
        sensor = m_vDevice[i]->hasSensor(SENSOR_DEPTH); if (sensor) devicesSupported+="DEPTH ";
        ROS_INFO_STREAM(devicesSupported);


        rc = m_vDevice[i]->setDepthColorSyncEnabled(true);
        // handle rc
        if (rc != STATUS_OK)
        {
            ROS_INFO_STREAM("Could not set the depth-color sync. Some error occured");
        }

        if (m_vDevice[i]->getSensorInfo(SENSOR_DEPTH) != NULL)
        {
            m_vDepth[i] = new VideoStream;
            rc = m_vDepth[i]->create(*m_vDevice[i], SENSOR_DEPTH);
            if (rc != STATUS_OK)
            {
                ROS_INFO_STREAM("Couldn't create depth stream " << OpenNI::getExtendedError());
            }
        }
        VideoMode depthVideoMode = m_vDepth[i]->getVideoMode();
        depthVideoMode.setResolution(640,480);
        rc = m_vDepth[i]->setVideoMode(depthVideoMode);
        if (rc != STATUS_OK)
        {
            ROS_INFO_STREAM("Couldn't set increased resolution for depth stream  "<<OpenNI::getExtendedError());
        }

        rc = m_vDepth[i]->start();
        if (rc != STATUS_OK)
        {
            ROS_INFO_STREAM("Couldn't start the depth stream " << OpenNI::getExtendedError());
        }

        if (m_vDevice[i]->getSensorInfo(SENSOR_COLOR) != NULL)
        {
            m_vColor[i] = new VideoStream;
            rc = m_vColor[i]->create(*m_vDevice[i], SENSOR_COLOR);
            if (rc != STATUS_OK)
            {
                ROS_INFO_STREAM("Couldn't create color stream " << OpenNI::getExtendedError());
            }
        }
        VideoMode colorVideoMode = m_vColor[i]->getVideoMode();
        colorVideoMode.setResolution(640,480);
        rc = m_vColor[i]->setVideoMode(colorVideoMode);
        if (rc != STATUS_OK)
        {
            ROS_INFO_STREAM("Couldn't set increased resolution for color stream " << OpenNI::getExtendedError());
        }

        rc = m_vColor[i]->start();
        if (rc != STATUS_OK)
        {
            ROS_INFO_STREAM("Couldn't start the color stream " << OpenNI::getExtendedError());
        }

        // Register to new frame
        m_vDepth[i]->addNewFrameListener(m_vDepthCallback[i]);
        m_vColor[i]->addNewFrameListener(m_vColorCallback[i]);
        
        if (m_bHwDepthRegister)
        {
            bool registrationSupported = m_vDevice[i]->isImageRegistrationModeSupported(IMAGE_REGISTRATION_DEPTH_TO_COLOR);
            if(registrationSupported)
            {
                ROS_INFO_STREAM("Hardware image registration enabled");
                rc = m_vDevice[i]->setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);
                // handle ret
                if (rc != STATUS_OK)
                {
                    ROS_INFO_STREAM("Could not set the hardware image registration on. Some error occured  "<<rc);
                }
            } else {
                ROS_INFO_STREAM("Hardware image registration NOT SUPPORTED");
            }
        } else {
            rc = m_vDevice[i]->setImageRegistrationMode(IMAGE_REGISTRATION_OFF);
            if (rc != STATUS_OK)
            {
                ROS_INFO_STREAM("Could not set the hardware image registration off. Some error occured  "<<rc);
            } else {
                ROS_INFO_STREAM("Hardware image registration turned OFF");
            }
        }
    }
}

void OpenniWrapperNode::terminateOpenni()
{
    ROS_INFO_STREAM("Shutting down Openni driver ");




    for (size_t i=0; i<m_DevicesDefined; i++)
    {
        m_vDepth[i]->removeNewFrameListener(m_vDepthCallback[i]);
        m_vColor[i]->removeNewFrameListener(m_vColorCallback[i]);

        m_vDepth[i]->stop();
        m_vDepth[i]->destroy();
        m_vColor[i]->stop();
        m_vColor[i]->destroy();
        m_vDevice[i]->close();

        delete m_vDepth[i];
        delete m_vColor[i];
        delete m_vDevice[i];

        delete m_vColorCallback[i];
        delete m_vDepthCallback[i];
    }

    OpenNI::shutdown();
}

void OpenniWrapperNode::configCallback(openni_wrapper::dynamic_parametersConfig &config, uint32_t level)
{
//    ROS_INFO_STREAM("Dynamic reconfigure "<<level);
    for (auto depthCallback : m_vDepthCallback) {
        depthCallback->setUndistortDepth(config.correct_depth);
    }

    if (m_bHwDepthRegister == config.hw_depth_registration)
    {
        return; // same value, nothing to do
    }
    m_bHwDepthRegister = config.hw_depth_registration;

    for (size_t i=0; i<m_DevicesDefined;i++)
    {
        if (m_bHwDepthRegister)
        {
            bool registrationSupported = m_vDevice[i]->isImageRegistrationModeSupported(IMAGE_REGISTRATION_DEPTH_TO_COLOR);
            if(registrationSupported)
            {
                ROS_INFO_STREAM("Hardware image registration enabled");
                Status rc = m_vDevice[i]->setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);
                // handle ret
                if (rc != STATUS_OK)
                {
                    ROS_INFO_STREAM("Could not set the hardware image registration on. Some error occured  "<<rc);
                }
            } else {
                ROS_INFO_STREAM("Hardware image registration NOT SUPPORTED");
            }
        } else {
            Status rc = m_vDevice[i]->setImageRegistrationMode(IMAGE_REGISTRATION_OFF);
            if (rc != STATUS_OK)
            {
                ROS_INFO_STREAM("Could not set the hardware image registration off. Some error occured  "<<rc);
            } else {
                ROS_INFO_STREAM("Hardware image registration turned OFF");
            }
        }
    }

}
