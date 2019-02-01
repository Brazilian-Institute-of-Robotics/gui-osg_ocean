#ifndef GAZEBO_INTEGRATION_H
#define GAZEBO_INTEGRATION_H

#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osg/Group>
#include <osg/PositionAttitudeTransform>
#include <ros/ros.h>
#include <iostream>
#include <osg/NodeCallback>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetLinkState.h>
#include <osg/ShapeDrawable>
#include <osg/Geode>

class CallbackAdapter : public osg::NodeCallback {

    private:

        // ROS-related attributes
        ros::NodeHandle nh_;
        ros::ServiceClient osg_client_link_;
        gazebo_msgs::GetLinkState gazebo_link_state_;

        // OSG-related attributes
        osg::ref_ptr<osg::Group> root_;
        osg::ref_ptr<osg::PositionAttitudeTransform> rov_transform_;
        osg::ref_ptr<osg::PositionAttitudeTransform> tank_transform_;
        osg::ref_ptr<osg::PositionAttitudeTransform> thruster0_transform_;
        osg::ref_ptr<osg::PositionAttitudeTransform> thruster1_transform_;
        osg::ref_ptr<osg::PositionAttitudeTransform> thruster2_transform_;
        osg::ref_ptr<osg::PositionAttitudeTransform> thruster3_transform_;
        osg::ref_ptr<osg::PositionAttitudeTransform> thruster4_transform_;
        osg::ref_ptr<osg::PositionAttitudeTransform> thruster5_transform_;
        osg::ref_ptr<osg::PositionAttitudeTransform> fj_body_transform_;
        osg::ref_ptr<osg::PositionAttitudeTransform> fj_coverrubber_transform_;
        osg::ref_ptr<osg::PositionAttitudeTransform> fj_riser_transform_;
        osg::ref_ptr<osg::PositionAttitudeTransform> tool_radialmovingbase_transform_;
        osg::ref_ptr<osg::PositionAttitudeTransform> tool_rotationalmove_transform_;
        osg::ref_ptr<osg::PositionAttitudeTransform> tool_rotationalstatic_transform_;
        osg::ref_ptr<osg::ShapeDrawable> shape_;

    public:

        CallbackAdapter(osg::ref_ptr<osg::Group>);
        ~CallbackAdapter();
        void operator()(osg::Node*, osg::NodeVisitor*);
        void startROSClient();
        void loadLink(std::string, std::string, osg::PositionAttitudeTransform*);
        void loadLink(osg::Drawable*, osg::PositionAttitudeTransform*);
        void updateLinkPosition(std::string, osg::PositionAttitudeTransform*);
        void updateLinksPosition();
};

#endif