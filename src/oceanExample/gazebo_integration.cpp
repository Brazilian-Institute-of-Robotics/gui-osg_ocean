// * Copyright (c) 1982, 1986, 1990, 1991, 1993
// *      The Regents of the University of California.  All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without
// * modification, are permitted provided that the following conditions
// * are met:
// * 1. Redistributions of source code must retain the above copyright
// *    notice, this list of conditions and the following disclaimer.
// * 2. Redistributions in binary form must reproduce the above copyright
// *    notice, this list of conditions and the following disclaimer in the
// *    documentation and/or other materials provided with the distribution.
// * 3. All advertising materials mentioning features or use of this software
// *    must display the following acknowledgement:
// *      This product includes software developed by the University of
// *      California, Berkeley and its contributors.
// * 4. Neither the name of the University nor the names of its contributors
// *    may be used to endorse or promote products derived from this software
// *    without specific prior written permission.
// *
// * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
// * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
// * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
// * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
// * SUCH DAMAGE.

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
#include "gazebo_integration.h"


std::string USER = "baqueiro";

std::string FILE_PATH_ROV = "/home/" + USER + "/dev/src/osg_gazebo_integration/models/cougarxt_base.osg";
std::string FILE_PATH_TANK = "/home/" + USER + "/dev/src/osg_gazebo_integration/models/tank.osg";
std::string FILE_PATH_THRUSTER = "/home/" + USER +"/dev/src/osg_gazebo_integration/models/prop.stl";
std::string FILE_PATH_FJ_BODY = "/home/" + USER +"/dev/src/osg_gazebo_integration/models/flexjoint6_body.stl";
std::string FILE_PATH_FJ_COVERRUBBER = "/home/" + USER + "/dev/src/osg_gazebo_integration/models/cover_rubber_bumps.osg";
std::string FILE_PATH_TOOL_RADIALMOVINGBASE = "/home/" + USER +"/dev/src/osg_gazebo_integration/models/RadialMovingBase.stl";
std::string FILE_PATH_TOOL_ROTATIONALMOVE = "/home/" + USER +"/dev/src/osg_gazebo_integration/models/RotationalMove_simple.stl";
std::string FILE_PATH_TOOL_ROTATIONALSTATIC = "/home/" + USER +"/dev/src/osg_gazebo_integration/models/RotationalStatic_simple.stl";

/* Links from cover rubber and riser extension are not being published in /gazebo/link_states.
   TO DO: Find a workaround for this, since we need their poses. */


// Just the constructor. Check if ROS is ok and creates the transforms. The loadLink method will
// add the node (given the mesh path) as child of the transform and then add the transform as child
// of the root.
CallbackAdapter::CallbackAdapter(osg::ref_ptr<osg::Group> root) {
    if (ros::ok()) {

        // Transforms instantiation
        rov_transform_ = new osg::PositionAttitudeTransform();
        tank_transform_ = new osg::PositionAttitudeTransform();
        thruster0_transform_ = new osg::PositionAttitudeTransform();
        thruster1_transform_ = new osg::PositionAttitudeTransform();
        thruster2_transform_ = new osg::PositionAttitudeTransform();
        thruster3_transform_ = new osg::PositionAttitudeTransform();
        thruster4_transform_ = new osg::PositionAttitudeTransform();
        thruster5_transform_ = new osg::PositionAttitudeTransform();
        fj_body_transform_ = new osg::PositionAttitudeTransform();
        fj_coverrubber_transform_ = new osg::PositionAttitudeTransform();
        fj_riser_transform_ = new osg::PositionAttitudeTransform();
        tool_radialmovingbase_transform_ = new osg::PositionAttitudeTransform();
        tool_rotationalmove_transform_ = new osg::PositionAttitudeTransform();
        tool_rotationalstatic_transform_ = new osg::PositionAttitudeTransform();

        this->root_ = root;

        // Load links
        this->loadLink("dfki_tank::tank_body", FILE_PATH_TANK, this->tank_transform_.get());
            //
        this->loadLink("cougarxt::cougarxt/base_link", FILE_PATH_ROV, this->rov_transform_.get());
        this->loadLink("cougarxt::cougarxt/thruster_0", FILE_PATH_THRUSTER, this->thruster0_transform_.get());
        this->loadLink("cougarxt::cougarxt/thruster_1", FILE_PATH_THRUSTER, this->thruster1_transform_.get());
        this->loadLink("cougarxt::cougarxt/thruster_2", FILE_PATH_THRUSTER, this->thruster2_transform_.get());
        this->loadLink("cougarxt::cougarxt/thruster_3", FILE_PATH_THRUSTER, this->thruster3_transform_.get());
        this->loadLink("cougarxt::cougarxt/thruster_4", FILE_PATH_THRUSTER, this->thruster4_transform_.get());
        this->loadLink("cougarxt::cougarxt/thruster_5", FILE_PATH_THRUSTER, this->thruster5_transform_.get());

        this->loadLink("flexjoint_0::flexjoint/fj_body_0_link", FILE_PATH_FJ_BODY, this->fj_body_transform_.get());
        this->loadLink("flexjoint_0::flexjoint/cover_rubber_0_link", FILE_PATH_FJ_COVERRUBBER, this->fj_coverrubber_transform_.get());

        /* Radius, length and origin offset values here are not thrustworthy because they do not reflect the real values in the URDF file */
        /*                                                                             radius length */
        shape_ = new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3(0.0f, 0.0f, -2.5f), 0.15f, 5.0f));
        this->loadLink(shape_.get(), this->fj_riser_transform_.get());

        this->loadLink("flexjoint_0::RotationalUnitStatic", FILE_PATH_TOOL_ROTATIONALSTATIC, this->tool_rotationalstatic_transform_.get());
        this->loadLink("flexjoint_0::RotationalUnitMove", FILE_PATH_TOOL_ROTATIONALMOVE, this->tool_rotationalmove_transform_.get());
        this->loadLink("flexjoint_0::RadialMovingBase", FILE_PATH_TOOL_RADIALMOVINGBASE, this->tool_radialmovingbase_transform_.get());
        
        this->startROSClient();

    }
    else { // The code never enters here. (why?)
        std::cout << "ERROR: ROS is not online" << std::endl;
    }
}


CallbackAdapter::~CallbackAdapter() {}

void CallbackAdapter::operator()(osg::Node* node, osg::NodeVisitor* nv) {
    updateLinksPosition();
    osg::NodeCallback::operator()(node, nv);
}


// Define from which the service the client
void CallbackAdapter::startROSClient() {
    osg_client_link_ = nh_.serviceClient<gazebo_msgs::GetLinkState>("gazebo/get_link_state");
}


// Add the node (given the mesh path) as child of a transform, and the transform as child of the root
void CallbackAdapter::loadLink(std::string link_name, std::string mesh_path, osg::PositionAttitudeTransform* osg_transform) {
    if ( osgDB::readNodeFile(mesh_path) != NULL) {
        osg_transform->addChild(osgDB::readNodeFile(mesh_path));
    }
    else {
        std::cout << "Error loading link \"" << link_name << "\"." << std::endl;
    }
    this->root_->addChild(osg_transform);
}

// Overloaded function in case the link is a Geode
/* void CallbackAdapter::loadLink(osg::ShapeDrawable shape, osg::PositionAttitudeTransform* osg_transform) { */
void CallbackAdapter::loadLink(osg::Drawable* shape, osg::PositionAttitudeTransform* osg_transform) {
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;

    geode->addDrawable(shape);
    /* geode->addDrawable(new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3(0.0f, 0.0f, 0.0f), 0.2f, 2.0f))); */

    osg_transform->addChild(geode.get());
    this->root_->addChild(osg_transform);
}


// Update the class attribute gazebo_link_state_ according to Gazebo's link_state
void CallbackAdapter::updateLinkPosition(std::string link_name, osg::PositionAttitudeTransform* osg_transform) {
    this->gazebo_link_state_.request.link_name = link_name;
    this->gazebo_link_state_.request.reference_frame = "";

    osg_client_link_.call(gazebo_link_state_);

    if (gazebo_link_state_.response.success) {
        osg_transform->setPosition(osg::Vec3(
            gazebo_link_state_.response.link_state.pose.position.x,
            gazebo_link_state_.response.link_state.pose.position.y,
            gazebo_link_state_.response.link_state.pose.position.z));

        osg_transform->setAttitude(osg::Quat(
            gazebo_link_state_.response.link_state.pose.orientation.x,
            gazebo_link_state_.response.link_state.pose.orientation.y,
            gazebo_link_state_.response.link_state.pose.orientation.z,
            gazebo_link_state_.response.link_state.pose.orientation.w));
    } else {
        std::cout << "Error getting link state" << std::endl;
    }
}


// Calls updateLinkPosition for every link
void CallbackAdapter::updateLinksPosition() {
    // ROV
    this->updateLinkPosition("cougarxt::cougarxt/base_link", rov_transform_.get());
    this->updateLinkPosition("cougarxt::cougarxt/thruster_0", thruster0_transform_.get());
    this->updateLinkPosition("cougarxt::cougarxt/thruster_1", thruster1_transform_.get());
    this->updateLinkPosition("cougarxt::cougarxt/thruster_2", thruster2_transform_.get());
    this->updateLinkPosition("cougarxt::cougarxt/thruster_3", thruster3_transform_.get());
    this->updateLinkPosition("cougarxt::cougarxt/thruster_4", thruster4_transform_.get());
    this->updateLinkPosition("cougarxt::cougarxt/thruster_5", thruster5_transform_.get());

    // Flexjoint
    this->updateLinkPosition("flexjoint_0::flexjoint/fj_body_0_link", fj_body_transform_.get());
    this->updateLinkPosition("flexjoint_0::flexjoint/cover_rubber_0_link", fj_coverrubber_transform_.get());
    this->updateLinkPosition("flexjoint_0::flexjoint/fj_extension_0_link", fj_riser_transform_.get());

    // Tool
    this->updateLinkPosition("flexjoint_0::RotationalUnitStatic", tool_rotationalstatic_transform_.get());
    this->updateLinkPosition("flexjoint_0::RotationalUnitMove", tool_rotationalmove_transform_.get());
    this->updateLinkPosition("flexjoint_0::RadialMovingBase", tool_radialmovingbase_transform_.get());
}
