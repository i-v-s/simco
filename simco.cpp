#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <svo_msgs/Info.h>

#include <string>
#include <vector>
#include <algorithm>

#include "integrator.h"
#include "copter.h"

struct Imu
{
    itg::real t; // Time
    itg::real a[3]; // Accelerations
    itg::real w[3]; // Angular velocities
};

struct Pose
{
    itg::real t;    // Time
    itg::real p[3]; // Position
    itg::real q[4]; // Orientation quaternion
};

struct Info
{
    itg::real t, processing_time;
    int stage;
};

void analize(const std::vector<Imu> & imuData,
             const std::vector<Pose> & poseData,
             const std::vector<Info> & infoData,
             itg::real period)
{
    auto imu = imuData.begin();
    auto pose = poseData.begin();
    auto info = infoData.begin();
    while(info->stage != 3 && info != infoData.end()) info++;
    if(info == infoData.end())
    {
        ROS_ERROR("svo info stage 3 not found.");
        return;
    }
    while(pose->t + 0.01 < info->t && pose != poseData.end()) pose++;
    if(pose == poseData.end())
    {
        ROS_ERROR("svo pose stage 3 not found.");
        return;
    }
    itg::real svoDelay = 0.01;
    while(imu->t < pose->t - svoDelay && imu != imuData.end()) imu++;
    if(imu == imuData.end())
    {
        ROS_ERROR("imu stage 3 not found.");
        return;
    }
    itg::IntegratorEuler integrator;
    Copter copter;
    integrator._models.push_back(&copter);
    integrator.initialize();
    PoseRef & copterPose = integrator.modelState(copter);
    while(pose != poseData.end() && info != infoData.end())
    {
        if(info->stage == 3)
        {
            Pose start = *pose;
            pose++;
            info++;
            if(info == infoData.end() || pose == poseData.end() || info->stage != 3 || abs(pose->t - info->t) > 0.001) continue;
            while(imu->t < start.t - svoDelay && imu != imuData.end()) imu++;
            if(imu == imuData.end()) continue;
            integrator.setTime(start.t);
            //copter.
            while(imu->t < pose->t - svoDelay && imu != imuData.end())
            {
                copter.w[0] = imu->w[0];
                copter.w[1] = imu->w[1];
                copter.w[2] = imu->w[2];
                integrator.stepTo(imu->t);
                imu++;
            }
        }
        else
        {
            pose++;
            info++;
        }
    }


}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simco");
    ros::NodeHandle nh("~");

    rosbag::Bag bag;
    std::string fn, imuTopicName, poseTopicName, infoTopicName;
    nh.param<std::string>("bag_file", fn, "/home/igor/bag/svo_2016-01-27-10-09-56.bag");
    bag.open(fn, rosbag::bagmode::Read);
    nh.param<std::string>("imu_topic_name", imuTopicName, "/mavros/imu/data");
    nh.param<std::string>("pose_topic_name", poseTopicName, "/svo/pose");
    nh.param<std::string>("info_topic_name", infoTopicName, "/svo/info");
    std::vector<std::string> topics = {imuTopicName, poseTopicName, infoTopicName};

    std::vector<Imu> imuData;
    std::vector<Pose> poseData;
    std::vector<Info> infoData;

    rosbag::View view(bag, rosbag::TopicQuery(topics));
    itg::real beginTime = view.getBeginTime().toSec();
    itg::real endTime = view.getEndTime().toSec();
    bool useHeadTime = true;
    std::for_each (view.begin(), view.end(), [&](rosbag::MessageInstance const msg){
        if(msg.getTopic() == imuTopicName)
        {
            auto imuMsg = msg.instantiate<sensor_msgs::Imu>();
            if(imuMsg == nullptr) return;
            const ros::Time time = useHeadTime ? imuMsg->header.stamp : msg.getTime();
            imuData.push_back(
            {
                time.toSec() - beginTime,
                {imuMsg->linear_acceleration.x, imuMsg->linear_acceleration.y, imuMsg->linear_acceleration.z},
                {imuMsg->   angular_velocity.x, imuMsg->   angular_velocity.y, imuMsg->   angular_velocity.z},
            });
        }
        else if(msg.getTopic() == poseTopicName)
        {
            auto poseMsg = msg.instantiate<geometry_msgs::PoseWithCovarianceStamped>();
            if(poseMsg == nullptr) return;
            ros::Time time = useHeadTime ? poseMsg->header.stamp : msg.getTime();
            geometry_msgs::Point & point = poseMsg->pose.pose.position;
            geometry_msgs::Quaternion & quat = poseMsg->pose.pose.orientation;
            poseData.push_back(
            {
                time.toSec() - beginTime,
                {point.x, point.y, point.z},
                {quat.x, quat.y, quat.z, quat.w}
            });
        }
        else if(msg.getTopic() == infoTopicName)
        {
            auto infoMsg = msg.instantiate<svo_msgs::Info>();
            if(infoMsg == nullptr) return;
            ros::Time time = useHeadTime ? infoMsg->header.stamp : msg.getTime();
            infoData.push_back(
            {
                time.toSec() - beginTime,
                infoMsg->processing_time,
                infoMsg->stage
            });
        }
    });

    analize(imuData, poseData, infoData, endTime - beginTime);
    return 0;
}
