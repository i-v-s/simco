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
#include "trapint.h"
#include "copter.h"

#include <gtest/gtest.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "linearregression.h"
#include <memory>

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
    Eigen::Quaternion<itg::real> q; // Orientation quaternion
};

void normalize(itg::real * q)
{
    itg::real l = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    for(int x = 0; x < 4; x++) q[x] /= l;
}

struct Info
{
    itg::real t, processing_time;
    int stage;
};


typedef LinearRegression<double, 3, 3> LinReg;

std::unique_ptr<LinReg> analize(const std::vector<Imu> & imuData,
             const std::vector<Pose> & poseData,
             const std::vector<Info> & infoData,
             itg::real period, itg::real svoDelay = 0.02)
{
    auto imu = imuData.begin();
    auto pose = poseData.begin();
    auto info = infoData.begin();
    itg::real cov[4] = {0}, sd[4] = {0}, si[4] = {0};

    while(info->stage != 3 && info != infoData.end()) info++;
    if(info == infoData.end())
    {
        ROS_ERROR("svo info stage 3 not found.");
        return nullptr;
    }
    while(pose->t + 0.01 < info->t && pose != poseData.end()) pose++;
    if(pose == poseData.end())
    {
        ROS_ERROR("svo pose stage 3 not found.");
        return nullptr;
    }

    TrapInt integ;
    while(imu->t < pose->t - svoDelay && imu != imuData.end())
    {
        integ.add(imu->t, {imu->w[0], imu->w[1], imu->w[2]});
        imu++;
    }
    if(imu == imuData.end())
    {
        ROS_ERROR("imu stage 3 not found.");
        return nullptr;
    }
    /*itg::IntegratorEuler integrator;
    Copter copter;
    integrator._models.push_back(&copter);
    integrator.initialize();*/

    //PoseRef & copterPose = integrator.modelState(copter);
    std::vector<XYStruct<double, 3, 3> > diffs;
    while(pose != poseData.end() && info != infoData.end())
    {
        if(info->stage == 3)
        {
            Pose start = *pose;
            pose++;
            info++;
            if(info == infoData.end() || pose == poseData.end() || info->stage != 3 || abs(pose->t - info->t) > 0.001) continue;
            while(imu->t < start.t - svoDelay && imu != imuData.end())
            {
                integ.add(imu->t, {imu->w[0], imu->w[1], imu->w[2]});
                imu++;
            }
            if(imu == imuData.end()) continue;
            if(pose->t > start.t + 0.1) continue;

            /*integrator.setTime(start.t);
            copterPose.Q[0] = start.q[0];
            copterPose.Q[1] = start.q[1];
            copterPose.Q[2] = start.q[2];
            copterPose.Q[3] = start.q[3];*/
            integ.begin(start.t - svoDelay);
            while(imu->t < pose->t - svoDelay && imu != imuData.end())
            {
                /*copter.w[0] = imu->w[0];
                copter.w[1] = imu->w[1];
                copter.w[2] = imu->w[2];
                integrator.stepTo(imu->t);*/
                integ.add(imu->t, {imu->w[0], imu->w[1], imu->w[2]});
                imu++;
            }
            Pose end = *pose;
            Eigen::Quaterniond dq = end.q * start.q.conjugate();
            double dt = end.t - start.t;
            std::vector<double> wm = integ.endMed(end.t - svoDelay, imu->t, {imu->w[0], imu->w[1], imu->w[2]});
            XYStruct<double, 3, 3> diff;
            diff.y << wm[0], wm[1], wm[2];
            diff.x << dq.x() / dt, dq.y() / dt, dq.z() / dt;
            diffs.push_back(diff);
            //svoDiff.push_back();
            //normalize(copterPose.Q.get());
            /*normalize(end.q);
            itg::real dd[4] = {0}, di[4] = {0};
            for(int x = 0; x < 4; x++)
            {
                dd[x] = end.q[x] - start.q[x];
                di[x] = copterPose.Q[x] - start.q[x];
                LinReg & l = lr[x].back();
                l.add(dd[x], di[x]);
                if(l._count >= 500)
                    lr[x].push_back(LinReg());
                cov[x] += dd[x] * di[x];
                sd[x] += dd[x] * dd[x];
                si[x] += di[x] * di[x];
            }*/
        }
        else
        {
            pose++;
            info++;
        }
    }

    return std::unique_ptr<LinReg>(new LinReg(diffs));
    //ROS_INFO_STREAM("B: %f" << lr.b << " error:" << lr.e);

    /*std::vector<itg::real> b[4], a[4];
    for(int x = 0; x < 4; x++)
    {
        for(auto l : lr[x])
        {
            l.calc();
            b[x].push_back(l._b);
            a[x].push_back(l._a);
        }
        cov[x] /= sqrt(sd[x] * si[x]);
    }
    for(int x = 0; x < 4; x++)
    {
        ROS_INFO("corr: %f", cov[x]);
    }*/
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simco");
    ros::NodeHandle nh("~");

    rosbag::Bag bag;
    std::string fn, imuTopicName, poseTopicName, infoTopicName;
    nh.param<std::string>("bag_file", fn, "/home/igor/bag/svo_2016-01-27-10-09-56.bag");
    //nh.param<std::string>("bag_file", fn, "/home/igor/bag/svo_2016-01-27-10-04-16.bag");
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
                {quat.w, quat.x, quat.y, quat.z}
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
    double emin = 1E10;
    double dtmin = 0;
    Eigen::Matrix<double, 3, 3> b;
    for(double dt = -0.5; dt < 0.5; dt += 0.0001)
    {
        std::unique_ptr<LinReg> lr = analize(imuData, poseData, infoData, endTime - beginTime, dt);
        if(!lr) continue;
        if(lr->e.sum() < emin)
        {
            emin = lr->e.sum();
            dtmin = dt;
            b = lr->b;
        }
    }
    ROS_INFO_STREAM("B: %f" << b << " error:" << emin << "dtmin:" << dtmin);
    return 0;
}
