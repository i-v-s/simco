#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

ros::Subscriber subPose, subImu;
ros::Publisher pubAnVel, pubAnVelImu;

double prevTime = 0;
Eigen::Quaterniond prevq;

geometry_msgs::Vector3 vec;

void onImu(const sensor_msgs::Imu::ConstPtr imu)
{
    geometry_msgs::Vector3Stamped res;
    res.header.stamp = imu->header.stamp;
    double m = 0.1;
    vec.x += (imu->angular_velocity.x - vec.x) * m;
    vec.y += (imu->angular_velocity.y - vec.y) * m;
    vec.z += (imu->angular_velocity.z - vec.z) * m;
    res.vector = vec;
    pubAnVelImu.publish(res);
}

void onPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr pose)
{
    using namespace Eigen;
    geometry_msgs::Quaternion rq = pose->pose.pose.orientation;
    double time = pose->header.stamp.toSec();
    Quaterniond q(rq.w, rq.x, rq.y, rq.z);
    Quaterniond d = q * prevq.conjugate();
    double dt = time - prevTime;
    geometry_msgs::Vector3Stamped res;
    res.vector.x = d.x() / dt;
    res.vector.y = d.y() / dt;
    res.vector.z = d.z() / dt;
    res.header.stamp = pose->header.stamp;
    pubAnVel.publish(res);

    prevTime = time;
    prevq = q;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simco");
    ros::NodeHandle nh("~");
    subPose = nh.subscribe("/svo/pose", 10, &onPose);
    subImu = nh.subscribe("/mavros/imu/data", 10, &onImu);
    pubAnVel = nh.advertise<geometry_msgs::Vector3Stamped>("/svo/an_vel", 10);
    pubAnVelImu = nh.advertise<geometry_msgs::Vector3Stamped>("/mavros/an_vel", 10);
    ros::spin();
    return 0;
}
