#include "ros/ros.h"
#include "hw4.h"

// Messages that you might need
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "std_srvs/Empty.h"

// For transform support
#include "tf2/LinearMath/Transform.h"
#include "tf2/convert.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_eigen/tf2_eigen.h"
#include <fstream>
#include <random>
#include <iostream>
#include <Eigen/Dense>
#include <numeric>
#include <map>

ros::Publisher particlecloud_pub_; // publisher

ros::Publisher estimate_tag_pub_;
ros::Publisher car_control_pub_;
ros::Publisher finish_pub_;

std::default_random_engine generator;

std::vector<Eigen::Isometry3d> sample_poses;
std::vector<double> sample_weights;

Eigen::Isometry3d lastOdometry = Eigen::Isometry3d::Identity();
Eigen::Isometry3d goal ;
bool initialized = false;
bool isMoving = false;
int initSampleNum = 1000;
double trans_sigma =5.0;
double rot_sigma = 0.05;

std::vector<std::vector<Eigen::Isometry3d>> tag_poses;

double uniform_sample(double min, double max)
{
    std::uniform_real_distribution<double> pdf(min, max);
    return pdf(generator);
}
double gaussion_sample(double mean, double sigma)
{
    std::normal_distribution<double> pdf(mean, sigma);
    return pdf(generator);
}
double gaussion_pdf(double x, double mean, double sigma)
{
    return (1 / (sigma * sqrt(2 * M_PI))) * exp(-0.5 * pow((x - mean) / sigma, 2.0));
}
Eigen::Vector3d getPoseEuler(Eigen::Isometry3d &pose)
{
    return pose.rotation().eulerAngles(2, 1, 0);
}
Eigen::Vector3d getPoseTrans(Eigen::Isometry3d &pose)
{
    return pose.translation();
}

double pose_prob(Eigen::Isometry3d x, Eigen::Isometry3d mean, double trans_sigma, double rot_sigma)
{
    Eigen::Vector3d trans = getPoseTrans(x);
    Eigen::Vector3d trans_mean = getPoseTrans(mean);
    Eigen::Vector3d rot = getPoseEuler(x);
    Eigen::Vector3d rot_mean = getPoseEuler(mean);
    return gaussion_pdf(trans.x(), trans_mean.x(), trans_sigma) *
           gaussion_pdf(trans.y(), trans_mean.y(), trans_sigma) 
        //    *gaussion_pdf(trans.z(), trans_mean.z(), trans_sigma)
        // * gaussion_pdf(rot.z(),rot_mean.z(),rot_sigma)
        ;
}

Eigen::Isometry3d getCurPose(){

    Eigen::Isometry3d cur_pose;
    for(int i =0 ;i<sample_poses.size();i++){
        if (i == 0)
        {
            cur_pose =sample_poses[i].affine()*sample_weights[i];
        }
        else
        {

            cur_pose =cur_pose.affine()+sample_poses[i].affine()*sample_weights[i];
        }
    }
    return  cur_pose;
}

void moveCarToGoal(){
    if(isMoving){return;}
    isMoving = true;
    Eigen::Isometry3d curPose = getCurPose();
    Eigen::Isometry3d goalPose = goal;

    if((goalPose.translation()-curPose.translation()).norm()<0.1){
        std::cout<<"Finished"<<std::endl;
        finish_pub_.publish(std_msgs::Empty());
        return;
    }

    Eigen::Vector3d curPoseOri = getPoseEuler(curPose).normalized();
    Eigen::Vector3d expectedOri = (goalPose.translation()-curPose.translation()).normalized();
    double euler_diff = acos(expectedOri.dot(curPoseOri));

    geometry_msgs::Twist twist;
    twist.angular.z= 0.1*euler_diff;
    twist.linear.x = 0.3;
    car_control_pub_.publish(twist);
    isMoving = false;
}

Eigen::Isometry3d updatePoseWithSigma(Eigen::Isometry3d pose, Eigen::Isometry3d deltaChange, double trans_sigma, double rot_sigma)
{
    return deltaChange * pose;
    // Eigen::Isometry3d pose_new  = deltaChange*pose;
    // Eigen::Vector3d trans = getPoseTrans(pose_new);
    // trans+=Eigen::Vector3d(gaussion_sample(0,trans_sigma),gaussion_sample(0,trans_sigma),gaussion_sample(0,trans_sigma));
    // Eigen::Vector3d euler = getPoseEuler(pose_new);
    // euler+=Eigen::Vector3d(0,0,gaussion_sample(0,rot_sigma));

    // pose_new.setIdentity();
    // pose_new.rotate(Eigen::AngleAxisd(euler[0],Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(euler[1],Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(euler[2],Eigen::Vector3d::UnitX()));
    // pose_new.translation() = trans;
    // return pose_new;
}

Eigen::Isometry3d getOdometryFromTagPose(Eigen::Isometry3d &tagPose, Eigen::Isometry3d tagTransform)
{
    return tagTransform.inverse() * tagPose;
}
Eigen::Isometry3d getTagPoseFromOdometry(Eigen::Isometry3d &odometry, Eigen::Isometry3d tagTransform)
{
    return tagTransform * odometry;
}
void SampleInit(int sampleNum, Eigen::Isometry3d initOdom)
{
    sample_poses.clear();
    sample_weights.clear();
    for (int i = 0; i < sampleNum; i++)
    {
        sample_weights.push_back(1.0 / sampleNum);

        Eigen::Vector3d trans = getPoseTrans(initOdom);
        trans += Eigen::Vector3d(uniform_sample(-10, 10), uniform_sample(-10, 10), 0);
        Eigen::Vector3d euler = getPoseEuler(initOdom);
        euler += Eigen::Vector3d(0, 0, uniform_sample(-rot_sigma / 2, rot_sigma / 2));

        Eigen::Isometry3d pose_new;
        pose_new.setIdentity();
        pose_new.rotate(Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitX()));
        pose_new.translation() = trans;
        sample_poses.push_back(pose_new);
    }
}

void motionUpate(const geometry_msgs::Transform &odometry)
{
    if (!initialized)
    {
        initialized = true;
        lastOdometry = tf2::transformToEigen(odometry);
        SampleInit(initSampleNum, lastOdometry);
    }
    else
    {
        Eigen::Isometry3d newOdom = tf2::transformToEigen(odometry);
        Eigen::Isometry3d deltaChange = newOdom * lastOdometry.inverse();
        for (int i = 0; i < sample_poses.size(); i++)
        {
            sample_poses[i] = updatePoseWithSigma(sample_poses[i], deltaChange, trans_sigma, rot_sigma);
        }
        lastOdometry = newOdom;
    }
}

void pubSamples()
{
    visualization_msgs::MarkerArray cloud_msg;
    std::string global_frame_id_ = "map";

    cloud_msg.markers.resize(sample_poses.size());

    for (int i = 0; i < sample_poses.size(); i++)
    {
    cloud_msg.markers[i].header.stamp = ros::Time::now();
    cloud_msg.markers[i].header.frame_id =global_frame_id_;
    cloud_msg.markers[i].id = i;

    cloud_msg.markers[i].type = visualization_msgs::Marker::ARROW;
    cloud_msg.markers[i].pose =  tf2::toMsg(sample_poses[i]);
    cloud_msg.markers[i].scale.x =1.0*sample_weights[i]*200;
    cloud_msg.markers[i].scale.y = 0.2*sample_weights[i]*200;
    cloud_msg.markers[i].scale.z = 0.2*sample_weights[i]*200;
    cloud_msg.markers[i].color.a=1.0;
    cloud_msg.markers[i].color.g=1.0;
    }
    particlecloud_pub_.publish(cloud_msg);
}
void odom_callback(const tf2_msgs::TFMessage &msg)
{
    // msg.transforms[0].transform
    motionUpate(msg.transforms[0].transform);
    pubSamples();
    moveCarToGoal();
}
void goal_callback(const tf2_msgs::TFMessage &msg){
    goal = tf2::transformToEigen(msg.transforms[0].transform);
}

double ESS()
{
    double cvt2 = 0;
    for (int i = 0; i < sample_weights.size(); i++)
    {
        cvt2 += (sample_weights.size() * sample_weights[i] - 1) * (sample_weights.size() * sample_weights[i] - 1);
    }
    cvt2 = cvt2 / sample_weights.size();
    return sample_weights.size() / (1 + cvt2);
}
void resample()
{
    std::vector<double> Q;
    double running_sum = 0;
    for (int i = 0; i < sample_weights.size(); i++)
    {
        running_sum += sample_weights[i];
        Q.push_back(running_sum);
    }
    std::vector<double> T;
    std::uniform_real_distribution<double> dis2(0.0, 1.0);
    for (int i = 0; i <initSampleNum+1; i++)
    {
        T.push_back(dis2(generator));
    }
    std::sort(T.begin(), T.end());
    *T.end() = 1.0;
    int i = 0;
    int j = 0;

    std::vector<int> indices;
    indices.resize(initSampleNum);
    while (i <=initSampleNum-1)
    {
        if (T[i] < Q[j])
        {
            indices[i] = j;
            i++;
        }
        else
        {
            j++;
        }
    }
    std::vector<Eigen::Isometry3d> new_poses;
    std::vector<double> new_weights;
    //form new poses and new weights
    for (int i = 0; i < initSampleNum; i++)
    {
        new_poses.push_back(sample_poses[indices[i]]);
        new_weights.push_back(1.0/initSampleNum);
    }
    sample_weights.clear();
    sample_poses.clear();
        for (int i = 0; i < initSampleNum; i++)
    {
        sample_poses.push_back(new_poses[i]);
        sample_weights.push_back(new_weights[i]);
    }
    //normalize sample weights
}

void weight_update(int tagId, Eigen::Isometry3d &transformToRobot)
{
    double beta = 0.5;
    if (ESS() < beta * sample_poses.size())
    {
        std::cout << "Trigger Resample" << std::endl;
        resample();
        std::cout << "Resample ended" << std::endl;
    }

    // update Odom sample weights
    std::cout << "update Odom sample weights" << std::endl;
    double weightSum = 0;
    for (int i = 0; i < sample_poses.size(); i++)
    {
        Eigen::Isometry3d tagPose = getTagPoseFromOdometry(sample_poses[i], transformToRobot.inverse());
        // find which is the closest tag pose
        double d = 99999;
        Eigen::Isometry3d realTagPose;
        for (auto p : tag_poses[tagId])
        {
            double dist = (p.translation() - tagPose.translation()).norm();
            if (dist < d)
            {
                d = dist;
                realTagPose = p;
            }
        }
        // from the true tag pose  get the odom pose
        Eigen::Isometry3d realOdomPose = getOdometryFromTagPose(realTagPose, transformToRobot.inverse());

        double weight = pose_prob(sample_poses[i], realOdomPose, trans_sigma, rot_sigma);
        sample_weights[i] = sample_weights[i] * weight;
        weightSum += sample_weights[i];
    }
    //normalize their weights
    for (int i = 0; i < sample_poses.size(); i++)
    {
        sample_weights[i] = sample_weights[i] / weightSum;
    }
    std::cout << "update Odom sample weights Complete" << std::endl;
    // weightSum = 0;
    // for (int i = 0; i < sample_poses.size(); i++)
    // {
    //     weightSum += sample_weights[i];
    // }
    // std::cout << "after " << weightSum << std::endl;


    geometry_msgs::PoseArray  tf;
    tf.header.stamp = ros::Time::now();
    tf.header.frame_id = "map";
    tf.poses.resize(tag_poses[tagId].size());
    for(int i= 0;i<tag_poses[tagId].size();i++){
        tf.poses[i] = tf2::toMsg(transformToRobot*tag_poses[tagId][i]);
    }
    // Eigen::Vector3d t = lastOdometry.translation()+transformToRobot.translation();
    // tf.pose.position = tf2::toMsg(t);
    estimate_tag_pub_.publish(tf);
}

void tags_callback(const tf2_msgs::TFMessage &msg)
{
    Eigen::Isometry3d tagTransfromFromRobot = tf2::transformToEigen(msg.transforms[0].transform);
    int tagId = atoi(msg.transforms[0].child_frame_id.c_str());
    // std::cout<<"Detect tag Id "<<tagId<<std::endl;
    weight_update(tagId, tagTransfromFromRobot);
    //weight update
    moveCarToGoal();
}

void readTagPoses()
{

    std::ifstream infile("./src/homework/src/tags_pose_map.txt");
    std::string line;
    int i = 0;
    //  std::cout<<"Read started"<<std::endl;

    tag_poses.resize(16);
    while (std::getline(infile, line))
    {
        geometry_msgs::Pose pose;
        std::istringstream iss(line);
        int tagId;
        double x, y, z, roll, pitch, yaw;
        iss >> tagId >> x >> y >> z >> roll >> pitch >> yaw;
        tf2::Quaternion q = tf2::Quaternion(yaw, pitch, roll);
        // std::cout<<tagId<<" "<<x<<" "<<y<<" "<<z<<" "<<roll<<" "<<pitch<<" "<<yaw<<std::endl;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
        i++;
        Eigen::Isometry3d iso;
        Eigen::fromMsg(pose, iso);
        tag_poses[tagId].push_back(iso);
    }
    //  std::cout<<"Read ended"<<std::endl;
}

int main(int argc, char **argv)
{

    readTagPoses();

    ros::init(argc, argv, "hw4");
    ros::NodeHandle n;

    ros::Subscriber odom_sub_; // subscriber
    odom_sub_ = n.subscribe("odom", 1000, odom_callback);
    ros::Subscriber tags_sub_; // subscriber
    tags_sub_ = n.subscribe("tags", 1000, tags_callback);
    ros::Subscriber goal_sub_; // subscriber
    goal_sub_ = n.subscribe("goal",1000,goal_callback);


    

    estimate_tag_pub_ = n.advertise<geometry_msgs::PoseArray>("car_pose_from_tag", 2, true);

    // There are other things that you may need to subscirbe to and publish

    particlecloud_pub_ = n.advertise<visualization_msgs::MarkerArray>("particlecloud", 2, true);
    car_control_pub_ =n.advertise<geometry_msgs::Twist>("cmd_vel",2,true);
    finish_pub_ =n.advertise<std_msgs::Empty>("finished",2,true);
    ros::spin();

    return 0;
}
