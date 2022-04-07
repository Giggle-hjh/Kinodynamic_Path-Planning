#include <iostream>
#include <fstream>
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <hw_tool.h>
#include "backward.hpp"

using namespace std;
using namespace Eigen;

namespace backward {
backward::SignalHandling sh;
}

// simulation param from launch file
double _resolution, _inv_resolution, _cloud_margin;
double _x_size, _y_size, _z_size;
Vector3d _start_pt, _start_velocity;

// useful global variables
bool _has_map   = false;

Vector3d _map_lower, _map_upper;
int _max_x_id, _max_y_id, _max_z_id;

// ros related
ros::Subscriber _map_sub, _pts_sub;
ros::Publisher  _grid_map_vis_pub, _path_vis_pub, _goal_vis_pub;

// Integral parameter
double _max_input_acc     = 1.0;
int    _discretize_step   = 2;
double _time_interval     = 1.25;
int    _time_step         = 50;

Homeworktool * _homework_tool     = new Homeworktool();
TrajectoryStatePtr *** TraLibrary;

void rcvWaypointsCallback(const nav_msgs::Path & wp);
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map);
void trajectoryLibrary(const Eigen::Vector3d start_pt, const Eigen::Vector3d start_velocity, const Eigen::Vector3d target_pt);
void visTraLibrary(TrajectoryStatePtr *** TraLibrary);

void rcvWaypointsCallback(const nav_msgs::Path & wp)
{     
    if( wp.poses[0].pose.position.z < 0.0 || _has_map == false )
        return;

    Vector3d target_pt;
    target_pt << wp.poses[0].pose.position.x,
                 wp.poses[0].pose.position.y,
                 wp.poses[0].pose.position.z;

    ROS_INFO("[node] receive the planning target");

    visualization_msgs::Marker vis_goal;
    vis_goal.header.frame_id  = "world";
    vis_goal.header.stamp = ros::Time::now();
    vis_goal.ns = "demo_node/goal";
    vis_goal.id = 0;
    vis_goal.type = visualization_msgs::Marker::ARROW;
    vis_goal.action = visualization_msgs::Marker::ADD;

    vis_goal.color.r = 0.0;
    vis_goal.color.g = 1.0;
    vis_goal.color.b = 0.0;
    vis_goal.color.a = 1.0;

    vis_goal.scale.x = _resolution;
    vis_goal.scale.y = _resolution / 2.0;
    vis_goal.scale.z = _resolution / 2.0;

    vis_goal.pose.position.x = wp.poses[0].pose.position.x;
    vis_goal.pose.position.y = wp.poses[0].pose.position.y;
    vis_goal.pose.position.z = wp.poses[0].pose.position.z;
    
    vis_goal.pose.orientation.x = wp.poses[0].pose.orientation.x;
    vis_goal.pose.orientation.y = wp.poses[0].pose.orientation.y;
    vis_goal.pose.orientation.z = wp.poses[0].pose.orientation.z;
    vis_goal.pose.orientation.w = wp.poses[0].pose.orientation.w;
    vis_goal.lifetime = ros::Duration(); // never delete

    _goal_vis_pub.publish(vis_goal);

    trajectoryLibrary(_start_pt,_start_velocity,target_pt);
}

void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map)
{   
    if(_has_map ) 
        return;

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_vis;
    sensor_msgs::PointCloud2 map_vis;

    pcl::fromROSMsg(pointcloud_map, cloud);
    
    if( (int)cloud.points.size() == 0 ) 
        return;

    pcl::PointXYZ pt;
    for (int idx = 0; idx < (int)cloud.points.size(); idx++)
    {    
        pt = cloud.points[idx];        

        // set obstalces into grid map for path planning
        _homework_tool->setObs(pt.x, pt.y, pt.z);

        // for visualize only
        Vector3d cor_round = _homework_tool->coordRounding(Vector3d(pt.x, pt.y, pt.z));
        pt.x = cor_round(0);
        pt.y = cor_round(1);
        pt.z = cor_round(2);
        cloud_vis.points.push_back(pt);
    }

    cloud_vis.width    = cloud_vis.points.size();
    cloud_vis.height   = 1;
    cloud_vis.is_dense = true;

    pcl::toROSMsg(cloud_vis, map_vis);

    map_vis.header.frame_id = "/world";
    _grid_map_vis_pub.publish(map_vis);

    _has_map = true;
}

void trajectoryLibrary(const Vector3d start_pt, const Vector3d start_velocity, const Vector3d target_pt)
{
    Vector3d acc_input;
    Vector3d pos,vel;
    int a =0 ;
    int b =0 ;
    int c =0 ;

    double min_Cost = 100000.0;
    double Trajctory_Cost;
    TraLibrary  = new TrajectoryStatePtr ** [_discretize_step + 1];     //recored all trajectories after input

    Matrix<double, 6, 1> status_old;  // s0
    Matrix<double, 6, 1> status_new;  // s
    Matrix<double, 6, 6> state_transform;   // e^At
    Matrix<double, Dynamic, Dynamic> identity_matrix; // I 
    Matrix<double, Dynamic, Dynamic> A_matrix;  // A
    Matrix<double, Dynamic, Dynamic> B_Matrix;  // B
    Matrix<double, 3, 1> u_matrix;              // u
    
    identity_matrix.setIdentity(6, 6);
    A_matrix.setZero(6, 6);
    B_Matrix.setZero(6, 3);

    A_matrix(0, 3) = 1.0;
    A_matrix(1, 4) = 1.0;
    A_matrix(2, 5) = 1.0;

    B_Matrix(3, 0) = 1.0;
    B_Matrix(4, 1) = 1.0;
    B_Matrix(5, 2) = 1.0;

    double best_time = 10000000;
    for(int i=0; i <= _discretize_step; i++)
    {           
        //acc_input_ax
        TraLibrary[i] = new TrajectoryStatePtr * [_discretize_step + 1];
        for(int j=0;j <= _discretize_step; j++)
        {        
            //acc_input_ay
            TraLibrary[i][j] = new TrajectoryStatePtr [_discretize_step + 1];
            for(int k=0; k <= _discretize_step; k++)
            {   
                //acc_input_az
                
                vector<Vector3d> Position;
                vector<Vector3d> Velocity;
                acc_input(0) = double(-_max_input_acc + i * (2 * _max_input_acc / double(_discretize_step)) );
                acc_input(1) = double(-_max_input_acc + j * (2 * _max_input_acc / double(_discretize_step)) );
                acc_input(2) = double( k * (2 * _max_input_acc / double(_discretize_step) ) + 0.1);                          //acc_input_az >0.1
                
                pos(0) = start_pt(0);
                pos(1) = start_pt(1);
                pos(2) = start_pt(2);

                vel(0) = start_velocity(0);
                vel(1) = start_velocity(1);
                vel(2) = start_velocity(2);

                Position.push_back(pos);
                Velocity.push_back(vel);

                bool collision = false;
                double delta_time;
                delta_time = _time_interval / double(_time_step);

                u_matrix << acc_input(0), acc_input(1), acc_input(2);
                status_old << pos(0), pos(1), pos(2), vel(0), vel(1), vel(2);
                
                for(int step=0 ; step <= _time_step ; step ++)
                {

                    /*
                    STEP 1: finish the forward integration, the modelling has been given in the document
                    the parameter of forward integration: _max_input_acc|_discretize_step|_time_interval|_time_step   
                    all have been given, use the pos and vel to recored the steps in the trajectory
                    */
                    // state transform
                    state_transform = identity_matrix + A_matrix * delta_time;
                    status_new = state_transform * status_old;   //  Zero input response
                    status_new += 0.5 * (state_transform + identity_matrix) * B_Matrix * delta_time * u_matrix;  // Zero state response
                    status_old = status_new;

                    pos(0) = status_new(0, 0);
                    pos(1) = status_new(1, 0);
                    pos(2) = status_new(2, 0);

                    vel(0) = status_new(3, 0);
                    vel(1) = status_new(4, 0);
                    vel(2) = status_new(5, 0);

                    // Velocity integral formula
                    /* pos(0) = pos(0) + vel(0) * delta_time + 0.5 * acc_input(0) * delta_time * delta_time;
                    pos(1) = pos(1) + vel(1) * delta_time + 0.5 * acc_input(1) * delta_time * delta_time;
                    pos(2) = pos(2) + vel(2) * delta_time + 0.5 * acc_input(2) * delta_time * delta_time;
                    vel(0) = vel(0) + acc_input(0) * delta_time;
                    vel(1) = vel(1) + acc_input(1) * delta_time;
                    vel(2) = vel(2) + acc_input(2) * delta_time; */

                    Position.push_back(pos);
                    Velocity.push_back(vel);
                    double coord_x = pos(0);
                    double coord_y = pos(1);
                    double coord_z = pos(2);

                    //check if if the trajectory face the obstacle
                    if(_homework_tool->isObsFree(coord_x,coord_y,coord_z) != 1)
                    {
                        collision = true; // mean there is a collision
                    }
                }

                double current_time;
                
                // the Trajctory_Cost will record the optimal cost of this trajectory
                Trajctory_Cost = _homework_tool -> OptimalBVP(pos, vel, target_pt, current_time);

                //input the trajetory in the trajectory library
                TraLibrary[i][j][k] = new TrajectoryState(Position,Velocity,Trajctory_Cost);
                
                //if there is not any obstacle in the trajectory we need to set 'collision_check = true', so this trajectory is useable
                if(collision)
                    TraLibrary[i][j][k]->setCollisionfree();
                
                //record the min_cost in the trajectory Library, and this is the part pf selecting the best trajectory cloest to the planning traget
                if(Trajctory_Cost < min_Cost && TraLibrary[i][j][k]->collision_check == false)
                {
                    a = i;
                    b = j;
                    c = k;
                    min_Cost = Trajctory_Cost;
                    best_time = current_time;
                }
            }
        }
    }
    
    // set the  optimal
    TraLibrary[a][b][c] -> setOptimal();
    ROS_INFO_STREAM("The best time is: " << best_time);
    visTraLibrary(TraLibrary);
    return;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "demo_node");
    ros::NodeHandle nh("~");

    _map_sub  = nh.subscribe( "map",       1, rcvPointCloudCallBack );
    _pts_sub  = nh.subscribe( "waypoints", 1, rcvWaypointsCallback );

    _grid_map_vis_pub         = nh.advertise<sensor_msgs::PointCloud2>("grid_map_vis", 1);
    _path_vis_pub             = nh.advertise<visualization_msgs::MarkerArray>("RRTstar_path_vis",1);
    _goal_vis_pub             = nh.advertise<visualization_msgs::Marker>("goal_vis", 1);

    nh.param("map/cloud_margin",  _cloud_margin, 0.0);
    nh.param("map/resolution",    _resolution,   0.2);
    
    nh.param("map/x_size",        _x_size, 50.0);
    nh.param("map/y_size",        _y_size, 50.0);
    nh.param("map/z_size",        _z_size, 5.0 );
    
    nh.param("planning/start_x",  _start_pt(0),  0.0);
    nh.param("planning/start_y",  _start_pt(1),  0.0);
    nh.param("planning/start_z",  _start_pt(2),  0.0);

    nh.param("planning/start_vx",  _start_velocity(0),  0.0);
    nh.param("planning/start_vy",  _start_velocity(1),  0.0);
    nh.param("planning/start_vz",  _start_velocity(2),  0.0);    
    
    _map_lower << - _x_size/2.0, - _y_size/2.0,     0.0;
    _map_upper << + _x_size/2.0, + _y_size/2.0, _z_size;
    
    _inv_resolution = 1.0 / _resolution;
    
    _max_x_id = (int)(_x_size * _inv_resolution);
    _max_y_id = (int)(_y_size * _inv_resolution);
    _max_z_id = (int)(_z_size * _inv_resolution);

    _homework_tool  = new Homeworktool();
    _homework_tool  -> initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id, _max_z_id);
    
    ros::Rate rate(100);
    bool status = ros::ok();
    while(status) 
    {
        ros::spinOnce();      
        status = ros::ok();
        rate.sleep();
    }

    delete _homework_tool;
    return 0;
}

void visTraLibrary(TrajectoryStatePtr *** TraLibrary)
{
    double _resolution = 0.2;
    visualization_msgs::MarkerArray  LineArray;
    visualization_msgs::Marker       Line;

    Line.header.frame_id = "world";
    Line.header.stamp    = ros::Time::now();
    Line.ns              = "demo_node/TraLibrary";
    Line.action          = visualization_msgs::Marker::ADD;
    Line.pose.orientation.w = 1.0;
    Line.type            = visualization_msgs::Marker::LINE_STRIP;
    Line.scale.x         = _resolution/5;

    Line.color.r         = 0.0;
    Line.color.g         = 0.0;
    Line.color.b         = 1.0;
    Line.color.a         = 1.0;

    int marker_id = 0;

    for(int i = 0; i <= _discretize_step; i++)
    {
        for(int j = 0; j<= _discretize_step;j++)
        {  
            for(int k = 0; k<= _discretize_step;k++)
            {
                if(TraLibrary[i][j][k]->collision_check == false)
                {
                    if(TraLibrary[i][j][k]->optimal_flag == true)
                    {
                        Line.color.r         = 0.0;
                        Line.color.g         = 1.0;
                        Line.color.b         = 0.0;
                        Line.color.a         = 1.0;
                    }
                    else
                    {
                        Line.color.r         = 0.0;
                        Line.color.g         = 0.0;
                        Line.color.b         = 1.0;
                        Line.color.a         = 1.0;
                    }
                }
                else
                {
                    Line.color.r         = 1.0;
                    Line.color.g         = 0.0;
                    Line.color.b         = 0.0;
                    Line.color.a         = 1.0;
                }
                    Line.points.clear();
                    geometry_msgs::Point pt;
                    Line.id = marker_id;
                    for(int index = 0; index < int(TraLibrary[i][j][k]->Position.size());index++)
                    {
                        Vector3d coord = TraLibrary[i][j][k]->Position[index];
                        pt.x = coord(0);
                        pt.y = coord(1);
                        pt.z = coord(2);
                        Line.points.push_back(pt);
                    }
                    LineArray.markers.push_back(Line);
                    _path_vis_pub.publish(LineArray);
                    ++marker_id; 
            }
        }
    }    
}