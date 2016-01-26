#include "ros/ros.h"
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
//#include "Eigen/Core"
//#include "Eigen/Geometry"
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
//#include <rwrc15_msgs/GetGoalsId.h>
#include <rwrc15_msgs/GetGoalsId.h>
#include <rwrc15_msgs/InitialValue.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <math.h>
#include <vector>
#include "Visualize_lib.h"
#include <boost/thread.hpp>
#include <stdio.h>
#include <stdlib.h>
#include "GPStoXY/GPStoXY.h"
#include <std_msgs/Bool.h>
#include "init_visu.h"
#include "read_wp_file.h"
//2015/02/22 592, 978行目追加
using namespace std;

//wp_mngr.cpp, init_visu.cpp, read_wp_file.cppのheader_frameを使用するグローバル座標系の名前に変更すること//
const string header_frame("/map");
const string ongaku_command("gnome-terminal -e \"/home/amsl/AMSL_ros_pkg/l/script/beep.sh\" --geometry=0x0+0+0");

float now_Rc=2;
ros::Publisher marker_pub5;

geometry_msgs::TransformStamped frame_in;
geometry_msgs::PoseStamped current;

bool fin_flag = false;

int current_id_;
///////////////////////////////////////////////////////
//////-----------Callback function-------------////////
///////////////////////////////////////////////////////
//おそらく現在使われていない. 動作チェック後破棄
boost::mutex goal_mutex_;
bool callback(rwrc15_msgs::GetGoalsId::Request  &req,
         rwrc15_msgs::GetGoalsId::Response &res )
{

	boost::mutex::scoped_lock(goal_mutex_);
	res.goals.clear();
	res.goals.push_back(current);
	res.id = current_id_;
	//res.finish = fin_flag;

	/*cout<<"server response way_point x:"<<current.pose.position.x<<endl;
	cout<<"server response way_point y:"<<current.pose.position.y<<endl;*/

	return true;
}

void housePointCloud(sensor_msgs::PointCloud& poi_cl, nav_msgs::Path wayp)
{
	poi_cl.header.frame_id = header_frame;
	poi_cl.header.stamp = ros::Time::now();
	for(int i=0; i<(int)wayp.poses.size(); i++){
		geometry_msgs::Point32 point;
		point.x = wayp.poses[i].pose.position.x;
		point.y = wayp.poses[i].pose.position.y;
		point.z = 0;
		poi_cl.points.push_back(point);
	}
}

void setCircleParameter(visualization_msgs::Marker& now, geometry_msgs::PoseStamped& traced_waypoint,
 geometry_msgs::PoseStamped& now_state,
 geometry_msgs::PoseStamped& traced_state,
 visualization_msgs::Marker& circle_dr,
 visualization_msgs::Marker& traced_circle_dr)
{
	std_msgs::ColorRGBA rgba_in;
	rgba_in.r = 0.8;
	rgba_in.g = 0.8;
	rgba_in.b = 0.0;
	rgba_in.a = 0.8;
	Visualization* marker_circle = new Visualization(rgba_in, ADD, 0.1, "area",header_frame);
    
    now_state.pose.position.x = now.points[0].x;
    now_state.pose.position.y = now.points[0].y;
    now_state.pose.position.z = now.points[0].z;
    
    traced_state.pose.position.x = now.points[0].x;
    traced_state.pose.position.y = now.points[0].y;
    traced_state.pose.position.z = now.points[0].z;
    
    traced_waypoint.header.frame_id=header_frame;
    traced_waypoint.pose.position.x = traced_state.pose.position.x;
    traced_waypoint.pose.position.y = traced_state.pose.position.y;
    traced_waypoint.pose.position.z = 0.5;
    
    // marker_circle->potint2MarkerCircles(circle_dr, now_state, 222, now_Rc);
    marker_circle->potint2MarkerCircles(traced_circle_dr, traced_state, 222, now_Rc);
}

///////////////////////////////////////////////////////
/////-------Cordinate Convert function---------////////
///////////////////////////////////////////////////////
void pathLocal2Global(nav_msgs::Path& path, const tf::StampedTransform transform)
{
	int length=path.poses.size();
	nav_msgs::Odometry zero;
	float angle = tf::getYaw(transform.getRotation()) - 0;
	for(int i=0;i<length;i++){
		float tmp_x = path.poses[i].pose.position.x - zero.pose.pose.position.x;
		float tmp_y = path.poses[i].pose.position.y - zero.pose.pose.position.y;
		float conv_x = cos(angle)*tmp_x - sin(angle)*tmp_y;
		float conv_y = sin(angle)*tmp_x + cos(angle)*tmp_y;
		path.poses[i].pose.position.x = conv_x + transform.getOrigin().x();
		path.poses[i].pose.position.y = conv_y + transform.getOrigin().y();
	}
}


void pose2DLocal2Global(const geometry_msgs::Pose2D& target, 
						 geometry_msgs::Pose2D robot,
						 geometry_msgs::Pose2D& target_global)
{
	float angle = robot.theta - 0;
	float tmp_x = target.x - 0;
	float tmp_y = target.y - 0;
	float conv_x = cos(angle)*tmp_x - sin(angle)*tmp_y;
	float conv_y = sin(angle)*tmp_x + cos(angle)*tmp_y;
	target_global.x = conv_x + robot.x;
	target_global.y = conv_y + robot.y;
	target_global.theta = 0;
}


void vLinePub(	geometry_msgs::Pose2D P, 
				geometry_msgs::Pose2D H)
{
	
	std_msgs::ColorRGBA rgba_in;
	rgba_in.r = 0.0;
	rgba_in.g = 0.8;
	rgba_in.b = 0.0;
	rgba_in.a = 0.8;
	Visualization* marker_line = new Visualization(rgba_in, ADD, 0.2, "vline",header_frame);
	
	nav_msgs::Path line;
	geometry_msgs::PoseStamped point;
	point.pose.position.x = P.x;
	point.pose.position.y = P.y;
	line.poses.push_back(point);
	point.pose.position.x = H.x;
	point.pose.position.y = H.y;
	line.poses.push_back(point);
	
	visualization_msgs::Marker m_line;
	marker_line->convertPath2MarkerLine(line, m_line, 1212);
	
	marker_pub5.publish(m_line);
	delete marker_line;
}



geometry_msgs::Pose2D nearest(	geometry_msgs::Pose2D A, 
								geometry_msgs::Pose2D B, 
								geometry_msgs::Pose2D P )
{
	geometry_msgs::Pose2D a, b;
	double r;
	
	//a = (next wp - now wp)
	//b = (robo pos - now_wp) 
	a.x = B.x - A.x;
	a.y = B.y - A.y;
	b.x = P.x - A.x;
	b.y = P.y - A.y;
	
	//waypoint間のベクトルに対する内積
	r = (a.x*b.x + a.y*b.y) / (a.x*a.x + a.y*a.y);
	cout<<"robot:"<<P.x<<","<<P.y<<endl;
	cout<<"r:"<<r<<endl;
	
	if( r<= 0 ){
		return A;
	}else if( r>=1 ){
		return B;
	}else{
		geometry_msgs::Pose2D result;
		result.x = A.x + r*a.x;
		result.y = A.y + r*a.y;
		return result;
	}
}

string IntToString(int number)
{
	stringstream ss;
	ss << number;
	return ss.str();
}
///////////////////////////////////////////////////////
//////-------Input waypoint data from txt------////////
///////////////////////////////////////////////////////
void setMarkers(nav_msgs::Path waypoints,
				visualization_msgs::Marker& lines,
				visualization_msgs::Marker& points,
				visualization_msgs::Marker& fin,
				visualization_msgs::MarkerArray& txts)
{
	std_msgs::ColorRGBA rgba_in;
	rgba_in.r = 0.0;
	rgba_in.g = 0.8;
	rgba_in.b = 0.9;
	rgba_in.a = 0.8;
	Visualization* marker_txt = new Visualization(rgba_in, ADD, 2.0, "txtSet",header_frame);
	float yaw = 0;
	geometry_msgs::Point pt;
	int waypoint_n = waypoints.poses.size();
	for(int i=0; i<waypoint_n; i++) {
		pt.x = waypoints.poses[i].pose.position.x;
		pt.y = waypoints.poses[i].pose.position.y;
		pt.z = 0;
		
		geometry_msgs::PoseStamped point;
		point.pose.position.x = pt.x;
		point.pose.position.y = pt.y; 
		point.pose.position.z = pt.z;
		
		visualization_msgs::Marker txt1;
		marker_txt->txtMarker(IntToString(i), point, txt1, i);
		
		txts.markers.push_back(txt1);
		waypoints.poses.push_back(point);
		lines.points.push_back(pt);
		points.points.push_back(pt);
	}
	geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,yaw);
	fin.pose.position = pt;
	fin.pose.orientation = pose_quat;
}

void waypointAngleSet(nav_msgs::Path& waypoints)
{
	for(unsigned int i=0; i<waypoints.poses.size()-1; i++){
		float angle=atan2(waypoints.poses[i+1].pose.position.y - waypoints.poses[i].pose.position.y,
				waypoints.poses[i+1].pose.position.x - waypoints.poses[i].pose.position.x);
		geometry_msgs::Quaternion angle_quat = tf::createQuaternionMsgFromYaw(angle);
		waypoints.poses[i].pose.orientation=angle_quat;
	}
}

float calcDistance(float x1, float y1, float x2, float y2)
{
	float dis;
	
	dis = sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));

	return dis;
}

void convertGlobalAngleTo(float& angle)
{
	angle = -(angle - M_PI_2);
}

void whereAmI(		const geometry_msgs::PoseStamped& robo,
					const nav_msgs::Path& waypoints,
					int& wp_count)
{
	
	geometry_msgs::Pose2D A, B, C, P, H;
	//P: robot position, A:now waypoints, B:next waypoints
	P.x = robo.pose.position.x;
	P.y = robo.pose.position.y;
	A.x = waypoints.poses[wp_count].pose.position.x;
	A.y = waypoints.poses[wp_count].pose.position.y;
	B.x = waypoints.poses[wp_count+1].pose.position.x;
	B.y = waypoints.poses[wp_count+1].pose.position.y;
	//ロボットと現在のwaypointの距離
	float dx1 = P.x - A.x;
	float dy1 = P.y - A.y;
	float dist1 = sqrt(dx1*dx1 + dy1*dy1);
	//ロボットと次のwaypointの距離
	float dx2 = P.x - B.x;
	float dy2 = P.y - B.y;
	float dist2 = sqrt(dx2*dx2 + dy2*dy2);
	
	H = nearest(A, B, P);

	vLinePub(P, H);
	
    //垂線足と次のwaypointの距離
	float dx3 = H.x - B.x;
	float dy3 = H.y - B.y;
	float dist3 = sqrt(dx3*dx3 + dy3*dy3);
	//垂線足と現在のwaypointの距離
	float dx4 = H.x - A.x;
	float dy4 = H.y - A.y;
	float dist4 = sqrt(dx4*dx4 + dy4*dy4);
	

	cout<<"wp_count = "<<wp_count<<" dist1:"<<dist1<<"\t\twp_count = "<<wp_count+1<<" dist2:"<<dist2<<" dist3:"<<dist3<<endl;

	if (dist2<1.0){
		if(wp_count >= (int)waypoints.poses.size()-2){
			fin_flag = true;
		}else{
			cout<<"change!!"<<endl;
			wp_count++;
            system(ongaku_command.c_str());
		}
	}
}

void anglePub(nav_msgs::Path waypoints, int idid, geometry_msgs::PoseStamped& pub_pose)
{
	float angle1 = tf::getYaw(waypoints.poses[idid-1].pose.orientation);
	//float angle2 = tf::getYaw(waypoints.poses[idid-2].pose.orientation);
	//geometry_msgs::Quaternion angle_quat = tf::createQuaternionMsgFromYaw(angle1 - angle2);
	geometry_msgs::Quaternion angle_quat = tf::createQuaternionMsgFromYaw(angle1);
	pub_pose.pose.orientation = angle_quat;
	pub_pose.pose.position = waypoints.poses[idid-1].pose.position;
	pub_pose.header.frame_id = header_frame;
}

int main(int argc, char** argv){
	
	ros::init(argc, argv, "rwrc15_waypoint_manager");
	ros::NodeHandle n;

	cout<<"waypoint server"<<endl;	
	// for visualization//
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("localization/waypoints", 100);
	ros::Publisher t_circle_pub = n.advertise<visualization_msgs::Marker>("localization/traced_waypoint_circle", 100);
	ros::Publisher marker_pub4 = n.advertise<visualization_msgs::MarkerArray>("localization/waypoints/num", 100);
	marker_pub5 = n.advertise<visualization_msgs::Marker>("/waypoint/v", 100);
	//-------------------//
	ros::Publisher num_pub = n.advertise<std_msgs::Int32>("/waypoint/mode", 1);
	ros::Publisher target_pub = n.advertise<geometry_msgs::PoseStamped>("/target/pose", 100);
	ros::Publisher prev_wp_pub = n.advertise<geometry_msgs::PoseStamped>("/waypoint/prev", 100);
	ros::Publisher fin_pub = n.advertise<std_msgs::Bool>("/tinypower/finish_flag", 100);
	ros::Publisher poi_pub = n.advertise<sensor_msgs::PointCloud>("/waypoint/point_cloud", 1);
	
    // for human search ? this is not used now//
	ros::ServiceServer service = n.advertiseService("/plan/waypoints", callback);
	
    tf::TransformBroadcaster odom_broadcaster;
	tf::TransformListener listener1;
	
	//--------------initVisualizationMsgs-----------------//
	visualization_msgs::MarkerArray txts;
	visualization_msgs::Marker lines, points, now, fin;
	initVisualizationMsgs1(lines);
	initVisualizationMsgs2(points);
	now=points;
	initVisualizationMsgs3(now);
	initVisualizationMsgs4(fin);
	
	nav_msgs::Path waypoints;
	sensor_msgs::PointCloud poi_cl;
	
    bool tf_flag = false;
	int wp_count = 0;
	int id_now = 1;
	geometry_msgs::PoseStamped traced_waypoint;
	
    //-----------read and set waypoints-----------------//
    string file_name;
    nav_msgs::Path input_latlon;
    file_name = selectWaypointFile(n);
    readWaypointInXY(waypoints,file_name);
    //set the variables for visualization
    setMarkers(waypoints, lines, points, fin, txts);
    //calc relative angle between wp[i] and wp[i+1]
    waypointAngleSet(waypoints);
    //おそらく使ってない
    housePointCloud(poi_cl, waypoints);
    current = waypoints.poses[0];
    
    ros::Rate loop_rate(10); // ループの周期 
	while(ros::ok()){
		
        //get robot position on the global_coordinate(=map)
        tf::StampedTransform transform;
        try{
            listener1.lookupTransform(header_frame,"/matching_base_link",ros::Time(0),transform);
            tf_flag=true;
        }
        catch(tf::TransformException ex){
            // ROS_ERROR("%s",ex.what());
            cout<<"waiting for global_frame and robot_frame relation"<<endl;
            tf_flag = false;
        }
		
        if(tf_flag){	
            //copy robot position
            geometry_msgs::PoseStamped robo;
            robo.pose.position.x=transform.getOrigin().x();
            robo.pose.position.y=transform.getOrigin().y();
            robo.pose.position.z=0;
            float angle = tf::getYaw(transform.getRotation());
            robo.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,angle);

            whereAmI(robo, waypoints,wp_count);//ここで現在のwaypointの計算をしている
            id_now = wp_count + 1;
            
            std_msgs::Int32 now_num;
            now_num.data = id_now;
            num_pub.publish(now_num);
                
            //now.points[0]には次のwaypointの座標が入ってる
            cout<<"pub:"<<id_now<<endl;
            geometry_msgs::Point pt;
            pt.x = waypoints.poses[wp_count+1].pose.position.x;
            pt.y = waypoints.poses[wp_count+1].pose.position.y;	
            pt.z = 0;
            cout<<"count:"<<wp_count<<"	x:"<<pt.x<<"	y"<<pt.y<<endl;
            now.points.clear();
            now.points.push_back(pt);
            
            /////////////////Circle drawing!!!//////////////////////////
            geometry_msgs::PoseStamped now_state, traced_state, prev_wp;
            visualization_msgs::Marker circle_dr, traced_circle_dr;
            setCircleParameter(now, traced_waypoint, now_state, traced_state, circle_dr, traced_circle_dr);
            
            prev_wp.header.frame_id=header_frame;
            prev_wp.pose.position.x = waypoints.poses[wp_count].pose.position.x;
            prev_wp.pose.position.y = waypoints.poses[wp_count].pose.position.y;
            prev_wp.pose.position.z = 0.5;
            
            
            //おそらく探索モードに入ったときに次に向かうwaypointを忘れないための処置?
            {
                boost::mutex::scoped_lock(goal_mutex_);
                current=now_state;
                current_id_=id_now;
            }
            
            target_pub.publish(traced_waypoint);
            prev_wp_pub.publish(prev_wp);
            
            //////////////publish visualization msg/////////////
            marker_pub.publish(now);
            marker_pub.publish(circle_dr);
            t_circle_pub.publish(traced_circle_dr);
            poi_pub.publish(poi_cl);
        }

        // publish msg to stop when infant reaches final waypoint// //2015/02/21 
        if (wp_count==(int)waypoints.poses.size()-1){
            std_msgs::Bool fin_flag;
            fin_flag.data=true;
            fin_pub.publish(fin_flag);
            std::cout<<"******************************goal***********************************************************************"<<std::endl;
        }
		
        ////////////publish visualization msg/////////////
        marker_pub.publish(now);
        marker_pub.publish(points);
        marker_pub.publish(fin);
        marker_pub.publish(lines);
        marker_pub4.publish(txts);
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	ros::spin();
	return 0;
}
