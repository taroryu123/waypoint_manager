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
//#include <rwrc14_msgs/GetGoalsId.h>
//#include <rwrc14_msgs/GetGoalsId.h>
//#include <rwrc14_msgs/InitialValue.h>
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
//2015/02/22 592, 978行目追加
using namespace std;

int jrm_fin_wp_id;
//const int jrm_stop_wp_id=-1;
const int jrm_stop_wp_id=4;//この数字のwaypointに入るとINFANTは停止する
//	2014.11.11 written by masada-------------->added traced waypoint
//	2014.11.13 written by masanobu------------>added Initial-State subscriber


//-----------------原点座標入力---------------------//
//海上自衛隊浦郷 2012/12/10 追浜実験
//double start_latlon[2] = {139.6363793,35.3165939};
//旧研究室	ikutachallenge 2014
double start_latlon[2] = {35.613415,139.549145};
//いつものスタートマンホール
//double start_latlon[2] = {35.614784,139.549492};
//左折矢印の根元
//double start_latlon[2] = {35.614756,139.54951};//今までの
//double start_latlon[2] = {35.614745 139.549445};//0211try2
//double start_latlon[2] = {35.614784,139.54949};//0211try1
//駅伝
//double start_latlon[2] = {35.613921, 139.549194};

//tkb 1
//double start_latlon[2] = {36.086091,140.110657};
//tkb start
//double start_latlon[2] = {36.079341,140.115387};
//2014tkb 原点
double tkb_start_latlon[2] = {36.079131,140.115302};
//-------------------------------------------------//


//-------------------------------------------------//


//const string file_name("/home/amsl/AMSL_ros_pkg/rwrc14/rwrc14_waypoint_manager/lat_lon_data/tkb2014.txt");

//const string file_name("/home/amsl/AMSL_ros_pkg/rwrc14_waypoint_manager/lat_lon_data/ikuta-2km_easy.txt");
//const string file_name("/home/amsl/AMSL_ros_pkg/rwrc14/rwrc14_waypoint_manager/lat_lon_data/ikuta-2km_easy.txt");
//const string file_name("/home/amsl/AMSL_ros_pkg/rwrc14/rwrc14_waypoint_manager/lat_lon_data/ikuta-test.txt");

//const string file_name("/home/amsl/AMSL_ros_pkg/rwrc14/rwrc14_waypoint_manager/lat_lon_data/ikuta-msd1109.txt");//1109
//const string file_name("/home/amsl/AMSL_ros_pkg/rwrc14/rwrc14_waypoint_manager/lat_lon_data/Dkan.txt");//Dkan

const float CARROT_L = 15;
float now_Rc=7;
const float Rc = 7;
const float Rc2 = 5;
const int Rc2_id1 = 33443;
const int Rc2_id2 = 333;
const int Rc2_id3 = 346346;
const int Rc2_id4 = 345345;
//const int Rc2_id5 = 7;
//const int Rc2_id6 = 8;
//const int Rc2_id7 = 9;
const int Rc2_id8 = 33333;

const string header_frame("/map");
const string child_frame("/waypoint");
const string ongaku_command("gnome-terminal -e \"/home/amsl/AMSL_ros_pkg/l/script/beep.sh\" --geometry=0x0+0+0");

#define Change_WP_ID 1

ros::Publisher marker_pub5;
ros::Publisher carrot_pub;

geometry_msgs::TransformStamped frame_in;
geometry_msgs::PoseStamped current;

bool msg_check_ = false;
bool fin_flag = false;

int current_id_;

///////////////////////////////////////////////////////
//////-----------Callback function-------------////////
///////////////////////////////////////////////////////
boost::mutex init_mutex_;
rwrc14_msgs::InitialValue init_state_;
void initialStateCallback(const rwrc14_msgs::InitialValueConstPtr &msg)
{
	boost::mutex::scoped_lock(init_mutex_);
	init_state_ = *msg;
	cout<<"Initial setup msgs GET!!!!!!!!!!!!!"<<endl;
	msg_check_=true;
}

boost::mutex ps_mutex_;
geometry_msgs::PointStamped ps_;
void psCallback(const geometry_msgs::PointStampedConstPtr &msg)
{
	boost::mutex::scoped_lock(ps_mutex_);
	ps_ = *msg;
	cout<<"wayp_change"<<endl;
}

boost::mutex goal_mutex_;
bool callback(rwrc14_msgs::GetGoalsId::Request  &req,
         rwrc14_msgs::GetGoalsId::Response &res )
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

///////////////////////////////////////////////////////
//////-------Input waypoint data from txt------////////
///////////////////////////////////////////////////////
string selectWaypointFile(rwrc14_msgs::InitialValue init_state)
{

	string waypoint_file_name("/home/amsl/AMSL_ros_pkg/rwrc15_waypoint_manager/lat_lon_data/");
	if(init_state.LocateFlag==0){
		switch(init_state.course_num){
			cout<<"init_state.course_num "<<init_state.course_num<<endl;
			case 0:
				waypoint_file_name += "node_saver.txt";
				break;
/*
	string waypoint_file_name("/home/amsl/AMSL_ros_pkg/rwrc15/rwrc15_waypoint_manager/lat_lon_data/");
	if(init_state.LocateFlag==0){// if flag==0 --> ikuta file
		switch(init_state.course_num){
			cout<<"init_state.course_num "<<init_state.course_num<<endl;
			case 0:
				waypoint_file_name += "ikuta-2km_easy.txt";
				break;
			case 1:
				waypoint_file_name += "Dkan_lat_lon.txt";
				break;
			case 2:	
				waypoint_file_name += "forSI_C3.txt";
				break;
			case 3:	
				waypoint_file_name += "dkan_1shu_shimizu1.txt";
				break;
			case 4:	
				waypoint_file_name += "dkan_seimon.txt";
				break;
			case 5:	
				waypoint_file_name += "jrm15_1.txt";
				break;
			case 6:	
				waypoint_file_name += "jrm15_2.txt";
				break;
			case 7:	
				waypoint_file_name += "jrm15_3.txt";
				break;
			case 8:	
				waypoint_file_name += "jrm15_4.txt";
				break;
			case 9:	
				waypoint_file_name += "jrm15_5.txt";
				break;
			case 10:	
				waypoint_file_name += "jrm15_6.txt";
				break;
			case 11:	
				waypoint_file_name += "jrm15_7.txt";
				break;
			case 12:	
				waypoint_file_name += "jrm15_8.txt";
				break;
			case 13:	
				waypoint_file_name += "jrm15_11.txt";
				break;
			case 14:	
				waypoint_file_name += "jrm15_12.txt";
				break;
			case 15:	
				waypoint_file_name += "jrm15_13.txt";
				break;
			case 16:	
				waypoint_file_name += "jrm15_14.txt";
				break;
			case 17:	
				waypoint_file_name += "20150217map.txt";//D館内スタート->D館一周->D館内で停止
				break;
			case 18:	
				waypoint_file_name += "confirm_stop.txt";
				break;
			case 19:	
				waypoint_file_name += "DEntrance.txt";
				break;
			default:
				break;
		}
	}else if(init_state.LocateFlag==1){// if flag==1 --> tsukuba file
		switch(init_state.course_num){
			case 0:
				waypoint_file_name += "tkb2014.txt";
				break;
			case 1:
				waypoint_file_name += "tkb2014_msd.txt";
				break;
			case 2:
				waypoint_file_name += "tkb2014_slope.txt";
				break;
			default:
				waypoint_file_name += "tkb2014.txt";
				break;
*/
		}
	}
	
	return waypoint_file_name;
}

///////////////////////////////////////////////////////
//////-------Input waypoint data from txt------////////
///////////////////////////////////////////////////////
void inputFromTxt(nav_msgs::Path& waypoints, const string& file_name)
{
	ifstream ifs(file_name.c_str());
	string buf;
	int wp_count=0;
	double x,y,yaw;
	geometry_msgs::Point pt;
	if( !ifs ){
		ROS_FATAL( "Error:Non file!!!!!---------Is this file's name correct?\n------->%s\n",file_name.c_str());
		exit(0);
	}else{
		while(ifs && getline(ifs, buf)) {
			istringstream is( buf );
			is>>x>>y>>yaw;
			pt.x=x;
			pt.y=y;
			pt.z=0;
			
			geometry_msgs::PoseStamped point;
			point.pose.position.x=pt.x;
			point.pose.position.y=pt.y; 
			point.pose.position.z=pt.z;
			
			cout<<"count:"<<wp_count<<"\tx = "<<pt.x<<"\ty = "<<pt.y<<"\tyaw = "<< yaw <<endl;
			waypoints.poses.push_back(point);
			wp_count++;
		}
		for(int i=0; i<(int)waypoints.poses.size(); i++){
			cout<<"x : "<<waypoints.poses[i].pose.position.x<<" y : "<<waypoints.poses[i].pose.position.y<<endl;
		}
		cout<<"Input Complete!(inputFromTxt)\t"<<wp_count<<endl;
	}
}


void convertWaypoints(rwrc14_msgs::InitialValue init_state, 
						nav_msgs::Path waypoint, 
						nav_msgs::Path& converts)
{
	GPStoXY* gps_xy;
	/////////////////////////初期位置(0, 0)///////////////////////////////
	//---------GPStoXY(lon, lat)---------------//
	printf("Lat:%4.8f \tLon:%4.8f\n",init_state.InLat,init_state.InLon);
	if(init_state.LocateFlag)	gps_xy = new GPStoXY(tkb_start_latlon[1], tkb_start_latlon[0]);	
	else						gps_xy = new GPStoXY(start_latlon[1], start_latlon[0]);	
//	else						gps_xy = new GPStoXY();	
	//gps_xy = new GPStoXY;	
	//gps_xy->Convert(init_state.InLon, init_state.InLat);
	converts.poses.clear();		
	/////////////////////////目標位置(x, y)//////////////////////////
	geometry_msgs::PoseStamped p_g;
	p_g.header.frame_id=header_frame;
					
	for(int i = 0 ; i < (int)waypoint.poses.size() ; i++){
		float xx = waypoint.poses[i].pose.position.x;
		float yy = waypoint.poses[i].pose.position.y;
		gps_xy->Convert( yy, xx);
		
		cout<<"asdfafx = "<<gps_xy->X()<<"\tasdfy = "<<gps_xy->Y()<<endl;
		
		geometry_msgs::Pose pt;
		pt.position.x = gps_xy->X();
		pt.position.y = gps_xy->Y();
		pt.position.z = 0;
		p_g.pose.position = pt.position;
		p_g.pose.orientation.x = 0;
		p_g.pose.orientation.y = 0;
		p_g.pose.orientation.z = 0;
		p_g.pose.orientation.w = 0;
		
		converts.poses.push_back(p_g);
	}
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

///////////////////////////////////////////////////////
//////-----VisualizationMsgs Configuration-----////////
///////////////////////////////////////////////////////
void initVisualizationMsgs1(visualization_msgs::Marker& lines)
{
	lines.header.frame_id = header_frame;
	lines.header.stamp = ros::Time::now();
	lines.ns = "lines";
	//lines.action = visualization_msgs::Marker::ADD;
	lines.id = 0; // set each ID number of lines
	lines.type = visualization_msgs::Marker::LINE_STRIP;
		
	lines.pose.position.x=0;
	lines.pose.position.y=0;
	lines.pose.position.z=0;
	lines.pose.orientation.x = 0.0;
	lines.pose.orientation.y = 0.0;
	lines.pose.orientation.z = 0.0;
	lines.pose.orientation.w = 0.0;

	lines.scale.x=0.2;
	lines.scale.y=0.2;
	lines.scale.z=0.2;

	lines.color.g=0.0;
	lines.color.r=0.0;
	lines.color.b=1;
/*	lines.color.g=1.0;
	lines.color.r=1.0;
	lines.color.b=0.9;
*/	lines.color.a=0.9;
}

void initVisualizationMsgs2(visualization_msgs::Marker& points)
{
	points.header.frame_id=header_frame;
	points.header.stamp=ros::Time::now();
	points.ns="points";
	//points.action = visualization_msgs::Marker::ADD;
	points.id=1; // set each ID number of lines
	points.type=visualization_msgs::Marker::POINTS;
		
	points.pose.position.x=0;
	points.pose.position.y=0;
	points.pose.position.z=0.7;
	points.pose.orientation.x = 0.0;
	points.pose.orientation.y = 0.0;
	points.pose.orientation.z = 0.0;
	points.pose.orientation.w = 0.0;

	points.scale.x=0.7;
	points.scale.y=0.7;
	points.scale.z=0.7;

	//points.color.g=0.0;
	//points.color.r=0.0;
	//points.color.b=1;
	//points.color.a=0.8;
	points.color.g=0.0;
	points.color.r=1.0;
	points.color.b=1.0;
	points.color.a=0.7;
}

void initVisualizationMsgs3(visualization_msgs::Marker& now)
{
	now.header.frame_id=header_frame;
	now.header.stamp=ros::Time::now();
	now.ns="now";
	//now.action = visualization_msgs::Marker::ADD;
	now.id=2; // set each ID number of lines
	now.type=visualization_msgs::Marker::CUBE_LIST;
		
	now.pose.position.x=0;
	now.pose.position.y=0;
	now.pose.position.z=0.7;
	now.pose.orientation.x = 0.0;
	now.pose.orientation.y = 0.0;
	now.pose.orientation.z = 0.0;
	now.pose.orientation.w = 0.0;

	now.scale.x=0.4;
	now.scale.y=0.4;
	now.scale.z=0.7;

	now.color.a=0.8;
	now.color.g=0.8;
	now.color.r=1;
	now.color.b=0;
}
void initVisualizationMsgs4(visualization_msgs::Marker& points)
{
	points.header.frame_id=header_frame;
	points.header.stamp=ros::Time::now();
	points.ns="goal";
	//points.action = visualization_msgs::Marker::ADD;
	points.id=2; // set each ID number of lines
	points.type=visualization_msgs::Marker::ARROW;
	points.scale.x=1.2;
	points.scale.y=0.2;
	points.scale.z=0.2;

	points.color.a=0.8;
	points.color.g=0.2;
	points.color.r=0.9;
	points.color.b=0.0;
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


void carrotPub(geometry_msgs::Pose2D carrot_p)
{
	geometry_msgs::PoseStamped c_pub;
	c_pub.header.frame_id = header_frame;
	c_pub.header.stamp = ros::Time::now();
	c_pub.pose.position.x = carrot_p.x;
	c_pub.pose.position.y = carrot_p.y;
	c_pub.pose.position.z = 0;
	c_pub.pose.orientation = tf::createQuaternionMsgFromYaw(carrot_p.theta);
	
	carrot_pub.publish(c_pub);
}

geometry_msgs::Pose2D nearest(	geometry_msgs::Pose2D A, 
								geometry_msgs::Pose2D B, 
								geometry_msgs::Pose2D P )
{
	geometry_msgs::Pose2D a, b;
	double r;
	
	a.x = B.x - A.x;
	a.y = B.y - A.y;
	b.x = P.x - A.x;
	b.y = P.y - A.y;
	
	r = (a.x*b.x + a.y*b.y) / (a.x*a.x + a.y*a.y);
	
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
//void inputFromTxt(nav_msgs::Path& waypoints,
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
	cout<<"waypoints.poses.size() : "<<waypoints.poses.size()<<endl;
	int waypoint_n = waypoints.poses.size();//semi-waypointを10個ずつ設置
	for(int i=0; i<waypoint_n; i++) {
		pt.x = waypoints.poses[i].pose.position.x;
		pt.y = waypoints.poses[i].pose.position.y;
		pt.z = 0;

		geometry_msgs::PoseStamped point;
		point.pose.position.x = pt.x;
		point.pose.position.y = pt.y; 
		point.pose.position.z = pt.z;
		
		//cout<<"count:"<<i<<"\tx = "<<pt.x<<"\ty = "<<pt.y<<"\tyaw = "<< yaw <<endl;
		visualization_msgs::Marker txt1;
		marker_txt->txtMarker(IntToString(i), point, txt1, i);
		
		txts.markers.push_back(txt1);
		waypoints.poses.push_back(point);
		lines.points.push_back(pt);
		points.points.push_back(pt);
	}
/*
	for(int i=0; i<waypoint_n-1; i++) {
		for(int j=0; j<10; j++)	{
			pt.x = (waypoints.poses[i+1].pose.position.x - waypoints.poses[i].pose.position.x)/10*j+waypoints.poses[i].pose.position.x;
			pt.y = (waypoints.poses[i+1].pose.position.y - waypoints.poses[i].pose.position.y)/10*j+waypoints.poses[i].pose.position.y;
			pt.z = 0;
		
			geometry_msgs::PoseStamped point;
			point.pose.position.x = pt.x;
			point.pose.position.y = pt.y; 
			point.pose.position.z = pt.z;
		
			//cout<<"count:"<<i<<"\tx = "<<pt.x<<"\ty = "<<pt.y<<"\tyaw = "<< yaw <<endl;
			visualization_msgs::Marker txt1;
			marker_txt->txtMarker(IntToString(i*10+j), point, txt1, i*10+j);
		
			txts.markers.push_back(txt1);
			waypoints.poses.push_back(point);
			lines.points.push_back(pt);
			points.points.push_back(pt);
		}
	}
*/
	geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,yaw);
	fin.pose.position = pt;
	fin.pose.orientation = pose_quat;
	cout<<"Input Complete!(setMarkers)\t"<<waypoint_n<<endl;
}

void waipointAngleSet(nav_msgs::Path& waypoints)
{
	for(unsigned int i=0; i<waypoints.poses.size()-1; i++){
		float angle=atan2(waypoints.poses[i+1].pose.position.y - waypoints.poses[i].pose.position.y,
				waypoints.poses[i+1].pose.position.x - waypoints.poses[i].pose.position.x);
		cout<<"i = "<<i<<"\tangle = "<<angle<<endl;
		geometry_msgs::Quaternion angle_quat = tf::createQuaternionMsgFromYaw(angle);
		waypoints.poses[i].pose.orientation=angle_quat;
	}
}


float
calcDistance(float x1, float y1, float x2, float y2)
{
	float dis;
	
	dis = sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));

	return dis;
}


void checkNearestPoint(int& now_id, geometry_msgs::PointStamped ps, nav_msgs::Path waypoints)
{
	static unsigned int pre_seq;
	jrm_fin_wp_id=now_id;//2017/02/21追加
	if(pre_seq != ps.header.seq){
		pre_seq = ps.header.seq;
		int id = 0;
		float min_dist = INFINITY;
		for(unsigned int i=0; i<waypoints.poses.size()-1; i++){
			float dx = waypoints.poses[i].pose.position.x - ps.point.x;
			float dy = waypoints.poses[i].pose.position.y - ps.point.y;
			float distance = sqrt(dx*dx + dy*dy);
			if(min_dist > distance){
				min_dist = distance;
				id = i;
			}
		}
		now_id = id;
	}else{
		cout<<"wp no change!"<<endl;
	}
}

void convertGlobalAngleTo(float& angle)
{
	angle = -(angle - M_PI_2);
}


geometry_msgs::Pose2D carrotSeeking(float length, 
									geometry_msgs::Pose2D now_wayp, 
									geometry_msgs::Pose2D next_wayp, 
									geometry_msgs::Pose2D H)
{
	geometry_msgs::Pose2D carrot_goal, target, robot;
	//target input
	target.x = length;
	target.y = 0;
	target.theta = 0;
	//for angle calculate
	float dx_wayp = next_wayp.x - now_wayp.x;
	float dy_wayp = next_wayp.y - now_wayp.y;
	float wayp_angle = atan2(dy_wayp,dx_wayp);
	//for distans calculate
	float dx_H2wayp = next_wayp.x - H.x;
	float dy_H2wayp = next_wayp.y - H.y;
	float distans_H2wayp = sqrt(dx_H2wayp*dx_H2wayp + dy_H2wayp*dy_H2wayp);
	
	robot.x = H.x;
	robot.y = H.y;
	robot.theta = wayp_angle;
	
	pose2DLocal2Global(target, robot, carrot_goal);
	
	if(distans_H2wayp < length){
		return next_wayp;
	}else{
		return carrot_goal;
	}
}


float whereIsMe(	geometry_msgs::PoseStamped& current_ps,
					const geometry_msgs::PoseStamped& robo,
					const nav_msgs::Path& path,
					int& wp_count,
					geometry_msgs::Pose2D& gap_btwn
					)
{
	
	geometry_msgs::Pose2D A, B, C, P, H, carrot_goal;
	P.x = robo.pose.position.x;
	P.y = robo.pose.position.y;
	A.x = path.poses[wp_count].pose.position.x;
	A.y = path.poses[wp_count].pose.position.y;
	B.x = path.poses[wp_count+1].pose.position.x;
	B.y = path.poses[wp_count+1].pose.position.y;
	//ロボットと現在のwaypointの距離
	float dx1 = P.x - A.x;
	float dy1 = P.y - A.y;
	float dist1 = sqrt(dx1*dx1 + dy1*dy1);
	//ロボットと次のwaypointの距離
	float dx2 = P.x - B.x;
	float dy2 = P.y - B.y;
	float dist2 = sqrt(dx2*dx2 + dy2*dy2);
	
	H = nearest(A, B, P);

	//gap_btwn.x = P.x - H.x;
	//gap_btwn.y = P.y - H.y;
	gap_btwn.x = 0;
	gap_btwn.y = 0;
	cout<<"gap.x:"<<gap_btwn.x<<" gap.y:"<<gap_btwn.y<<endl;

	vLinePub(P, H);
	carrot_goal = carrotSeeking(CARROT_L, A, B, H);
	carrotPub(carrot_goal);
	//垂線足と次のwaypointの距離
	float dx3 = H.x - B.x;
	float dy3 = H.y - B.y;
	float dist3 = sqrt(dx3*dx3 + dy3*dy3);
	//垂線足と現在のwaypointの距離
	float dx4 = H.x - A.x;
	float dy4 = H.y - A.y;
	float dist4 = sqrt(dx4*dx4 + dy4*dy4);
	
	float dx5 = H.x - B.x - gap_btwn.x;
	float dy5 = H.y - B.y - gap_btwn.y;
	float dist5 = sqrt(dx5*dx5 + dy5*dy5);

	cout<<"wp_count = "<<wp_count<<" dist1:"<<dist1<<"\t\twp_count = "<<wp_count+1<<" dist2:"<<dist2<<" dist3:"<<dist3<<endl;
	//if((dist2 < dist1)||(dist2 < Rc)){
	//if(dist2 < Rc){

	/*switch(wp_count){
		case Rc2_id1:
		case Rc2_id2:
		case Rc2_id3:
		case Rc2_id4:
		//case Rc2_id5:
		//case Rc2_id6:
		//case Rc2_id7:
		case Rc2_id8:
			now_Rc=Rc2;
			if( (dist2 < Rc2) || (dist3 < Rc2) || (dist5 < Rc2)){
				if(wp_count >= (int)path.poses.size()-2){
					fin_flag = true;
				}else{
					cout<<"change!!"<<endl;
					wp_count++;
					current_ps = path.poses[wp_count+1];
				}
			}
			break;
		default:
			now_Rc=Rc;
			if( (dist2 < Rc) || (dist3 < Rc) || (dist5 < Rc2)){
				if(wp_count >= (int)path.poses.size()-2){
					fin_flag = true;
				}else{
					cout<<"change!!"<<endl;
					wp_count++;
					current_ps = path.poses[wp_count+1];
				}
			}
			break;
	}*/

	if(wp_count<Change_WP_ID){
		now_Rc=Rc2;
		if( (dist2 < Rc2) || (dist3 < Rc2) || (dist5 < Rc2)){
			if(wp_count >= (int)path.poses.size()-2){
				fin_flag = true;
			}else{
				cout<<"change!!"<<endl;
				wp_count++;
				system(ongaku_command.c_str());
				current_ps = path.poses[wp_count+1];
			}
		}
	}else{

		now_Rc=Rc;
		if( (dist2 < Rc) || (dist3 < Rc) || (dist5 < Rc2)){
			if(wp_count >= (int)path.poses.size()-2){
				fin_flag = true;
			}else{
				cout<<"change!!"<<endl;
				wp_count++;
				system(ongaku_command.c_str());
				current_ps = path.poses[wp_count+1];
			}
		}

	}
	current_ps = path.poses[wp_count+1];
	//cout<<"current_ps : "<<current_ps<<endl;
	
	if(dist3 < dist4){
		return dist3;
	}else{
		return dist4;
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

void setTf(geometry_msgs::PoseStamped pose,ros::Time current_time,
			geometry_msgs::TransformStamped& odom_trans)
{
	odom_trans.header.stamp = current_time;
	odom_trans.header.frame_id = header_frame;
	odom_trans.child_frame_id = child_frame;
	odom_trans.transform.translation.x = pose.pose.position.x;
	odom_trans.transform.translation.y = pose.pose.position.y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = pose.pose.orientation;
}

int main(int argc, char** argv){
	
	ros::init(argc, argv, "sgt15_waypoint_manager");
	ros::NodeHandle n;

	cout<<"waypoint server"<<endl;	
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("localization/waypoints", 100);
	ros::Publisher t_circle_pub = n.advertise<visualization_msgs::Marker>("localization/traced_waypoint_circle", 100);
	//ros::Publisher marker_pub2 = n.advertise<visualization_msgs::Marker>("localization/waypoints/dr", 100);
	ros::Publisher marker_pub4 = n.advertise<visualization_msgs::MarkerArray>("localization/waypoints/num", 100);
	marker_pub5 = n.advertise<visualization_msgs::Marker>("/waypoint/v", 100);
	ros::Publisher inter_pub = n.advertise<geometry_msgs::PoseStamped>("/waypoint/inter", 100);
	ros::Publisher dist_pub = n.advertise<std_msgs::Float32>("/waypoint/distance", 1);
	ros::Publisher num_pub = n.advertise<std_msgs::Int32>("/waypoint/now", 1);
	ros::Publisher poi_pub = n.advertise<sensor_msgs::PointCloud>("/waypoint/point_cloud", 1);
	ros::Publisher target_pub = n.advertise<geometry_msgs::PoseStamped>("/target/pose", 100);
	ros::Publisher prev_wp_pub = n.advertise<geometry_msgs::PoseStamped>("/waypoint/prev", 100);
	ros::Publisher jrm_fin_pub = n.advertise<std_msgs::Bool>("/tinypower/finish_flag", 100);//2015/02/21
	carrot_pub = n.advertise<geometry_msgs::PoseStamped>("/waypoint/carrot", 100);
	
	ros::Subscriber sub = n.subscribe("/ini_pose", 100, initialStateCallback);
	ros::Subscriber sub2 = n.subscribe("/clicked_point", 100, psCallback);
	ros::ServiceServer service = n.advertiseService("/plan/waypoints", callback);
	tf::TransformBroadcaster odom_broadcaster;
	tf::TransformListener listener1;
	
	ros::Time current_time;
	std_msgs::ColorRGBA rgba_in;
	rgba_in.r = 0.8;
	rgba_in.g = 0.8;
	rgba_in.b = 0.0;
	rgba_in.a = 0.8;
	Visualization* marker_circle = new Visualization(rgba_in, ADD, 0.1, "area",header_frame);
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
	
	int wp_count = 0;//////////////////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	int id_now = 1;
	bool initialize_flag = false;
	geometry_msgs::Pose2D gap_btwn;
	geometry_msgs::PoseStamped traced_waypoint;
	ros::Rate loop_rate(10); // ループの周期 
	while(ros::ok()){
		
		current_time = ros::Time::now();
		tf::StampedTransform transform;
		rwrc14_msgs::InitialValue init_state;
		{
			boost::mutex::scoped_lock(init_mutex_);
			init_state = init_state_;
		}
		geometry_msgs::PointStamped ps;
		{
			boost::mutex::scoped_lock(ps_mutex_);
			ps = ps_;
		}
		if(msg_check_==false){
			cout<<"Waiting for initial msgs!!"<<endl;
	}else{
			if(!initialize_flag){
				//--------------Input data from file-----------------//
				string file_name;
				nav_msgs::Path input_latlon;
				file_name = selectWaypointFile(init_state);
				inputFromTxt( input_latlon, file_name);
				convertWaypoints(init_state, input_latlon, waypoints);
				setMarkers(waypoints, lines, points, fin, txts);
				waipointAngleSet(waypoints);
				housePointCloud(poi_cl, waypoints);
				current = waypoints.poses[0];
				initialize_flag = true;
			}else{
				cout<<"Initialized already!!"<<endl;
			}
			
			bool tf_ok_flag = true;
			try{
				listener1.lookupTransform(header_frame,"/matching_base_link",ros::Time(0),transform);
			}
			catch(tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
				tf_ok_flag = false;
			}
			//global_position Inmporting
			geometry_msgs::PoseStamped robo;
			robo.pose.position.x=transform.getOrigin().x();
			robo.pose.position.y=transform.getOrigin().y();
			robo.pose.position.z=0;
			float angle = tf::getYaw(transform.getRotation());
			robo.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,angle);

			float distance = calcDistance(0, 0, robo.pose.position.x, robo.pose.position.y);
			if(distance<1000000000 && tf_ok_flag){	
				checkNearestPoint(wp_count, ps, waypoints);
				geometry_msgs::PoseStamped current_tmp;
				std_msgs::Float32 dist_way;
				dist_way.data = whereIsMe(current_tmp, robo, waypoints,wp_count, gap_btwn);//ここで現在のwaypointの計算をしている
				id_now = wp_count + 1;
				dist_pub.publish(dist_way);
				
				std_msgs::Int32 now_num;
				now_num.data = id_now;
				num_pub.publish(now_num);
					
				//	pre -> 一個前	near -> 次のやつ	now->今のやつ
				//cout<<"near id:"<<id<<" id_now:"<<id_now<<endl;
				cout<<"pub:"<<id_now<<endl;
				geometry_msgs::Point pt;
				//pt.x=current.pose.position.x;	pt.y=current.pose.position.y;	pt.z=0;
				pt.x = waypoints.poses[wp_count+1].pose.position.x;
				pt.y = waypoints.poses[wp_count+1].pose.position.y;	
				pt.z = 0;
				cout<<"count:"<<wp_count<<"	x:"<<pt.x<<"	y"<<pt.y<<endl;
				now.points.clear();
				now.points.push_back(pt);
				
				////////////ここの関数がrvizへの描画を行っている/////////////
				marker_pub.publish(now);
				marker_pub.publish(fin);
				
				///////////////Circle drawing!!!//////////////////////////
				geometry_msgs::PoseStamped now_state, traced_state,prev_wp;
				visualization_msgs::Marker circle_dr, traced_circle_dr;
				now_state.pose.position.x = now.points[0].x;
				now_state.pose.position.y = now.points[0].y;
				now_state.pose.position.z = now.points[0].z;
				
				traced_state.pose.position.x = now.points[0].x + gap_btwn.x;
				traced_state.pose.position.y = now.points[0].y + gap_btwn.y;
				traced_state.pose.position.z = now.points[0].z;
				
				traced_waypoint.header.frame_id="/map";
				traced_waypoint.pose.position.x = traced_state.pose.position.x;
				traced_waypoint.pose.position.y = traced_state.pose.position.y;
				traced_waypoint.pose.position.z = 0.5;
				
				prev_wp.header.frame_id="/map";
				prev_wp.pose.position.x = waypoints.poses[wp_count].pose.position.x;
				prev_wp.pose.position.y = waypoints.poses[wp_count].pose.position.y;
				prev_wp.pose.position.z = 0.5;
				
				marker_circle->potint2MarkerCircles(circle_dr, now_state, 222, now_Rc);
				marker_circle->potint2MarkerCircles(traced_circle_dr, traced_state, 222, now_Rc);
				
				{
					boost::mutex::scoped_lock(goal_mutex_);
					current=now_state;
					current_id_=id_now;
				}
				
				marker_pub.publish(circle_dr);
				t_circle_pub.publish(traced_circle_dr);
				
				target_pub.publish(traced_waypoint);
				prev_wp_pub.publish(prev_wp);
				
				marker_pub4.publish(txts);
				poi_pub.publish(poi_cl);
				cout<<"----------------------------------------------------------------"<<endl;
			}
			geometry_msgs::PoseStamped for_inter;
			anglePub(waypoints, id_now, for_inter);
			inter_pub.publish(for_inter);
			//TF publish
			geometry_msgs::TransformStamped odom_trans;
			setTf(for_inter, current_time, odom_trans);
			//--------------publish----------------//
			odom_broadcaster.sendTransform(odom_trans);
			marker_pub.publish(lines);
			marker_pub.publish(points);
		}

		//publish msg to stop when infant reaches final waypoint// //2015/02/21 
		if (jrm_fin_wp_id==jrm_stop_wp_id){
			std_msgs::Bool jrm_fin_flag;
			jrm_fin_flag.data=true;
			jrm_fin_pub.publish(jrm_fin_flag);
			std::cout<<"******************************goal***********************************************************************"<<std::endl;
	    }
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	ros::spin();
	return 0;
}
