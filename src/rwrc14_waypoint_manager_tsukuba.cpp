#include "ros/ros.h"
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
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
#include <rwrc14_msgs/GetGoalsId.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <math.h>
#include <vector>
#include "Visualize_lib.h"
#include <boost/thread.hpp>

#include "GPStoXY/GPStoXY.h"

using namespace std;


//	last update 2014.11.11 written by masada 


//-----------------原点座標入力---------------------//
//海上自衛隊浦郷 2012/12/10 追浜実験
//double start_latlon[2] = {139.6363793,35.3165939};
//旧研究室	ikutachallenge 2014
double start_latlon[2] = {35.613415,139.549145};
//いつものスタートマンホール
//double start_latlon[2] = {35.614784,139.549492};
//左折矢印の根元
//double start_latlon[2] = {35.614756,139.54951};
//駅伝
//double start_latlon[2] = {35.613921, 139.549194};

//tkb 1
//double start_latlon[2] = {36.086091,140.110657};
//tkb start
//double start_latlon[2] = {36.079341,140.115387};
//2014tkb 原点
//double start_latlon[2] = {36.079146,140.115312};
//-------------------------------------------------//


//-------------------------------------------------//


//const string file_name("/home/amsl/AMSL_ros_pkg/rwrc14/rwrc14_waypoint_manager/lat_lon_data/tkb2014.txt");

//const string file_name("/home/amsl/AMSL_ros_pkg/rwrc14_waypoint_manager/lat_lon_data/ikuta-2km_easy.txt");
const string file_name("/home/amsl/AMSL_ros_pkg/rwrc14/rwrc14_waypoint_manager/lat_lon_data/ikuta-2km_easy.txt");
//const string file_name("/home/amsl/AMSL_ros_pkg/rwrc14/rwrc14_waypoint_manager/lat_lon_data/ikuta-test.txt");

//const string file_name("/home/amsl/AMSL_ros_pkg/rwrc14/rwrc14_waypoint_manager/lat_lon_data/ikuta-msd1109.txt");//1109
//const string file_name("/home/amsl/AMSL_ros_pkg/rwrc14/rwrc14_waypoint_manager/lat_lon_data/Dkan.txt");//Dkan

const float CARROT_L = 15;
float now_Rc=10;
const float Rc = 10;
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

#define Change_WP_ID 1

ros::Publisher marker_pub5;
ros::Publisher carrot_pub;

boost::mutex odom_mutex_, goal_mutex_, init_mutex_,init2_mutex_;
geometry_msgs::TransformStamped frame_in;
geometry_msgs::PoseStamped current;
nav_msgs::Odometry initial_state_;
geometry_msgs::PoseWithCovarianceStamped initial_state2_;

bool msg_check_=true;
bool fin_flag=false;

int current_id_;

///////////////////////////////////////////////////////
//////-----------Callback function-------------////////
///////////////////////////////////////////////////////
void
initialStateCallback(const nav_msgs::OdometryConstPtr &msg)
{
	boost::mutex::scoped_lock(init_mutex_);
	initial_state_=*msg;
	cout<<"msgs GET!!!!!!!!!!!!!"<<endl;
	msg_check_=true;
}
void
initialState2Callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
	boost::mutex::scoped_lock(init2_mutex_);
	initial_state2_=*msg;
	initial_state_.pose=initial_state2_.pose;
	cout<<"msgs GET!!!!!!!!!!!!!"<<endl;
	msg_check_=true;
}


bool callback(rwrc14_msgs::GetGoalsId::Request  &req,
         rwrc14_msgs::GetGoalsId::Response &res )
{

	boost::mutex::scoped_lock(goal_mutex_);
	res.goals.clear();
	res.goals.push_back(current);
	res.id = current_id_;
	//res.finish = fin_flag;

	/*cout<<"server response way_point x:"<<current.pose.position.x<<endl;
	cout<<"server response way_point y:"<<current.pose.position.y<<endl;//*/

	return true;
}



///////////////////////////////////////////////////////
//////-------Input waypoint data from txt------////////
///////////////////////////////////////////////////////
void inputFromTxt(nav_msgs::Path& waypoints)
{
	ifstream ifs(file_name.c_str());
	string buf;
	int wp_count=0;
	double x,y,yaw;
	geometry_msgs::Point pt;
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
	cout<<"Input Complete!\t"<<wp_count<<endl;
}


void convertWaypoints(nav_msgs::Path waypoint, nav_msgs::Path& converts)
{
	
	GPStoXY* gps_xy;
	/////////////////////////初期位置(0, 0)///////////////////////////////
	//cout<<"initial_LAT = "<<start_latlon[1]<<"\tinitial_LON = "<<start_latlon[0]<<endl;
	gps_xy = new GPStoXY(start_latlon[1], start_latlon[0]);	
	//gps_xy = new GPStoXY;	
	gps_xy->Convert(start_latlon[1], start_latlon[0]);
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
	//lines.color.g=1.0;
	//lines.color.r=1.0;
	//lines.color.b=0.9;
	lines.color.a=0.9;
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
	int weypoint_n = waypoints.poses.size();
	for(int i=0; i<weypoint_n; i++) {
		pt.x = waypoints.poses[i].pose.position.x;
		pt.y = waypoints.poses[i].pose.position.y;
		pt.z = 0;
		
		geometry_msgs::PoseStamped point;
		point.pose.position.x = pt.x;
		point.pose.position.y = pt.y; 
		point.pose.position.z = pt.z;
		
		cout<<"count:"<<i<<"\tx = "<<pt.x<<"\ty = "<<pt.y<<"\tyaw = "<< yaw <<endl;
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
	cout<<"Input Complete!\t"<<weypoint_n<<endl;
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

	gap_btwn.x = P.x - H.x;
	gap_btwn.y = P.y - H.y;
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
				current_ps = path.poses[wp_count+1];
			}
		}

	}



	current_ps = path.poses[wp_count+1];
	
	if(dist3 < dist4){
		return dist3;
	}else{
		return dist4;
	}
}

float
calcDistance(float x1, float y1, float x2, float y2)
{
	float dis;
	
	dis = sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));

	return dis;
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
	
	ros::init(argc, argv, "rwrc14_waypoint_manager");
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
	carrot_pub = n.advertise<geometry_msgs::PoseStamped>("/waypoint/carrot", 100);
	
	ros::Subscriber sub = n.subscribe("localization/initial_state", 100, initialStateCallback);
	//ros::Subscriber sub = n.subscribe("/initialpose", 100, initialState2Callback);
	ros::ServiceServer service = n.advertiseService("/plan/waypoints", callback);
	tf::TransformBroadcaster odom_broadcaster;
	
	ros::Time current_time;
	std_msgs::ColorRGBA rgba_in;
	rgba_in.r = 0.8;
	rgba_in.g = 0.8;
	rgba_in.b = 0.0;
	rgba_in.a = 0.8;
	Visualization* marker_circle = new Visualization(rgba_in, ADD, 0.1, "area",header_frame);
	//--------------initVisualizationMsgs-----------------//
	visualization_msgs::Marker lines, points, now, fin;
	visualization_msgs::Marker lines_dr, points_dr, now_dr;
	initVisualizationMsgs1(lines);
	initVisualizationMsgs2(points);
	now=points;
	initVisualizationMsgs3(now);
	initVisualizationMsgs4(fin);
	lines_dr = lines;	points_dr = points;	now_dr = now;
	
	//--------------Input data from file-----------------//
	visualization_msgs::MarkerArray txts;
	nav_msgs::Path input_latlon, waypoints;
	sensor_msgs::PointCloud poi_cl;
	inputFromTxt(input_latlon);
	convertWaypoints(input_latlon, waypoints);
	setMarkers(waypoints, lines, points, fin, txts);
	waipointAngleSet(waypoints);
	housePointCloud(poi_cl, waypoints);
	
	tf::TransformListener listener1;
	//bool init_check=false;
	int wp_count = 0;//////////////////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	int id_now = 1;
	geometry_msgs::Pose2D gap_btwn;
	geometry_msgs::PoseStamped traced_waypoint;
	current = waypoints.poses[0];
	ros::Rate loop_rate(10); // ループの周期 
	while(ros::ok()){
		
		current_time = ros::Time::now();
		tf::StampedTransform transform;
		if(msg_check_==false){
		}else{
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
				pt.x=current_tmp.pose.position.x;	pt.y=current_tmp.pose.position.y;	pt.z=0;
				cout<<"count:"<<wp_count<<"	x:"<<pt.x<<"	y"<<pt.y<<endl;
				now.points.clear();
				now.points.push_back(pt);
				
				geometry_msgs::PoseStamped for_inter;
				anglePub(waypoints, id_now, for_inter);
				inter_pub.publish(for_inter);
				//TF publish
				geometry_msgs::TransformStamped odom_trans;
				setTf(for_inter, current_time, odom_trans);
				//--------------publish----------------//
				odom_broadcaster.sendTransform(odom_trans);
				
				
				////////////ここの関数がrvizへの描画を行っている/////////////
				marker_pub.publish(lines);
				marker_pub.publish(points);
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
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	ros::spin();
	return 0;
}
