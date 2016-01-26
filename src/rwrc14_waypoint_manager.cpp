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

int jrm_fin_wp_id;
const int jrm_stop_wp_id=7;//この数字のwaypointに入るとINFANTは停止する

//	2014.11.11 written by masada-------------->added traced waypoint
//	2014.11.13 written by masanobu------------>added Initial-State subscriber

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

//wp_mngr.cpp, init_visu.cpp, read_wp_file.cppのheader_frameを使用するグローバル座標系の名前に変更すること//
const string header_frame("/map");
const string child_frame("/waypoint");
const string ongaku_command("gnome-terminal -e \"/home/amsl/AMSL_ros_pkg/l/script/beep.sh\" --geometry=0x0+0+0");

#define Change_WP_ID 1

ros::Publisher marker_pub5;
// ros::Publisher carrot_pub;

geometry_msgs::TransformStamped frame_in;
geometry_msgs::PoseStamped current;

bool msg_check_ = false;
bool fin_flag = false;

int current_id_;

///////////////////////////////////////////////////////
//////-----------Callback function-------------////////
///////////////////////////////////////////////////////
boost::mutex init_mutex_;
rwrc15_msgs::InitialValue init_state_;
void initialStateCallback(const rwrc15_msgs::InitialValueConstPtr &msg)
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

void setCircleParameter(visualization_msgs::Marker& now, geometry_msgs::Pose2D& gap_btwn, geometry_msgs::PoseStamped& traced_waypoint,
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
    
    traced_state.pose.position.x = now.points[0].x + gap_btwn.x;
    traced_state.pose.position.y = now.points[0].y + gap_btwn.y;
    traced_state.pose.position.z = now.points[0].z;
    
    traced_waypoint.header.frame_id=header_frame;
    traced_waypoint.pose.position.x = traced_state.pose.position.x;
    traced_waypoint.pose.position.y = traced_state.pose.position.y;
    traced_waypoint.pose.position.z = 0.5;
    
    marker_circle->potint2MarkerCircles(circle_dr, now_state, 222, now_Rc);
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


// void carrotPub(geometry_msgs::Pose2D carrot_p)
// {
	// geometry_msgs::PoseStamped c_pub;
	// c_pub.header.frame_id = header_frame;
	// c_pub.header.stamp = ros::Time::now();
	// c_pub.pose.position.x = carrot_p.x;
	// c_pub.pose.position.y = carrot_p.y;
	// c_pub.pose.position.z = 0;
	// c_pub.pose.orientation = tf::createQuaternionMsgFromYaw(carrot_p.theta);
	
	// carrot_pub.publish(c_pub);
// }

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
	cout<<"Input Complete!\t"<<waypoint_n<<endl;
}

void waypointAngleSet(nav_msgs::Path& waypoints)
{
	for(unsigned int i=0; i<waypoints.poses.size()-1; i++){
		float angle=atan2(waypoints.poses[i+1].pose.position.y - waypoints.poses[i].pose.position.y,
				waypoints.poses[i+1].pose.position.x - waypoints.poses[i].pose.position.x);
		cout<<"i = "<<i<<"\tangle = "<<angle<<endl;
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

void checkNearestPoint(int& now_id, geometry_msgs::PointStamped ps, nav_msgs::Path waypoints)
{
	static unsigned int pre_seq;
	jrm_fin_wp_id=now_id;//2015/02/21追加
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

// geometry_msgs::Pose2D carrotSeeking(float length, 
									// geometry_msgs::Pose2D now_wayp, 
									// geometry_msgs::Pose2D next_wayp, 
									// geometry_msgs::Pose2D H)
// {
	// geometry_msgs::Pose2D carrot_goal, target, robot;
	// //target input
	// target.x = length;
	// target.y = 0;
	// target.theta = 0;
	// //for angle calculate
	// float dx_wayp = next_wayp.x - now_wayp.x;
	// float dy_wayp = next_wayp.y - now_wayp.y;
	// float wayp_angle = atan2(dy_wayp,dx_wayp);
	// //for distans calculate
	// float dx_H2wayp = next_wayp.x - H.x;
	// float dy_H2wayp = next_wayp.y - H.y;
	// float distans_H2wayp = sqrt(dx_H2wayp*dx_H2wayp + dy_H2wayp*dy_H2wayp);
	
	// robot.x = H.x;
	// robot.y = H.y;
	// robot.theta = wayp_angle;
	
	// pose2DLocal2Global(target, robot, carrot_goal);
	
	// if(distans_H2wayp < length){
		// return next_wayp;
	// }else{
		// return carrot_goal;
	// }
// }


float whereIsMe(	geometry_msgs::PoseStamped& current_ps,
					const geometry_msgs::PoseStamped& robo,
					const nav_msgs::Path& path,
					int& wp_count,
					geometry_msgs::Pose2D& gap_btwn
					)
{
	
	geometry_msgs::Pose2D A, B, C, P, H, carrot_goal;
	//P: robot position, A:now waypoints, B:next waypoints
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
	// carrot_goal = carrotSeeking(CARROT_L, A, B, H);
	// carrotPub(carrot_goal);
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
	
	ros::init(argc, argv, "rwrc15_waypoint_manager");
	ros::NodeHandle n;

	cout<<"waypoint server"<<endl;	
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("localization/waypoints", 100);
	ros::Publisher t_circle_pub = n.advertise<visualization_msgs::Marker>("localization/traced_waypoint_circle", 100);
	//ros::Publisher marker_pub2 = n.advertise<visualization_msgs::Marker>("localization/waypoints/dr", 100);
	ros::Publisher marker_pub4 = n.advertise<visualization_msgs::MarkerArray>("localization/waypoints/num", 100);
	marker_pub5 = n.advertise<visualization_msgs::Marker>("/waypoint/v", 100);
    //おそらく使っていない
	ros::Publisher inter_pub = n.advertise<geometry_msgs::PoseStamped>("/waypoint/inter", 100);
	ros::Publisher dist_pub = n.advertise<std_msgs::Float32>("/waypoint/distance", 1);
	ros::Publisher num_pub = n.advertise<std_msgs::Int32>("/waypoint/now", 1);
	ros::Publisher poi_pub = n.advertise<sensor_msgs::PointCloud>("/waypoint/point_cloud", 1);
	ros::Publisher target_pub = n.advertise<geometry_msgs::PoseStamped>("/target/pose", 100);
	ros::Publisher prev_wp_pub = n.advertise<geometry_msgs::PoseStamped>("/waypoint/prev", 100);
	ros::Publisher jrm_fin_pub = n.advertise<std_msgs::Bool>("/tinypower/finish_flag", 100);//2015/02/21
	// carrot_pub = n.advertise<geometry_msgs::PoseStamped>("/waypoint/carrot", 100);
	
	ros::Subscriber sub = n.subscribe("/ini_pose", 100, initialStateCallback);
    // for human search ? this is not used now//
	ros::ServiceServer service = n.advertiseService("/plan/waypoints", callback);
	
    tf::TransformBroadcaster odom_broadcaster;
	tf::TransformListener listener1;
	
	ros::Time current_time;
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
	
	int wp_count = 0;
	int id_now = 1;
	bool initialize_flag = false;
	geometry_msgs::Pose2D gap_btwn;
	geometry_msgs::PoseStamped traced_waypoint;
	
    ros::Rate loop_rate(10); // ループの周期 
	while(ros::ok()){
		
        current_time = ros::Time::now();
        rwrc15_msgs::InitialValue init_state;
        {
            boost::mutex::scoped_lock(init_mutex_);
            init_state = init_state_;
        }
        geometry_msgs::PointStamped ps;
        {
            boost::mutex::scoped_lock(ps_mutex_);
            ps = ps_;
        }
		
        //wait for intial pose msg//
        if(msg_check_==false){
            cout<<"Waiting for initial msgs!!"<<endl;
        }else{
            //when intial pose msg come, read text file//
            if(!initialize_flag){
                string file_name;
                nav_msgs::Path input_latlon;
                file_name = selectWaypointFile(n);
                inputFromTxt( input_latlon, file_name);
                convertWaypoints(init_state, input_latlon, waypoints);
                //set the variables for visualization
                setMarkers(waypoints, lines, points, fin, txts);
                //calc relative angle between wp[i] and wp[i+1]
                waypointAngleSet(waypoints);
                //おそらく使ってない
                housePointCloud(poi_cl, waypoints);
                current = waypoints.poses[0];
                initialize_flag = true;
            }else{
                cout<<"Initialized already!!"<<endl;
            }
			
            //get robot position on the global_coordinate(=map)
            bool tf_ok_flag = true;
            tf::StampedTransform transform;
            try{
                listener1.lookupTransform(header_frame,"/matching_base_link",ros::Time(0),transform);
            }
            catch(tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
                tf_ok_flag = false;
            }
			
            //copy robot position
            geometry_msgs::PoseStamped robo;
            robo.pose.position.x=transform.getOrigin().x();
            robo.pose.position.y=transform.getOrigin().y();
            robo.pose.position.z=0;
            float angle = tf::getYaw(transform.getRotation());
            robo.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,angle);

            //"tf is ok" and "check whether robot position is not correct"
            float distance = calcDistance(0, 0, robo.pose.position.x, robo.pose.position.y);
            if(distance<1000000000 && tf_ok_flag){	
                //checkNearestPoint(wp_count, ps, waypoints);//おそらく探索が終わってから一番近いwaypointに復帰するため?
                checkNearestPoint(wp_count, ps, waypoints);
                geometry_msgs::PoseStamped current_tmp;
                std_msgs::Float32 dist_way;
                dist_way.data = whereIsMe(current_tmp, robo, waypoints,wp_count, gap_btwn);//ここで現在のwaypointの計算をしている
                id_now = wp_count + 1;
                dist_pub.publish(dist_way);
                
                std_msgs::Int32 now_num;
                now_num.data = id_now;
                num_pub.publish(now_num);
                    
                //push_backをしてるがclearをすることでnow.points[0]には次のwaypointの座標が入ってる
                //多分push_backの意味はないから後で直す+now
                cout<<"pub:"<<id_now<<endl;
                geometry_msgs::Point pt;
                pt.x = waypoints.poses[wp_count+1].pose.position.x;
                pt.y = waypoints.poses[wp_count+1].pose.position.y;	
                pt.z = 0;
                cout<<"count:"<<wp_count<<"	x:"<<pt.x<<"	y"<<pt.y<<endl;
                now.points.clear();
                now.points.push_back(pt);
                
                ///////////////Circle drawing!!!//////////////////////////
                geometry_msgs::PoseStamped now_state, traced_state, prev_wp;
                visualization_msgs::Marker circle_dr, traced_circle_dr;
                setCircleParameter(now, gap_btwn, traced_waypoint, now_state, traced_state, circle_dr, traced_circle_dr);
                
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
                
                ////////////publish visualization msg/////////////
                marker_pub.publish(now);
                marker_pub.publish(fin);
                marker_pub.publish(lines);
                marker_pub.publish(points);
                marker_pub.publish(circle_dr);
                t_circle_pub.publish(traced_circle_dr);
                marker_pub4.publish(txts);
                poi_pub.publish(poi_cl);
            }
            //--------------publish----------------//
            //おそらく使っていない
            geometry_msgs::PoseStamped for_inter;
            anglePub(waypoints, id_now, for_inter);
            inter_pub.publish(for_inter);
            //TF publish
            geometry_msgs::TransformStamped odom_trans;
            setTf(for_inter, current_time, odom_trans);
            odom_broadcaster.sendTransform(odom_trans);
        }

        // publish msg to stop when infant reaches final waypoint// //2015/02/21 
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
