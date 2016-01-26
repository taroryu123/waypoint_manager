
#include <ros/ros.h>
#include <stdlib.h>
#include <math.h>
#include <vector>

using namespace std;

struct point{
	float x;
	float y;
};

void smooth(vector<point> &waypoint,const float weight_data,const float weight_smooth,const float tolerance)
{
	vector<point> path = waypoint;
	
	float change = tolerance;
	while(change >= tolerance){
		change = 0.0;
		for(int i=1;i<waypoint.size()-1;i++){
				point aux  = path[i];
	//			cout<<"i="<<i<<":x="<<path[i].x<<" y="<<path[i].y<<endl;
				path[i].x += weight_data * (waypoint[i].x - path[i].x) + weight_smooth * (path[i+1].x + path[i-1].x - (2*path[i].x));
				path[i].y += weight_data * (waypoint[i].y - path[i].y) + weight_smooth * (path[i+1].y + path[i-1].y - (2*path[i].y));
				
	//			cout<<"smooth: x="<<path[i].x<<" y="<<path[i].y<<endl;
				change += abs(aux.x-path[i].x) + abs(aux.y - path[i].y);
		}
	}

	waypoint = path;
}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"path_smoother");
	
	FILE* fp;
	FILE* wfp;
	const char r_filename[100] = "/home/amsl/AMSL_ros_pkg/rwrc15/rwrc15_waypoint_manager/wp_list_xy/dis4_waypoints.txt";//filename;
	const char w_filename[100] = "/home/amsl/AMSL_ros_pkg/rwrc15/rwrc15_waypoint_manager/wp_list_xy/ikuchare_smooth_dkan3.txt";//filename;
	if((fp = fopen(r_filename,"r")) == NULL){
		cout<<"read file open error"<<endl;
		exit(EXIT_FAILURE);
	}
	

	if((wfp = fopen(w_filename,"w")) == NULL){
		cout<<"write file open error"<<endl;
		exit(EXIT_FAILURE);
	}
	
	point p;
	vector<point> waypoint;
	vector<point> cp_waypoint;
	cout<<"insert"<<endl;
	while(fscanf(fp,"%f %f\n",&p.x,&p.y)!=EOF){
		waypoint.push_back(p);
		cp_waypoint.push_back(p);
	}
	cout<<"smooth"<<endl;
	smooth(waypoint,0.5,0.25,0.000001);//,weight_data,weight_smooth(0.1~0.25),tolerance,  weight_smoothがカーブの緩やかさのパラメータ大きいほど滑らかに
	
	cout<<"output"<<endl;
	for(int i=0;i<waypoint.size();i++){
		fprintf(wfp,"%f %f\n",waypoint[i].x,waypoint[i].y);
		if(cp_waypoint[i].x-waypoint[i].x !=0){
			cout<<"i="<<i<<endl<<"old:x="<<cp_waypoint[i].x<<"y="<<cp_waypoint[i].y<<endl;
			cout<<"\tnew:x="<<waypoint[i].x<<"y="<<waypoint[i].y<<endl;
		}
	}
	fclose(fp);
	cout<<"finish"<<endl;

	return 0;
}
