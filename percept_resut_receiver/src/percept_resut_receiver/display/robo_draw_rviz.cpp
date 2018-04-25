/*
 *  Robosense display tool, service for Robosense perception debug.
 *  This is in processing, some more tools will be added in the future.
 */

#include "robo_draw_rviz.h"
#define DRAW_DEBUG 1

#define TEXT_SCALE (1.0)

static int marker_track_num_=0, marker_seg_num_ =0, marker_label_num_=0, marker_per_num =0,marker_box_num_=0;
static std::vector<cv::Point3f> colors_;



DrawRviz::DrawRviz(){

	generateColors(colors_,100);

}
DrawRviz::~DrawRviz() {
}


void DrawRviz::generateColors(std::vector<cv::Point3f>& colors, int num)
{
	colors.clear();
	colors.resize(num);
	for(int i=0;i<num;++i)
	{
		cv::Point3f color;
		color.x = rand()%255;
		color.y = rand()%255;
		color.z = rand()%255;

		color+=cv::Point3f(60,60,60);

		color.x=color.x<255?color.x:255;
		color.y=color.y<255?color.y:255;
		color.z=color.z<255?color.z:255;

		colors[i] = color;
	}
}


std_msgs::Header DrawRviz::convertPCLHeader2ROSHeader(const pcl::PCLHeader& _header)
{
	std_msgs::Header header;
	header.seq = _header.seq;
	header.stamp = pcl_conversions::fromPCL(_header.stamp);
//	header.stamp.fromNSec(_header.stamp*1e3);
//	header.stamp.sec = (int32_t)(double(_header.stamp)*1e-9);
//	header.stamp.nsec = (int32_t)(((double(_header.stamp)*1e-9) - (int)(double(_header.stamp)*1e-9))*1e9);
	header.frame_id = _header.frame_id;

	return  header;
}


void DrawRviz::show_labelmsg_info_box(const ros::Publisher& pub_percept_info, const pcl::PCLHeader& _header,
																	const std::vector<Robosense::PerceptResultMsg>& msgs)
{
	visualization_msgs::MarkerArrayPtr marker_array(new visualization_msgs::MarkerArray);
	visualization_msgs::Marker marker_text;//标记label文字信息
	visualization_msgs::Marker marker_box; //标记分类点云

	marker_text.header = convertPCLHeader2ROSHeader(_header);
	marker_text.ns = "percept_info";
	marker_text.scale.x = TEXT_SCALE;
	marker_text.scale.y = TEXT_SCALE;
	marker_text.scale.z = TEXT_SCALE;
	marker_text.color.a = 1.0;

	marker_box.header = convertPCLHeader2ROSHeader(_header);
	marker_box.ns = "cube_info";
	marker_box.scale.x = 0.01;
	marker_box.scale.y = 0.01;
	marker_box.scale.z = 0.01;
	marker_box.color.r = 0.2;
	marker_box.color.g = 0.8;
	marker_box.color.b = 0.0;
	marker_box.color.a = 0.7;

	cv::Point3f colors[5];
	colors[0]=cv::Point3f(1,0,1);//淡紫色 background
	colors[1]=cv::Point3f(1,1,0);//黄色 pedestrian
	colors[2]=cv::Point3f(0,1,1);//青色 bike
	colors[3]=cv::Point3f(1,0,0);//红色 car
	colors[4]=cv::Point3f(0,0,1);//蓝色 truck

	std::string info[5] = {"unknow", "ped", "bike", "car", "truck"};

	int marker_id=0;
	for(int i=0;i<msgs.size();++i)
	{
		// 给不同的label的点云赋予不同的颜色j
		if (msgs[i].label > 0 && msgs[i].label < 5)
		{
			cv::Point3f color = colors[msgs[i].label];
			if (msgs[i].label>0)
			{
				marker_box.scale.x = 0.1;
				marker_box.scale.y = 0.1;
				marker_box.scale.z = 0.1;
			}
			else{
				marker_box.scale.x = 0.02;
				marker_box.scale.y = 0.02;
				marker_box.scale.z = 0.02;
			}
			//画框
			BoxInfo box;
			convertBoxMsg(msgs[i].box,box);
			draw_cube(box, marker_id, marker_box, color);
			marker_array->markers.push_back(marker_box);

			//label 显示
			marker_text.color.r = color.x;
			marker_text.color.g = color.y;
			marker_text.color.b = color.z;
			cv::Point3f pos = box.center;
			pos.z = box.corners[6].z + 0.5f;

			draw_text(pos, info[msgs[i].label], marker_id, marker_text);
			marker_array->markers.push_back(marker_text);

			//TODO: add some ...

			marker_id++;
		}
	}

	if(marker_id<marker_box_num_)
	{
		int k=0;
		for (int i = marker_id; i < marker_box_num_; ++i) {
			marker_text.id = marker_id+k;
			marker_text.color.a = 0.f;
			marker_array->markers.push_back(marker_text);

			marker_box.id = marker_id+k;
			marker_box.color.a = 0.f;
			marker_array->markers.push_back(marker_box);

			k++;
		}
	}
	marker_box_num_ = marker_id;
	pub_percept_info.publish(marker_array);
}
void DrawRviz::show_labelmsg_info(const ros::Publisher& pub_percept_info, const pcl::PCLHeader& _header,
																		 const std::vector<Robosense::PerceptResultMsg>& msgs)
{
	visualization_msgs::MarkerArrayPtr marker_array(new visualization_msgs::MarkerArray);
	visualization_msgs::Marker marker_text;//标记label文字信息
	visualization_msgs::Marker marker_box; //标记分类点云

	marker_text.header = convertPCLHeader2ROSHeader(_header);
	marker_text.ns = "percept_info";
	marker_text.scale.x = TEXT_SCALE;
	marker_text.scale.y = TEXT_SCALE;
	marker_text.scale.z = TEXT_SCALE;
	marker_text.color.a = 1.0;

	marker_box.header = convertPCLHeader2ROSHeader(_header);
	marker_box.ns = "percept_box";
	marker_box.scale.x = 0.01;
	marker_box.scale.y = 0.01;
	marker_box.scale.z = 0.01;
	marker_box.color.r = 0.2;
	marker_box.color.g = 0.8;
	marker_box.color.b = 0.0;
	marker_box.color.a = 0.7;

	cv::Point3f colors[5];
	colors[0]=cv::Point3f(1,0,1);//淡紫色 background
	colors[1]=cv::Point3f(1,1,0);//黄色 pedestrian
	colors[2]=cv::Point3f(0,1,1);//青色 bike
	colors[3]=cv::Point3f(1,0,0);//红色 car
	colors[4]=cv::Point3f(0,0,1);//蓝色 truck

	std::string info[5] = {"unknow", "ped", "bike", "car", "truck"};

	int marker_id=0;
	for(int i=0;i<msgs.size();++i)
	{
		// 给不同的label的点云赋予不同的颜色j
		if (msgs[i].label > 0 && msgs[i].label < 5)
		{
			cv::Point3f color = colors[msgs[i].label];
			if (msgs[i].label>0)
			{
				marker_box.scale.x = 0.1;
				marker_box.scale.y = 0.1;
				marker_box.scale.z = 0.1;
			}
			else{
				marker_box.scale.x = 0.02;
				marker_box.scale.y = 0.02;
				marker_box.scale.z = 0.02;
			}
			//画框
			BoxInfo box;
			convertBoxMsg(msgs[i].box,box);
			draw_box(box, marker_id, marker_box, 1.0);
			marker_array->markers.push_back(marker_box);

			//label 显示
			marker_text.color.r = color.x;
			marker_text.color.g = color.y;
			marker_text.color.b = color.z;
			cv::Point3f pos = box.center;
			pos.z = box.corners[6].z + 0.5f;

			draw_text(pos, info[msgs[i].label], marker_id, marker_text);
			marker_array->markers.push_back(marker_text);

			//TODO: add some ...

			marker_id++;
		}
	}

	if(marker_id<marker_label_num_)
	{
		int k=0;
		for (int i = marker_id; i < marker_label_num_; ++i) {
			marker_text.id = marker_id+k;
			marker_text.color.a = 0.f;
			marker_array->markers.push_back(marker_text);

			marker_box.id = marker_id+k;
			marker_box.color.a = 0.f;
			marker_array->markers.push_back(marker_box);

			k++;
		}
	}
	marker_label_num_ = marker_id;
	pub_percept_info.publish(marker_array);
}

void DrawRviz::draw_box(const BoxInfo& box, const int& marker_id, visualization_msgs::Marker& marker, float scale)
{
	marker.id = marker_id;
	marker.type = visualization_msgs::Marker::LINE_LIST;
	marker.action = visualization_msgs::Marker::ADD;

	float box_size = box.size.x*box.size.y*box.size.z;
	if(box_size>0)// && box_size<=50 && box.size.z < 5)//工程型限制
	{
		marker.color.a=1.f;
		std::vector<geometry_msgs::Point> cub_points;

		for (int i = 0; i < 8; ++i) {
			geometry_msgs::Point pts;
			pts.x = box.corners[i].x;
			pts.y = box.corners[i].y;
			pts.z = box.corners[i].z;
			cub_points.push_back(pts);
		}

		marker.points.push_back(cub_points[0]);
		marker.points.push_back(cub_points[1]);
		marker.points.push_back(cub_points[1]);
		marker.points.push_back(cub_points[2]);
		marker.points.push_back(cub_points[2]);
		marker.points.push_back(cub_points[3]);
		marker.points.push_back(cub_points[3]);
		marker.points.push_back(cub_points[0]);
		// horizontal high points for lines
		marker.points.push_back(cub_points[4]);
		marker.points.push_back(cub_points[5]);
		marker.points.push_back(cub_points[5]);
		marker.points.push_back(cub_points[6]);
		marker.points.push_back(cub_points[6]);
		marker.points.push_back(cub_points[7]);
		marker.points.push_back(cub_points[7]);
		marker.points.push_back(cub_points[4]);
		// vertical points for lines
		marker.points.push_back(cub_points[0]);
		marker.points.push_back(cub_points[4]);
		marker.points.push_back(cub_points[1]);
		marker.points.push_back(cub_points[5]);
		marker.points.push_back(cub_points[2]);
		marker.points.push_back(cub_points[6]);
		marker.points.push_back(cub_points[3]);
		marker.points.push_back(cub_points[7]);
	}
	else
	{
		marker.color.a=0;
	}

}

void DrawRviz::show_cluster_msg_info(const ros::Publisher& pub_cluster_msg_info,const pcl::PCLHeader& _header,const std::vector<Robosense::PerceptResultMsg>& msgs)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	visualization_msgs::MarkerArrayPtr marker_array(new visualization_msgs::MarkerArray);
	visualization_msgs::Marker marker_box;//标记框
	visualization_msgs::Marker marker_text;//标记object 位置文字信息
	visualization_msgs::Marker marker_arrow; //标记box主方向箭头

	marker_box.header = convertPCLHeader2ROSHeader(_header);
	marker_box.ns = "cluster_box";
	marker_box.scale.x = 0.05;
	marker_box.scale.y = 0.05;
	marker_box.scale.z = 0.05;
	marker_box.color.r = 0.8;
	marker_box.color.g = 0.8;
	marker_box.color.b = 0.0f;
	marker_box.color.a = 0.9;

	marker_text.header = convertPCLHeader2ROSHeader(_header);
	marker_text.ns = "pos_info";
	marker_text.scale.x = TEXT_SCALE;
	marker_text.scale.y = TEXT_SCALE;
	marker_text.scale.z = TEXT_SCALE;
	marker_text.color.r = 0.7;
	marker_text.color.g = 0.4;
	marker_text.color.b = 0.f;
	marker_text.color.a = 1.0;

	marker_arrow.header = convertPCLHeader2ROSHeader(_header);
	marker_arrow.ns = "box_direction";
	marker_arrow.color.r = 0.f;
	marker_arrow.color.g = 0.f;
	marker_arrow.color.b = 1.f;
	marker_arrow.color.a = 1.f;

	int marker_id = 0;
	for (int i = 0; i < msgs.size(); ++i)
	{
		//绘制每个cluster的bbox
		BoxInfo box;
		convertBoxMsg(msgs[i].box,box);
		draw_box(box, marker_id, marker_box, 1.0);
		marker_array->markers.push_back(marker_box);

		//绘制箭头
		draw_box_arrow(box, marker_id, marker_arrow);
		marker_array->markers.push_back(marker_arrow);

		cv::Point3f pos = box.center;
		cv::Point3f size = box.size;
		float dis = sqrtf(pos.x*pos.x + pos.y*pos.y);
		std::string info = num2str<float>(dis, 1)+
											 "<" + num2str<float>(pos.x, 1) + ", " + num2str<float>(pos.y, 1) + ">";// + ", " + num2str<float>(pos.z, 1)
//			               + "[" + num2str<float>(size.x, 1) + ", " + num2str<float>(size.y, 1) + ", " + num2str<float>(size.z, 1) + "]" ;

		pos.z = box.corners[6].z+1.7f;
		draw_text(pos, info, marker_id, marker_text);
		marker_array->markers.push_back(marker_text);

		//TODO: add some ...

		marker_id++;
	}

	if(marker_id<marker_seg_num_)
	{
		int k=0;
		for (int i = marker_id; i < marker_seg_num_; ++i)
		{
			marker_box.id = marker_id+k;
			marker_box.color.a = 0.f;
			marker_array->markers.push_back(marker_box);

			marker_arrow.id = marker_id+k;
			marker_arrow.color.a = 0.f;
			marker_array->markers.push_back(marker_arrow);

			marker_text.id = marker_id+k;
			marker_text.color.a = 0.f;
			marker_array->markers.push_back(marker_text);

			k++;
		}
	}

	marker_seg_num_ = marker_id;

	//发布cluster的bbox信息
	pub_cluster_msg_info.publish(marker_array);

}

void DrawRviz::show_track_msg_info(const ros::Publisher& pub_track_msg_info, const pcl::PCLHeader& _header, const std::vector<Robosense::PerceptResultMsg>& msgs,const float yaw)
{
//  std::cout<<"yaw: "<<yaw<<std::endl;
	visualization_msgs::MarkerArrayPtr marker_array(new visualization_msgs::MarkerArray);
	visualization_msgs::Marker marker_text;//标记tracking文字信息
	visualization_msgs::Marker marker_arrow; //标记速度箭头

	marker_text.header = convertPCLHeader2ROSHeader(_header);
	marker_text.ns = "tracker_info";
	marker_text.scale.x = TEXT_SCALE;
	marker_text.scale.y = TEXT_SCALE;
	marker_text.scale.z = TEXT_SCALE;
	marker_text.color.r = 0.f;
	marker_text.color.g = 0.7f;
	marker_text.color.b = 0.6f;
	marker_text.color.a = 1.f;

	marker_arrow.header = convertPCLHeader2ROSHeader(_header);
	marker_arrow.ns = "velocity_direction";
	marker_arrow.color.r = 0.f;
	marker_arrow.color.g = 0.7f;
	marker_arrow.color.b = 0.3f;
	marker_arrow.color.a = 0.8;

	int marker_id=0;

	for(int i=0;i<msgs.size();++i)
	{
		float length = sqrtf(msgs[i].velocity.x*msgs[i].velocity.x + msgs[i].velocity.y*msgs[i].velocity.y);
		if (length > 0.3f)
		{
			draw_track_msg_arrow(msgs[i], marker_id, marker_arrow,yaw);
			marker_array->markers.push_back(marker_arrow);

			BoxInfo box;
			convertBoxMsg(msgs[i].box,box);
			cv::Point3f pos = box.center;
			pos.z = box.corners[6].z + 0.5f;

			float velocity = length;
//      std::string text = "(" + num2str<float>(perception_info[i].tracker.acceleration.getLength(), 2) + "," +
//                         num2str<float>(perception_info[i].tracker.angle_acc, 2) +")"+
//              "<" + num2str<int>(perception_info[i].tracker.tracker_id, 0) + ">" +
//                           num2str<int>(perception_info[i].tracker.sequence_length, 0) +
//                         ":" + num2str<float>(velocity*3.6f, 1) + "km/h";

			std::string text ="<"+num2str<int>(msgs[i].track_id, 0)+">" + num2str<float>(velocity*3.6f, 1) + "km/h";// +
			//       "+"+ num2str<float>(perception_info[i].tracker.robustness, 3);

			draw_text(pos, text, marker_id, marker_text);
			marker_array->markers.push_back(marker_text);

			marker_id++;
		}
	}


	if(marker_id<marker_track_num_)
	{
		int k=0;
		for (int i = marker_id; i < marker_track_num_; ++i) {
			marker_text.id = marker_id+k;
			marker_text.color.a = 0.f;
			marker_array->markers.push_back(marker_text);

			marker_arrow.id = marker_id+k;
			marker_arrow.color.a = 0.f;
			marker_array->markers.push_back(marker_arrow);

			k++;
		}
	}

	marker_track_num_ = marker_id;
	pub_track_msg_info.publish(marker_array);

}

void DrawRviz::draw_cube(const BoxInfo& box, const int& marker_id, visualization_msgs::Marker& marker,cv::Point3f color, float scale)
{
	marker.id = marker_id;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;

	float box_size = box.size.x*box.size.y*box.size.z;
	if(box_size>0)// && box_size<=50 && box.size.z < 5)//工程型限制
	{
		marker.color.a= 0.5f;
		marker.color.r = color.x;
		marker.color.g = color.y;
		marker.color.b = color.z;

		marker.pose.position.x = box.center.x;
		marker.pose.position.y = box.center.y;
		marker.pose.position.z = box.center.z;

		marker.scale.x = box.size.x;
		marker.scale.y = box.size.y;
		marker.scale.z = box.size.z;

		tf::Quaternion quat = tf::createQuaternionFromRPY(0., 0.,box.angle);
//		if(fabs(box.angle - 1.5708) > 0.1)
//			std::cout<<box.angle<<std::endl;
		tf::quaternionTFToMsg(quat, marker.pose.orientation);

	}
  else
  {
		marker.color.a=0;
  }

}

void DrawRviz::draw_track_msg_arrow(const Robosense::PerceptResultMsg& per, const int& marker_id, visualization_msgs::Marker& marker,const float& yaw)
{
	marker.id = marker_id;
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;

	float box_size = per.box.length*per.box.width*per.box.height;
	if (box_size>0)
	{
		marker.color.a = 1;

		float arrow_length = sqrtf(per.velocity.x*per.velocity.x + per.velocity.y*per.velocity.y);
		float main_direction = atan2f(per.velocity.y, per.velocity.x) - yaw;
//    main_direction -= yaw;
//    main_direction = atan2f(sin(main_direction),cos(main_direction));

		marker.scale.x = sqrtf(arrow_length + 1.0) - 1.0;
		marker.scale.y = 0.2;
		marker.scale.z = 0.2;

		tf::Quaternion quat = tf::createQuaternionFromRPY(0., 0., main_direction);
		tf::quaternionTFToMsg(quat, marker.pose.orientation);
		marker.pose.position.x = per.box.location.x;
		marker.pose.position.y = per.box.location.y;
		marker.pose.position.z = per.box.location.z;
	}
	else
	{
		marker.color.a = 0;
	}
}

void DrawRviz::draw_box_arrow(const BoxInfo& box, const int& marker_id, visualization_msgs::Marker& marker)
{
	marker.id = marker_id;
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;

	float box_size = box.size.x*box.size.y*box.size.z;;
	if(box_size>0)
	{
		marker.color.a = 1.f;

		marker.scale.x = 1.0;
		marker.scale.y = 0.15;
		marker.scale.z = 0.15;
		tf::Quaternion quat = tf::createQuaternionFromRPY(0., 0., box.angle);
		tf::quaternionTFToMsg(quat, marker.pose.orientation);
		marker.pose.position.x = box.center.x;
		marker.pose.position.y = box.center.y;
		marker.pose.position.z = box.center.z;
	}
	else
	{
		marker.color.a = 0.f;
	}
}

void DrawRviz::draw_text(const cv::Point3f& pos, const std::string& info, const int& marker_id, visualization_msgs::Marker& marker)
{
	//----------------标记跟踪信息----------
	marker.id = marker_id;
	marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = pos.x;
	marker.pose.position.y = pos.y;
	marker.pose.position.z = pos.z;

	marker.text = info;
}

void DrawRviz::convertBoxMsg(const Robosense::boxMsg& BoxMsg,BoxInfo& box)
{

  BoxInfo tmp_box;
  tmp_box.center.x = BoxMsg.location.x;
  tmp_box.center.y = BoxMsg.location.y;
  tmp_box.center.z = BoxMsg.location.z;

  tmp_box.size.x = BoxMsg.length;
  tmp_box.size.y = BoxMsg.width;
  tmp_box.size.z = BoxMsg.height;

  tmp_box.corners[0].x = BoxMsg.location.x - BoxMsg.length / 2;
  tmp_box.corners[0].y = BoxMsg.location.y - BoxMsg.width / 2;
  tmp_box.corners[0].z = BoxMsg.location.z - BoxMsg.height / 2;

  tmp_box.corners[1].x = BoxMsg.location.x + BoxMsg.length / 2;
  tmp_box.corners[1].y = BoxMsg.location.y - BoxMsg.width / 2;
  tmp_box.corners[1].z = BoxMsg.location.z - BoxMsg.height / 2;

  tmp_box.corners[2].x = BoxMsg.location.x + BoxMsg.length / 2;
  tmp_box.corners[2].y = BoxMsg.location.y + BoxMsg.width / 2;
  tmp_box.corners[2].z = BoxMsg.location.z - BoxMsg.height / 2;

  tmp_box.corners[3].x = BoxMsg.location.x - BoxMsg.length / 2;
  tmp_box.corners[3].y = BoxMsg.location.y + BoxMsg.width / 2;
  tmp_box.corners[3].z = BoxMsg.location.z - BoxMsg.height / 2;
//	tmp_box.angle = CV_PI/2 - BoxMsg.direction.yaw;
	tmp_box.angle = 0;
//	std::cout<<BoxMsg.direction.yaw<<std::endl;

	for(int i = 0; i < 4; i++)
	{
    tmp_box.corners[i+4] = tmp_box.corners[i];
    tmp_box.corners[i+4].z = BoxMsg.location.z + BoxMsg.height / 2;
	}

	box = rotateBox2D(tmp_box,BoxMsg.yaw);
}

BoxInfo DrawRviz::rotateBox2D(const BoxInfo &box, float angle)
{
	if(fabsf(angle) < 1e-6)
		return box;

	cv::Point3f center = box.center;
	cv::Point2f corners[4];

	float cos_ang = cosf(angle), sin_ang = sinf(angle);
	for(int i = 0; i < 4; ++i)
	{
		corners[i].x = (box.corners[i].x - center.x) * cos_ang - (box.corners[i].y - center.y) * sin_ang + center.x;
		corners[i].y = (box.corners[i].x - center.x) * sin_ang + (box.corners[i].y - center.y) * cos_ang + center.y;
	}

	BoxInfo box_out = box;

	for(int i = 0; i < 4; ++i)
	{
		box_out.corners[i] = cv::Point3f(corners[i].x, corners[i].y, box.corners[i].z);
		box_out.corners[i + 4] = cv::Point3f(corners[i].x, corners[i].y, box.corners[i + 4].z);
	}

	box_out.angle = box.angle + angle;

//  box_out.anchor = box.corners[findBoxAnchor(box_out)];

	return box_out;
}

