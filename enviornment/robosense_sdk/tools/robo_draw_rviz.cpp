/*
 *  Robosense display tool, service for Robosense perception debug.
 *  This is in processing, some more tools will be added in the future.
 */

#include "robo_draw_rviz.h"
#define DRAW_DEBUG 1
#define USE_ClUSER_COLOR 0
#define TEXT_SCALE (0.7)

namespace Robosense
{

static int marker_per_num =0;
static std::vector<Point3f> colors_;

template <typename PointT>
DrawRviz<PointT>::DrawRviz(){

	generateColors(colors_,100);

}

template <typename PointT>
void DrawRviz<PointT>::generateColors(std::vector<Point3f>& colors, int num)
{
	colors.clear();
	colors.resize(num);
	for(int i=0;i<num;++i)
	{
		Point3f color;
		color.x = rand()%255;
		color.y = rand()%255;
		color.z = rand()%255;

		color+=Point3f(60,60,60);

		color.x=color.x<255?color.x:255;
		color.y=color.y<255?color.y:255;
		color.z=color.z<255?color.z:255;

		colors[i] = color;
	}
}

template <typename PointT>
std_msgs::Header DrawRviz<PointT>::convertPCLHeader2ROSHeader(const pcl::PCLHeader& _header)
{
	std_msgs::Header header;
	header.seq = _header.seq;
	header.stamp = pcl_conversions::fromPCL(_header.stamp);
	header.frame_id = _header.frame_id;

	return  header;
}

template <typename PointT>
void DrawRviz<PointT>::show_car_self(const ros::Publisher& pub_car_info,
																		 const pcl::PCLHeader& _header, Eigen::VectorXf car_pos, const std::string &frame_id)
{
	visualization_msgs::MarkerArrayPtr marker_array(new visualization_msgs::MarkerArray);

	visualization_msgs::Marker marker_car;
	marker_car.header = convertPCLHeader2ROSHeader(_header);
	marker_car.id = 0;
	marker_car.ns = "car_vel_info";
	marker_car.scale.x = TEXT_SCALE*1.1;
	marker_car.scale.y = TEXT_SCALE*1.1;
	marker_car.scale.z = TEXT_SCALE*1.1;
	marker_car.color.r=1.0;
	marker_car.color.g=0.7;
	marker_car.color.a=1.0;

	Eigen::Vector3f self_positon = car_pos.head(3);
	Eigen::Vector3f self_angle = car_pos.segment(3, 3);
	Eigen::Vector3f self_velocity = car_pos.tail(3);

	std::string info_self = num2str<float>(self_velocity.norm()*3.6f, 1)+"km/h";
	Point3f pos_self(0,0,0);
	draw_text(pos_self, info_self, 0, marker_car, 1.0);
	marker_array->markers.push_back(marker_car);

	visualization_msgs::Marker marker_yaw_dir;
	pcl::PCLHeader car_header = _header;
	car_header.frame_id = frame_id;

	marker_yaw_dir.pose.position.x = self_positon(0);
	marker_yaw_dir.pose.position.y = self_positon(1);
	marker_yaw_dir.pose.position.z = self_positon(2);
	marker_yaw_dir.header = convertPCLHeader2ROSHeader(car_header);
	marker_yaw_dir.id = 0;
	marker_yaw_dir.ns = "car_dir_info";
	marker_yaw_dir.scale.x = 2.0;
	marker_yaw_dir.scale.y = 0.15;
	marker_yaw_dir.scale.z = 0.15;
	marker_yaw_dir.color.r=1.0;
	marker_yaw_dir.color.g=0.0;
	marker_yaw_dir.color.b=1.0;
	marker_yaw_dir.color.a=1.0;
	marker_yaw_dir.type = visualization_msgs::Marker::ARROW;
	marker_yaw_dir.action = visualization_msgs::Marker::ADD;
	tf::Quaternion quat = tf::createQuaternionFromRPY(self_angle(0), self_angle(1), self_angle(2));
	tf::quaternionTFToMsg(quat, marker_yaw_dir.pose.orientation);

	marker_array->markers.push_back(marker_yaw_dir);


	pub_car_info.publish(marker_array);
}

template <typename PointT>
void DrawRviz<PointT>::show_percept(const ros::Publisher& pub_percept_info, const pcl::PCLHeader& _header,
									const std::vector<RoboPerceptron<PointT> >& perception_info, float yaw)
{
	visualization_msgs::MarkerArrayPtr marker_array(new visualization_msgs::MarkerArray);
	visualization_msgs::Marker marker_box;
	visualization_msgs::Marker marker_cube;
	visualization_msgs::Marker marker_text_box;
  visualization_msgs::Marker marker_text_label;
  visualization_msgs::Marker marker_text_track;
  visualization_msgs::Marker marker_track_arrow;

	Point3f colors[5];
	colors[0]=Point3f(0.6,0.5,0.6);//淡紫色 background
	colors[1]=Point3f(1,1,0);//黄色 pedestrian
	colors[2]=Point3f(0,1,1);//青色 bike
	colors[3]=Point3f(0.5,0,0);//红色 car
	colors[4]=Point3f(0,0,1);//蓝色 truck

	std::string info[5] = {"unknow", "ped", "bike", "car", "truck"};

	marker_text_box.header = convertPCLHeader2ROSHeader(_header);
	marker_text_box.ns = "box_info";
	marker_text_box.scale.x = TEXT_SCALE;
	marker_text_box.scale.y = TEXT_SCALE;
	marker_text_box.scale.z = TEXT_SCALE;
	marker_text_box.color.r = 0.f;
	marker_text_box.color.g = 0.6f;
	marker_text_box.color.b = 0.7f;
	marker_text_box.color.a = 1.f;

  marker_text_track.header = convertPCLHeader2ROSHeader(_header);
  marker_text_track.ns = "track_info";
  marker_text_track.scale.x = TEXT_SCALE;
  marker_text_track.scale.y = TEXT_SCALE;
  marker_text_track.scale.z = TEXT_SCALE;
  marker_text_track.color.r = 0.f;
  marker_text_track.color.g = 0.7f;
  marker_text_track.color.b = 0.6f;
  marker_text_track.color.a = 1.f;

  marker_text_label.header = convertPCLHeader2ROSHeader(_header);
  marker_text_label.ns = "label_info";
  marker_text_label.scale.x = TEXT_SCALE;
  marker_text_label.scale.y = TEXT_SCALE;
  marker_text_label.scale.z = TEXT_SCALE;
  marker_text_label.color.a = 1.0;

  marker_track_arrow.header = convertPCLHeader2ROSHeader(_header);
	marker_track_arrow.ns = "velocity_dir";
  marker_track_arrow.color.r = 0.f;
  marker_track_arrow.color.g = 0.7f;
  marker_track_arrow.color.b = 0.3f;
  marker_track_arrow.color.a = 0.8f;

	marker_box.header = convertPCLHeader2ROSHeader(_header);
	marker_box.ns = "box";
	marker_box.color.r = colors[0].x;
	marker_box.color.g = colors[0].y;
	marker_box.color.b = colors[0].z;
	marker_box.scale.x = marker_box.scale.y = marker_box.scale.z = 0.03;

	marker_cube.header = convertPCLHeader2ROSHeader(_header);
	marker_cube.ns = "cube";

//	std::string link_mode[4] ={"Bary", "Center", "S-center", "Corner"};

	int marker_id=0;
	for(int i=0;i<perception_info.size();++i)
	{
		const RoboPerceptron<PointT>& perceptron = perception_info[i];
		if (perceptron.is_segmented)
		{
			//-------------------------------box----------------------------
			draw_box(perceptron.cluster.box, marker_id, marker_box, 0.5, 1.0);

			Point3f nearest = findNearestBoxCorner(perceptron.cluster.box);
			std::string text_box = num2str<float>(nearest.getLength(), 1) +
														 " (" + num2str<float>(perceptron.cluster.box.center.x, 1) + " " +
														 num2str<float>(perceptron.cluster.box.center.y, 1) + " " +
														 num2str<float>(perceptron.cluster.box.center.z, 1) + ")";

			Point3f pos0 = perceptron.cluster.box.center;
			pos0.z = perceptron.cluster.box.center.z + perceptron.cluster.box.size.z * 0.5f + 0.2f;
			draw_text(pos0, text_box, marker_id, marker_text_box, 1.0);
		} else{
			marker_box.id = marker_id;
			marker_box.color.a = 0;
			marker_text_box.id = marker_id;
			marker_text_box.color.a = 0;
		}
		marker_array->markers.push_back(marker_box);
		marker_array->markers.push_back(marker_text_box);

		if (perceptron.is_tracked)
		{
			//--------------------------------tracking------------------------------
			bool is_valid_vel = perceptron.tracker.velocity_abs.getLength() > 0.3f;
			draw_track_arrow(perceptron, marker_id, marker_track_arrow, yaw, is_valid_vel ? 0.8 : 0);

			Point3f pos1 = perceptron.cluster.box.center;
			pos1.z = perceptron.cluster.box.center.z + perceptron.cluster.box.size.z * 0.5f + 0.7f;

			float velocity = perceptron.tracker.velocity_abs.getLength();
			float angle_vel = perceptron.tracker.angle_velocity / PI_OVER_180;

//			std::string link;
//			if(perceptron.tracker.link_mode >= 0)
//				link = link_mode[perceptron.tracker.link_mode];

			std::string text_track =
							"<" + num2str<int>(perceptron.tracker.tracker_id, 0) + ">" + num2str<float>(velocity * 3.6f, 1) + "km/h"
							+ " >> " + num2str<float>(perceptron.tracker.possibility, 2);
			// +" >>" + num2str<float>(perceptron.tracker.possibility, 3) + ">" + link;
			//num2str<float>(angle_vel, 1);
			//+ " / " + num2str<float>(perceptron.tracker.sequence_robustness, 3);
			draw_text(pos1, text_track, marker_id, marker_text_track, is_valid_vel ? 0.8 : 0);
		}else{

			marker_track_arrow.id = marker_id;
			marker_track_arrow.color.a = 0;
			marker_text_track.id = marker_id;
			marker_text_track.color.a = 0;
		}
		marker_array->markers.push_back(marker_track_arrow);
		marker_array->markers.push_back(marker_text_track);

		if (perceptron.is_classified)
		{
      //--------------------------------classification---------------------------
			int label = perceptron.labeler.label;
			Point3f color = colors[label];
			bool is_bgd = (label == 0);
			//cube
			marker_cube.color.r = color.x;
			marker_cube.color.g = color.y;
			marker_cube.color.b = color.z;

			draw_cube(perceptron.cluster.box, marker_id, marker_cube, is_bgd?0.3:0.4, 1.0);

      marker_text_label.color.r = color.x;
      marker_text_label.color.g = color.y;
      marker_text_label.color.b = color.z;
      Point3f pos2 = perceptron.cluster.box.center;
      pos2.z = perceptron.cluster.box.center.z + perceptron.cluster.box.size.z*0.5f + 1.2f;

			std::string text_label = info[label] + " >>"+num2str<float>(perceptron.labeler.confidence, 2);

      draw_text(pos2, text_label, marker_id, marker_text_label, is_bgd?0:0.95);
		}else{
			marker_cube.id = marker_id;
			marker_cube.color.a = 0;
			marker_text_label.id = marker_id;
			marker_text_label.color.a = 0;
		}

		marker_array->markers.push_back(marker_cube);
		marker_array->markers.push_back(marker_text_label);

		marker_id++;
	}

	if(marker_id<marker_per_num)
	{
		int k=0;
		for (int i = marker_id; i < marker_per_num; ++i)
		{
			marker_box.id = marker_id+k;
			marker_box.color.a = 0.f;
			marker_array->markers.push_back(marker_box);

			marker_cube.id = marker_id+k;
			marker_cube.color.a = 0.f;
			marker_array->markers.push_back(marker_cube);

			marker_text_box.id = marker_id+k;
			marker_text_box.color.a = 0.f;
			marker_array->markers.push_back(marker_text_box);

      marker_text_track.id = marker_id+k;
      marker_text_track.color.a = 0.f;
      marker_array->markers.push_back(marker_text_track);

      marker_track_arrow.id = marker_id+k;
      marker_track_arrow.color.a = 0.f;
      marker_array->markers.push_back(marker_track_arrow);

      marker_text_label.id = marker_id+k;
      marker_text_label.color.a = 0.f;
      marker_array->markers.push_back(marker_text_label);

			k++;
		}
	}

	marker_per_num = marker_id;
	pub_percept_info.publish(marker_array);

}


template <typename PointT>
void DrawRviz<PointT>::show_cluster(const ros::Publisher& pub_cluster_cloud,
                            const pcl::PCLHeader& _header, const std::vector<RoboPerceptron<PointT> >& clusters)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

#if !USE_ClUSER_COLOR
  Point3f colors[5];
  colors[0]=Point3f(0.6,0.5,0.6);//淡紫色 background
  colors[1]=Point3f(1,1,0);//黄色 pedestrian
  colors[2]=Point3f(0,1,1);//青色 bike
  colors[3]=Point3f(0.5,0,0);//红色 car
  colors[4]=Point3f(0,0,1);//蓝色 truck
#endif

	for (int i = 0; i < clusters.size(); ++i)
	{
#if !USE_ClUSER_COLOR
    Point3f color = colors[clusters[i].labeler.label];
#endif
		for (int j = 0; j < clusters[i].cluster.pointCloud->size(); ++j)
		{
			PointT tmp_point = clusters[i].cluster.pointCloud->points[j];
			pcl::PointXYZRGB tmp_point_rgb;
			tmp_point_rgb.x = tmp_point.x;
			tmp_point_rgb.y = tmp_point.y;
			tmp_point_rgb.z = tmp_point.z;

#if USE_ClUSER_COLOR
			tmp_point_rgb.r = (uint8_t)colors_[i % 100].x;
			tmp_point_rgb.g = (uint8_t)colors_[i % 100].y;
			tmp_point_rgb.b = (uint8_t)colors_[i % 100].z;
#else
      tmp_point_rgb.r = (uint8_t)(color.x*255);
      tmp_point_rgb.g = (uint8_t)(color.y*255);
      tmp_point_rgb.b = (uint8_t)(color.z*255);
#endif
			cluster_cloud_ptr->push_back(tmp_point_rgb);
		}

	}

	cluster_cloud_ptr->header = _header;
	pub_cluster_cloud.publish(cluster_cloud_ptr);
}


template <typename PointT>
void DrawRviz<PointT>::draw_cube(const BoundingBox& box, const int& marker_id, visualization_msgs::Marker& marker, float alpha, float scale)
{
	marker.id = marker_id;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;

	float box_size = getBoxVolume(box);
	if(box_size>0)
	{
		marker.color.a=alpha;

		BoundingBox box_s = scaleBox(box, scale);

		marker.pose.position.x = box_s.center.x;
		marker.pose.position.y = box_s.center.y;
		marker.pose.position.z = box_s.center.z;

		marker.scale.x = box_s.size.x;
		marker.scale.y = box_s.size.y;
		marker.scale.z = box_s.size.z;

		tf::Quaternion quat = tf::createQuaternionFromYaw(box.angle);
		tf::quaternionTFToMsg(quat, marker.pose.orientation);

	}
	else
	{
		marker.color.a=0;
	}

}


template <typename PointT>
void DrawRviz<PointT>::draw_box(const BoundingBox& box, const int& marker_id, visualization_msgs::Marker& marker, float alpha, float scale)
{
	marker.id = marker_id;
	marker.type = visualization_msgs::Marker::LINE_LIST;
	marker.action = visualization_msgs::Marker::ADD;

	float box_size = getBoxVolume(box);
	if(box_size>0)
	{

		tf::Quaternion quat = tf::createQuaternionFromRPY(0, 0, 0);
		tf::quaternionTFToMsg(quat, marker.pose.orientation);

		marker.color.a=alpha;
		std::vector<geometry_msgs::Point> cub_points;

		std::vector<Point3f> corners;
		if (getCornersFromBox2D(box, corners))
		{
			for (int i = 0; i < 8; ++i) {
				geometry_msgs::Point pts;
				pts.x = corners[i].x;
				pts.y = corners[i].y;
				pts.z = corners[i].z;
				cub_points.push_back(pts);
			}
		}else{
			for (int i = 0; i < 8; ++i) {
				geometry_msgs::Point pts;
				pts.x = 0;
				pts.y = 0;
				pts.z = 0;
				cub_points.push_back(pts);
			}
			marker.color.a=0;
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

template <typename PointT>
void DrawRviz<PointT>::draw_track_arrow(const RoboPerceptron<PointT> & per, const int& marker_id, visualization_msgs::Marker& marker, float yaw, float alpha)
{
	marker.id = marker_id;
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;

  float box_size = getBoxVolume(per.cluster.box);
  if (box_size>0)
  {
		marker.color.a = alpha;

    float arrow_length = per.tracker.velocity_abs.getLength();
    float main_direction = atan2f(per.tracker.velocity_abs.y, per.tracker.velocity_abs.x)-yaw;

		marker.scale.x = sqrtf(arrow_length + 1.0) - 1.0;
		if (per.labeler.label == 0)
		{
			marker.scale.y = 0.1;
			marker.scale.z = 0.1;
		}else{
			marker.scale.y = 0.2;
			marker.scale.z = 0.2;
		}

		tf::Quaternion quat = tf::createQuaternionFromRPY(0., 0., main_direction);
		tf::quaternionTFToMsg(quat, marker.pose.orientation);
		marker.pose.position.x = per.cluster.box.center.x;
		marker.pose.position.y = per.cluster.box.center.y;
//		marker.pose.position.x = per.cluster.barycenter.x;
//		marker.pose.position.y = per.cluster.barycenter.y;
		marker.pose.position.z = per.cluster.box.center.z;

  }
  else
  {
		marker.color.a = 0;
  }
}


template <typename PointT>
void DrawRviz<PointT>::draw_text(const Point3f& pos, const std::string& info, const int& marker_id, visualization_msgs::Marker& marker, float alpha)
{
	marker.id = marker_id;
	marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = pos.x;
	marker.pose.position.y = pos.y;
	marker.pose.position.z = pos.z;

	tf::Quaternion quat = tf::createQuaternionFromRPY(0, 0, 0);
	tf::quaternionTFToMsg(quat, marker.pose.orientation);

	marker.color.a = alpha;

	marker.text = info;
}

template class DrawRviz<pcl::PointXYZI>;
template class DrawRviz<pcl::PointXYZINormal>;

}

