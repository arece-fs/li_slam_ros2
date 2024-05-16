#include "scanmatcher/scanmatcher_component.h"

#include <chrono>

using namespace std::chrono_literals;

namespace graphslam
{
ScanMatcherComponent::ScanMatcherComponent(const rclcpp::NodeOptions & options)
: Node("scan_matcher", options),
  clock_(RCL_ROS_TIME),
  tfbuffer_(std::make_shared<rclcpp::Clock>(clock_)),
  listener_(tfbuffer_),
  broadcaster_(this)
{
  std::string registration_method;
  double ndt_resolution;
  int ndt_num_threads;
  double gicp_corr_dist_threshold;

  declare_parameter("global_frame_id", "map");
  get_parameter("global_frame_id", global_frame_id_);
  declare_parameter("robot_frame_id", "base_link");
  get_parameter("robot_frame_id", robot_frame_id_);
  declare_parameter("odom_frame_id", "odom");
  get_parameter("odom_frame_id", odom_frame_id_);
  declare_parameter("registration_method", "NDT");
  get_parameter("registration_method", registration_method);
  declare_parameter("ndt_resolution", 5.0);
  get_parameter("ndt_resolution", ndt_resolution);
  declare_parameter("ndt_num_threads", 0);
  get_parameter("ndt_num_threads", ndt_num_threads);
  declare_parameter("gicp_corr_dist_threshold", 5.0);
  get_parameter("gicp_corr_dist_threshold", gicp_corr_dist_threshold);
  declare_parameter("trans_for_mapupdate", 1.5);
  get_parameter("trans_for_mapupdate", trans_for_mapupdate_);
  declare_parameter("vg_size_for_input", 0.2);
  get_parameter("vg_size_for_input", vg_size_for_input_);
  declare_parameter("vg_size_for_map", 0.1);
  get_parameter("vg_size_for_map", vg_size_for_map_);
  declare_parameter("use_min_max_filter", false);
  get_parameter("use_min_max_filter", use_min_max_filter_);
  declare_parameter("scan_min_range", 0.1);
  get_parameter("scan_min_range", scan_min_range_);
  declare_parameter("scan_max_range", 100.0);
  get_parameter("scan_max_range", scan_max_range_);
  declare_parameter("map_publish_period", 15.0);
  get_parameter("map_publish_period", map_publish_period_);
  declare_parameter("num_targeted_cloud", 10);
  get_parameter("num_targeted_cloud", num_targeted_cloud_);
  if (num_targeted_cloud_ < 1) {
    std::cout << "num_tareged_cloud should be positive" << std::endl;
    num_targeted_cloud_ = 1;
  }

  declare_parameter("initial_pose_x", 0.0);
  get_parameter("initial_pose_x", initial_pose_x_);
  declare_parameter("initial_pose_y", 0.0);
  get_parameter("initial_pose_y", initial_pose_y_);
  declare_parameter("initial_pose_z", 0.0);
  get_parameter("initial_pose_z", initial_pose_z_);
  declare_parameter("initial_pose_qx", 0.0);
  get_parameter("initial_pose_qx", initial_pose_qx_);
  declare_parameter("initial_pose_qy", 0.0);
  get_parameter("initial_pose_qy", initial_pose_qy_);
  declare_parameter("initial_pose_qz", 0.0);
  get_parameter("initial_pose_qz", initial_pose_qz_);
  declare_parameter("initial_pose_qw", 1.0);
  get_parameter("initial_pose_qw", initial_pose_qw_);

  declare_parameter("set_initial_pose", false);
  get_parameter("set_initial_pose", set_initial_pose_);
  declare_parameter("use_odom", false);
  get_parameter("use_odom", use_odom_);
  declare_parameter("use_imu", false);
  get_parameter("use_imu", use_imu_);
  declare_parameter("debug_flag", false);
  get_parameter("debug_flag", debug_flag_);

  std::cout << "registration_method:" << registration_method << std::endl;
  std::cout << "ndt_resolution[m]:" << ndt_resolution << std::endl;
  std::cout << "ndt_num_threads:" << ndt_num_threads << std::endl;
  std::cout << "gicp_corr_dist_threshold[m]:" << gicp_corr_dist_threshold << std::endl;
  std::cout << "trans_for_mapupdate[m]:" << trans_for_mapupdate_ << std::endl;
  std::cout << "vg_size_for_input[m]:" << vg_size_for_input_ << std::endl;
  std::cout << "vg_size_for_map[m]:" << vg_size_for_map_ << std::endl;
  std::cout << "use_min_max_filter:" << std::boolalpha << use_min_max_filter_ << std::endl;
  std::cout << "scan_min_range[m]:" << scan_min_range_ << std::endl;
  std::cout << "scan_max_range[m]:" << scan_max_range_ << std::endl;
  std::cout << "set_initial_pose:" << std::boolalpha << set_initial_pose_ << std::endl;
  std::cout << "use_odom:" << std::boolalpha << use_odom_ << std::endl;
  std::cout << "use_imu:" << std::boolalpha << use_imu_ << std::endl;
  std::cout << "debug_flag:" << std::boolalpha << debug_flag_ << std::endl;
  std::cout << "map_publish_period[sec]:" << map_publish_period_ << std::endl;
  std::cout << "num_targeted_cloud:" << num_targeted_cloud_ << std::endl;
  std::cout << "------------------" << std::endl;

  if (registration_method == "NDT") {

    pclomp::NormalDistributionsTransform<PointType, PointType>::Ptr
      ndt(new pclomp::NormalDistributionsTransform<PointType, PointType>());
    ndt->setResolution(ndt_resolution);
    ndt->setTransformationEpsilon(0.01);
    // ndt_omp
    ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    if (ndt_num_threads > 0) {ndt->setNumThreads(ndt_num_threads);}

    registration_ = ndt;

  } else if (registration_method == "GICP") {
    boost::shared_ptr<pclomp::GeneralizedIterativeClosestPoint<PointType, PointType>>
      gicp(new pclomp::GeneralizedIterativeClosestPoint<PointType, PointType>());
    gicp->setMaxCorrespondenceDistance(gicp_corr_dist_threshold);
    //gicp->setCorrespondenceRandomness(20);
    gicp->setTransformationEpsilon(1e-8);
    registration_ = gicp;
  } else {
    RCLCPP_ERROR(get_logger(), "invalid registration method");
    exit(1);
  }

  map_array_msg_.header.frame_id = global_frame_id_;
  map_array_msg_.cloud_coordinate = map_array_msg_.LOCAL;

  path_.header.frame_id = global_frame_id_;

  initializePubSub();

  if (set_initial_pose_) {
    auto msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
    msg->header.stamp = now();
    msg->header.frame_id = global_frame_id_;
    msg->pose.position.x = initial_pose_x_;
    msg->pose.position.y = initial_pose_y_;
    msg->pose.position.z = initial_pose_z_;
    msg->pose.orientation.x = initial_pose_qx_;
    msg->pose.orientation.y = initial_pose_qy_;
    msg->pose.orientation.z = initial_pose_qz_;
    msg->pose.orientation.w = initial_pose_qw_;
    corrent_pose_stamped_ = *msg;
    pose_pub_->publish(corrent_pose_stamped_);
    initial_pose_received_ = true;

    path_.poses.push_back(*msg);
  }

  RCLCPP_INFO(get_logger(), "initialization end");
}

void ScanMatcherComponent::initializePubSub()
{
  RCLCPP_INFO(get_logger(), "initialize Publishers and Subscribers");
  // sub
  auto initial_pose_callback =
    [this](const typename geometry_msgs::msg::PoseStamped::SharedPtr msg) -> void
    {
      if (msg->header.frame_id != global_frame_id_) {
        RCLCPP_WARN(get_logger(), "This initial_pose is not in the global frame");
        return;
      }
      RCLCPP_INFO(get_logger(), "initial_pose is received");

      corrent_pose_stamped_ = *msg;
      previous_position_.x() = corrent_pose_stamped_.pose.position.x;
      previous_position_.y() = corrent_pose_stamped_.pose.position.y;
      previous_position_.z() = corrent_pose_stamped_.pose.position.z;
      initial_pose_received_ = true;

      pose_pub_->publish(corrent_pose_stamped_);
    };

  auto cloud_callback =
    [this](const typename sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void
    {
      if (!initial_pose_received_)
      {
        RCLCPP_WARN(get_logger(), "initial_pose is not received");
        return;
      }
      sensor_msgs::msg::PointCloud2 transformed_msg;
      try {
        tf2::TimePoint time_point = tf2::TimePoint(
          std::chrono::seconds(msg->header.stamp.sec) +
          std::chrono::nanoseconds(msg->header.stamp.nanosec));
        const geometry_msgs::msg::TransformStamped transform = tfbuffer_.lookupTransform(
          robot_frame_id_, msg->header.frame_id, time_point);
        tf2::doTransform(*msg, transformed_msg, transform); // TODO:slow now(https://github.com/ros/geometry2/pull/432)
      } catch (tf2::TransformException & e) {
        RCLCPP_ERROR(this->get_logger(), "%s", e.what());
        return;
      }

      pcl::PointCloud<PointType>::Ptr tmp_ptr(new pcl::PointCloud<PointType>());
      pcl::fromROSMsg(transformed_msg, *tmp_ptr);

      if (use_min_max_filter_) {
        double r;
        pcl::PointCloud<PointType> tmp;
        for (const auto & p : tmp_ptr->points) {
          r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
          if (scan_min_range_ < r && r < scan_max_range_) {tmp.push_back(p);}
        }
      }

      if (!initial_cloud_received_) {
        RCLCPP_INFO(get_logger(), "initial_cloud is received");
        initial_cloud_received_ = true;
        initializeMap(tmp_ptr, msg->header);
        last_map_time_ = clock_.now();
      }

      if (initial_cloud_received_) {receiveCloud(tmp_ptr, msg->header.stamp);}

    };


  auto odom_callback =
    [this](const typename nav_msgs::msg::Odometry::SharedPtr msg) -> void
    {
      if (initial_pose_received_) {receiveOdom(*msg);}
    };


  initial_pose_sub_ =
    create_subscription<geometry_msgs::msg::PoseStamped>(
    "initial_pose", rclcpp::QoS(10), initial_pose_callback);

  odom_sub_ =
    create_subscription<nav_msgs::msg::Odometry>(
    "preintegrated_odom", rclcpp::SensorDataQoS(), odom_callback);

  input_cloud_sub_ =
    create_subscription<sensor_msgs::msg::PointCloud2>(
    "input_cloud", rclcpp::SensorDataQoS(), cloud_callback);

  // pub
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    "current_pose",
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "map", rclcpp::QoS(
      rclcpp::KeepLast(
        1)).reliable());
  map_array_pub_ =
    create_publisher<lidarslam_msgs::msg::MapArray>(
    "map_array", rclcpp::QoS(
      rclcpp::KeepLast(
        1)).reliable());
  path_pub_ = create_publisher<nav_msgs::msg::Path>("path", 1);
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
    "odom", rclcpp::QoS(
      rclcpp::KeepLast(
        1)).reliable());
}

void ScanMatcherComponent::initializeMap(const pcl::PointCloud <pcl::PointXYZI>::Ptr & tmp_ptr, const std_msgs::msg::Header & header)
{
  RCLCPP_INFO(get_logger(), "create a first map");
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
  voxel_grid.setLeafSize(vg_size_for_map_, vg_size_for_map_, vg_size_for_map_);
  voxel_grid.setInputCloud(tmp_ptr);
  voxel_grid.filter(*cloud_ptr);

  Eigen::Matrix4f sim_trans = getTransformation(corrent_pose_stamped_.pose);
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_ptr(
    new pcl::PointCloud<pcl::PointXYZI>());
  pcl::transformPointCloud(*cloud_ptr, *transformed_cloud_ptr, sim_trans);
  registration_->setInputTarget(transformed_cloud_ptr);

  // map
  sensor_msgs::msg::PointCloud2::SharedPtr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*transformed_cloud_ptr, *map_msg_ptr);

  // map array
  sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_ptr(
    new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*cloud_ptr, *cloud_msg_ptr);
  lidarslam_msgs::msg::SubMap submap;
  submap.header = header;
  submap.distance = 0;
  submap.pose = corrent_pose_stamped_.pose;
  submap.cloud = *cloud_msg_ptr;
  map_array_msg_.header = header;
  map_array_msg_.submaps.push_back(submap);

  map_pub_->publish(submap.cloud);
}

void ScanMatcherComponent::receiveCloud(
  const pcl::PointCloud<PointType>::ConstPtr & cloud_ptr,
  const rclcpp::Time stamp)
{
  if (mapping_flag_ && mapping_future_.valid()) {
    auto status = mapping_future_.wait_for(0s);
    if (status == std::future_status::ready) {
      if (is_map_updated_ == true) {
        pcl::PointCloud<PointType>::Ptr targeted_cloud_ptr(new pcl::PointCloud<PointType>(
            targeted_cloud_));
        registration_->setInputTarget(targeted_cloud_ptr);
        is_map_updated_ = false;
      }
      mapping_flag_ = false;
      mapping_thread_.detach();
    }
  }

  pcl::PointCloud<PointType>::Ptr filtered_cloud_ptr(new pcl::PointCloud<PointType>());
  pcl::VoxelGrid<PointType> voxel_grid;
  voxel_grid.setLeafSize(vg_size_for_input_, vg_size_for_input_, vg_size_for_input_);
  voxel_grid.setInputCloud(cloud_ptr);
  voxel_grid.filter(*filtered_cloud_ptr);
  registration_->setInputSource(filtered_cloud_ptr);

  Eigen::Matrix4f sim_trans = getTransformation(corrent_pose_stamped_.pose);

  if (use_odom_) {
    geometry_msgs::msg::TransformStamped odom_trans;
    try {
      odom_trans = tfbuffer_.lookupTransform(
        odom_frame_id_, robot_frame_id_, tf2_ros::fromMsg(
          stamp));
    } catch (tf2::TransformException & e) {
      RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    }
    Eigen::Affine3d odom_affine = tf2::transformToEigen(odom_trans);
    Eigen::Matrix4f odom_mat = odom_affine.matrix().cast<float>();
    if (previous_odom_mat_ != Eigen::Matrix4f::Identity()) {
      sim_trans = sim_trans * previous_odom_mat_.inverse() * odom_mat;
    }
    previous_odom_mat_ = odom_mat;
  }

  if (use_imu_ && odom_ptr_last_ != -1) {
    int odom_ptr = odom_ptr_front_;
    while (odom_ptr != odom_ptr_last_) {
      rclcpp::Time odom_stamp = odom_que_[odom_ptr].header.stamp;
      if (odom_stamp.nanoseconds() > stamp.nanoseconds()) {break;}
      odom_ptr = (odom_ptr + 1) % odom_que_length_;
    }
    Eigen::Matrix4f odom_position = getTransformation(odom_que_[odom_ptr].pose.pose);
    sim_trans = odom_position;
    odom_ptr_front_ = odom_ptr;
  }

  pcl::PointCloud<PointType>::Ptr output_cloud(new pcl::PointCloud<PointType>);
  rclcpp::Clock system_clock;
  rclcpp::Time time_align_start = system_clock.now();
  registration_->align(*output_cloud, sim_trans);
  rclcpp::Time time_align_end = system_clock.now();

  Eigen::Matrix4f final_transformation = registration_->getFinalTransformation();


  getCovariance(filtered_cloud_ptr,output_cloud);

  publishMapAndPose(cloud_ptr, final_transformation, stamp);

  if (!debug_flag_) {return;}

  tf2::Quaternion quat_tf;
  double roll, pitch, yaw;
  tf2::fromMsg(corrent_pose_stamped_.pose.orientation, quat_tf);
  tf2::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);

  std::cout << "---------------------------------------------------------" << std::endl;
  std::cout << "nanoseconds: " << stamp.nanoseconds() << std::endl;
  std::cout << "trans: " << trans_ << std::endl;
  std::cout << "align time:" << time_align_end.seconds() - time_align_start.seconds() << "s" <<
    std::endl;
  std::cout << "number of filtered cloud points: " << filtered_cloud_ptr->size() << std::endl;
  std::cout << "initial transformation:" << std::endl;
  std::cout << sim_trans << std::endl;
  std::cout << "has converged: " << registration_->hasConverged() << std::endl;
  std::cout << "fitness score: " << registration_->getFitnessScore() << std::endl;
  std::cout << "final transformation:" << std::endl;
  std::cout << final_transformation << std::endl;
  std::cout << "rpy" << std::endl;
  std::cout << "roll:" << roll * 180 / M_PI << "," <<
    "pitch:" << pitch * 180 / M_PI << "," <<
    "yaw:" << yaw * 180 / M_PI << std::endl;
  int num_submaps = map_array_msg_.submaps.size();
  std::cout << "num_submaps:" << num_submaps << std::endl;
  std::cout << "moving distance:" << latest_distance_ << std::endl;
  std::cout << "---------------------------------------------------------" << std::endl;
}

void ScanMatcherComponent::getCovariance(
  const pcl::PointCloud < PointType >::ConstPtr & cloud_in,
  const pcl::PointCloud < PointType >::ConstPtr & cloud_out)
{
  pcl::PointCloud<PointType> cld1, cld2;
  pcl::copyPointCloud(*cloud_in, cld1);
  pcl::copyPointCloud(*cloud_out,cld2);
  Eigen::Matrix4f final_transformation;
  final_transformation << 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;

  pcl::Correspondences correspondeces_reciprocal_shot;

  int iterations = 50;// We just hard code the number of ICP iterations to be 50
    for (int i = 0; i < iterations; i++)
    {


        pcl::registration::CorrespondenceEstimation<PointType, PointType> corr_est;
        corr_est.setInputSource(cld1.makeShared()); // + setIndices(...)
        corr_est.setInputTarget(cld2.makeShared());
        corr_est.determineReciprocalCorrespondences(correspondeces_reciprocal_shot);
        //cout << "No. of Reciprocal Correspondences : " << correspondeces_reciprocal_shot.size() << endl;


        Eigen::Matrix4f transform_eigen;
        pcl::registration::TransformationEstimationSVD<PointType, PointType> pp_svd;
        pp_svd.estimateRigidTransformation (cld1, cld2, correspondeces_reciprocal_shot, transform_eigen);


        // rotate/transform data based on this estimated transformation
        pcl::transformPointCloud(cld1, cld1, transform_eigen);


        // accumulate incremental tf
        final_transformation = transform_eigen * final_transformation;


    }

    std::vector<int> data_idx;
    std::vector<int> model_idx;

    pcl::Correspondence temp1;
    for (int i = 0; i < correspondeces_reciprocal_shot.size(); i++)
    {
        temp1 = correspondeces_reciprocal_shot[i];
        data_idx.push_back(temp1.index_query);
        model_idx.push_back(temp1.index_match);
    }

    pcl::PointCloud<PointType> data_pi; //Put all the pi in this cloud and its size will be equal to number of correspondences
    pcl::PointCloud<PointType> model_qi;// Put all the qi in this cloud and its size will be equal to number of correspondences

    pcl::copyPointCloud(cld1,data_idx,data_pi);
    pcl::copyPointCloud(cld2,model_idx,model_qi);

    Eigen::MatrixXd ICP_COV(6,6);
    ICP_COV = Eigen::MatrixXd::Zero(6,6);


    calculateIcpCov(data_pi, model_qi, final_transformation, ICP_COV);

}


void ScanMatcherComponent::calculateIcpCov(pcl::PointCloud<PointType>& data_pi, pcl::PointCloud<PointType>& model_qi, Eigen::Matrix4f& transform, Eigen::MatrixXd& ICP_COV)
{
    double Tx = transform(0,3);
    double Ty = transform(1,3);
    double Tz = transform(2,3);
    double roll  = atan2f(transform(2,1), transform(2,2));
    double pitch = asinf(-transform(2,0));
    double yaw   = atan2f(transform(1,0), transform(0,0));

    double x, y, z, a, b, c;
    x = Tx; y = Ty; z = Tz;
    a = yaw; b = pitch; c = roll;// important // According to the rotation matrix I used and after verification, it is Yaw Pitch ROLL = [a,b,c]== [R] matrix used in the MatLab also :)

    /* Flushing out in the form of XYZ ABC */
    std::cout << "\nPrinting out [x, y, z, a, b, c] =  " <<x<<"    "<<y<<"    "<<z<<"    "<<a<<"    "<<b<<"    "<<c<<std::endl;

    //Matrix initialization
    Eigen::MatrixXd d2J_dX2(6,6);
    d2J_dX2 = Eigen::MatrixXd::Zero(6,6);

    /****  Calculating d2J_dX2  ****/
    for (size_t s = 0; s < data_pi.points.size(); ++s )
    {
        double pix = data_pi.points[s].x;
        double piy = data_pi.points[s].y;
        double piz = data_pi.points[s].z;
        double qix = model_qi.points[s].x;
        double qiy = model_qi.points[s].y;
        double qiz = model_qi.points[s].z;

        /************************************************************

d2J_dX2 -- X is the [R|T] in the form of [x, y, z, a, b, c]
x, y, z is the translation part 
a, b, c is the rotation part in Euler format
[x, y, z, a, b, c] is acquired from the Transformation Matrix returned by ICP.

Now d2J_dX2 is a 6x6 matrix of the form

d2J_dx2                   
d2J_dxdy    d2J_dy2
d2J_dxdz    d2J_dydz    d2J_dz2
d2J_dxda    d2J_dyda    d2J_dzda   d2J_da2
d2J_dxdb    d2J_dydb    d2J_dzdb   d2J_dadb   d2J_db2
d2J_dxdc    d2J_dydc    d2J_dzdc   d2J_dadc   d2J_dbdc   d2J_dc2


*************************************************************/

        double d2J_dx2,     d2J_dydx,	  d2J_dzdx,   d2J_dadx,   d2J_dbdx,     d2J_dcdx,
                d2J_dxdy,    d2J_dy2,	  d2J_dzdy,   d2J_dady,   d2J_dbdy,     d2J_dcdy,
                d2J_dxdz,    d2J_dydz,    d2J_dz2,    d2J_dadz,   d2J_dbdz,     d2J_dcdz,
                d2J_dxda,    d2J_dyda,    d2J_dzda,   d2J_da2,	  d2J_dbda,     d2J_dcda,
                d2J_dxdb,    d2J_dydb,    d2J_dzdb,   d2J_dadb,   d2J_db2,      d2J_dcdb,
                d2J_dxdc,    d2J_dydc,    d2J_dzdc,   d2J_dadc,   d2J_dbdc,     d2J_dc2;


        // These terms are generated from the provided Matlab scipts. We just have to copy
        // the expressions from the matlab output with two very simple changes.
        // The first one being the the sqaure of a number 'a' is shown as a^2 in matlab,
        // which is converted to pow(a,2) in the below expressions.
        // The second change is to add ';' at the end of each expression :)
        // In this way, matlab can be used to generate these terms for various objective functions of ICP
        // and they can simply be copied to the C++ files and with appropriate changes to ICP estimation,
        // its covariance can be easily estimated.




        d2J_dx2 =

                2;


        d2J_dy2 =

                2;


        d2J_dz2 =

                2;


        d2J_dydx =

                0;


        d2J_dxdy =

                0;


        d2J_dzdx =

                0;


        d2J_dxdz =

                0;


        d2J_dydz =

                0;


        d2J_dzdy =

                0;


        d2J_da2 =

                (piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + pix*cos(a)*cos(b))*(2*piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - 2*piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + 2*pix*cos(a)*cos(b)) - (2*piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - 2*piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + 2*pix*cos(b)*sin(a))*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)) + (piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a))*(2*piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - 2*piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + 2*pix*cos(b)*sin(a)) - (2*piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - 2*piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + 2*pix*cos(a)*cos(b))*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b));


        d2J_db2 =

                (pix*cos(b) + piz*cos(c)*sin(b) + piy*sin(b)*sin(c))*(2*pix*cos(b) + 2*piz*cos(c)*sin(b) + 2*piy*sin(b)*sin(c)) - (2*piz*cos(b)*cos(c) - 2*pix*sin(b) + 2*piy*cos(b)*sin(c))*(z - qiz - pix*sin(b) + piz*cos(b)*cos(c) + piy*cos(b)*sin(c)) - (2*pix*cos(a)*cos(b) + 2*piz*cos(a)*cos(c)*sin(b) + 2*piy*cos(a)*sin(b)*sin(c))*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b)) + (piz*cos(a)*cos(b)*cos(c) - pix*cos(a)*sin(b) + piy*cos(a)*cos(b)*sin(c))*(2*piz*cos(a)*cos(b)*cos(c) - 2*pix*cos(a)*sin(b) + 2*piy*cos(a)*cos(b)*sin(c)) - (2*pix*cos(b)*sin(a) + 2*piz*cos(c)*sin(a)*sin(b) + 2*piy*sin(a)*sin(b)*sin(c))*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)) + (piz*cos(b)*cos(c)*sin(a) - pix*sin(a)*sin(b) + piy*cos(b)*sin(a)*sin(c))*(2*piz*cos(b)*cos(c)*sin(a) - 2*pix*sin(a)*sin(b) + 2*piy*cos(b)*sin(a)*sin(c));


        d2J_dc2 =

                (piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)))*(2*piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + 2*piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c))) + (piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)))*(2*piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + 2*piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c))) - (2*piz*cos(b)*cos(c) + 2*piy*cos(b)*sin(c))*(z - qiz - pix*sin(b) + piz*cos(b)*cos(c) + piy*cos(b)*sin(c)) + (2*piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) - 2*piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)))*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b)) + (piy*cos(b)*cos(c) - piz*cos(b)*sin(c))*(2*piy*cos(b)*cos(c) - 2*piz*cos(b)*sin(c)) - (2*piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - 2*piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)))*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a));


        d2J_dxda =

                2*piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) - 2*piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - 2*pix*cos(b)*sin(a);


        d2J_dadx =

                2*piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) - 2*piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - 2*pix*cos(b)*sin(a);


        d2J_dyda =

                2*piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - 2*piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + 2*pix*cos(a)*cos(b);


        d2J_dady =

                2*piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - 2*piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + 2*pix*cos(a)*cos(b);


        d2J_dzda =

                0;


        d2J_dadz =

                0;


        d2J_dxdb =

                2*piz*cos(a)*cos(b)*cos(c) - 2*pix*cos(a)*sin(b) + 2*piy*cos(a)*cos(b)*sin(c);


        d2J_dbdx =

                2*piz*cos(a)*cos(b)*cos(c) - 2*pix*cos(a)*sin(b) + 2*piy*cos(a)*cos(b)*sin(c);


        d2J_dydb =

                2*piz*cos(b)*cos(c)*sin(a) - 2*pix*sin(a)*sin(b) + 2*piy*cos(b)*sin(a)*sin(c);


        d2J_dbdy =

                2*piz*cos(b)*cos(c)*sin(a) - 2*pix*sin(a)*sin(b) + 2*piy*cos(b)*sin(a)*sin(c);


        d2J_dzdb =

                - 2*pix*cos(b) - 2*piz*cos(c)*sin(b) - 2*piy*sin(b)*sin(c);


        d2J_dbdz =

                - 2*pix*cos(b) - 2*piz*cos(c)*sin(b) - 2*piy*sin(b)*sin(c);


        d2J_dxdc =

                2*piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + 2*piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c));


        d2J_dcdx =

                2*piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + 2*piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c));


        d2J_dydc =

                - 2*piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) - 2*piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c));


        d2J_dcdy =

                - 2*piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) - 2*piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c));


        d2J_dzdc =

                2*piy*cos(b)*cos(c) - 2*piz*cos(b)*sin(c);


        d2J_dcdz =

                2*piy*cos(b)*cos(c) - 2*piz*cos(b)*sin(c);


        d2J_dadb =

                (2*piz*cos(b)*cos(c)*sin(a) - 2*pix*sin(a)*sin(b) + 2*piy*cos(b)*sin(a)*sin(c))*(piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + pix*cos(a)*cos(b)) - (2*piz*cos(a)*cos(b)*cos(c) - 2*pix*cos(a)*sin(b) + 2*piy*cos(a)*cos(b)*sin(c))*(piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)) + (2*piz*cos(a)*cos(b)*cos(c) - 2*pix*cos(a)*sin(b) + 2*piy*cos(a)*cos(b)*sin(c))*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)) - (2*piz*cos(b)*cos(c)*sin(a) - 2*pix*sin(a)*sin(b) + 2*piy*cos(b)*sin(a)*sin(c))*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b));


        d2J_dbda =

                (piz*cos(b)*cos(c)*sin(a) - pix*sin(a)*sin(b) + piy*cos(b)*sin(a)*sin(c))*(2*piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - 2*piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + 2*pix*cos(a)*cos(b)) - (piz*cos(a)*cos(b)*cos(c) - pix*cos(a)*sin(b) + piy*cos(a)*cos(b)*sin(c))*(2*piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - 2*piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + 2*pix*cos(b)*sin(a)) + (2*piz*cos(a)*cos(b)*cos(c) - 2*pix*cos(a)*sin(b) + 2*piy*cos(a)*cos(b)*sin(c))*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)) - (2*piz*cos(b)*cos(c)*sin(a) - 2*pix*sin(a)*sin(b) + 2*piy*cos(b)*sin(a)*sin(c))*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b));


        d2J_dbdc =

                (2*piy*cos(a)*cos(b)*cos(c) - 2*piz*cos(a)*cos(b)*sin(c))*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b)) + (2*piy*cos(b)*cos(c)*sin(a) - 2*piz*cos(b)*sin(a)*sin(c))*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)) - (2*piy*cos(b)*cos(c) - 2*piz*cos(b)*sin(c))*(pix*cos(b) + piz*cos(c)*sin(b) + piy*sin(b)*sin(c)) - (2*piy*cos(c)*sin(b) - 2*piz*sin(b)*sin(c))*(z - qiz - pix*sin(b) + piz*cos(b)*cos(c) + piy*cos(b)*sin(c)) + (2*piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + 2*piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)))*(piz*cos(a)*cos(b)*cos(c) - pix*cos(a)*sin(b) + piy*cos(a)*cos(b)*sin(c)) - (2*piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + 2*piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)))*(piz*cos(b)*cos(c)*sin(a) - pix*sin(a)*sin(b) + piy*cos(b)*sin(a)*sin(c));


        d2J_dcdb =

                (2*piy*cos(a)*cos(b)*cos(c) - 2*piz*cos(a)*cos(b)*sin(c))*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b)) + (2*piy*cos(b)*cos(c)*sin(a) - 2*piz*cos(b)*sin(a)*sin(c))*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)) - (piy*cos(b)*cos(c) - piz*cos(b)*sin(c))*(2*pix*cos(b) + 2*piz*cos(c)*sin(b) + 2*piy*sin(b)*sin(c)) - (2*piy*cos(c)*sin(b) - 2*piz*sin(b)*sin(c))*(z - qiz - pix*sin(b) + piz*cos(b)*cos(c) + piy*cos(b)*sin(c)) + (piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)))*(2*piz*cos(a)*cos(b)*cos(c) - 2*pix*cos(a)*sin(b) + 2*piy*cos(a)*cos(b)*sin(c)) - (piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)))*(2*piz*cos(b)*cos(c)*sin(a) - 2*pix*sin(a)*sin(b) + 2*piy*cos(b)*sin(a)*sin(c));


        d2J_dcda =

                (2*piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + 2*piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)))*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b)) - (piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)))*(2*piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - 2*piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + 2*pix*cos(b)*sin(a)) - (piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)))*(2*piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - 2*piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + 2*pix*cos(a)*cos(b)) + (2*piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + 2*piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)))*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a));


        d2J_dadc =

                (2*piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + 2*piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)))*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b)) - (2*piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + 2*piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)))*(piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)) - (2*piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + 2*piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)))*(piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + pix*cos(a)*cos(b)) + (2*piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + 2*piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)))*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a));





        Eigen::MatrixXd d2J_dX2_temp(6,6);

        d2J_dX2_temp << d2J_dx2,     d2J_dydx,	  d2J_dzdx,   d2J_dadx,   d2J_dbdx,     d2J_dcdx,
                        d2J_dxdy,    d2J_dy2,	  d2J_dzdy,   d2J_dady,   d2J_dbdy,     d2J_dcdy,
                        d2J_dxdz,    d2J_dydz,    d2J_dz2,    d2J_dadz,   d2J_dbdz,     d2J_dcdz,
                        d2J_dxda,    d2J_dyda,    d2J_dzda,   d2J_da2,	  d2J_dbda,     d2J_dcda,
                        d2J_dxdb,    d2J_dydb,    d2J_dzdb,   d2J_dadb,   d2J_db2,      d2J_dcdb,
                        d2J_dxdc,    d2J_dydc,    d2J_dzdc,   d2J_dadc,   d2J_dbdc,     d2J_dc2;


        d2J_dX2 = d2J_dX2 + d2J_dX2_temp;

    }// End of the FOR loop!!!

    std::cout << "\n**************\n Successfully Computed d2J_dX2 \n**************\n" << std::endl;



    // Now its time to calculate d2J_dZdX , where Z are the measurements Pi and Qi, X = [x,y,z,a,b,c]

    // n is the number of correspondences
    int n = data_pi.points.size();





    /*  Here we check if the number of correspondences between the source and the target point clouds are greater than 200.
      if yes, we only take the first 200 correspondences to calculate the covariance matrix
      You can try increasing it but if its too high, the system may run out of memory and give an exception saying

     terminate called after throwing an instance of 'std::bad_alloc'
     what():  std::bad_alloc
     Aborted (core dumped)

  */


    if (n > 200) n = 200;////////////****************************IMPORTANT CHANGE********but may not affect*****************/////////////////////////////////////////

    std::cout << "\nNumber of Correspondences used for ICP's covariance estimation = " << n << std::endl;

    Eigen::MatrixXd d2J_dZdX(6,6*n);

    for (int k = 0; k < n ; ++k) // row
    {
        //here the current correspondences are loaded into Pi and Qi
        double pix = data_pi.points[k].x;
        double piy = data_pi.points[k].y;
        double piz = data_pi.points[k].z;
        double qix = model_qi.points[k].x;
        double qiy = model_qi.points[k].y;
        double qiz = model_qi.points[k].z;

        Eigen::MatrixXd d2J_dZdX_temp(6,6);


        double 	d2J_dpix_dx,    d2J_dpiy_dx,	d2J_dpiz_dx,  	   d2J_dqix_dx,    d2J_dqiy_dx,	   d2J_dqiz_dx,
                d2J_dpix_dy,    d2J_dpiy_dy,	d2J_dpiz_dy,   	   d2J_dqix_dy,    d2J_dqiy_dy,	   d2J_dqiz_dy,
                d2J_dpix_dz,    d2J_dpiy_dz,    d2J_dpiz_dz,       d2J_dqix_dz,    d2J_dqiy_dz,    d2J_dqiz_dz,
                d2J_dpix_da,    d2J_dpiy_da,    d2J_dpiz_da,       d2J_dqix_da,    d2J_dqiy_da,    d2J_dqiz_da,
                d2J_dpix_db,    d2J_dpiy_db,    d2J_dpiz_db,       d2J_dqix_db,    d2J_dqiy_db,    d2J_dqiz_db,
                d2J_dpix_dc,    d2J_dpiy_dc,    d2J_dpiz_dc,       d2J_dqix_dc,    d2J_dqiy_dc,    d2J_dqiz_dc;




        d2J_dpix_dx =

                2*cos(a)*cos(b);


        d2J_dpix_dy =

                2*cos(b)*sin(a);


        d2J_dpix_dz =

                -2*sin(b);


        d2J_dpix_da =

                cos(b)*sin(a)*(2*piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - 2*piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + 2*pix*cos(a)*cos(b)) - cos(a)*cos(b)*(2*piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - 2*piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + 2*pix*cos(b)*sin(a)) - 2*cos(b)*sin(a)*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b)) + 2*cos(a)*cos(b)*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a));


        d2J_dpix_db =

                sin(b)*(2*pix*cos(b) + 2*piz*cos(c)*sin(b) + 2*piy*sin(b)*sin(c)) - 2*cos(b)*(z - qiz - pix*sin(b) + piz*cos(b)*cos(c) + piy*cos(b)*sin(c)) + cos(a)*cos(b)*(2*piz*cos(a)*cos(b)*cos(c) - 2*pix*cos(a)*sin(b) + 2*piy*cos(a)*cos(b)*sin(c)) - 2*sin(a)*sin(b)*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)) + cos(b)*sin(a)*(2*piz*cos(b)*cos(c)*sin(a) - 2*pix*sin(a)*sin(b) + 2*piy*cos(b)*sin(a)*sin(c)) - 2*cos(a)*sin(b)*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b));


        d2J_dpix_dc =

                cos(a)*cos(b)*(2*piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + 2*piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c))) - sin(b)*(2*piy*cos(b)*cos(c) - 2*piz*cos(b)*sin(c)) - cos(b)*sin(a)*(2*piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + 2*piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)));


        d2J_dpiy_dx =

                2*cos(a)*sin(b)*sin(c) - 2*cos(c)*sin(a);


        d2J_dpiy_dy =

                2*cos(a)*cos(c) + 2*sin(a)*sin(b)*sin(c);


        d2J_dpiy_dz =

                2*cos(b)*sin(c);


        d2J_dpiy_da =

                (cos(a)*cos(c) + sin(a)*sin(b)*sin(c))*(2*piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - 2*piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + 2*pix*cos(a)*cos(b)) + (cos(c)*sin(a) - cos(a)*sin(b)*sin(c))*(2*piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - 2*piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + 2*pix*cos(b)*sin(a)) - (2*cos(a)*cos(c) + 2*sin(a)*sin(b)*sin(c))*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b)) - (2*cos(c)*sin(a) - 2*cos(a)*sin(b)*sin(c))*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a));


        d2J_dpiy_db =

                (cos(a)*cos(c) + sin(a)*sin(b)*sin(c))*(2*piz*cos(b)*cos(c)*sin(a) - 2*pix*sin(a)*sin(b) + 2*piy*cos(b)*sin(a)*sin(c)) - (cos(c)*sin(a) - cos(a)*sin(b)*sin(c))*(2*piz*cos(a)*cos(b)*cos(c) - 2*pix*cos(a)*sin(b) + 2*piy*cos(a)*cos(b)*sin(c)) - 2*sin(b)*sin(c)*(z - qiz - pix*sin(b) + piz*cos(b)*cos(c) + piy*cos(b)*sin(c)) - cos(b)*sin(c)*(2*pix*cos(b) + 2*piz*cos(c)*sin(b) + 2*piy*sin(b)*sin(c)) + 2*cos(a)*cos(b)*sin(c)*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b)) + 2*cos(b)*sin(a)*sin(c)*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a));


        d2J_dpiy_dc =

                (2*sin(a)*sin(c) + 2*cos(a)*cos(c)*sin(b))*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b)) - (2*cos(a)*sin(c) - 2*cos(c)*sin(a)*sin(b))*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)) - (cos(a)*cos(c) + sin(a)*sin(b)*sin(c))*(2*piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + 2*piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c))) - (cos(c)*sin(a) - cos(a)*sin(b)*sin(c))*(2*piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + 2*piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c))) + 2*cos(b)*cos(c)*(z - qiz - pix*sin(b) + piz*cos(b)*cos(c) + piy*cos(b)*sin(c)) + cos(b)*sin(c)*(2*piy*cos(b)*cos(c) - 2*piz*cos(b)*sin(c));


        d2J_dpiz_dx =

                2*sin(a)*sin(c) + 2*cos(a)*cos(c)*sin(b);


        d2J_dpiz_dy =

                2*cos(c)*sin(a)*sin(b) - 2*cos(a)*sin(c);


        d2J_dpiz_dz =

                2*cos(b)*cos(c);


        d2J_dpiz_da =

                (2*cos(a)*sin(c) - 2*cos(c)*sin(a)*sin(b))*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b)) - (sin(a)*sin(c) + cos(a)*cos(c)*sin(b))*(2*piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - 2*piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + 2*pix*cos(b)*sin(a)) - (cos(a)*sin(c) - cos(c)*sin(a)*sin(b))*(2*piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - 2*piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + 2*pix*cos(a)*cos(b)) + (2*sin(a)*sin(c) + 2*cos(a)*cos(c)*sin(b))*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a));


        d2J_dpiz_db =

                (sin(a)*sin(c) + cos(a)*cos(c)*sin(b))*(2*piz*cos(a)*cos(b)*cos(c) - 2*pix*cos(a)*sin(b) + 2*piy*cos(a)*cos(b)*sin(c)) - (cos(a)*sin(c) - cos(c)*sin(a)*sin(b))*(2*piz*cos(b)*cos(c)*sin(a) - 2*pix*sin(a)*sin(b) + 2*piy*cos(b)*sin(a)*sin(c)) - 2*cos(c)*sin(b)*(z - qiz - pix*sin(b) + piz*cos(b)*cos(c) + piy*cos(b)*sin(c)) - cos(b)*cos(c)*(2*pix*cos(b) + 2*piz*cos(c)*sin(b) + 2*piy*sin(b)*sin(c)) + 2*cos(a)*cos(b)*cos(c)*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b)) + 2*cos(b)*cos(c)*sin(a)*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a));


        d2J_dpiz_dc =

                (2*cos(c)*sin(a) - 2*cos(a)*sin(b)*sin(c))*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b)) - (2*cos(a)*cos(c) + 2*sin(a)*sin(b)*sin(c))*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)) + (sin(a)*sin(c) + cos(a)*cos(c)*sin(b))*(2*piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + 2*piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c))) + (cos(a)*sin(c) - cos(c)*sin(a)*sin(b))*(2*piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + 2*piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c))) + cos(b)*cos(c)*(2*piy*cos(b)*cos(c) - 2*piz*cos(b)*sin(c)) - 2*cos(b)*sin(c)*(z - qiz - pix*sin(b) + piz*cos(b)*cos(c) + piy*cos(b)*sin(c));


        d2J_dqix_dx =

                -2;


        d2J_dqix_dy =

                0;


        d2J_dqix_dz =

                0;


        d2J_dqix_da =

                2*piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - 2*piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + 2*pix*cos(b)*sin(a);


        d2J_dqix_db =

                2*pix*cos(a)*sin(b) - 2*piz*cos(a)*cos(b)*cos(c) - 2*piy*cos(a)*cos(b)*sin(c);


        d2J_dqix_dc =

                - 2*piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - 2*piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c));


        d2J_dqiy_dx =

                0;


        d2J_dqiy_dy =

                -2;


        d2J_dqiy_dz =

                0;


        d2J_dqiy_da =

                2*piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) - 2*piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - 2*pix*cos(a)*cos(b);


        d2J_dqiy_db =

                2*pix*sin(a)*sin(b) - 2*piz*cos(b)*cos(c)*sin(a) - 2*piy*cos(b)*sin(a)*sin(c);


        d2J_dqiy_dc =

                2*piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + 2*piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c));


        d2J_dqiz_dx =

                0;


        d2J_dqiz_dy =

                0;


        d2J_dqiz_dz =

                -2;


        d2J_dqiz_da =

                0;


        d2J_dqiz_db =

                2*pix*cos(b) + 2*piz*cos(c)*sin(b) + 2*piy*sin(b)*sin(c);


        d2J_dqiz_dc =

                2*piz*cos(b)*sin(c) - 2*piy*cos(b)*cos(c);






        d2J_dZdX_temp <<    d2J_dpix_dx,    d2J_dpiy_dx,	d2J_dpiz_dx,  	   d2J_dqix_dx,    d2J_dqiy_dx,	   d2J_dqiz_dx,
                            d2J_dpix_dy,    d2J_dpiy_dy,	d2J_dpiz_dy,   	   d2J_dqix_dy,    d2J_dqiy_dy,	   d2J_dqiz_dy,
                            d2J_dpix_dz,    d2J_dpiy_dz,        d2J_dpiz_dz,       d2J_dqix_dz,    d2J_dqiy_dz,    d2J_dqiz_dz,
                            d2J_dpix_da,    d2J_dpiy_da,        d2J_dpiz_da,       d2J_dqix_da,    d2J_dqiy_da,    d2J_dqiz_da,
                            d2J_dpix_db,    d2J_dpiy_db,        d2J_dpiz_db,       d2J_dqix_db,    d2J_dqiy_db,    d2J_dqiz_db,
                            d2J_dpix_dc,    d2J_dpiy_dc,        d2J_dpiz_dc,       d2J_dqix_dc,    d2J_dqiy_dc,    d2J_dqiz_dc;


        d2J_dZdX.block<6,6>(0,6*k) = d2J_dZdX_temp;

    }


    //By reaching here both the matrices d2J_dX2 and d2J_dZdX are calculated and lets print those values out;

    //std::cout << "\n Finally here are the results \n\n" << "d2J_dX2 = \n " << d2J_dX2 <<std::endl;
    //std::cout << "\n\n\n" << "d2J_dZdX = \n " << d2J_dZdX <<std::endl;



    //By reaching here both the matrices d2J_dX2 and d2J_dZdX are calculated and lets print those values out;

    //std::cout << "\n Finally here are the two matrices \n\n" << "d2J_dX2 = \n " << d2J_dX2 <<std::endl;
    //std::cout << "\n\n\n" << "d2J_dZdX = \n " << d2J_dZdX <<std::endl;



    /**************************************
     *
     * Here we create the matrix cov(z) as mentioned in Section 3.3 in the paper, "Covariance of ICP with 3D Point to Point and Point to Plane Error Metrics"
     *
     * ************************************/

    Eigen::MatrixXd cov_z(6*n,6*n);
    cov_z = 0.01 * Eigen::MatrixXd::Identity(6*n,6*n);



    ICP_COV =  d2J_dX2.inverse() * d2J_dZdX * cov_z * d2J_dZdX.transpose() * d2J_dX2.inverse();




    std::cout << "\n\n********************** \n\n" << "ICP_COV = \n" << ICP_COV <<"\n*******************\n\n"<< std::endl;

    std::cout << "\nSuccessfully Computed the ICP's Covariance !!!\n" << std::endl;
}




void ScanMatcherComponent::publishMapAndPose(
  const pcl::PointCloud<PointType>::ConstPtr & cloud_ptr,
  const Eigen::Matrix4f final_transformation, const rclcpp::Time stamp)
{

  Eigen::Vector3d position = final_transformation.block<3, 1>(0, 3).cast<double>();

  Eigen::Matrix3d rot_mat = final_transformation.block<3, 3>(0, 0).cast<double>();
  Eigen::Quaterniond quat_eig(rot_mat);
  geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig);

  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = stamp;
  transform_stamped.header.frame_id = global_frame_id_;
  transform_stamped.child_frame_id = robot_frame_id_;
  transform_stamped.transform.translation.x = position.x();
  transform_stamped.transform.translation.y = position.y();
  transform_stamped.transform.translation.z = position.z();
  transform_stamped.transform.rotation = quat_msg;
  broadcaster_.sendTransform(transform_stamped);

  corrent_pose_stamped_.header.stamp = stamp;
  corrent_pose_stamped_.pose.position.x = position.x();
  corrent_pose_stamped_.pose.position.y = position.y();
  corrent_pose_stamped_.pose.position.z = position.z();
  corrent_pose_stamped_.pose.orientation = quat_msg;
  pose_pub_->publish(corrent_pose_stamped_);

  nav_msgs::msg::Odometry odom;
  odom.header = transform_stamped.header;
  odom.pose.pose = corrent_pose_stamped_.pose;
  odom.twist = odom_que_[odom_ptr_front_].twist;
  odom_pub_->publish(odom);

  path_.poses.push_back(corrent_pose_stamped_);
  path_pub_->publish(path_);

  trans_ = (position - previous_position_).norm();
  if (trans_ >= trans_for_mapupdate_ && !mapping_flag_) {
    geometry_msgs::msg::PoseStamped corrent_pose_stamped;
    corrent_pose_stamped = corrent_pose_stamped_;
    previous_position_ = position;
    mapping_task_ =
      std::packaged_task<void()>(
      std::bind(
        &ScanMatcherComponent::updateMap, this, cloud_ptr,
        final_transformation, corrent_pose_stamped));
    mapping_future_ = mapping_task_.get_future();
    mapping_thread_ = std::thread(std::move(std::ref(mapping_task_)));
    mapping_flag_ = true;
  }
}

void ScanMatcherComponent::updateMap(
  const pcl::PointCloud<PointType>::ConstPtr cloud_ptr,
  const Eigen::Matrix4f final_transformation,
  const geometry_msgs::msg::PoseStamped corrent_pose_stamped)
{
  pcl::PointCloud<PointType>::Ptr filtered_cloud_ptr(new pcl::PointCloud<PointType>());
  pcl::VoxelGrid<PointType> voxel_grid;
  voxel_grid.setLeafSize(vg_size_for_map_, vg_size_for_map_, vg_size_for_map_);
  voxel_grid.setInputCloud(cloud_ptr);
  voxel_grid.filter(*filtered_cloud_ptr);

  pcl::PointCloud<PointType>::Ptr transformed_cloud_ptr(new pcl::PointCloud<PointType>());
  pcl::transformPointCloud(*filtered_cloud_ptr, *transformed_cloud_ptr, final_transformation);

  targeted_cloud_.clear();
  targeted_cloud_ += *transformed_cloud_ptr;
  int num_submaps = map_array_msg_.submaps.size();
  for (int i = 0; i < num_targeted_cloud_ - 1; i++) {
    if (num_submaps - 1 - i < 0) {continue;}
    pcl::PointCloud<PointType>::Ptr tmp_ptr(new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(map_array_msg_.submaps[num_submaps - 1 - i].cloud, *tmp_ptr);
    pcl::PointCloud<PointType>::Ptr transformed_tmp_ptr(new pcl::PointCloud<PointType>());
    Eigen::Affine3d submap_affine;
    tf2::fromMsg(map_array_msg_.submaps[num_submaps - 1 - i].pose, submap_affine);
    pcl::transformPointCloud(*tmp_ptr, *transformed_tmp_ptr, submap_affine.matrix());
    targeted_cloud_ += *transformed_tmp_ptr;
  }

  /* map array */
  sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_ptr(
    new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*filtered_cloud_ptr, *cloud_msg_ptr);

  lidarslam_msgs::msg::SubMap submap;
  submap.header.frame_id = global_frame_id_;
  submap.header.stamp = corrent_pose_stamped.header.stamp;
  latest_distance_ += trans_;
  submap.distance = latest_distance_;
  submap.pose = corrent_pose_stamped.pose;
  submap.cloud = *cloud_msg_ptr;
  submap.cloud.header.frame_id = global_frame_id_;
  map_array_msg_.header.stamp = corrent_pose_stamped.header.stamp;
  map_array_msg_.submaps.push_back(submap);
  map_array_pub_->publish(map_array_msg_);

  is_map_updated_ = true;

  rclcpp::Time map_time = clock_.now();
  double dt = map_time.seconds() - last_map_time_.seconds();
  if (dt > map_publish_period_) {
    publishMap(map_array_msg_, global_frame_id_);
    last_map_time_ = map_time;
  }
}

Eigen::Matrix4f ScanMatcherComponent::getTransformation(const geometry_msgs::msg::Pose pose)
{
  Eigen::Affine3d affine;
  tf2::fromMsg(pose, affine);
  Eigen::Matrix4f sim_trans = affine.matrix().cast<float>();
  return sim_trans;
}

void ScanMatcherComponent::receiveOdom(const nav_msgs::msg::Odometry odom_msg)
{
  if (!use_imu_) {return;}
  // RCLCPP_WARN(get_logger(), "The odom was received.");
  odom_ptr_last_ = (odom_ptr_last_ + 1) % odom_que_length_;
  odom_que_[odom_ptr_last_] = odom_msg;
  if ((odom_ptr_last_ + 1) % odom_que_length_ == odom_ptr_front_) {
    odom_ptr_front_ = (odom_ptr_front_ + 1) % odom_que_length_;
  }
}

void ScanMatcherComponent::publishMap(const lidarslam_msgs::msg::MapArray & map_array_msg , const std::string & map_frame_id)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  for (auto & submap : map_array_msg.submaps) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr submap_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_submap_cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(submap.cloud, *submap_cloud_ptr);

    Eigen::Affine3d affine;
    tf2::fromMsg(submap.pose, affine);
    pcl::transformPointCloud(
      *submap_cloud_ptr, *transformed_submap_cloud_ptr,
      affine.matrix().cast<float>());

    *map_ptr += *transformed_submap_cloud_ptr;
  }
  RCLCPP_INFO(get_logger(), "publish a map, number of points in the map : %ld", map_ptr->size());

  sensor_msgs::msg::PointCloud2::SharedPtr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*map_ptr, *map_msg_ptr);
  map_msg_ptr->header.frame_id = map_frame_id;
  map_pub_->publish(*map_msg_ptr);
}

} // namespace graphslam

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(graphslam::ScanMatcherComponent)
