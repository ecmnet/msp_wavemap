
#include <msp_wavemap/msp_wavemap.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>

#include <wavemap/core/indexing/index_conversions.h>
#include <wavemap/core/utils/query/probability_conversions.h>
#include <wavemap/core/map/hashed_chunked_wavelet_octree.h>
#include <wavemap/core/map/map_base.h>
#include <wavemap/core/map/map_factory.h>
#include <wavemap/pipeline/map_operations/map_operation_factory.h>
#include <msp_wavemap/lib/map_operations/msp_map_operation_factory.h>
#include <wavemap/core/utils/query/map_interpolator.h>

#include <opencv2/core/eigen.hpp>
#include <wavemap/pipeline/pipeline.h>
#include <wavemap/core/utils/query/map_interpolator.h>
#include <wavemap/core/utils/iterate/grid_iterator.h>
#include <wavemap/core/utils/query/query_accelerator.h>
#include <msp_msgs/msg/trajectory.hpp>

#include <msp_wavemap/lib/config/stream_conversions.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <sstream>

#define ROI_X___BORDER 20
#define ROI_TOP_BORDER 20
#define ROI_BOT_BORDER 100

using namespace msp;
using namespace wavemap;

void MSWaveMapNode::initialize()
{

  gz_node->Subscribe("/depth_camera", &MSWaveMapNode::onDepthReceived, this);

  // trajectory_publisher_ = this->create_publisher<msp_msgs::msg::Trajectory>("msp/in/trajectory", this->getQos());
  obstacle_publisher_ = this->create_publisher<px4_msgs::msg::ObstacleDistance>("msp/in/obstacle_distance", this->getQos());
  scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  std::string base_path = ament_index_cpp::get_package_share_directory("msp_wavemap");
  std::string config_name = base_path + "/config/pipeline.yaml";

  std::ifstream file(config_name);
  if (!file.is_open())
  {
    RCLCPP_ERROR(this->get_logger(), "Could not open pipeline file: %s", config_name.c_str());
    return;
  }

  std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  std::istringstream iss(content);

  const auto params = wavemap::io::yamlStreamToParams(iss);
  CHECK(params.has_value());

  // setup the map
  const auto map_config = params->getChild("map");
  CHECK(map_config.has_value());
  occupancy_map_ = MapFactory::create(map_config.value());
  CHECK_NOTNULL(occupancy_map_);

  // Setup thread pool
  RCLCPP_INFO(this->get_logger(), "Creating thread pool with 5 threads.");
  thread_pool_ = std::make_shared<ThreadPool>(5);
  CHECK_NOTNULL(thread_pool_);

  // Setup the pipeline
  pipeline_ = std::make_shared<Pipeline>(occupancy_map_, thread_pool_);
  CHECK_NOTNULL(pipeline_);

  // Add map operations to pipeline
  const auto map_operations =
      params->getChildAs<param::Array>("map_operations");
  CHECK(map_operations);

  for (const auto &operation_params : map_operations.value())
  {
    // pipeline_->addOperation(operation_params);

    const auto type_name = param::getTypeStr(operation_params);


    if (const auto type = MapMSPOperationType{type_name.value()};
        type.isValid())
    {
      auto operation = MapMSPOperationFactory::create(
          type, operation_params, occupancy_map_, &pos_, thread_pool_);
      pipeline_->addOperation(std::move(operation));
    }

    if (const auto type = MapOperationType{type_name.value()}; type.isValid())
    {
      auto operation =
          MapOperationFactory::create(type, operation_params, occupancy_map_);
      pipeline_->addOperation(std::move(operation));
    }
  }

  // Add measurement integrators to pipeline
  const auto measurement_integrators =
      params->getChildAs<param::Map>("measurement_integrators");
  CHECK(measurement_integrators);
  for (const auto &[integrator_name, integrator_params] :
       measurement_integrators.value())
  {
    pipeline_->addIntegrator(integrator_name, integrator_params);
  }

  map_publisher.setMap(occupancy_map_);

   // Publish Camera Tranform
   publish_camera_transform();

  RCLCPP_INFO(this->get_logger(), "MSP WaveMap Node started");
}

void MSWaveMapNode::onDepthReceived(const gz::msgs::Image &msg)
{
  if (++count % 3 != 0 || occupancy_map_ == nullptr)
    return;

  const auto T_W_C =
      transformer_->lookupLatestTransform("world", "camera_link");

  if(T_W_C == std::nullopt)
    return;

  pos_ = T_W_C->getPosition();

  cv::Mat floatImg = cv::Mat(msg.height(), msg.width(), CV_32FC1, (void *)msg.data().data());
  // cv::Rect roi(ROI_X___BORDER, ROI_TOP_BORDER, floatImg.cols-ROI_X___BORDER, floatImg.rows - ROI_BOT_BORDER);
  cv::transpose(floatImg, floatImg);
  PosedImage<> depth_image(floatImg.rows, floatImg.cols);
  cv::cv2eigen<FloatingPoint>(floatImg, depth_image.getData());
  depth_image.setPose(*T_W_C);

  auto t1 = std::chrono::high_resolution_clock::now();

  pipeline_->runPipeline({"gazebo_short", "gazebo_long"}, depth_image);
  auto t2 = std::chrono::high_resolution_clock::now();
 // checkCollision(T_W_C->getPosition(), 7.0f, 72);

  auto t3 = std::chrono::high_resolution_clock::now();
  std::cout << std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1000 << "us\n";
  std::cout << std::chrono::duration_cast<std::chrono::nanoseconds>(t3 - t2).count() / 1000 << "us\n";
  const size_t map_size_KB = occupancy_map_->getMemoryUsage() / 1024;
  std::cout << "Created map of size: " << map_size_KB << " KB" << std::endl;
}

void MSWaveMapNode::onTrajectoryCheck(const std::shared_ptr<msp_msgs::srv::TrajectoryCheck::Request> request,
                                      std::shared_ptr<msp_msgs::srv::TrajectoryCheck::Response> response)
{
  response->reply.status = msp_msgs::msg::TrajectoryCheckAck::STATUS_NO_COLLISION;
}

bool MSWaveMapNode::checkCollision(Eigen::Vector3f position, const float radius, const uint8_t points)
{

  // if (pos_.z() < 1.0f)
  //   return false;

  const float min_cell_width = occupancy_map_->getMinCellWidth();
  const Point3D radius_vec(radius, radius, min_cell_width);

  const Point3D lower_bound(position - radius_vec);
  const Point3D upper_bound(position + radius_vec);

  Point3D lower_bound_b;

  const auto T_W_C =
      transformer_->lookupLatestTransform("world", "camera_link");
  

  const float inv_min_cell_width = 1.0f / min_cell_width;

  const Index3D min_corner_index = convert::pointToFloorIndex(lower_bound, inv_min_cell_width);
  const Index3D max_corner_index = convert::pointToCeilIndex(upper_bound, inv_min_cell_width);

  std::vector<Point3D> occupied_cells;

  const auto *hashed_map = dynamic_cast<const HashedChunkedWaveletOctree *>(occupancy_map_.get());
  CHECK_NOTNULL(hashed_map);

  QueryAccelerator<HashedChunkedWaveletOctree> query_accelerator(*hashed_map);

  for (const auto &query_index : Grid<3>(min_corner_index, max_corner_index))
  {

    auto value = query_accelerator.getCellValue(query_index);
    const float probability = convert::logOddsToProbability(value);
    if (probability > 0.7f)
    {
      auto cell_location = convert::indexToCenterPoint(query_index, min_cell_width);
      if ((cell_location - position).norm() < radius)
      {
        occupied_cells.push_back(cell_location);
      }
    }
  }

  // if (occupied_cells.size() == 0)
  // {
  //   in_collision = false;
  //   return false;
  // }

  const float PIP = points / M_PI;

  std::vector<float> distance_bins(points, std::numeric_limits<float>::max());

  const float ref_angle = std::atan2(position.y(), position.x());

  for (auto point : occupied_cells)
  {

    float delta_x = point.x() - position.x();
    float delta_y = point.y() - position.y();
    float xy_angle = std::atan2(delta_y, delta_x) - ref_angle;

    if ((xy_angle < -M_PI_2) || xy_angle > M_PI_2)
      continue;

    float distance = (point - position).norm();
    const int index = static_cast<int>((xy_angle)*PIP);
    if (index < 0 || index > 71)
      continue;

    if (distance < distance_bins[index])
      distance_bins[index] = distance;
  }

  publishScan(distance_bins, position);

  // if (!in_collision && occupied_cells.size() > 1)
  // {
  //   this->log_message("[msp] Emergency breaking.", MAV_SEVERITY_ALERT);
  //   in_collision = true;
  //   return true;
  // }

  return false;
}

void MSWaveMapNode::publishScan(std::vector<float> scans, Eigen::Vector3f position)
{

  const float PI2 = M_PI / 2;

  auto scan_msg = sensor_msgs::msg::LaserScan();
  scan_msg.header.stamp = this->get_clock()->now();
  scan_msg.header.frame_id = "base_line"; // Set the correct TF frame

  // Laser properties
  scan_msg.angle_min = -M_PI_2;
  scan_msg.angle_max =  M_PI_2;
  scan_msg.angle_increment = M_PI / 72;
  scan_msg.time_increment = 0.0;
  scan_msg.scan_time = 0.1;
  scan_msg.range_min = 0.1;
  scan_msg.range_max = 7.0;

  scan_msg.ranges.resize(72, 0.0);
  scan_msg.intensities.resize(72, 0.0);

  // Generate fake distance readings
  for (int i = 0; i < 72; i++)
  {
    if (scans[i] > 7.0)
      continue;
    scan_msg.intensities[i] = 1.0f;
    scan_msg.ranges[i] = scans[i];
  }

  // Publish the scan message
  scan_publisher_->publish(scan_msg);
}

int main(int argc, char *argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<msp::MSWaveMapNode>());
  rclcpp::shutdown();
  return 0;
}