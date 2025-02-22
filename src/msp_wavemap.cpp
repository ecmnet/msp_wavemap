
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

#include <wavemap/core/utils/math/approximate_trigonometry.h>

#include <opencv2/core/eigen.hpp>
#include <wavemap/pipeline/pipeline.h>
#include <wavemap/core/utils/query/map_interpolator.h>
#include <wavemap/core/utils/iterate/grid_iterator.h>

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

  pcl_publisher.setMap(occupancy_map_);
  msp_publisher.setMap(occupancy_map_);

  // Publish Camera Tranform
  publish_camera_transform();

  RCLCPP_INFO(this->get_logger(), "MSP WaveMap Node started");
}

void MSWaveMapNode::onDepthReceived(const gz::msgs::Image &msg)
{

  if (this->arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED)
    return;

  if (++count % 3 != 0 || occupancy_map_ == nullptr)
    return;

  const auto T_W_C =
      transformer_->lookupLatestTransform("world", "camera_link");

  if (T_W_C == std::nullopt)
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
  // wave_rider_.updateMap(occupancy_map_);
  auto t3 = std::chrono::high_resolution_clock::now();

  // std::cout << std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1000000 << "ms\n";
  // std::cout << std::chrono::duration_cast<std::chrono::nanoseconds>(t3 - t2).count() / 1000000 << "ms\n";
  // const size_t map_size_KB = occupancy_map_->getMemoryUsage() / 1024;
  // std::cout << "Created map of size: " << map_size_KB << " KB" << std::endl;
}

void MSWaveMapNode::onTrajectoryCheck(const std::shared_ptr<msp_msgs::srv::TrajectoryCheck::Request> request,
                                      std::shared_ptr<msp_msgs::srv::TrajectoryCheck::Response> response)
{
  //std::cout << "Checking trajectory" << std::endl;
  for (int i = 0; i < request->plan.count; i++)
  {
    auto item = msp::ros2::convert::fromTrajectoryPlanItemMessage(request->plan.segments[i]);
    response->reply.status = checkPlanItem(item);
    if (response->reply.status != msp_msgs::msg::TrajectoryCheckAck::STATUS_NO_COLLISION)
      break;
  }
  path_publisher.publish();
}

uint8_t MSWaveMapNode::checkPlanItem(msp::PlanItem item)
{

  StateTriplet s0;
  
  const FloatingPoint query_min_cell_width = 1.0f; // in meters

  //std::cout << "Checking item: \n" << item << std::endl;
  planner_.generate(&item);
  const double time_slot = 1 / ( (item.max_velocity > 0 ? item.max_velocity : 2.0f) * 10 ); // note: max_velocity in some cases 0
  for (double t = time_slot; t < item.estimated_time_s; t += time_slot)
  {
    planner_.getSetpointAt(t, s0);

    // Transform from ned to world
    const Point3D query_point(s0.pos.x, -s0.pos.y, -s0.pos.z);
    path_publisher.push(query_point);
    if (auto *hashed_wavelet_octree =
            dynamic_cast<HashedWaveletOctree *>(occupancy_map_.get());
        hashed_wavelet_octree)
    {
      const FloatingPoint map_min_cell_width = hashed_wavelet_octree->getMinCellWidth();
      const IndexElement query_height = convert::cellWidthToHeight(query_min_cell_width, 1.f / map_min_cell_width);
      const OctreeIndex query_index = convert::pointToNodeIndex(query_point, map_min_cell_width, query_height);
      const FloatingPoint occupancy_log_odds = hashed_wavelet_octree->getCellValue(query_index);
      //std::cout << occupancy_log_odds << std::endl;
      if (occupancy_log_odds > 0.5)
      {
        std::cout << "Collision at " << s0.pos << std::endl;
        return msp_msgs::msg::TrajectoryCheckAck::STATUS_EMERGENCY_STOP;
      }
    }
  }
  return msp_msgs::msg::TrajectoryCheckAck::STATUS_NO_COLLISION;
}

int main(int argc, char *argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<msp::MSWaveMapNode>());
  rclcpp::shutdown();
  return 0;
}