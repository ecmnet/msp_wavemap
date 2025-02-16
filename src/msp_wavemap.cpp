
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
  obstacle_publisher_ = this->create_publisher<px4_msgs::msg::ObstacleDistance>("msp/in/obstacle_distance", this->getQos());
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

  std::cout << std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1000000 << "ms\n";
  std::cout << std::chrono::duration_cast<std::chrono::nanoseconds>(t3 - t2).count() / 1000000 << "ms\n";
  const size_t map_size_KB = occupancy_map_->getMemoryUsage() / 1024;
  std::cout << "Created map of size: " << map_size_KB << " KB" << std::endl;
  

}

void MSWaveMapNode::onTrajectoryCheck(const std::shared_ptr<msp_msgs::srv::TrajectoryCheck::Request> request,
                                      std::shared_ptr<msp_msgs::srv::TrajectoryCheck::Response> response)
{
  response->reply.status = msp_msgs::msg::TrajectoryCheckAck::STATUS_NO_COLLISION;
}

int main(int argc, char *argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<msp::MSWaveMapNode>());
  rclcpp::shutdown();
  return 0;
}