/*
 * Copyright 2018 Simbe Robotics, Inc.
 * Author: Steve Macenski (stevenmacenski@gmail.com)
 */

#include <unordered_map>
#include <string>
#include <utility>
#include <thread>
#include <stdexcept>
#include "robot_slam_solver/ceres_solver.hpp"

namespace robot_slam_solver
{

/*****************************************************************************/
ConfigurableCeresSolver::ConfigurableCeresSolver()
: nodes_(new std::unordered_map<int, Eigen::Vector3d>()),
  blocks_(new std::unordered_map<std::size_t,
    ceres::ResidualBlockId>()),
  problem_(NULL), was_constant_set_(false)
/*****************************************************************************/
{
}

/*****************************************************************************/
void ConfigurableCeresSolver::Configure(rclcpp_lifecycle::LifecycleNode::SharedPtr node)
/*****************************************************************************/
{
  logger_ = node->get_logger();

  std::string solver_type, preconditioner_type, dogleg_type,
    trust_strategy, loss_fn, mode;
  if (!node->has_parameter("ceres_linear_solver")) {
    node->declare_parameter(
      "ceres_linear_solver",
      rclcpp::ParameterValue(std::string("SPARSE_NORMAL_CHOLESKY")));
  }
  solver_type = node->get_parameter("ceres_linear_solver").as_string();

  if (!node->has_parameter("ceres_preconditioner")) {
    node->declare_parameter(
      "ceres_preconditioner",
      rclcpp::ParameterValue(std::string("JACOBI")));
  }
  preconditioner_type = node->get_parameter("ceres_preconditioner").as_string();

  if (!node->has_parameter("ceres_dogleg_type")) {
    node->declare_parameter(
      "ceres_dogleg_type",
      rclcpp::ParameterValue(std::string("TRADITIONAL_DOGLEG")));
  }
  dogleg_type = node->get_parameter("ceres_dogleg_type").as_string();

  if (!node->has_parameter("ceres_trust_strategy")) {
    node->declare_parameter(
      "ceres_trust_strategy",
      rclcpp::ParameterValue(std::string("LM")));
  }
  trust_strategy = node->get_parameter("ceres_trust_strategy").as_string();

  if (!node->has_parameter("ceres_loss_function")) {
    node->declare_parameter(
      "ceres_loss_function",
      rclcpp::ParameterValue(std::string("None")));
  }
  loss_fn = node->get_parameter("ceres_loss_function").as_string();

  if (!node->has_parameter("mode")) {
    node->declare_parameter(
      "mode",
      rclcpp::ParameterValue(std::string("mapping")));
  }
  mode = node->get_parameter("mode").as_string();

  debug_logging_ = node->get_parameter("debug_logging").as_bool();

  corrections_.clear();
  first_node_ = nodes_->end();

  // formulate problem
  angle_manifold_ = AngleManifold::Create();

  // choose loss function default squared loss (NULL)
  loss_function_ = NULL;
  if (loss_fn == "HuberLoss") {
    RCLCPP_INFO(
      node->get_logger(),
      "ConfigurableCeresSolver: Using HuberLoss loss function.");
    loss_function_ = new ceres::HuberLoss(0.7);
  } else if (loss_fn == "CauchyLoss") {
    RCLCPP_INFO(
      node->get_logger(),
      "ConfigurableCeresSolver: Using CauchyLoss loss function.");
    loss_function_ = new ceres::CauchyLoss(0.7);
  }

  // choose linear solver default CHOL
  options_.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  if (solver_type == "SPARSE_SCHUR") {
    RCLCPP_INFO(
      node->get_logger(),
      "ConfigurableCeresSolver: Using SPARSE_SCHUR solver.");
    options_.linear_solver_type = ceres::SPARSE_SCHUR;
  } else if (solver_type == "ITERATIVE_SCHUR") {
    RCLCPP_INFO(
      node->get_logger(),
      "ConfigurableCeresSolver: Using ITERATIVE_SCHUR solver.");
    options_.linear_solver_type = ceres::ITERATIVE_SCHUR;
  } else if (solver_type == "CGNR") {
    RCLCPP_INFO(
      node->get_logger(),
      "ConfigurableCeresSolver: Using CGNR solver.");
    options_.linear_solver_type = ceres::CGNR;
  }

  // choose preconditioner default Jacobi
  options_.preconditioner_type = ceres::JACOBI;
  if (preconditioner_type == "IDENTITY") {
    RCLCPP_INFO(
      node->get_logger(),
      "ConfigurableCeresSolver: Using IDENTITY preconditioner.");
    options_.preconditioner_type = ceres::IDENTITY;
  } else if (preconditioner_type == "SCHUR_JACOBI") {
    RCLCPP_INFO(
      node->get_logger(),
      "ConfigurableCeresSolver: Using SCHUR_JACOBI preconditioner.");
    options_.preconditioner_type = ceres::SCHUR_JACOBI;
  }

  if (options_.preconditioner_type == ceres::CLUSTER_JACOBI ||
    options_.preconditioner_type == ceres::CLUSTER_TRIDIAGONAL)
  {
    // default canonical view is O(n^2) which is unacceptable for
    // problems of this size
    options_.visibility_clustering_type = ceres::SINGLE_LINKAGE;
  }

  // choose trust region strategy default LM
  options_.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  if (trust_strategy == "DOGLEG") {
    RCLCPP_INFO(
      node->get_logger(),
      "ConfigurableCeresSolver: Using DOGLEG trust region strategy.");
    options_.trust_region_strategy_type = ceres::DOGLEG;
  }

  // choose dogleg type default traditional
  if (options_.trust_region_strategy_type == ceres::DOGLEG) {
    options_.dogleg_type = ceres::TRADITIONAL_DOGLEG;
    if (dogleg_type == "SUBSPACE_DOGLEG") {
      RCLCPP_INFO(
        node->get_logger(),
        "ConfigurableCeresSolver: Using SUBSPACE_DOGLEG dogleg type.");
      options_.dogleg_type = ceres::SUBSPACE_DOGLEG;
    }
  }

  // a typical ros map is 5cm, this is 0.001, 50x the resolution
  options_.function_tolerance = 1e-3;
  options_.gradient_tolerance = 1e-6;
  options_.parameter_tolerance = 1e-3;

  options_.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
  options_.max_num_consecutive_invalid_steps = 3;
  options_.max_consecutive_nonmonotonic_steps =
    options_.max_num_consecutive_invalid_steps;
  if (!node->has_parameter("ceres_num_threads")) {
    node->declare_parameter("ceres_num_threads", rclcpp::ParameterValue(int64_t{12}));
  }
  const int64_t requested_threads =
    node->get_parameter("ceres_num_threads").as_int();
  const unsigned int detected_threads = std::thread::hardware_concurrency();
  const int64_t available_threads =
    detected_threads == 0 ? 1 : static_cast<int64_t>(detected_threads);
  if (requested_threads < 1) {
    throw std::invalid_argument(
            "ceres_num_threads must be at least 1, got " +
            std::to_string(requested_threads));
  }
  int64_t configured_threads = requested_threads;
  if (requested_threads > available_threads) {
    configured_threads = available_threads;
    RCLCPP_WARN(
      node->get_logger(),
      "ConfigurableCeresSolver: requested ceres_num_threads=%ld exceeds "
      "hardware concurrency=%ld; using %ld.",
      requested_threads, available_threads, configured_threads);
    const auto result = node->set_parameter(
      rclcpp::Parameter("ceres_num_threads", configured_threads));
    if (!result.successful) {
      throw std::invalid_argument(
              "failed to publish effective ceres_num_threads: " +
              result.reason);
    }
  }
  options_.num_threads = static_cast<int>(configured_threads);
  RCLCPP_INFO(
    node->get_logger(),
    "ConfigurableCeresSolver: ceres_num_threads=%d (available=%ld)",
    options_.num_threads, available_threads);
  options_.use_nonmonotonic_steps = true;
  options_.jacobi_scaling = true;

  options_.min_relative_decrease = 1e-3;

  options_.initial_trust_region_radius = 1e4;
  options_.max_trust_region_radius = 1e8;
  options_.min_trust_region_radius = 1e-16;

  options_.min_lm_diagonal = 1e-6;
  options_.max_lm_diagonal = 1e32;

  if (options_.linear_solver_type == ceres::SPARSE_NORMAL_CHOLESKY) {
    options_.dynamic_sparsity = true;
  }

  if (mode == std::string("localization")) {
    // doubles the memory footprint, but lets us remove contraints faster
    options_problem_.enable_fast_removal = true;
  }

  // we do not want the problem definition to own these objects, otherwise they get
  // deleted along with the problem
  options_problem_.loss_function_ownership = ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;

  problem_ = new ceres::Problem(options_problem_);
}

/*****************************************************************************/
ConfigurableCeresSolver::~ConfigurableCeresSolver()
/*****************************************************************************/
{
  if (loss_function_ != NULL) {
    delete loss_function_;
  }
  if (nodes_ != NULL) {
    delete nodes_;
  }
  if (blocks_ != NULL) {
    delete blocks_;
  }
  if (problem_ != NULL) {
    delete problem_;
  }
}

/*****************************************************************************/
void ConfigurableCeresSolver::Compute()
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(nodes_mutex_);

  if (nodes_->size() == 0) {
    RCLCPP_WARN(
      logger_,
      "ConfigurableCeresSolver: Ceres was called when there are no nodes."
      " This shouldn't happen.");
    return;
  }

  // populate contraint for static initial pose
  if (!was_constant_set_ && first_node_ != nodes_->end() &&
    problem_->HasParameterBlock(&first_node_->second(0)) &&
    problem_->HasParameterBlock(&first_node_->second(1)) &&
    problem_->HasParameterBlock(&first_node_->second(2)))
  {
    RCLCPP_DEBUG(
      logger_,
      "ConfigurableCeresSolver: Setting first node as a constant pose:"
      "%0.2f, %0.2f, %0.2f.", first_node_->second(0),
      first_node_->second(1), first_node_->second(2));
    problem_->SetParameterBlockConstant(&first_node_->second(0));
    problem_->SetParameterBlockConstant(&first_node_->second(1));
    problem_->SetParameterBlockConstant(&first_node_->second(2));
    was_constant_set_ = !was_constant_set_;
  }

  ceres::Solver::Summary summary;
  ceres::Solve(options_, problem_, &summary);
  if (debug_logging_) {
    std::cout << summary.FullReport() << '\n';
  }

  if (!summary.IsSolutionUsable()) {
    RCLCPP_WARN(
      logger_, "ConfigurableCeresSolver: "
      "Ceres could not find a usable solution to optimize.");
    return;
  }

  // store corrected poses
  if (!corrections_.empty()) {
    corrections_.clear();
  }
  corrections_.reserve(nodes_->size());
  karto::Pose2 pose;
  ConstGraphIterator iter = nodes_->begin();
  for (iter; iter != nodes_->end(); ++iter) {
    pose.SetX(iter->second(0));
    pose.SetY(iter->second(1));
    pose.SetHeading(iter->second(2));
    corrections_.push_back(std::make_pair(iter->first, pose));
  }
}

/*****************************************************************************/
const karto::ScanSolver::IdPoseVector & ConfigurableCeresSolver::GetCorrections() const
/*****************************************************************************/
{
  return corrections_;
}

/*****************************************************************************/
void ConfigurableCeresSolver::Clear()
/*****************************************************************************/
{
  corrections_.clear();
}

/*****************************************************************************/
void ConfigurableCeresSolver::Reset()
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(nodes_mutex_);

  corrections_.clear();
  was_constant_set_ = false;

  if (problem_) {
    // Note that this also frees anything the problem owns (i.e. local parameterization, cost
    // function)
    delete problem_;
  }

  if (nodes_) {
    delete nodes_;
  }

  if (blocks_) {
    delete blocks_;
  }

  nodes_ = new std::unordered_map<int, Eigen::Vector3d>();
  blocks_ = new std::unordered_map<std::size_t, ceres::ResidualBlockId>();
  problem_ = new ceres::Problem(options_problem_);
  first_node_ = nodes_->end();

  angle_manifold_ = AngleManifold::Create();
}

/*****************************************************************************/
void ConfigurableCeresSolver::AddNode(karto::Vertex<karto::LocalizedRangeScan> * pVertex)
/*****************************************************************************/
{
  // store nodes
  if (!pVertex) {
    return;
  }

  karto::Pose2 pose = pVertex->GetObject()->GetCorrectedPose();
  Eigen::Vector3d pose2d(pose.GetX(), pose.GetY(), pose.GetHeading());

  const int id = pVertex->GetObject()->GetUniqueId();

  boost::mutex::scoped_lock lock(nodes_mutex_);
  nodes_->insert(std::pair<int, Eigen::Vector3d>(id, pose2d));

  if (nodes_->size() == 1) {
    first_node_ = nodes_->find(id);
  }
}

/*****************************************************************************/
void ConfigurableCeresSolver::AddConstraint(karto::Edge<karto::LocalizedRangeScan> * pEdge)
/*****************************************************************************/
{
  // get IDs in graph for this edge
  boost::mutex::scoped_lock lock(nodes_mutex_);

  if (!pEdge) {
    return;
  }

  const int node1 = pEdge->GetSource()->GetObject()->GetUniqueId();
  GraphIterator node1it = nodes_->find(node1);
  const int node2 = pEdge->GetTarget()->GetObject()->GetUniqueId();
  GraphIterator node2it = nodes_->find(node2);

  if (node1it == nodes_->end() ||
    node2it == nodes_->end() || node1it == node2it)
  {
    RCLCPP_WARN(
      logger_,
      "ConfigurableCeresSolver: Failed to add constraint, could not find nodes.");
    return;
  }

  // extract transformation
  karto::LinkInfo * pLinkInfo = (karto::LinkInfo *)(pEdge->GetLabel());
  karto::Pose2 diff = pLinkInfo->GetPoseDifference();
  Eigen::Vector3d pose2d(diff.GetX(), diff.GetY(), diff.GetHeading());

  karto::Matrix3 precisionMatrix = pLinkInfo->GetCovariance().Inverse();
  Eigen::Matrix3d information;
  information(0, 0) = precisionMatrix(0, 0);
  information(0, 1) = information(1, 0) = precisionMatrix(0, 1);
  information(0, 2) = information(2, 0) = precisionMatrix(0, 2);
  information(1, 1) = precisionMatrix(1, 1);
  information(1, 2) = information(2, 1) = precisionMatrix(1, 2);
  information(2, 2) = precisionMatrix(2, 2);
  Eigen::Matrix3d sqrt_information = information.llt().matrixU();

  // populate residual and parameterization for heading normalization
  ceres::CostFunction * cost_function = PoseGraph2dErrorTerm::Create(pose2d(0),
      pose2d(1), pose2d(2), sqrt_information);
  ceres::ResidualBlockId block = problem_->AddResidualBlock(
    cost_function, loss_function_,
    &node1it->second(0), &node1it->second(1), &node1it->second(2),
    &node2it->second(0), &node2it->second(1), &node2it->second(2));
  problem_->SetManifold(&node1it->second(2),
    angle_manifold_);
  problem_->SetManifold(&node2it->second(2),
    angle_manifold_);

  blocks_->insert(std::pair<std::size_t, ceres::ResidualBlockId>(
      GetHash(node1, node2), block));
}

/*****************************************************************************/
void ConfigurableCeresSolver::RemoveNode(kt_int32s id)
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(nodes_mutex_);
  GraphIterator nodeit = nodes_->find(id);
  if (nodeit != nodes_->end()) {
    if (problem_->HasParameterBlock(&nodeit->second(0)) &&
      problem_->HasParameterBlock(&nodeit->second(1)) &&
      problem_->HasParameterBlock(&nodeit->second(2)))
    {
      problem_->RemoveParameterBlock(&nodeit->second(0));
      problem_->RemoveParameterBlock(&nodeit->second(1));
      problem_->RemoveParameterBlock(&nodeit->second(2));
      RCLCPP_DEBUG(
        logger_,
        "RemoveNode: Removed node id %d", nodeit->first);
    } else {
      RCLCPP_DEBUG(
        logger_,
        "RemoveNode: Missing parameter blocks for "
        "node id %d", nodeit->first);
    }
    nodes_->erase(nodeit);
  } else {
    RCLCPP_ERROR(
      logger_, "RemoveNode: Failed to find node matching id %i",
      (int)id);
  }
}

/*****************************************************************************/
void ConfigurableCeresSolver::RemoveConstraint(kt_int32s sourceId, kt_int32s targetId)
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(nodes_mutex_);
  std::unordered_map<std::size_t, ceres::ResidualBlockId>::iterator it_a =
    blocks_->find(GetHash(sourceId, targetId));
  std::unordered_map<std::size_t, ceres::ResidualBlockId>::iterator it_b =
    blocks_->find(GetHash(targetId, sourceId));
  if (it_a != blocks_->end()) {
    problem_->RemoveResidualBlock(it_a->second);
    blocks_->erase(it_a);
  } else if (it_b != blocks_->end()) {
    problem_->RemoveResidualBlock(it_b->second);
    blocks_->erase(it_b);
  } else {
    RCLCPP_ERROR(
      logger_,
      "RemoveConstraint: Failed to find residual block for %i %i",
      (int)sourceId, (int)targetId);
  }
}

/*****************************************************************************/
void ConfigurableCeresSolver::ModifyNode(const int & unique_id, Eigen::Vector3d pose)
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(nodes_mutex_);
  GraphIterator it = nodes_->find(unique_id);
  if (it != nodes_->end()) {
    double yaw_init = it->second(2);
    it->second = pose;
    it->second(2) += yaw_init;
  }
}

/*****************************************************************************/
void ConfigurableCeresSolver::GetNodeOrientation(const int & unique_id, double & pose)
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(nodes_mutex_);
  GraphIterator it = nodes_->find(unique_id);
  if (it != nodes_->end()) {
    pose = it->second(2);
  }
}

/*****************************************************************************/
std::unordered_map<int, Eigen::Vector3d> * ConfigurableCeresSolver::getGraph()
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(nodes_mutex_);
  return nodes_;
}

}  // namespace robot_slam_solver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(robot_slam_solver::ConfigurableCeresSolver, karto::ScanSolver)
