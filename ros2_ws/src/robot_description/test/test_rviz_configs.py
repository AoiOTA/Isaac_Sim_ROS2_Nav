from pathlib import Path

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
RVIZ_ROOT = PACKAGE_ROOT / 'rviz'


def _config(name):
    return yaml.safe_load(
        (RVIZ_ROOT / name).read_text(encoding='utf-8'))


def _displays(config):
    result = []

    def visit(display):
        result.append(display)
        for child in display.get('Displays', []):
            visit(child)

    for display in config['Visualization Manager']['Displays']:
        visit(display)
    return result


def _named(config, name):
    return next(item for item in _displays(config) if item.get('Name') == name)


def _assert_topic(display, topic, *, reliability, durability):
    configured = display['Topic']
    assert configured['Value'] == topic
    assert configured['Reliability Policy'] == reliability
    assert configured['Durability Policy'] == durability


@pytest.mark.parametrize(
    'name', ['mapping.rviz', 'localization.rviz', 'navigation.rviz'])
def test_every_workflow_has_map_frame_robot_tf_and_sensor_qos(name):
    config = _config(name)
    manager = config['Visualization Manager']

    assert manager['Global Options']['Fixed Frame'] == 'map'
    classes = {item['Class'] for item in _displays(config)}
    assert 'rviz_default_plugins/RobotModel' in classes
    assert 'rviz_default_plugins/TF' in classes
    _assert_topic(
        _named(config, 'LaserScan'),
        '/scan',
        reliability='Best Effort',
        durability='Volatile',
    )
    _assert_topic(
        _named(config, 'Raw LiDAR PointCloud2'),
        '/lidar/points_raw',
        reliability='Best Effort',
        durability='Volatile',
    )
    assert _named(config, 'Raw LiDAR PointCloud2')['Enabled'] is False
    _assert_topic(
        _named(config, 'Robot Front Camera'),
        '/camera/front/image_raw',
        reliability='Best Effort',
        durability='Volatile',
    )
    assert _named(config, 'Robot Front Camera')['Topic']['Depth'] == 2
    assert _named(config, 'Robot Front Camera')['Enabled'] is (name != 'navigation.rviz')
    assert _named(config, 'Robot Front Camera')['Normalize Range'] is False
    assert _named(config, 'Robot Front Camera').get('Transport Hint', 'raw') == 'raw'
    window = config['Window Geometry']
    assert window['Robot Front Camera']['collapsed'] is (name == 'navigation.rviz')
    assert len(window['QMainWindow State']) > 100


def test_mapping_workflow_uses_live_map_and_no_navigation_goal_tool():
    config = _config('mapping.rviz')
    _assert_topic(
        _named(config, 'Mapping Map'),
        '/map',
        reliability='Reliable',
        durability='Transient Local',
    )
    panel_classes = {panel['Class'] for panel in config['Panels']}
    tool_classes = {
        tool['Class'] for tool in config['Visualization Manager']['Tools']}
    assert 'slam_toolbox::SlamToolboxPlugin' not in panel_classes
    assert 'nav2_rviz_plugins/Navigation 2' not in panel_classes
    assert 'nav2_rviz_plugins/GoalTool' not in tool_classes


def test_localization_workflow_has_static_and_diagnostic_maps_and_pose_tool():
    config = _config('localization.rviz')
    _assert_topic(
        _named(config, 'Static Map'),
        '/map',
        reliability='Reliable',
        durability='Transient Local',
    )
    diagnostic = _named(config, 'SLAM Toolbox Diagnostic Map')
    assert diagnostic['Topic']['Value'] == '/slam_toolbox/map'
    assert diagnostic['Enabled'] is False
    pose_tool = next(
        tool for tool in config['Visualization Manager']['Tools']
        if tool['Class'] == 'rviz_default_plugins/SetInitialPose')
    assert pose_tool['Topic']['Value'] == '/initialpose'


def test_navigation_workflow_has_complete_official_nav2_interaction():
    config = _config('navigation.rviz')
    expected_topics = {
        'Global Costmap': '/global_costmap/costmap',
        'Local Costmap': '/local_costmap/costmap',
        'Global Plan': '/plan',
        'Local Plan': '/optimal_trajectory',
        'Global Footprint': '/global_costmap/published_footprint',
        'Local Footprint': '/local_costmap/published_footprint',
        'Stop Zone': '/collision_monitor/stop_zone',
        'Slowdown Zone': '/collision_monitor/slowdown_zone',
    }
    for display_name, topic in expected_topics.items():
        assert _named(config, display_name)['Topic']['Value'] == topic

    global_plan = _named(config, 'Global Plan')
    odometry = _named(config, 'Odometry')
    assert global_plan['Color'] == '255; 215; 0'
    assert global_plan['Pose Color'] == global_plan['Color']
    assert global_plan['Line Width'] >= 0.1
    assert global_plan['Color'] != odometry['Shape']['Color']

    optimal = _named(config, 'MPPI Optimal Trajectory')
    candidates = _named(config, 'MPPI Candidate Trajectories')
    assert optimal['Enabled'] is True
    assert optimal['Topic']['Value'] == '/optimal_trajectory'
    assert optimal['Line Width'] >= 0.07
    assert candidates['Enabled'] is True
    assert candidates['Topic']['Value'] == '/trajectories'

    panels = [panel['Class'] for panel in config['Panels']]
    tools = [tool['Class'] for tool in config['Visualization Manager']['Tools']]
    assert panels.count('robot_rviz_plugins/Navigation 2 Safe') == 1
    assert 'nav2_rviz_plugins/Navigation 2' not in panels
    assert tools.count('rviz_default_plugins/SetGoal') == 1
    assert tools.count('rviz_default_plugins/SetInitialPose') == 1
    assert 'nav2_rviz_plugins/GoalTool' not in tools
    assert '/goal_pose' not in (RVIZ_ROOT / 'navigation.rviz').read_text(
        encoding='utf-8')
    assert '/local_plan' not in (RVIZ_ROOT / 'navigation.rviz').read_text(
        encoding='utf-8')
    assert _named(config, 'Transformed Reference Plan')['Topic'][
        'Value'] == '/transformed_global_plan'
    assert _named(config, 'MPPI Candidate Trajectories')['Enabled'] is True

    module2 = _named(config, 'Module2 Live Overlay')
    _assert_topic(
        module2,
        '/bio_nav/module2/rviz_markers',
        reliability='Reliable',
        durability='Volatile',
    )
    assert module2['Enabled'] is True
    assert module2['Namespaces']['Motion Belief'] is True
    assert module2['Namespaces']['Motion Peak'] is True
    assert module2['Namespaces']['Dynamic Risk'] is True
    assert module2['Namespaces']['Local BEV Prediction'] is True
    assert module2['Namespaces']['Local BEV Label'] is False
    assert module2['Namespaces']['Status'] is False
    assert module2['Namespaces']['Visual Candidate'] is False
    applied = _named(config, 'Module2 Applied Nav2 Risk')
    _assert_topic(
        applied,
        '/bio_nav/local_risk_layer/rviz_markers',
        reliability='Reliable',
        durability='Volatile',
    )
    assert applied['Namespaces']['Projected Global Risk'] is True
    assert applied['Namespaces']['Nav2 Risk Status'] is False
    planning = _named(config, 'Module2 Planning Decision')
    _assert_topic(
        planning,
        '/bio_nav/planner/rviz_markers',
        reliability='Reliable',
        durability='Volatile',
    )
    assert planning['Namespaces']['Planning Decision'] is False
    raw_risk = _named(config, 'Module2 Dynamic Risk Raw')
    assert raw_risk['Enabled'] is False
    assert raw_risk['Color Scheme'] == 'costmap'
    assert raw_risk['Topic']['Value'] == '/bio_nav/module2/dynamic_cost_grid'

    depth_cloud = _named(config, 'Depth PointCloud2')
    _assert_topic(
        depth_cloud,
        '/camera/front/depth/points',
        reliability='Best Effort',
        durability='Volatile',
    )
    assert depth_cloud['Enabled'] is False
    assert depth_cloud['Color Transformer'] == 'FlatColor'
    assert depth_cloud['Color'] == '0; 255; 255'
    assert depth_cloud['Style'] == 'Flat Squares'
    assert depth_cloud['Decay Time'] == 0.5
    assert depth_cloud['Size (m)'] >= 0.05
    voxel_grid = _named(config, 'Marked Voxels (3D)')
    assert voxel_grid['Class'] == 'robot_rviz_plugins/Voxel Grid'
    assert voxel_grid['Topic']['Value'] == '/local_costmap/voxel_grid'
    assert voxel_grid['Enabled'] is True
    assert voxel_grid['Value'] is True
    assert voxel_grid['Color Transformer'] == 'FlatColor'
    assert voxel_grid['Style'] == 'Boxes'
    assert voxel_grid['Size (m)'] == pytest.approx(0.05)
    temporal_voxels = _named(config, 'Temporal Voxels (3D)')
    assert temporal_voxels['Class'] == 'rviz_default_plugins/PointCloud2'
    assert temporal_voxels['Topic']['Value'] == (
        '/local_costmap/stvl_voxel_grid')
    assert temporal_voxels['Enabled'] is True
    assert temporal_voxels['Color Transformer'] == 'FlatColor'
    assert temporal_voxels['Style'] == 'Boxes'
    assert temporal_voxels['Size (m)'] == pytest.approx(0.05)


def test_robot_description_cmake_installs_all_rviz_configs():
    cmake = (PACKAGE_ROOT / 'CMakeLists.txt').read_text(encoding='utf-8')
    assert 'install(DIRECTORY' in cmake
    assert 'rviz' in cmake


def test_dedicated_camera_view_uses_front_image_and_optical_frame():
    config = _config('camera_view.rviz')

    _assert_topic(
        _named(config, 'Robot Front Camera'),
        '/camera/front/image_raw',
        reliability='Best Effort',
        durability='Volatile',
    )
    assert config['Visualization Manager']['Global Options'][
        'Fixed Frame'] == 'camera_front_optical_frame'
    assert _named(config, 'Robot Front Camera')['Enabled'] is True
