import ast
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
    assert _named(config, 'TF')['Frames']['rtx_lidar']['Value'] is True
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


def test_navigation_workflow_has_single_indoor_cognitive_demo_view():
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

    assert _named(config, 'Static Map')['Enabled'] is True
    assert _named(config, 'RobotModel')['Enabled'] is True
    global_costmap = _named(config, 'Global Costmap')
    local_costmap = _named(config, 'Local Costmap')
    assert global_costmap['Enabled'] is True
    assert 0.08 <= global_costmap['Alpha'] <= 0.10
    assert local_costmap['Enabled'] is True
    assert local_costmap['Alpha'] <= 0.12

    # The public demo route is drawn once by the unified Integration marker
    # stream.  The raw Nav2 plans remain available as diagnostic toggles.
    for display_name in (
        'Grid',
        'TF',
        'LaserScan',
        'Odometry',
        'Module1 Candidate Odometry (No TF Authority)',
        'Global Plan',
        'Local Plan',
        'MPPI Optimal Trajectory',
        'MPPI Candidate Trajectories',
        'Collision Monitor',
        'RGB-D Fusion',
    ):
        assert _named(config, display_name)['Enabled'] is False

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
    module1_odom = _named(config, 'Module1 Candidate Odometry (No TF Authority)')
    _assert_topic(
        module1_odom,
        '/bio_nav/module1/odom',
        reliability='Reliable',
        durability='Volatile',
    )
    assert module1_odom['Enabled'] is False
    assert _named(config, 'SLAM Toolbox Diagnostic Map')['Enabled'] is False

    current = _named(config, 'Indoor Cognitive Navigation Demo')
    _assert_topic(
        current,
        '/bio_nav/v310/rviz',
        reliability='Reliable',
        durability='Volatile',
    )
    demo_namespaces = {
        'demo_grid_outline',
        'demo_m1_belief',
        'demo_m1_reference_pose',
        'demo_m1_cognitive_pose',
        'demo_m1_pose_delta',
        'demo_m2_srdr_grid',
        'demo_m2_applied_edges',
        'demo_m2_obstacle_offers',
        'demo_m2_predicted_risk',
        'demo_m2_consumers',
        'demo_navigation',
        'demo_story',
        'demo_m3_gvg',
        'demo_m3_lookahead',
        'demo_m3_smac',
        'demo_m3_mppi_candidates',
        'demo_m3_mppi_optimal',
        'demo_m3_pipeline',
    }
    assert demo_namespaces == {
        namespace
        for namespace, enabled in current['Namespaces'].items()
        if enabled
    }
    integration_visualizer = (
        PACKAGE_ROOT.parents[3]
        / 'bio_nav_integration'
        / 'ros2_ws/src/bio_nav_ros_bridge/bio_nav_ros_bridge'
        / 'v310_visualizer.py'
    )
    producer_tree = ast.parse(
        integration_visualizer.read_text(encoding='utf-8'))
    producer_demo_namespaces = {
        call.args[0].value
        for call in ast.walk(producer_tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Attribute)
        and call.func.attr == '_marker'
        and call.args
        and isinstance(call.args[0], ast.Constant)
        and isinstance(call.args[0].value, str)
        and call.args[0].value.startswith('demo_')
    }
    assert producer_demo_namespaces <= set(current['Namespaces'])
    assert all(
        not enabled
        for namespace, enabled in current['Namespaces'].items()
        if namespace not in demo_namespaces
    )

    module2 = _named(config, 'Module2 Live Overlay')
    _assert_topic(
        module2,
        '/bio_nav/module2/rviz_markers',
        reliability='Reliable',
        durability='Volatile',
    )
    assert module2['Enabled'] is False
    assert module2['Namespaces']['Motion Belief'] is True
    assert module2['Namespaces']['Motion Peak'] is True
    assert module2['Namespaces']['Dynamic Risk'] is False
    assert module2['Namespaces']['Local BEV Prediction'] is True
    assert module2['Namespaces']['Local BEV Label'] is False
    assert module2['Namespaces']['Status'] is True
    assert module2['Namespaces']['Visual Candidate'] is False
    navigation_source = (RVIZ_ROOT / 'navigation.rviz').read_text(
        encoding='utf-8')
    for retired_topic in (
        '/bio_nav/local_risk_layer/rviz_markers',
        '/bio_nav/planner/rviz_markers',
        '/bio_nav/sr_impact_probe/rviz_markers',
        '/bio_nav/risk_impact_probe/rviz_markers',
        '/bio_nav/planner/sr_impact_probe_plan',
    ):
        assert retired_topic not in navigation_source
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
    assert voxel_grid['Enabled'] is False
    assert voxel_grid['Value'] is False
    assert voxel_grid['Color Transformer'] == 'FlatColor'
    assert voxel_grid['Style'] == 'Boxes'
    assert voxel_grid['Size (m)'] == pytest.approx(0.05)
    temporal_voxels = _named(config, 'Temporal Voxels (3D)')
    assert temporal_voxels['Class'] == 'rviz_default_plugins/PointCloud2'
    assert temporal_voxels['Topic']['Value'] == (
        '/local_costmap/stvl_voxel_grid')
    assert temporal_voxels['Enabled'] is False
    assert temporal_voxels['Color Transformer'] == 'FlatColor'
    assert temporal_voxels['Style'] == 'Boxes'
    assert temporal_voxels['Size (m)'] == pytest.approx(0.05)

    view = config['Visualization Manager']['Views']['Current']
    assert view['Class'] == 'rviz_default_plugins/TopDownOrtho'
    assert view['Angle'] == pytest.approx(0.0)
    assert view['X'] == pytest.approx(0.0)
    assert view['Y'] == pytest.approx(0.5)
    assert view['Scale'] == pytest.approx(50.0)
    window = config['Window Geometry']
    assert (window['Width'], window['Height']) == (1920, 1080)
    # Reserve a conservative 120x80 px for window chrome.  The central grid
    # plus side panels occupy x=[-17.6, 17.6], y=[-8.0, 9.5] metres.
    visible_width_m = (window['Width'] - 120) / view['Scale']
    visible_height_m = (window['Height'] - 80) / view['Scale']
    assert 0.75 <= 16.0 / visible_height_m <= 0.80
    assert view['X'] - visible_width_m / 2.0 <= -17.6
    assert view['X'] + visible_width_m / 2.0 >= 17.6
    assert view['Y'] - visible_height_m / 2.0 <= -8.0
    assert view['Y'] + visible_height_m / 2.0 >= 9.5
    assert window['Hide Left Dock'] is True
    assert window['Hide Right Dock'] is True

    active_leaves = []

    def collect_active_leaves(display, parent_enabled=True):
        enabled = parent_enabled and display.get('Enabled', True)
        children = display.get('Displays', [])
        if children:
            for child in children:
                collect_active_leaves(child, enabled)
        elif enabled:
            active_leaves.append(display)

    for display in config['Visualization Manager']['Displays']:
        collect_active_leaves(display)
    assert {display['Name'] for display in active_leaves} == {
        'RobotModel',
        'Static Map',
        'Global Costmap',
        'Local Costmap',
        'Indoor Cognitive Navigation Demo',
    }
    active_topics = [
        display['Topic']['Value']
        for display in active_leaves
        if isinstance(display.get('Topic'), dict)
    ]
    assert len(active_topics) == len(set(active_topics))


def test_rivermark_workflow_exposes_tiles_module2_and_module3_authority():
    config = _config('rivermark.rviz')
    manager = config['Visualization Manager']
    assert manager['Global Options']['Fixed Frame'] == 'map'
    _assert_topic(
        _named(config, 'Rivermark Physical Occupancy 0.05m'),
        '/map',
        reliability='Reliable',
        durability='Transient Local',
    )
    physical_map = _named(config, 'Rivermark Physical Occupancy 0.05m')
    global_costmap = _named(config, 'Module3 Global Costmap')
    lidar = _named(config, 'Outdoor LiDAR Obstacles')
    assert physical_map['Enabled'] is True
    # The combined costmap includes the static map's inflation gradient.  It
    # remains available as a diagnostic toggle but must not obscure /map by
    # default or look like accumulated scan residue.
    assert global_costmap['Enabled'] is False
    assert global_costmap['Value'] is False
    assert global_costmap['Alpha'] <= 0.20
    assert lidar['Decay Time'] == 0
    unified = _named(config, 'Attempt31 Live Tile and Execution')
    _assert_topic(
        unified,
        '/bio_nav/v310/rviz',
        reliability='Reliable',
        durability='Transient Local',
    )
    static = _named(config, 'Attempt31 Outdoor Static Topology')
    _assert_topic(
        static,
        '/bio_nav/v310/rviz_static',
        reliability='Reliable',
        durability='Transient Local',
    )
    edge = _named(config, 'Attempt31 Module2 Module3 Edge Handoff')
    _assert_topic(
        edge,
        '/bio_nav/v310/rviz_edges',
        reliability='Reliable',
        durability='Transient Local',
    )
    required = {
        'active_tile_canvas_16x16',
        'active_tile_cells',
        'active_tile_core_12x12',
        'tile_switch_direction',
        'tile_switch_event',
        'module2_p_corr',
        'module2_sr',
        'module2_dr',
        'module2_dynamic_cost',
        'module2_remap',
        'bridge_module2',
        'module3_gvg_edges',
        'selected_canonical_route',
        'module3_runtime_blocked',
        'ownership_module2',
        'ownership_module3',
        'ownership_handoff',
    }
    enabled_namespaces = {
        name
        for display in (unified, static, edge)
        for name, enabled in display['Namespaces'].items()
        if enabled
    }
    assert required <= enabled_namespaces
    assert _named(config, 'Module3 Local Costmap')['Enabled'] is True
    assert _named(config, 'Module3 MPPI Optimal Trajectory')['Enabled'] is True
    actors = _named(config, 'Rivermark Physical Dynamic Actors')
    _assert_topic(
        actors,
        '/experiment/dynamic_obstacles/markers',
        reliability='Reliable',
        durability='Volatile',
    )
    assert actors['Namespaces']['dynamic_obstacles'] is True
    assert actors['Namespaces']['dynamic_future_trajectory'] is True
    goal_tool = next(
        tool for tool in manager['Tools']
        if tool['Class'] == 'rviz_default_plugins/SetGoal'
    )
    assert goal_tool['Topic']['Value'] == '/bio_nav/route_goal'


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
