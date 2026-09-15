"""Source/launch contracts, not a substitute for C++ compilation or bag replay.

Run with ROS sourced: /usr/bin/python3 scripts/test_internal_core_alignment.py
The stock tree is an audit reference only; no nodes or external package are run.
"""
import importlib.util
import re
from pathlib import Path
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[1]
OURS = ROOT / 'src/adaptive_fast_lio2'
REF = ROOT / 'src/FAST_LIO'


def read(path):
    return path.read_text()


def normalized(source):
    source = re.sub(r'/\*.*?\*/|//[^\n]*', '', source, flags=re.S)
    return re.sub(r'\s+', '', source)


def function(source, signature):
    start = source.index(signature)
    opening = source.index('{', start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


class InternalCoreTests(unittest.TestCase):
    def setUp(self):
        self.main = read(OURS / 'src/adaptive_laserMapping.cpp')
        self.ref_main = read(REF / 'src/laserMapping.cpp')

    def test_preprocessing_algorithm_is_internal_stock_port(self):
        expected = read(REF / 'src/preprocess.cpp').replace(
            '#include "preprocess.h"',
            '#include "adaptive_fast_lio2/adaptive_preprocess.hpp"\n#include <omp.h>')
        self.assertEqual(normalized(expected), normalized(read(OURS / 'src/adaptive_preprocess.cpp')))
        expected_header = read(REF / 'src/preprocess.h')
        actual_header = read(OURS / 'include/adaptive_fast_lio2/adaptive_preprocess.hpp')
        self.assertEqual(normalized(expected_header[expected_header.index('enum Feature'):]),
                         normalized(actual_header[actual_header.index('enum Feature'):]))

    def test_all_imu_algorithm_bodies_match(self):
        expected = read(REF / 'src/IMU_Processing.hpp')
        expected = expected[expected.index('ImuProcess::ImuProcess()'):]
        expected = expected.replace('ImuProcess', 'AdaptiveImuProcess').replace(
            'fout_imu.open(DEBUG_FILE_DIR("imu.txt"),ios::out);', '')
        actual = read(OURS / 'src/adaptive_imu_process.cpp')
        actual = actual[actual.index('AdaptiveImuProcess::AdaptiveImuProcess()'):]
        # Explicitly reviewed storage initialization, not an Adaptive-only algorithm branch.
        self.assertEqual(actual.count('acc_s_last = Zero3d;'), 2)
        self.assertEqual(actual.count('last_lidar_end_time_ = 0.0;'), 2)
        actual = actual.replace('acc_s_last = Zero3d;', '').replace('last_lidar_end_time_ = 0.0;', '')
        self.assertEqual(normalized(expected), normalized(actual))
        self.assertIn('#define MAX_INI_COUNT (10)', read(OURS / 'include/adaptive_fast_lio2/adaptive_imu_process.hpp'))
        self.assertIn('std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu;',
                      read(OURS / 'include/adaptive_fast_lio2/adaptive_common.hpp'))

    def test_internal_pose_storage_adapter(self):
        upstream = function(read(REF / 'include/common_lib.h'), 'auto set_pose6d(')
        own = function(read(OURS / 'include/adaptive_fast_lio2/fastlio_core_types.hpp'), 'auto set_pose6d(')
        self.assertEqual(normalized(upstream), normalized(own))

    def test_sync_algorithm_matches_reference(self):
        own = function(self.main, 'bool sync_packages(')
        self.assertEqual(normalized(own), normalized(function(self.ref_main, 'bool sync_packages(')))
        self.assertIn('double lidar_mean_scantime = 0.0;', self.main)

    def test_ros_time_conversion_matches_reference(self):
        reference = read(REF / 'include/common_lib.h')
        for signature in ('double get_time_sec(', 'rclcpp::Time get_ros_time('):
            self.assertEqual(normalized(function(reference, signature)),
                             normalized(function(self.main, signature)))

    def test_scan_end_ablation_is_removed_from_live_code(self):
        paths = [OURS / 'src/adaptive_laserMapping.cpp',
                 OURS / 'src/adaptive_runtime_logger.cpp',
                 OURS / 'include/adaptive_fast_lio2/adaptive_runtime_logger.hpp',
                 OURS / 'launch/adaptive_fast_lio2.launch.py']
        for path in paths:
            with self.subTest(file=path.name):
                self.assertNotRegex(read(path),
                                    r'scan_end_use_last_point|scan_last_offset_s|scan_max_offset_s|scan_end_fallback')
        self.assertFalse((ROOT / 'scripts/run_scan_end_ab.py').exists())

    def test_runtime_csv_schema_matches_rows(self):
        logger = read(OURS / 'src/adaptive_runtime_logger.cpp')
        header = logger.split('constexpr char kRuntimeCsvHeader[] =', 1)[1].split(';', 1)[0]
        columns = ''.join(re.findall(r'"([^"\n]*)"', header)).split(',')
        writer = function(logger, 'void AdaptiveRuntimeLogger::write(')
        fields = re.findall(r'row\.(\w+)', writer)
        self.assertEqual(columns, fields)
        self.assertIn('header != kRuntimeCsvHeader', logger)

    def test_invalid_quality_low_effective_recovery_boundary(self):
        insertion = function(self.main, 'void map_incremental()')
        self.assertIn('adaptive_invalid_quality_low_effective_relax_enable', insertion)
        self.assertIn('effct_feat_num < adaptive_min_effective_points', insertion)
        allowance = function(self.main, 'bool allow_map_insert_point(')
        self.assertIn('adaptive_invalid_quality_filter_enable &&\n                !invalid_quality_relax_active', allowance)
        self.assertIn('invalid_quality_relaxed_num++;', allowance)
        self.assertLess(allowance.index('invalid_quality_relaxed_num++;'),
                        allowance.index('if(frame_degenerate && has_quality && directional_selection_enable)'))

    def test_equal_point_count_control_boundary(self):
        insertion = function(self.main, 'void map_incremental()')
        allowance = function(self.main, 'bool allow_map_insert_point(')
        self.assertIn('adaptive_equal_point_count_control_enable', insertion)
        self.assertIn('frame_degenerate;', insertion)
        self.assertIn('!equal_point_count_control_active', insertion)
        self.assertIn('equal_point_count_accepted_num >=', insertion)
        self.assertIn('adaptive_equal_point_count_per_degenerate_frame', insertion)
        self.assertIn('frame_degenerate && has_quality && directional_selection_enable', allowance)
        self.assertLess(insertion.index('if (!allow_insert)'),
                        insertion.index('equal_point_count_accepted_num >='))

    def test_directional_selection_has_independent_ablation_switch(self):
        insertion = function(self.main, 'void map_incremental()')
        allowance = function(self.main, 'bool allow_map_insert_point(')
        self.assertIn('adaptive_directional_selection_enable', insertion)
        self.assertIn('adaptive_directional_selection_enable &&\n                    !equal_point_count_control_active', insertion)
        self.assertIn('frame_degenerate && has_quality && directional_selection_enable', allowance)
        self.assertIn('adaptive_map.directional_selection_enable', self.main)

    def test_frozen_c_candidate_is_the_project_default(self):
        launch = read(OURS / 'launch/adaptive_fast_lio2.launch.py')
        config = yaml.safe_load(read(OURS / 'config/adaptive_fast_lio2.yaml'))
        params = config['adaptive_fastlio_mapping']['ros__parameters']
        self.assertIn('bool adaptive_directional_selection_enable = false;', self.main)
        self.assertIn('bool adaptive_window_enable = false;', self.main)
        self.assertIn(
            'declare_parameter<bool>("adaptive_map.directional_selection_enable", false)',
            self.main)
        self.assertIn(
            'declare_parameter<bool>("adaptive_window.enable", false)',
            self.main)
        window_start = launch.index(
            'declare_adaptive_window_enable_cmd = DeclareLaunchArgument(')
        window_declaration = launch[
            window_start:launch.index('\n    )', window_start)]
        self.assertIn('default_value="false"', window_declaration)
        self.assertTrue(params['adaptive_map']['enable'])
        self.assertFalse(params['adaptive_map']['directional_selection_enable'])
        self.assertFalse(params['adaptive_map']['equal_point_count_control_enable'])
        self.assertFalse(params['adaptive_window']['enable'])
        self.assertEqual(params['adaptive_window']['persistent_insert_quota_max'], 0)

    def test_historical_e_configs_do_not_inherit_frozen_c_switches(self):
        config_dir = (ROOT / 'experiments/adaptive_map_v1/validation/subt_hawkins/'
                      'persistent_quota_v1/config')
        for name in ('P0_no_total_quota.yaml', 'P1_quota_0p3_2_5.yaml'):
            with self.subTest(config=name):
                config = yaml.safe_load(read(config_dir / name))
                params = config['adaptive_fastlio_mapping']['ros__parameters']
                self.assertTrue(params['adaptive_map']['directional_selection_enable'])
                self.assertFalse(params['adaptive_map']['equal_point_count_control_enable'])

    def test_invalid_quality_turn_guard_boundary(self):
        insertion = function(self.main, 'void map_incremental()')
        self.assertIn('adaptive_invalid_quality_turn_guard_enable', insertion)
        self.assertIn('degeneracy_window_ready', insertion)
        self.assertIn('window_yaw_change > adaptive_window_max_yaw_change', insertion)
        self.assertIn('invalid_quality_relax_candidate &&\n        !invalid_quality_turn_guard_active', insertion)
        allowance = function(self.main, 'bool allow_map_insert_point(')
        self.assertIn('invalid_quality_turn_guard_rejected_num++;', allowance)

    def test_skipped_map_update_is_logged_without_advancing_window(self):
        insertion = function(self.main, 'void map_incremental()')
        gate = function(insertion, 'if (adaptive_map_enable && effct_feat_num < scan_match_min_effective_points)')
        self.assertIn('write_runtime_log_row(', gate)
        self.assertLess(gate.index('write_runtime_log_row('), gate.index('return;'))
        for mutation in ('update_degeneracy_window(', 'map_update_count++', 'p_map->addPoints('):
            self.assertNotIn(mutation, gate)
        self.assertIn('const bool frame_degenerate = is_current_frame_degenerate();', gate)
        log_writer = function(self.main, 'void write_runtime_log_row(')
        self.assertIn('map_update_skipped ? "low_effective_points" : "none"', log_writer)
        allowance = function(self.main, 'bool allow_map_insert_point(')
        for condition, counter in [('if(range < adaptive_min_range)', 'range_near_reject_num++'),
                                   ('if(range > adaptive_max_range)', 'range_far_reject_num++')]:
            branch = function(allowance, condition)
            self.assertLess(branch.index(counter), branch.index('return false;'))

    def test_float_plane_fit_matches_stock(self):
        expected = function(read(REF / 'include/common_lib.h'), 'bool esti_plane(')
        expected = expected.replace(
            'bool esti_plane(Matrix<T, 4, 1> &pca_result, const PointVector &point, const T &threshold)',
            'bool estimate_plane_from_neighbors(const MapPointVector &point, Eigen::Vector4f &pca_result)')
        expected = expected.replace('NUM_MATCH_POINTS', '5').replace('Matrix<T,', 'Eigen::Matrix<float,')
        expected = expected.replace('T n =', 'float n =').replace('> threshold', '> 0.1f')
        self.assertEqual(normalized(expected), normalized(function(self.main, 'bool estimate_plane_from_neighbors(')))

    def test_local_cube_algorithm_matches_stock(self):
        expected = function(self.ref_main, 'void lasermap_fov_segment()')
        edits = {
            'cub_needrm.clear();': 'std::vector<BoxPointType> cub_needrm;',
            'kdtree_delete_counter = 0;': '', 'kdtree_delete_time = 0.0;': '',
            'pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);': '',
            'V3D pos_LiD = pos_lid;': 'Eigen::Vector3d pos_LiD = state_point.pos + state_point.rot * state_point.offset_T_L_I;',
            'MOV_THRESHOLD': 'move_threshold', 'DET_RANGE': 'det_range',
            'points_cache_collect();': 'p_map->collectRemovedPoints();',
            'double delete_begin = omp_get_wtime();': '',
            'kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);': 'p_map->Delete_Point_Boxes(cub_needrm);',
            'kdtree_delete_time = omp_get_wtime() - delete_begin;': '',
        }
        for a, b in edits.items():
            expected = expected.replace(a, b)
        self.assertEqual(normalized(expected), normalized(function(self.main, 'void lasermap_fov_segment()')))

    def test_dependency_algorithms_match(self):
        count = 0
        for folder in ('IKFoM_toolkit', 'ikd-Tree'):
            for ref in (REF / 'include' / folder).rglob('*'):
                if ref.is_file():
                    own = OURS / 'include' / ref.relative_to(REF / 'include')
                    with self.subTest(file=str(own)):
                        self.assertEqual(ref.read_bytes(), own.read_bytes())
                    count += 1
        self.assertGreaterEqual(count, 14)
        self.assertEqual(normalized(read(REF / 'include/use-ikfom.hpp')),
                         normalized(read(OURS / 'include/use-ikfom.hpp').replace('inline ', '')))

    def test_single_shared_estimator_and_pipeline_order(self):
        timer = function(self.main, 'void timer_callback()')
        self.assertNotIn('adaptive_map_enable', timer)
        matcher = function(self.main, 'void h_share_model(')
        # Adaptive may collect diagnostics, but must not alter measurement
        # construction. OFF must bypass the expensive diagnostic branches.
        svd = function(matcher, 'if (adaptive_map_enable && effct_feat_num > 0)')
        self.assertIn('Eigen::JacobiSVD', svd)
        self.assertNotIn('ekfom_data.h_x =', svd)
        self.assertNotIn('ekfom_data.h(i) =', svd)
        self.assertIn('const float residual =', matcher)
        self.assertIn('const float score =', matcher)
        self.assertIn('if (!(score > 0.9)) continue;', matcher)
        self.assertIn('std::fill(ikfom_epsi, ikfom_epsi + 23, 0.001);', self.main)
        self.assertIn('rclcpp::create_timer(this, this->get_clock()', self.main)
        self.assertIn('create_subscription<sensor_msgs::msg::Imu>(imu_topic, 10, imu_cbk)', self.main)
        steps = ['if (flg_first_scan)', 'p_imu->Process(', 'feats_undistort->empty()',
                 'lasermap_fov_segment();', 'downsample_current_scan(', '!p_map->hasRoot()',
                 'feats_down_body->size() < 5', 'kf.update_iterated_dyn_share_modified(',
                 'publish_odometry(Measures);', 'map_incremental();']
        offsets = [timer.index(step) for step in steps]
        self.assertEqual(offsets, sorted(offsets))

    def test_baseline_insertion_and_adaptive_boundary(self):
        insertion = function(self.main, 'void map_incremental()')
        self.assertIn('nearest_points != nullptr && flg_EKF_inited', insertion)
        self.assertIn('if (nearest_points->size() < 5) break;', insertion)
        self.assertIn('adaptive_map_enable && effct_feat_num < scan_match_min_effective_points', insertion)
        allowance = function(self.main, 'bool allow_map_insert_point(')
        self.assertRegex(allowance, r'if\s*\(!adaptive_map_enable\)\s*\{\s*return true;')
        self.assertIn('p_map->addPoints(point_to_add, true);', insertion)
        self.assertIn('p_map->addPoints(point_no_need_downsample, false);', insertion)

    def test_off_observation_path_excludes_adaptive_statistics(self):
        matcher = function(self.main, 'void h_share_model(')
        # Inspect the reachable OFF path by removing master-enabled blocks.
        svd = function(matcher, 'if (adaptive_map_enable && effct_feat_num > 0)')
        matcher = matcher.replace(svd, '')
        while 'if (adaptive_map_enable)\n    {' in matcher:
            branch = function(matcher, 'if (adaptive_map_enable)\n    {')
            matcher = matcher.replace(branch, '', 1)
        # Nested braced blocks have different indentation.
        matcher = re.sub(
            r'if \(adaptive_map_enable\)\s*\{[^{}]*\}', '', matcher)
        for operation in ('median_of_values(', 'Eigen::JacobiSVD',
                          'map_point_effective.assign(', 'map_point_effective[i] =',
                          'effective_residuals.push_back('):
            self.assertNotIn(operation, matcher)
        self.assertIn('ekfom_data.h_x =', matcher)
        self.assertIn('ekfom_data.h(i) = -norm_p.intensity;', matcher)
        self.assertIn('if (!(score > 0.9)) continue;', matcher)

    def test_mapping_visualization_does_not_traverse_live_tree(self):
        self.assertNotIn('getMapCloud(', self.main)
        self.assertNotIn('pub_ikdtree_map_', self.main)
        timer = function(self.main, 'void timer_callback()')
        self.assertNotIn('publish_map(', timer)
        self.assertIn('std::chrono::milliseconds(1000)', self.main)

    def test_map_wrapper_preserves_native_point_container(self):
        manager = read(OURS / 'src/adaptive_map_manager.cpp')
        header = read(OURS / 'include/adaptive_fast_lio2/adaptive_map_manager.hpp')
        self.assertIn('using PointVector = KD_TREE<PointType>::PointVector;', header)
        search = function(manager, 'bool AdaptiveMapManager::nearestSearch(')
        self.assertIn('Nearest_Search(point_world, k, nearest_points, squared_distances)', search)
        self.assertNotIn('.assign(', search)
        insertion = function(manager, 'void AdaptiveMapManager::addPoints(PointVector &points')
        self.assertIn('ikdtree_->Build(points)', insertion)
        self.assertIn('ikdtree_->Add_Points(points, need_downsample)', insertion)

    def test_publication_alignment_and_current_covariance(self):
        body = function(self.main, 'void publish_current_cloud_body(')
        self.assertIn('!scan_publish_en || !scan_bodyframe_pub_en', body)
        path = function(self.main, 'void publish_path(')
        self.assertIn('if (!path_publish_en) return;', path)
        self.assertIn('++path_count % 10 != 0', path)
        odom = function(self.main, 'void publish_odometry(')
        self.assertIn('state_point.rot.coeffs()', odom)
        self.assertLess(odom.index('odom.pose.covariance[i * 6 + j]'),
                        odom.index('pub_odom_->publish(odom)'))

    def test_current_configs_do_not_select_legacy_core(self):
        for config in (OURS / 'config').glob('*.yaml'):
            data = yaml.safe_load(read(config))
            for node in data.values():
                params = node.get('ros__parameters', {})
                mapping = params.get('mapping', {})
                with self.subTest(config=config.name):
                    self.assertEqual(mapping.get('imu_init_num', 10), 10)
                    self.assertNotIn('scan_end_use_last_point', mapping)
                    self.assertEqual(mapping.get('nearest_search_num', 5), 5)
                    self.assertTrue(mapping.get('local_map_enable', True))
                    self.assertTrue(mapping.get('local_map_delete_enable', True))

    def test_no_external_fast_lio_build_or_runtime_dependency(self):
        package = ET.parse(OURS / 'package.xml').getroot()
        self.assertNotIn('fast_lio', [n.text for n in package if 'depend' in n.tag])
        cmake = read(OURS / 'CMakeLists.txt')
        self.assertNotRegex(cmake, r'find_package\s*\(\s*fast_lio\b')
        self.assertNotRegex(cmake, r'(?:\.\./|/src/)FAST_LIO/')
        for path in list((OURS / 'include/adaptive_fast_lio2').glob('*')) + list((OURS / 'src').glob('*.cpp')):
            self.assertNotRegex(read(path), r'#include\s*[<"](?:fast_lio/|common_lib.h|IMU_Processing.hpp)')

    def test_three_paired_sensor_and_mapping_configs(self):
        def flat(path):
            result = {}
            def visit(data, prefix=''):
                for key, value in data.items():
                    if isinstance(value, dict):
                        visit(value, prefix + key + '.')
                    else:
                        result[prefix + key] = value
            visit(next(iter(yaml.safe_load(read(path)).values()))['ros__parameters'])
            return result
        aliases = {'max_iteration': 'mapping.scan_match_max_iteration',
                   'filter_size_surf': 'mapping.filter_size_surf',
                   'filter_size_map': 'mapping.filter_size_map',
                   'cube_side_length': 'mapping.cube_len'}
        pairs = [('ntu_spms.yaml', 'ntu_spms_baseline.yaml'),
                 ('subt_mrs_hawkins_long_corridor.yaml', 'subt_mrs_hawkins_long_corridor.yaml'),
                 ('geode_gamma_blind2.yaml', 'geode_gamma_source_check_baseline.yaml')]
        for own_name, ref_name in pairs:
            own, reference = flat(OURS / 'config' / own_name), flat(REF / 'config' / ref_name)
            for key, value in reference.items():
                if key.startswith(('publish.', 'pcd_save.')) or key in (
                        'runtime_pos_log_enable', 'map_file_path', 'mapping.fov_degree'):
                    continue
                with self.subTest(config=own_name, key=key):
                    self.assertEqual(own.get(aliases.get(key, key)), value)

    def test_core_revision_is_recorded(self):
        self.assertIn('fastlio2_internal_v1', read(OURS / 'include/adaptive_fast_lio2/adaptive_runtime_logger.hpp'))
        logger = read(OURS / 'src/adaptive_runtime_logger.cpp')
        self.assertIn('row.frontend_core_revision', logger)
        self.assertIn('sync_imu_last_time,frontend_core_revision', logger)
        self.assertIn('legacy schema', logger)

    def test_launch_on_and_off_use_own_executable(self):
        from launch import LaunchContext
        spec = importlib.util.spec_from_file_location('own_launch', OURS / 'launch/adaptive_fast_lio2.launch.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        with patch.object(module, 'Node', wraps=module.Node) as nodes, patch.object(
                module, 'get_package_share_directory', return_value=str(OURS)):
            module.generate_launch_description()
        frontend = [c.kwargs for c in nodes.call_args_list if c.kwargs.get('executable') == 'adaptive_fastlio_mapping']
        self.assertEqual(len(frontend), 1)
        self.assertEqual(frontend[0]['package'], 'adaptive_fast_lio2')
        self.assertFalse(any(c.kwargs.get('package') == 'fast_lio' for c in nodes.call_args_list))
        flag = next(p['adaptive_map.enable'] for p in frontend[0]['parameters'] if isinstance(p, dict) and 'adaptive_map.enable' in p)
        for value in ('true', 'false'):
            context = LaunchContext()
            context.launch_configurations['adaptive_map_enable'] = value
            self.assertEqual(flag.perform(context), value)


if __name__ == '__main__':
    unittest.main()
