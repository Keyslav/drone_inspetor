"""Geometria e dados inválidos da câmera depth sem iniciar ROS."""

from dataclasses import replace
import math

import numpy as np
import pytest

from drone_inspetor.nodes.depth_node.processing import (
    depth_statistics, filtered_depth, proximity_alerts, render_depth,
)
from drone_inspetor.nodes.depth_node.projection import (
    DepthCalibration, depth_in_meters, project_depth_scan,
)


def calibration(**kwargs):
    return replace(DepthCalibration(fx=1., fy=1., cx=1., cy=1., width=3, height=3,
                                    bins=3, band_half_height_m=.4), **kwargs)


def test_optical_depth_becomes_radial_and_right_image_is_negative_flu_angle():
    image = np.full((3, 3), np.nan)
    image[1] = [2, 3, 4]
    scan = project_depth_scan(image, calibration())
    assert scan.angle_min == pytest.approx(-math.pi / 4)
    assert scan.angle_max == pytest.approx(math.pi / 4)
    assert scan.ranges == pytest.approx((4 * math.sqrt(2), 3, 2 * math.sqrt(2)))
    assert scan.angle_increment > 0


def test_yaw_rotates_only_observed_fov_and_never_extrapolates():
    image = np.full((3, 3), 2.)
    scan = project_depth_scan(image, calibration(mount_yaw_deg=90))
    assert scan.angle_min == pytest.approx(math.pi / 4)
    assert scan.angle_max == pytest.approx(3 * math.pi / 4)
    assert len(scan.ranges) == 3


def test_vertical_band_excludes_ceiling_floor_and_honors_camera_height():
    image = np.array([[.2, .3, .4], [5., 5., 5.], [.2, .3, .4]])
    scan = project_depth_scan(image, calibration(band_half_height_m=.1))
    assert scan.ranges == pytest.approx((5 * math.sqrt(2), 5, 5 * math.sqrt(2)))
    # Com a câmera 1m acima do drone, a linha inferior a 1m está no plano do drone.
    image[:] = np.nan
    image[2, 1] = 1.
    at_height = project_depth_scan(image, calibration(camera_height_m=1.))
    assert at_height.ranges[1] == 1.
    assert math.isnan(project_depth_scan(image, calibration()).ranges[1])


@pytest.mark.parametrize('invalid', [math.nan, math.inf, -math.inf, 0., -1., 11.])
def test_invalid_measurements_never_become_clear_rays(invalid):
    scan = project_depth_scan(np.full((3, 3), invalid), calibration())
    assert all(math.isnan(value) for value in scan.ranges)


def test_positive_saturation_below_minimum_is_conservative_near_hit():
    image = np.full((3, 3), .02)
    scan = project_depth_scan(image, calibration())
    assert scan.ranges == pytest.approx((scan.range_min,) * 3)


def test_sparse_or_invalid_columns_are_not_interpolated():
    image = np.full((3, 3), np.nan)
    image[1, 1] = 2.
    scan = project_depth_scan(image, calibration(bins=5))
    assert scan.ranges[2] == 2.
    assert all(math.isnan(scan.ranges[index]) for index in (0, 1, 3, 4))


def test_closest_return_within_band_wins_not_the_average():
    image = np.full((3, 3), 7.)
    image[2, 1] = .25
    scan = project_depth_scan(image, calibration())
    assert scan.ranges[1] == .25


def test_hfov_requires_explicit_calibration_and_uses_pixel_centers():
    uncalibrated = DepthCalibration()
    assert not uncalibrated.calibrated
    with pytest.raises(ValueError):
        project_depth_scan(np.ones((3, 3)), uncalibrated)
    configured = DepthCalibration(horizontal_fov_deg=90, bins=3)
    scan = project_depth_scan(np.ones((3, 3)), configured)
    assert -math.pi / 4 < scan.angle_min < 0 < scan.angle_max < math.pi / 4


def test_intrinsics_scale_with_resolution_preserving_optical_center():
    fx, fy, cx, cy = calibration().intrinsics(6, 6)
    assert (fx, fy, cx, cy) == (2., 2., 2.5, 2.5)


def test_depth_encodings_are_explicit_metric_units():
    millimeters = np.asarray([[0, 250, 1000]], dtype=np.uint16)
    assert depth_in_meters(millimeters, '16UC1').tolist()[0] == pytest.approx([0, .25, 1.])
    meters = np.asarray([[.25, 1., np.inf]], dtype=np.float32)
    assert depth_in_meters(meters, '32FC1')[0, 1] == 1.
    with pytest.raises(ValueError):
        depth_in_meters(millimeters, 'mono16')


def test_statistics_and_alerts_clear_when_frame_has_no_close_returns():
    depth = filtered_depth(np.array([[np.nan, 0., -1., np.inf, .2, 4., 20.]]), .1, 10.)
    statistics = depth_statistics(depth, '10:00:00')
    assert statistics['valid_pixels'] == 2
    assert statistics['mean_distance'] == pytest.approx(2.1)
    assert len(proximity_alerts(depth, 1.)) == 1
    assert proximity_alerts(np.full((3, 3), 5.), 1.) == []
    assert depth_statistics(np.zeros((3, 3)))['valid_percentage'] == 0.
    assert render_depth(np.zeros((3, 3)), {}, [], 'colormap').sum() == 0


@pytest.mark.parametrize('kwargs', [dict(horizontal_fov_deg=180.), dict(fx=10., fy=0.),
                                    dict(band_half_height_m=0.), dict(min_depth=2., max_depth=1.),
                                    dict(fx=-1., fy=-1.), dict(bins=1)])
def test_invalid_calibration_rejected(kwargs):
    with pytest.raises(ValueError):
        DepthCalibration(**kwargs)
