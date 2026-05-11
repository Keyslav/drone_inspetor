"""Testes unitários para drone_inspetor.common.math_utils."""

import math

import pytest

from drone_inspetor.common.math_utils import (
    normalize_yaw_deg,
    yaw_deg_to_0_360,
    yaw_diff_shortest,
    yaw_step_toward,
    global_to_local_offset,
    horizontal_distance,
    distance_3d,
)


class TestNormalizeYawDeg:
    @pytest.mark.parametrize("input_deg,expected", [
        (0, 0),
        (90, 90),
        (180, -180),     # borda superior normaliza para -180
        (-180, -180),
        (181, -179),
        (360, 0),
        (-360, 0),
        (540, -180),
        (-540, -180),
        (720, 0),
        (45.5, 45.5),
    ])
    def test_normalize_yaw_deg(self, input_deg, expected):
        assert normalize_yaw_deg(input_deg) == pytest.approx(expected)

    def test_always_in_range(self):
        """Qualquer input deve resultar em [-180, 180)."""
        for deg in range(-1000, 1000, 37):
            result = normalize_yaw_deg(deg)
            assert -180 <= result < 180, f"normalize({deg}) = {result}"


class TestYawDegTo0_360:
    @pytest.mark.parametrize("input_deg,expected", [
        (0, 0),
        (90, 90),
        (180, 180),
        (-180, 180),
        (-90, 270),
        (360, 0),
        (359, 359),
        (720, 0),
        (-1, 359),
    ])
    def test_yaw_deg_to_0_360(self, input_deg, expected):
        assert yaw_deg_to_0_360(input_deg) == pytest.approx(expected)

    def test_always_in_range(self):
        for deg in range(-1000, 1000, 37):
            result = yaw_deg_to_0_360(deg)
            assert 0 <= result < 360


class TestYawDiffShortest:
    @pytest.mark.parametrize("current,target,expected", [
        (0, 0, 0),
        (90, 0, 90),       # 90° à direita do alvo
        (0, 90, -90),      # 90° à esquerda do alvo
        (179, -179, -2),   # wrap: caminho mais curto é -2°, não 358°
        (-179, 179, 2),
        (0, 180, -180),    # borda
        (45, 225, -180),   # oposto
        (10, 350, 20),     # wrap positivo
    ])
    def test_yaw_diff_shortest(self, current, target, expected):
        assert yaw_diff_shortest(current, target) == pytest.approx(expected)

    def test_range_bounded(self):
        """Diferença sempre em [-180, 180)."""
        import random
        random.seed(42)
        for _ in range(100):
            a = random.uniform(-720, 720)
            b = random.uniform(-720, 720)
            diff = yaw_diff_shortest(a, b)
            assert -180 <= diff < 180


class TestYawStepToward:
    def test_reaches_target_when_within_step(self):
        assert yaw_step_toward(10, 15, 10) == pytest.approx(15)
        assert yaw_step_toward(10, 5, 10) == pytest.approx(5)

    def test_steps_toward_positive(self):
        # target 90° à direita, passo de 15° → avança 15°
        assert yaw_step_toward(0, 90, 15) == pytest.approx(-15) or \
               yaw_step_toward(0, 90, 15) == pytest.approx(15)

    def test_steps_toward_shortest_path(self):
        # current=170, target=-170 → diferença mais curta: +20°
        # Com step=10, avança 10° → 180 → normaliza para -180
        result = yaw_step_toward(170, -170, 10)
        assert result == pytest.approx(-180) or result == pytest.approx(180)

    def test_result_normalized(self):
        import random
        random.seed(7)
        for _ in range(50):
            current = random.uniform(-180, 180)
            target = random.uniform(-180, 180)
            step = random.uniform(1, 45)
            result = yaw_step_toward(current, target, step)
            assert -180 <= result < 180


class TestGlobalToLocalOffset:
    def test_zero_offset(self):
        dx, dy, dz = global_to_local_offset(0, 0, 0, 0, 0, 0)
        assert dx == 0 and dy == 0 and dz == 0

    def test_altitude_offset(self):
        _, _, dz = global_to_local_offset(0, 0, 100, 0, 0, 150)
        assert dz == 50

    def test_latitude_offset_1_degree(self):
        """1° latitude ≈ 111132 m."""
        dx, _, _ = global_to_local_offset(0, 0, 0, 1, 0, 0)
        assert dx == pytest.approx(111132, rel=1e-6)

    def test_longitude_offset_equator(self):
        """No equador, 1° longitude ≈ 111132 m (cos(0)=1)."""
        _, dy, _ = global_to_local_offset(0, 0, 0, 0, 1, 0)
        assert dy == pytest.approx(111132, rel=1e-6)

    def test_longitude_offset_varies_with_latitude(self):
        """Em latitudes maiores, 1° longitude é menor."""
        _, dy_equador, _ = global_to_local_offset(0, 0, 0, 0, 1, 0)
        _, dy_60deg, _ = global_to_local_offset(60, 0, 0, 60, 1, 0)
        # cos(60°) = 0.5, então dy_60deg deve ser ~metade
        assert dy_60deg < dy_equador
        assert dy_60deg == pytest.approx(dy_equador * math.cos(math.radians(60)), rel=1e-6)


class TestHorizontalDistance:
    def test_same_point(self):
        assert horizontal_distance(0, 0, 0, 0) == 0

    def test_3_4_5_triangle(self):
        assert horizontal_distance(0, 0, 3, 4) == pytest.approx(5)

    def test_symmetric(self):
        assert horizontal_distance(1, 2, 5, 8) == horizontal_distance(5, 8, 1, 2)


class TestDistance3D:
    def test_same_point(self):
        assert distance_3d((0, 0, 0), (0, 0, 0)) == 0

    def test_cube_diagonal(self):
        # Diagonal de um cubo 1x1x1 = sqrt(3)
        assert distance_3d((0, 0, 0), (1, 1, 1)) == pytest.approx(math.sqrt(3))

    def test_only_z(self):
        assert distance_3d((0, 0, 0), (0, 0, 5)) == 5
