"""Geometria pinhole da câmera nivelada para distâncias horizontais FLU.

A imagem fornece profundidade no eixo óptico, não distância radial. A projeção
converte cada pixel em ponto 3D, seleciona a faixa de altura do drone e conserva
o retorno mais próximo por setor. Sem medição válida, o setor permanece NaN.
"""

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class DepthCalibration:
    """Intrínsecos da resolução calibrada ou HFOV explícito em graus.

    Na opção HFOV, pixels quadrados (fx=fy) são uma hipótese da calibração.
    A câmera deve estar nivelada: montagem com pitch/roll exige retificação
    antes deste componente. Yaw gira o campo observado no plano horizontal.
    """

    horizontal_fov_deg: float = 0.0
    fx: float = 0.0
    fy: float = 0.0
    cx: float = -1.0
    cy: float = -1.0
    width: int = 0
    height: int = 0
    mount_yaw_deg: float = 0.0
    camera_height_m: float = 0.0
    band_half_height_m: float = 0.35
    bins: int = 181
    min_depth: float = 0.1
    max_depth: float = 10.0

    def __post_init__(self):
        values = (self.horizontal_fov_deg, self.fx, self.fy, self.cx, self.cy,
                  self.mount_yaw_deg, self.camera_height_m, self.band_half_height_m,
                  self.min_depth, self.max_depth)
        if not all(math.isfinite(value) for value in values):
            raise ValueError('Calibração depth deve conter valores finitos')
        if not 0 < self.min_depth < self.max_depth or self.band_half_height_m <= 0:
            raise ValueError('Alcance e faixa vertical devem ser positivos')
        if self.bins < 2 or int(self.bins) != self.bins:
            raise ValueError('Scan depth deve conter pelo menos dois setores')
        if not 0 <= self.horizontal_fov_deg < 180:
            raise ValueError('HFOV deve estar entre 0 (não calibrado) e 180 graus')
        if self.fx < 0 or self.fy < 0:
            raise ValueError('Focais não podem ser negativas')
        if (self.fx > 0) != (self.fy > 0):
            raise ValueError('fx e fy devem ser configurados juntos')
        if self.fx > 0 and (self.width <= 1 or self.height <= 1):
            raise ValueError('Intrínsecos exigem resolução de calibração')

    @property
    def calibrated(self):
        """Não deduz campo de visão a partir de um sensor desconhecido."""
        return self.horizontal_fov_deg > 0 or (self.fx > 0 and self.fy > 0)

    def intrinsics(self, width, height):
        """Escala intrínsecos mantendo centros dos pixels ao mudar resolução."""
        if width < 2 or height < 1:
            raise ValueError('Imagem depth precisa de pelo menos duas colunas')
        if self.fx > 0:
            sx, sy = width / self.width, height / self.height
            cx = self.cx if self.cx >= 0 else (self.width - 1) / 2
            cy = self.cy if self.cy >= 0 else (self.height - 1) / 2
            if not 0 <= cx < self.width or not 0 <= cy < self.height:
                raise ValueError('Centro óptico deve pertencer à imagem calibrada')
            return self.fx * sx, self.fy * sy, (cx + .5) * sx - .5, (cy + .5) * sy - .5
        if self.horizontal_fov_deg > 0:
            focal = width / (2 * math.tan(math.radians(self.horizontal_fov_deg) / 2))
            return focal, focal, (width - 1) / 2, (height - 1) / 2
        raise ValueError('Scan depth requer intrínsecos ou HFOV calibrado')


@dataclass(frozen=True)
class DepthScan:
    """Scan uniforme: ângulos CCW/FLU e ranges radiais em metros."""

    ranges: tuple
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float

    @property
    def angle_max(self):
        return self.angle_min + (len(self.ranges) - 1) * self.angle_increment


def depth_in_meters(image, encoding):
    """Converte somente encodings com unidades conhecidas; não adivinha escala."""
    array = np.asarray(image)
    if array.ndim != 2 or array.size == 0:
        raise ValueError('Imagem depth deve ser uma matriz não vazia')
    if encoding == '32FC1':
        return array.astype(np.float32, copy=False)
    if encoding == '16UC1':
        return array.astype(np.float32) * 0.001
    raise ValueError(f'Encoding depth não suportado: {encoding}')


def project_depth_scan(depth_m, calibration):
    """Produz hits conservadores, sem extrapolar setores ou interpretar Inf como livre."""
    depth = np.asarray(depth_m, dtype=np.float64)
    if depth.ndim != 2 or not depth.size:
        raise ValueError('Imagem depth inválida')
    height, width = depth.shape
    fx, fy, cx, cy = calibration.intrinsics(width, height)
    horizontal = (np.arange(width) - cx) / fx
    vertical = (np.arange(height) - cy) / fy
    # Optical x=direita/y=baixo/z=frente -> FLU x=frente/y=esquerda/z=cima.
    bearings = math.radians(calibration.mount_yaw_deg) - np.arctan(horizontal)
    angle_min, angle_max = float(bearings[-1]), float(bearings[0])
    step = (angle_max - angle_min) / (calibration.bins - 1)
    bin_index = np.rint((bearings - angle_min) / step).astype(int)
    measured = np.isfinite(depth) & (depth > 0) & (depth <= calibration.max_depth)
    # Um retorno positivo abaixo do mínimo representa proximidade saturada;
    # conservá-lo em range_min evita descartá-lo como caminho livre.
    bounded_depth = np.where(measured, np.maximum(depth, calibration.min_depth), np.nan)
    z_flu = calibration.camera_height_m - bounded_depth * vertical[:, None]
    in_band = np.abs(z_flu) <= calibration.band_half_height_m
    radial = bounded_depth * np.sqrt(1.0 + horizontal[None, :] ** 2)
    radial = np.where(measured & (depth < calibration.min_depth), calibration.min_depth, radial)
    radial = np.where(measured & in_band, radial, np.inf)
    column_min = radial.min(axis=0)
    output = np.full(calibration.bins, np.inf)
    np.minimum.at(output, bin_index, column_min)
    output[~np.isfinite(output)] = np.nan
    max_radial = calibration.max_depth * math.sqrt(1 + float(np.max(horizontal ** 2)))
    return DepthScan(tuple(float(value) for value in output), angle_min, step,
                     calibration.min_depth, max_radial + 1e-6)
