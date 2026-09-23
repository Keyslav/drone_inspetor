"""Formato, nomes e falhas de persistência de fotografias."""

import re
from pathlib import Path

import cv2


def filename_component(value):
    """Impede que nomes de objetos sejam interpretados como caminhos."""
    return re.sub(r'[^\w.-]+', '_', str(value)).strip('._') or 'objeto'


def save_photo(path, image, quality=95):
    """Salva a imagem e informa falha mesmo quando OpenCV só retorna False."""
    path = Path(path)
    quality = max(1, min(100, int(quality)))
    if path.suffix.lower() in ('.jpg', '.jpeg'):
        params = [cv2.IMWRITE_JPEG_QUALITY, quality]
    elif path.suffix.lower() == '.png':
        params = [cv2.IMWRITE_PNG_COMPRESSION, max(0, 9 - int(quality / 11))]
    else:
        raise ValueError('Formato de fotografia deve ser jpg, jpeg ou png')
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), image, params):
        raise OSError(f'Não foi possível salvar fotografia: {path}')
