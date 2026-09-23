"""Um único proprietário serializa abertura, escrita e fechamento de vídeo."""

from pathlib import Path
from threading import RLock
import math

import cv2


class VideoRecorder:
    """Grava frames BGR; todas as operações do writer ocorrem sob o mesmo lock.

    Sem ``frame_size``, a abertura aguarda o primeiro frame. Mudanças de resolução
    são redimensionadas ao tamanho inicial, pois um arquivo não admite ambos.
    O FPS é a taxa declarada do arquivo: o chamador fornece um frame por amostra.
    """

    def __init__(self, codec, fps, *, writer_factory=None):
        if len(codec) != 4 or not math.isfinite(fps) or fps <= 0:
            raise ValueError('Codec deve ter 4 caracteres e FPS deve ser positivo')
        self.codec = codec
        self.fps = float(fps)
        self._factory = writer_factory or cv2.VideoWriter
        self._lock = RLock()
        self._writer = None
        self._active = False
        self._path = ''
        self._size = None

    @property
    def extension(self):
        """Container compatível com os codecs configuráveis do projeto."""
        return 'avi' if self.codec in ('MJPG', 'XVID') else 'mp4'

    @property
    def active(self):
        """Sessão iniciada, possivelmente aguardando o primeiro frame."""
        with self._lock:
            return self._active

    @property
    def opened(self):
        """Indica abertura efetiva, para publicação de status de gravação."""
        with self._lock:
            return self._writer is not None

    def start(self, path, frame_size=None):
        """Inicia uma sessão; falhas não deixam um writer ou sessão ativos."""
        with self._lock:
            if self._active:
                raise RuntimeError('Gravação já está em andamento')
            if frame_size is not None and (
                len(frame_size) != 2 or any(int(v) != v or v <= 0 for v in frame_size)
            ):
                raise ValueError('Resolução deve conter largura e altura positivas')
            Path(path).parent.mkdir(parents=True, exist_ok=True)
            self._path = str(path)
            self._size = tuple(frame_size) if frame_size is not None else None
            self._active = True
            if self._size is not None:
                self._open()
            return self._path

    def _open(self):
        try:
            writer = self._factory(
                self._path, cv2.VideoWriter_fourcc(*self.codec), self.fps, self._size,
            )
            self._writer = writer
            if not writer.isOpened():
                raise OSError(f'Não foi possível abrir o vídeo: {self._path}')
        except Exception:
            self.close()
            raise

    def write(self, frame):
        """Escreve se a sessão ainda está ativa; nunca escreve após release."""
        with self._lock:
            if not self._active:
                return False
            if frame is None or frame.ndim != 3 or frame.shape[2] != 3 or not frame.size:
                raise ValueError('Frame BGR vazio ou inválido')
            try:
                size = (frame.shape[1], frame.shape[0])
                if self._writer is None:
                    self._size = size
                    self._open()
                if size != self._size:
                    frame = cv2.resize(frame, self._size)
                self._writer.write(frame)
                return True
            except Exception:
                self.close()
                raise

    def close(self):
        """Fecha uma única vez; retorna o último caminho para a resposta ROS."""
        with self._lock:
            writer, self._writer = self._writer, None
            self._active = False
            self._size = None
            if writer is not None:
                writer.release()
            return self._path
