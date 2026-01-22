"""
depth_node.py
=================================================================================================
Nó ROS2 responsável pelo processamento de dados de câmera de profundidade.

Este nó processa imagens de profundidade recebidas de uma câmera de profundidade (ex: RealSense,
Kinect, ou simulador). Realiza análise estatística dos dados, detecta alertas de proximidade,
e gera visualizações processadas para exibição no dashboard.

ARQUITETURA:
- Assina: /drone_inspetor/externo/depth_camera/image_raw (imagens de profundidade externas)
- Publica: /drone_inspetor/interno/depth_node/image_processed (imagens processadas)
- Publica: /drone_inspetor/interno/depth_node/statistics (estatísticas em JSON)
- Publica: /drone_inspetor/interno/depth_node/proximity_alerts (alertas de proximidade em JSON)

FUNCIONALIDADES:
- Processamento de imagens de profundidade (suporta formatos 32FC1 e 16UC1)
- Cálculo de estatísticas (mínimo, máximo, média, mediana, desvio padrão)
- Detecção de alertas de proximidade com níveis (CRÍTICO, ALTO, MÉDIO, BAIXO)
- Visualização em tons de cinza ou mapa de cores
- Análise de distribuição por faixas de distância
=================================================================================================
"""

# ==================== IMPORTAÇÕES =====================================================================
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
import cv2
import numpy as np
import json
from datetime import datetime

class DepthNode(Node):
    """
    Nó ROS2 para processamento de câmera de profundidade.
    
    Este nó processa dados de profundidade recebidos de sensores de profundidade (câmeras
    de profundidade, sensores ToF, etc.). Realiza análise estatística completa dos dados,
    detecta situações de proximidade perigosa, e gera visualizações processadas.
    
    Responsabilidades:
    - Assinar o tópico da câmera de profundidade externa
    - Processar dados de profundidade (normalização, filtragem, análise)
    - Detectar alertas de proximidade com classificação por nível de severidade
    - Calcular estatísticas detalhadas (mínimo, máximo, média, mediana, desvio padrão)
    - Publicar estatísticas e alertas em tópicos padronizados em formato JSON
    - Publicar imagem processada para visualização no dashboard
    - Suportar múltiplos formatos de imagem de profundidade (32FC1, 16UC1)
    """
    
    def __init__(self):
        """
        Inicializa o nó de câmera de profundidade.
        """
        super().__init__("depth_node")
        self.get_logger().info("Nó DepthNode iniciado.")
        
        # ==================== INICIALIZAÇÃO ============================================================
        # Cria instância do CvBridge para conversão entre formatos ROS e OpenCV
        self.bridge = CvBridge()
        
        # ==================== CONFIGURAÇÕES DE PROCESSAMENTO ==========================================
        # Threshold de proximidade: distância mínima em metros para gerar alerta
        # Objetos mais próximos que este valor são considerados perigosos
        self.proximity_threshold = 1.0  # metros
        
        # Range válido de profundidade: valores fora deste range são ignorados
        # Valores muito pequenos podem ser ruído, valores muito grandes podem ser inválidos
        self.min_depth = 0.1           # metros (distância mínima válida)
        self.max_depth = 10.0          # metros (distância máxima válida)
        
        # Modo de visualização: determina como a imagem de profundidade é exibida
        # "grayscale": Tons de cinza (mais rápido, menor uso de memória)
        # "colormap": Mapa de cores (mais intuitivo visualmente)
        self.visualization_mode = "grayscale"
        
        # ==================== DADOS DE ANÁLISE =========================================================
        # Armazena estatísticas calculadas da última imagem processada
        self.depth_statistics = {}
        
        # Armazena lista de alertas de proximidade detectados
        self.proximity_alerts = []
        
        # ==================== CONFIGURAÇÃO DE QoS ========================
        # QoS para dados de sensores (imagens): VOLATILE + BEST_EFFORT (alta frequência, não crítico perder algumas)
        qos_sensor_data = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # ==================== SUBSCRIBERS EXTERNOS (ENTRADA DE DADOS DO DRONE) =========================

        # Imagem de profundidade raw do tópico externo
        self.depth_subscription = self.create_subscription(
            Image,
            "/drone_inspetor/externo/depth_camera/image_raw",
            self.depth_image_callback,
            qos_sensor_data
        )
        self.get_logger().info(f"Assinado tópico externo: {self.depth_subscription.topic_name}")
        
        # ==================== SUBSCRIBERS INTERNOS (COMANDOS DO DASHBOARD) ====================


        # ==================== PUBLISHERS INTERNOS (SAÍDA DE DADOS PARA O DASHBOARD) ====================

        # Imagem de profundidade processada (para visualização) no tópico padronizado
        self.processed_depth_publisher = self.create_publisher(
            Image,
            "/drone_inspetor/interno/depth_node/image_processed",
            qos_sensor_data
        )
        self.get_logger().info(f"Publicando no tópico: {self.processed_depth_publisher.topic_name}")
        
        # Estatísticas de profundidade no tópico padronizado
        self.depth_stats_publisher = self.create_publisher(
            String,
            "/drone_inspetor/interno/depth_node/statistics",
            qos_sensor_data
        )
        self.get_logger().info(f"Publicando no tópico: {self.depth_stats_publisher.topic_name}")
        
        # Alertas de proximidade no tópico padronizado
        self.proximity_alerts_publisher = self.create_publisher(
            String,
            "/drone_inspetor/interno/depth_node/proximity_alerts",
            qos_sensor_data
        )
        self.get_logger().info(f"Publicando no tópico: {self.proximity_alerts_publisher.topic_name}")
        
        # ==================== TIMER PARA RELATÓRIOS ====================
        # Timer para publicar estatísticas periódicas
        self.stats_timer = self.create_timer(2.0, self.publish_depth_statistics)
        
        self.get_logger().info("DepthNode inicializado com sucesso.")
    

    # ==================== CALLBACKS ============================================================

    def depth_image_callback(self, msg):
        """
        Callback para processar imagens de profundidade.
        
        Args:
            msg: Mensagem sensor_msgs/Image com dados de profundidade
        """
        self.get_logger().debug("Processando imagem de profundidade")
        
        try:
            # ==================== CONVERSÃO PARA OPENCV ====================
            # Depth images podem ser float32 ou uint16
            match msg.encoding:
                case "32FC1":
                    depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
                case "16UC1":
                    depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
                    # Converte para metros se necessário (assumindo mm)
                    depth_image = depth_image.astype(np.float32) / 1000.0
                case _:
                    depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                    depth_image = depth_image.astype(np.float32)
            
            self.get_logger().debug(f"Imagem de profundidade convertida: {depth_image.shape}, dtype: {depth_image.dtype}")
            
            # ==================== PROCESSAMENTO DE PROFUNDIDADE ====================
            processed_image, statistics, alerts = self.process_depth_image(depth_image)
            
            # ==================== PUBLICAÇÃO DE IMAGEM PROCESSADA ====================
            if processed_image is not None:
                try:
                    # Converte para formato adequado para publicação
                    if len(processed_image.shape) == 2:
                        processed_msg = self.bridge.cv2_to_imgmsg(processed_image, "mono8")
                    else:
                        processed_msg = self.bridge.cv2_to_imgmsg(processed_image, "bgr8")
                    
                    self.processed_depth_publisher.publish(processed_msg)
                    
                    self.get_logger().debug("Imagem de profundidade processada publicada")
                except Exception as e:
                    self.get_logger().error(f"Erro ao publicar imagem de profundidade: {e}")
            
            # ==================== EMISSÃO DE DADOS ====================
            if statistics:
                self.depth_statistics = statistics
            
            if alerts:
                self.proximity_alerts = alerts
                self.publish_proximity_alerts(alerts)
                
        except Exception as e:
            self.get_logger().error(f"Erro no processamento de profundidade: {e}")
    

    def process_depth_image(self, depth_image):
        """
        Processa imagem de profundidade para análise e visualização.
        
        Args:
            depth_image: Array numpy com dados de profundidade
            
        Returns:
            tuple: (imagem_processada, estatisticas, alertas)
        """
        try:
            # ==================== LIMPEZA DE DADOS ====================
            # Remove valores inválidos (NaN, inf, negativos)
            valid_mask = np.isfinite(depth_image) & (depth_image > 0)
            clean_depth = np.where(valid_mask, depth_image, 0)
            
            # Aplica range de profundidade
            range_mask = (clean_depth >= self.min_depth) & (clean_depth <= self.max_depth)
            filtered_depth = np.where(range_mask, clean_depth, 0)
            
            # ==================== CÁLCULO DE ESTATÍSTICAS ====================
            statistics = self.calculate_depth_statistics(filtered_depth, valid_mask)
            
            # ==================== DETECÇÃO DE ALERTAS ====================
            alerts = self.detect_proximity_alerts(filtered_depth)
            
            # ==================== GERAÇÃO DE VISUALIZAÇÃO ====================
            visualization = self.create_depth_visualization(filtered_depth, alerts)
            
            return visualization, statistics, alerts
            
        except Exception as e:
            self.get_logger().error(f"Erro no processamento de profundidade: {e}")
            return None, None, []
    
    def calculate_depth_statistics(self, depth_image, valid_mask):
        """
        Calcula estatísticas dos dados de profundidade.
        
        Args:
            depth_image: Array numpy com dados de profundidade
            valid_mask: Máscara de pixels válidos
            
        Returns:
            dict: Estatísticas calculadas
        """
        try:
            valid_depths = depth_image[valid_mask & (depth_image > 0)]
            
            if len(valid_depths) == 0:
                return {
                    "timestamp": datetime.now().strftime("%H:%M:%S"),
                    "error": "Nenhum pixel de profundidade válido"
                }
            
            statistics = {
                "timestamp": datetime.now().strftime("%H:%M:%S"),
                "total_pixels": depth_image.size,
                "valid_pixels": len(valid_depths),
                "valid_percentage": (len(valid_depths) / depth_image.size) * 100,
                "min_distance": float(np.min(valid_depths)),
                "max_distance": float(np.max(valid_depths)),
                "mean_distance": float(np.mean(valid_depths)),
                "median_distance": float(np.median(valid_depths)),
                "std_distance": float(np.std(valid_depths)),
                "range_meters": float(np.max(valid_depths) - np.min(valid_depths))
            }
            
            # Análise de distribuição por faixas
            close_pixels = np.sum((valid_depths > 0) & (valid_depths <= 1.0))
            medium_pixels = np.sum((valid_depths > 1.0) & (valid_depths <= 5.0))
            far_pixels = np.sum(valid_depths > 5.0)
            
            statistics.update({
                "close_pixels": int(close_pixels),
                "medium_pixels": int(medium_pixels),
                "far_pixels": int(far_pixels),
                "close_percentage": (close_pixels / len(valid_depths)) * 100,
                "medium_percentage": (medium_pixels / len(valid_depths)) * 100,
                "far_percentage": (far_pixels / len(valid_depths)) * 100
            })
            
            return statistics
            
        except Exception as e:
            self.get_logger().error(f"Erro no cálculo de estatísticas: {e}")
            return {"error": str(e)}
    
    def detect_proximity_alerts(self, depth_image):
        """
        Detecta alertas de proximidade baseados no threshold.
        
        Args:
            depth_image: Array numpy com dados de profundidade
            
        Returns:
            list: Lista de alertas
        """
        try:
            alerts = []
            
            # Encontra pixels próximos
            close_mask = (depth_image > 0) & (depth_image <= self.proximity_threshold)
            close_pixels = np.sum(close_mask)
            
            if close_pixels > 0:
                # Calcula porcentagem da imagem
                total_valid = np.sum(depth_image > 0)
                if total_valid > 0:
                    close_percentage = (close_pixels / total_valid) * 100
                    
                    # Encontra distância mínima
                    min_distance = float(np.min(depth_image[close_mask]))
                    
                    # Determina nível de alerta
                    alert_level = "BAIXO"
                    if close_percentage > 50:
                        alert_level = "CRÍTICO"
                    
                    if 25 < close_percentage <= 50:
                        alert_level = "ALTO"
                    
                    if 10 < close_percentage <= 25:
                        alert_level = "MÉDIO"
                    
                    alert = {
                        "id": f"proximity_{datetime.now().strftime("%H%M%S")}",
                        "type": "proximity",
                        "level": alert_level,
                        "min_distance": min_distance,
                        "affected_pixels": int(close_pixels),
                        "affected_percentage": round(close_percentage, 1),
                        "threshold": self.proximity_threshold,
                        "timestamp": datetime.now().strftime("%H:%M:%S")
                    }
                    alerts.append(alert)
            
            return alerts
            
        except Exception as e:
            self.get_logger().error(f"Erro na detecção de alertas: {e}")
            return []
    
    def create_depth_visualization(self, depth_image, alerts):
        """
        Cria visualização da imagem de profundidade.
        
        Args:
            depth_image: Array numpy com dados de profundidade
            alerts: Lista de alertas
            
        Returns:
            numpy.ndarray: Imagem de visualização
        """
        try:
            match self.visualization_mode:
                case "grayscale":
                    return self.create_grayscale_visualization(depth_image, alerts)
                case "colormap":
                    return self.create_colormap_visualization(depth_image, alerts)
                case _:
                    # Fallback para grayscale
                    return self.create_grayscale_visualization(depth_image, alerts)
                
        except Exception as e:
            self.get_logger().error(f"Erro na criação de visualização: {e}")
            return None
    
    def create_grayscale_visualization(self, depth_image, alerts):
        """
        Cria visualização em tons de cinza.
        
        Args:
            depth_image: Array numpy com dados de profundidade
            alerts: Lista de alertas
            
        Returns:
            numpy.ndarray: Imagem em tons de cinza
        """
        # Normaliza para 0-255
        valid_mask = depth_image > 0
        if np.any(valid_mask):
            min_val = np.min(depth_image[valid_mask])
            max_val = np.max(depth_image[valid_mask])
            
            if max_val > min_val:
                normalized = np.zeros_like(depth_image, dtype=np.uint8)
                normalized[valid_mask] = ((depth_image[valid_mask] - min_val) / (max_val - min_val) * 255).astype(np.uint8)
            else:
                normalized = np.zeros_like(depth_image, dtype=np.uint8)
        else:
            normalized = np.zeros_like(depth_image, dtype=np.uint8)
        
        # Converte para BGR para anotações
        visualization = cv2.cvtColor(normalized, cv2.COLOR_GRAY2BGR)
        
        # Adiciona anotações
        visualization = self.add_depth_annotations(visualization, depth_image, alerts)
        
        return visualization
    
    def create_colormap_visualization(self, depth_image, alerts):
        """
        Cria visualização com mapa de cores.
        
        Args:
            depth_image: Array numpy com dados de profundidade
            alerts: Lista de alertas
            
        Returns:
            numpy.ndarray: Imagem colorida
        """
        # Normaliza para 0-255
        valid_mask = depth_image > 0
        if np.any(valid_mask):
            min_val = np.min(depth_image[valid_mask])
            max_val = np.max(depth_image[valid_mask])
            
            if max_val > min_val:
                normalized = np.zeros_like(depth_image, dtype=np.uint8)
                normalized[valid_mask] = ((depth_image[valid_mask] - min_val) / (max_val - min_val) * 255).astype(np.uint8)
            else:
                normalized = np.zeros_like(depth_image, dtype=np.uint8)
        else:
            normalized = np.zeros_like(depth_image, dtype=np.uint8)
        
        # Aplica colormap
        visualization = cv2.applyColorMap(normalized, cv2.COLORMAP_JET)
        
        # Pixels inválidos ficam pretos
        visualization[~valid_mask] = [0, 0, 0]
        
        # Adiciona anotações
        visualization = self.add_depth_annotations(visualization, depth_image, alerts)
        
        return visualization
    
    def add_depth_annotations(self, image, depth_data, alerts):
        """
        Adiciona anotações à imagem de profundidade.
        
        Args:
            image: Imagem base
            depth_data: Dados de profundidade
            alerts: Lista de alertas
            
        Returns:
            numpy.ndarray: Imagem anotada
        """
        annotated = image.copy()
        height, width = annotated.shape[:2]
        
        # ==================== INFORMAÇÕES BÁSICAS ====================
        info_y = 30
        info_color = (255, 255, 255)
        
        # Estatísticas básicas
        valid_depths = depth_data[depth_data > 0]
        if len(valid_depths) > 0:
            min_dist = np.min(valid_depths)
            max_dist = np.max(valid_depths)
            mean_dist = np.mean(valid_depths)
            
            cv2.putText(annotated, f"Min: {min_dist:.2f}m", (10, info_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, info_color, 2)
            cv2.putText(annotated, f"Max: {max_dist:.2f}m", (10, info_y + 25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, info_color, 2)
            cv2.putText(annotated, f"Media: {mean_dist:.2f}m", (10, info_y + 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, info_color, 2)
        
        # ==================== ALERTAS DE PROXIMIDADE ====================
        if alerts:
            alert_y = height - 80
            for alert in alerts:
                level = alert["level"]
                min_distance = alert["min_distance"]
                
                # Cor baseada no nível
                alert_color = (255, 255, 0)  # Ciano (padrão)
                if level == "CRÍTICO":
                    alert_color = (0, 0, 255)  # Vermelho
                
                if level == "ALTO":
                    alert_color = (0, 165, 255)  # Laranja
                
                if level == "MÉDIO":
                    alert_color = (0, 255, 255)  # Amarelo
                
                cv2.putText(annotated, f"ALERTA {level}: {min_distance:.2f}m", 
                           (10, alert_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, alert_color, 2)
                alert_y += 25
        
        # ==================== LINHA DE THRESHOLD ====================
        # Adiciona indicador visual do threshold
        threshold_text = f"Threshold: {self.proximity_threshold:.1f}m"
        cv2.putText(annotated, threshold_text, (width - 200, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        # ==================== TIMESTAMP ====================
        timestamp = datetime.now().strftime("%H:%M:%S")
        cv2.putText(annotated, f"Depth - {timestamp}", (width - 200, height - 20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        return annotated
    
    def publish_proximity_alerts(self, alerts):
        """
        Publica alertas de proximidade via ROS2.
        
        Args:
            alerts: Lista de alertas
        """
        try:
            alert_msg = String()
            alert_msg.data = json.dumps(alerts)
            self.proximity_alerts_publisher.publish(alert_msg)
            
            self.get_logger().debug("Alertas de proximidade publicados")
            
        except Exception as e:
            self.get_logger().error(f"Erro ao publicar alertas: {e}")
    
    def publish_depth_statistics(self):
        """
        Publica estatísticas de profundidade periodicamente.
        """
        try:
            if self.depth_statistics:
                stats_msg = String()
                stats_msg.data = json.dumps(self.depth_statistics)
                self.depth_stats_publisher.publish(stats_msg)
                
                self.get_logger().debug("Estatísticas de profundidade publicadas")
                
        except Exception as e:
            self.get_logger().error(f"Erro ao publicar estatísticas: {e}")
    
    def set_proximity_threshold(self, threshold):
        """
        Define threshold de proximidade.
        
        Args:
            threshold: Novo threshold em metros
        """
        if threshold > 0:
            self.proximity_threshold = threshold
            self.get_logger().info(f"Threshold de proximidade alterado para: {threshold}m")
        else:
            self.get_logger().warn("Threshold deve ser positivo")
    
    def set_depth_range(self, min_depth, max_depth):
        """
        Define range de profundidade válido.
        
        Args:
            min_depth: Profundidade mínima
            max_depth: Profundidade máxima
        """
        if min_depth < max_depth and min_depth >= 0:
            self.min_depth = min_depth
            self.max_depth = max_depth
            self.get_logger().info(f"Range de profundidade alterado: {min_depth}-{max_depth}m")
        else:
            self.get_logger().warn("Range de profundidade inválido")
    
    def set_visualization_mode(self, mode):
        """
        Define modo de visualização.
        
        Args:
            mode: "grayscale" ou "colormap"
        """
        valid_modes = ["grayscale", "colormap"]
        if mode in valid_modes:
            self.visualization_mode = mode
            self.get_logger().info(f"Modo de visualização alterado para: {mode}")
        else:
            self.get_logger().warn(f"Modo de visualização inválido: {mode}")
    
    def toggle_visualization_mode(self):
        """
        Alterna entre modos de visualização.
        """
        if self.visualization_mode == "grayscale":
            self.set_visualization_mode("colormap")
        else:
            self.set_visualization_mode("grayscale")
    
    def reset_depth_data(self):
        """
        Reseta dados de profundidade.
        """
        self.depth_statistics = {}
        self.proximity_alerts = []
        self.get_logger().info("Dados de profundidade resetados")

def main(args=None):
    rclpy.init(args=args)
    depth_node = DepthNode()
    rclpy.spin(depth_node)
    depth_node.destroy_node()
    rclpy.try_shutdown()

if __name__ == "__main__":
    main()


