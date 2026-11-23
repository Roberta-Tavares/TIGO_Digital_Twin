#!/usr/bin/env python3
"""
yolo_depth_gap.py – Navegação autônoma reativa com fusão YOLO + Profundidade + GAP Following.

Combina a base comprovada de navegação por Gap (gap_phase35.py) com a camada de
percepção semântica do YOLO:
  1. Base reativa: detecção de gaps por perfil de profundidade horizontal (robusto e testado).
  2. Percepção Semântica: YOLO identifica classes de obstáculos na câmera RGB.
  3. Fusão Semântico-Espacial: para cada objeto detectado, a distância métrica Z é extraída
     da matriz de profundidade e uma margem de segurança proporcional à classe é inflacionada
     no perfil de navegação, forçando o GAP a contornar o objeto semântico com folga segura.
  4. Resiliência: se o YOLO ou a câmera RGB não estiverem disponíveis, o nó navega perfeitamente
     com Gap Following métrico puro, sem travar nem parar o robô.

Tópicos ROS 2:
  - Profundidade: /ascamera_hp60c/camera_publisher/depth0/image_raw (sensor_msgs/Image)
  - RGB:          /ascamera_hp60c/camera_publisher/rgb0/image_raw (sensor_msgs/Image)
  - Pose:         /world/{GZ_WORLD_NAME}/pose/info (tf2_msgs/TFMessage)
  - Comandos:     /cmd_vel (geometry_msgs/Twist)
"""

import argparse
import math
import os
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage

# Verificação segura de dependência YOLO
try:
    from ultralytics import YOLO
    ULTRALYTICS_DISPONIVEL = True
except ImportError:
    ULTRALYTICS_DISPONIVEL = False


# ============================================================================
# Detector YOLO Modular
# ============================================================================
class DetectorYOLO:
    def __init__(self, model_name='yolov8n.pt', conf_thresh=0.35, logger=None):
        self.logger = logger
        self.conf_thresh = conf_thresh
        self.modelo = None
        self.ativo = False

        # Margens de segurança adicionais em metros para inflação no perfil
        self.margens_classe = {
            'person': 0.40,
            'chair': 0.25,
            'table': 0.30,
            'couch': 0.35,
            'bottle': 0.15,
            'box': 0.25,
            'default': 0.20
        }

        if ULTRALYTICS_DISPONIVEL:
            try:
                if self.logger:
                    self.logger.info(f"[YOLO] Carregando modelo: {model_name}...")
                self.modelo = YOLO(model_name)
                self.ativo = True
                if self.logger:
                    self.logger.info("[YOLO] Modelo YOLO carregado com sucesso!")
            except Exception as e:
                if self.logger:
                    self.logger.warn(f"[YOLO] Não foi possível carregar '{model_name}': {e}. "
                                     "Operando em modo Gap Following puro.")
        else:
            if self.logger:
                self.logger.warn("[YOLO] 'ultralytics' não encontrado. Para habilitar YOLO: "
                                 "'pip install ultralytics'. Continuando com Gap Following métrico puro.")

    def obter_margem(self, classe):
        return self.margens_classe.get(classe.lower(), self.margens_classe['default'])

    def detectar(self, frame_bgr):
        if not self.ativo or self.modelo is None or frame_bgr is None:
            return []

        try:
            resultados = self.modelo(frame_bgr, verbose=False, conf=self.conf_thresh, imgsz=320)
            deteccoes = []
            for r in resultados:
                for box in r.boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                    conf = float(box.conf[0].cpu().numpy())
                    cls_id = int(box.cls[0].cpu().numpy())
                    cls_name = r.names.get(cls_id, f"obj_{cls_id}")
                    deteccoes.append({
                        'box': (int(x1), int(y1), int(x2), int(y2)),
                        'class': cls_name,
                        'conf': conf,
                        'margin_m': self.obter_margem(cls_name)
                    })
            return deteccoes
        except Exception:
            return []


# ============================================================================
# Nó Principal: YoloDepthGap
# ============================================================================
class YoloDepthGap(Node):
    def __init__(self, model_name='yolov8n.pt', conf_thresh=0.35, show_gui=False):
        super().__init__('yolo_depth_gap')

        self.depth_topic = '/ascamera_hp60c/camera_publisher/depth0/image_raw'
        self.rgb_topic   = '/ascamera_hp60c/camera_publisher/rgb0/image_raw'
        self.cmd_topic   = '/cmd_vel'
        world_name       = os.environ.get('GZ_WORLD_NAME') or 'lacea_world'
        self.show_gui = show_gui
        if self.show_gui and os.environ.get('DISPLAY') is None:
            self.get_logger().warn("[GUI] A variável DISPLAY não está definida. Janela gráfica desabilitada.")
            self.show_gui = False
        elif self.show_gui:
            self.get_logger().info("[GUI] Janela gráfica ativada! Pressione a janela do OpenCV para visualizar.")

        self.detector = DetectorYOLO(model_name=model_name, conf_thresh=conf_thresh, logger=self.get_logger())

        # Thread dedicada para inferência YOLO assíncrona (não bloqueia o controle)
        self.yolo_lock = threading.Lock()
        self.ultimas_deteccoes = []
        self.rgb_para_inferencia = None
        self.novo_rgb_evento = threading.Event()
        self.thread_yolo_ativa = True
        self.thread_yolo = threading.Thread(target=self._loop_inferencia_yolo, daemon=True)
        self.thread_yolo.start()

        # Subscrições
        self.subscription = self.create_subscription(
            Image, self.depth_topic, self.image_callback, 10
        )
        self.rgb_subscription = self.create_subscription(
            Image, self.rgb_topic, self.rgb_callback, 10
        )
        self.pose_subscription = self.create_subscription(
            TFMessage, f'/world/{world_name}/pose/info', self.pose_callback, 10
        )
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)

        # Watchdog
        self.watchdog = self.create_timer(0.1, self.sensor_watchdog)

        # --- PARÂMETROS DE NAVEGAÇÃO COMPATÍVEIS COM GAP_PHASE35 ---
        self.DIST_OBSTACULO       = 0.70   # Profundidade mínima p/ considerar coluna livre (m)
        self.DIST_GATILHO_RECUO   = 0.60   # Distância frontal p/ iniciar manobra de desvio (m)
        self.LARGURA_ROBO_PX      = 40     # Largura mínima do gap em pixels
        self.VX_MAX               = 0.30   # Velocidade linear máxima (m/s)
        self.VX_MIN               = 0.15   # Velocidade linear mínima (m/s)
        self.WZ_VARREDURA         = 0.65   # Velocidade angular de varredura (rad/s)
        self.WZ_GIRO_MAX          = 0.80   # Velocidade angular máxima de alinhamento (rad/s)
        self.TEMPO_RECUO_INICIAL  = 1.5    # Tempo de recuo (s)
        self.MAX_TENTATIVAS_RECUO = 3

        self.W_LARG               = 0.40
        self.W_PROF               = 0.60
        self.FOV_H                = math.radians(87.0)

        # Parâmetros Semânticos
        self.DIST_AVISO_SEMANTICO = 2.5    # Distância máxima de influência semântica (m)
        self.DIST_CRITICA_SEMANTICA = 1.0  # Distância de bloqueio forçado (m)

        # --- ESTADO ---
        self.estado               = 'NAVEGANDO'
        self.tempo_inicio_estado  = time.time()
        self.ultimo_print         = time.time()
        self.ultimo_diagnostico   = 0.0
        self.ultimo_erro_camera   = 0.0
        self.ultimo_erro_sensor   = 0.0
        self.ultimo_watchdog_log  = 0.0
        self.ultima_imagem_valida = time.monotonic()
        self.tentativas_recuo     = 0
        self.yaw_atual            = None
        self.yaw_ultima_varredura = None
        self.progresso_varredura  = 0.0
        self.direcao_varredura    = 1.0
        self.alvo_yaw             = None
        self.melhor_gap_score     = -1.0
        self.larg_ultima_imagem   = 640
        self.ultimo_gui_draw      = 0.0

        # Buffer RGB
        self.latest_rgb_cv        = None
        self.latest_rgb_time      = 0.0
        self.deteccoes_atuais     = []

        self.get_logger().info("=============================================================")
        self.get_logger().info("  TIGO - YOLO + Profundidade + Desvio Semântico (v2.0)       ")
        self.get_logger().info(f"  Dist Obstáculo={self.DIST_OBSTACULO}m | Min Gap={self.LARGURA_ROBO_PX}px")
        self.get_logger().info(f"  GUI: {'Ativada' if self.show_gui else 'Desativada (Modo Terminal)'}")
        self.get_logger().info("=============================================================")

    # ------------------------------------------------------------------
    # Comandos
    # ------------------------------------------------------------------
    def publicar_cmd(self, vx, vy, wz):
        msg = Twist()
        msg.linear.x  = float(vx)
        msg.linear.y  = float(vy)
        msg.angular.z = float(wz)
        if rclpy.ok():
            self.cmd_pub.publish(msg)

    def parar(self):
        self.publicar_cmd(0.0, 0.0, 0.0)

    # ------------------------------------------------------------------
    # Watchdog de Sensor
    # ------------------------------------------------------------------
    def sensor_watchdog(self):
        if time.monotonic() - self.ultima_imagem_valida > 1.2:
            self.parar()
            agora = time.monotonic()
            if agora - self.ultimo_watchdog_log > 2.0:
                self.get_logger().error(
                    "Sem imagem de profundidade recente; robô parado por segurança."
                )
                self.ultimo_watchdog_log = agora

    # ------------------------------------------------------------------
    # Utilitários Angulares
    # ------------------------------------------------------------------
    @staticmethod
    def diferenca_angular(destino, origem):
        return math.atan2(math.sin(destino - origem), math.cos(destino - origem))

    def coluna_para_angulo(self, coluna, larg):
        return (coluna / larg - 0.5) * self.FOV_H

    # ------------------------------------------------------------------
    # Worker Thread de Inferência YOLO Assíncrona
    # ------------------------------------------------------------------
    def _loop_inferencia_yolo(self):
        """Loop de alta velocidade executado em paralelo sem atrasar o controle."""
        while self.thread_yolo_ativa and rclpy.ok():
            if not self.novo_rgb_evento.wait(timeout=0.2):
                continue
            self.novo_rgb_evento.clear()

            with self.yolo_lock:
                frame = None if self.rgb_para_inferencia is None else self.rgb_para_inferencia.copy()

            if frame is None:
                continue

            # Roda inferência rápida (22ms @ 320x320)
            deteccoes = self.detector.detectar(frame)

            with self.yolo_lock:
                self.ultimas_deteccoes = deteccoes

    # ------------------------------------------------------------------
    # Conversão de Imagens ROS (Compatível com NumPy 1.x e 2.x)
    # ------------------------------------------------------------------
    @staticmethod
    def converter_imagem_ros(msg, modo='bgr'):
        """Converte sensor_msgs/Image sem cv_bridge para compatibilidade universal."""
        h, w = msg.height, msg.width
        enc = msg.encoding.lower()

        if modo == 'bgr':
            if enc == 'rgb8':
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 3))
                return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            elif enc == 'bgr8':
                return np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 3))
            elif enc == 'rgba8':
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 4))
                return cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
            elif enc == 'bgra8':
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 4))
                return cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
            elif enc in ['mono8', '8uc1']:
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w))
                return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
            else:
                arr = np.frombuffer(msg.data, dtype=np.uint8)
                if arr.size == h * w * 3:
                    return arr.reshape((h, w, 3))
                elif arr.size == h * w * 4:
                    return cv2.cvtColor(arr.reshape((h, w, 4)), cv2.COLOR_RGBA2BGR)
                return arr.reshape((h, w, -1))
        else: # modo == 'depth'
            if enc in ['32fc1', 'float32', '32f', 'passthrough']:
                return np.frombuffer(msg.data, dtype=np.float32).reshape((h, w))
            elif enc in ['16uc1', 'uint16', 'mono16']:
                arr = np.frombuffer(msg.data, dtype=np.uint16).reshape((h, w))
                return arr.astype(np.float32) / 1000.0
            else:
                try:
                    return np.frombuffer(msg.data, dtype=np.float32).reshape((h, w))
                except Exception:
                    arr = np.frombuffer(msg.data, dtype=np.uint16).reshape((h, w))
                    return arr.astype(np.float32) / 1000.0

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def rgb_callback(self, msg):
        try:
            rgb = self.converter_imagem_ros(msg, modo='bgr')
            self.latest_rgb_cv = rgb
            self.latest_rgb_time = time.time()
            with self.yolo_lock:
                self.rgb_para_inferencia = rgb
            self.novo_rgb_evento.set()
        except Exception:
            pass

    def pose_callback(self, msg):
        tigo_pose = next(
            (t for t in msg.transforms if t.child_frame_id == 'tigo'),
            None,
        )
        if tigo_pose is None:
            return

        q = tigo_pose.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        if self.estado == 'VARRENDO_360' and self.yaw_ultima_varredura is not None:
            delta = self.diferenca_angular(yaw, self.yaw_ultima_varredura)
            self.progresso_varredura += self.direcao_varredura * delta

        self.yaw_atual = yaw
        if self.estado == 'VARRENDO_360':
            self.yaw_ultima_varredura = yaw

    def iniciar_varredura(self, estado, direcao):
        if self.yaw_atual is None:
            return False
        self.estado = estado
        self.direcao_varredura   = direcao
        self.progresso_varredura = 0.0
        self.yaw_ultima_varredura = self.yaw_atual
        self.tempo_inicio_estado = time.time()
        return True

    # ------------------------------------------------------------------
    # Fusão Semântica: Extração de Profundidade dos Bounding Boxes
    # ------------------------------------------------------------------
    def extrair_profundidade_deteccoes(self, deteccoes, depth_array):
        alt, larg = depth_array.shape
        objetos = []

        for d in deteccoes:
            x1, y1, x2, y2 = d['box']
            x1 = max(0, min(larg - 1, x1))
            x2 = max(0, min(larg - 1, x2))
            y1 = max(0, min(alt - 1, y1))
            y2 = max(0, min(alt - 1, y2))

            if x2 <= x1 or y2 <= y1:
                continue

            # Região central do box (evita bordas)
            cx1 = int(x1 + (x2 - x1) * 0.2)
            cx2 = int(x2 - (x2 - x1) * 0.2)
            cy1 = int(y1 + (y2 - y1) * 0.2)
            cy2 = int(y2 - (y2 - y1) * 0.2)

            roi = depth_array[cy1:cy2, cx1:cx2]
            validos = roi[(roi > 0.08) & (roi <= 10.0)]

            if validos.size > 0:
                dist_z = float(np.percentile(validos, 20))
            else:
                roi_full = depth_array[y1:y2, x1:x2]
                val_full = roi_full[(roi_full > 0.08) & (roi_full <= 10.0)]
                dist_z = float(np.percentile(val_full, 20)) if val_full.size > 0 else 999.0

            centro_x = (x1 + x2) / 2.0
            angulo = self.coluna_para_angulo(centro_x, larg)

            objetos.append({
                'box': (x1, y1, x2, y2),
                'class': d['class'],
                'conf': d['conf'],
                'margin_m': d['margin_m'],
                'dist_z': dist_z,
                'angulo': angulo,
                'centro_x': centro_x
            })

        return objetos

    # ------------------------------------------------------------------
    # Inflação Semântica no Perfil de Profundidade
    # ------------------------------------------------------------------
    def aplicar_inflacao_semantica(self, perfil_bruto, objetos, larg):
        perfil_mod = np.copy(perfil_bruto)
        obstaculos_criticos = []

        for obj in objetos:
            z = obj['dist_z']
            if z > self.DIST_AVISO_SEMANTICO or z <= 0.08:
                continue

            x1, y1, x2, y2 = obj['box']
            margem_m = obj['margin_m']

            # Converte margem métrica em pixels na distância do objeto
            delta_px = int((margem_m / max(z * self.FOV_H, 0.1)) * larg)
            c_ini = max(0, x1 - delta_px)
            c_fim = min(larg, x2 + delta_px)

            if z < self.DIST_CRITICA_SEMANTICA:
                # Bloqueio forçado: garante que nenhum gap passe por este obstáculo
                perfil_mod[c_ini:c_fim] = np.minimum(perfil_mod[c_ini:c_fim], 0.20)
                obstaculos_criticos.append(obj)
            else:
                # Penalização virtual: reduz a profundidade para favorecer outros caminhos
                perfil_mod[c_ini:c_fim] = np.minimum(perfil_mod[c_ini:c_fim], z * 0.70)

        return perfil_mod, obstaculos_criticos

    # ------------------------------------------------------------------
    # Detecção e Avaliação de Gaps (Compatível com gap_phase35)
    # ------------------------------------------------------------------
    def detectar_gaps(self, perfil, larg):
        gaps = []
        em_gap = False
        inicio = 0

        for i in range(larg):
            if perfil[i] > self.DIST_OBSTACULO:
                if not em_gap:
                    em_gap = True
                    inicio = i
            elif em_gap:
                em_gap = False
                if (i - inicio) >= self.LARGURA_ROBO_PX:
                    gaps.append((inicio, i, float(np.mean(perfil[inicio:i])), i - inicio))

        if em_gap and (larg - inicio) >= self.LARGURA_ROBO_PX:
            gaps.append((inicio, larg, float(np.mean(perfil[inicio:larg])), larg - inicio))

        return gaps

    def _score_gap(self, largura_px, z_medio, larg_total):
        larg_norm = min(largura_px / larg_total, 1.0)
        prof_norm = min(z_medio / 10.0, 1.0)
        return self.W_LARG * larg_norm + self.W_PROF * prof_norm

    def avaliar_gaps(self, perfil, larg, obstaculos_criticos=None):
        gaps = self.detectar_gaps(perfil, larg)
        if not gaps:
            return 0.0, None

        melhor_gap = None
        melhor_score = -1.0

        for g in gaps:
            g_inicio, g_fim, z_medio, g_larg = g
            score = self._score_gap(g_larg, z_medio, larg)

            # Penalização semântica adicional para gaps alinhados a obstáculos críticos
            if obstaculos_criticos:
                centro_px = (g_inicio + g_fim) / 2.0
                angulo_gap = self.coluna_para_angulo(centro_px, larg)
                for obs in obstaculos_criticos:
                    d_theta = abs(self.diferenca_angular(angulo_gap, obs['angulo']))
                    if d_theta < math.radians(20.0):
                        score -= 0.35 * (1.0 - d_theta / math.radians(20.0))

            if score > melhor_score:
                melhor_score = score
                melhor_gap = g

        return melhor_score, melhor_gap

    def gap_lateral_imediato(self, perfil, larg):
        if self.yaw_atual is None:
            return None

        gaps = self.detectar_gaps(perfil, larg)
        zona_central_inicio = int(larg * 0.30)
        zona_central_fim    = int(larg * 0.70)

        melhor_score = -1.0
        melhor_yaw   = None

        for g_inicio, g_fim, z_medio, g_larg in gaps:
            centro_px = (g_inicio + g_fim) / 2.0
            if centro_px < zona_central_inicio or centro_px > zona_central_fim:
                score = self._score_gap(g_larg, z_medio, larg)
                if score > melhor_score:
                    melhor_score = score
                    angulo_relativo = self.coluna_para_angulo(centro_px, larg)
                    melhor_yaw = self.yaw_atual + angulo_relativo

        return melhor_yaw

    def registrar_gap_varredura(self, perfil, larg):
        if self.yaw_atual is None:
            return

        gaps = self.detectar_gaps(perfil, larg)
        if not gaps:
            return

        for g_inicio, g_fim, z_medio, g_larg in gaps:
            score = self._score_gap(g_larg, z_medio, larg)
            if score > self.melhor_gap_score:
                self.melhor_gap_score = score
                centro_px = (g_inicio + g_fim) / 2.0
                angulo_relativo = self.coluna_para_angulo(centro_px, larg)
                self.alvo_yaw = self.yaw_atual + angulo_relativo

    # ------------------------------------------------------------------
    # Loop Principal (Image Callback)
    # ------------------------------------------------------------------
    def image_callback(self, msg):
        try:
            depth_image = self.converter_imagem_ros(msg, modo='depth')
        except Exception as exc:
            self.parar()
            tempo_atual = time.time()
            if tempo_atual - self.ultimo_erro_camera > 2.0:
                self.get_logger().error(f"Falha ao converter profundidade: {exc}")
                self.ultimo_erro_camera = tempo_atual
            return

        depth_array = np.array(depth_image, dtype=np.float32)
        if depth_array.ndim != 2 or not depth_array.size:
            self.parar()
            return

        self.ultima_imagem_valida = time.monotonic()

        # Conversão mm -> m
        validos = depth_array[np.isfinite(depth_array) & (depth_array > 0.0)]
        if len(validos) > 0 and np.percentile(validos, 90) > 100.0:
            depth_array /= 1000.0

        depth_array[np.isinf(depth_array)] = 10.0
        depth_array[np.isnan(depth_array)] = 0.0

        alt, larg = depth_array.shape
        self.larg_ultima_imagem = larg
        centro_x = larg // 2
        tempo_atual = time.time()

        # 1. Medição frontal central de segurança
        y_inicio = int(alt * 0.25)
        y_fim    = int(alt * 0.75)
        x_inicio = max(0, int(centro_x - larg * 0.20))
        x_fim    = min(larg, int(centro_x + larg * 0.20))
        faixa_frontal = depth_array[y_inicio:y_fim, x_inicio:x_fim]
        frontal_valido = faixa_frontal[(faixa_frontal > 0.08) & (faixa_frontal <= 10.0)]
        dist_frontal = float(np.percentile(frontal_valido, 5)) if frontal_valido.size else None

        # 2. Perfil de profundidade geométrico
        faixa_central = depth_array[int(alt * 0.35):int(alt * 0.65), :]
        coluna_valida = (faixa_central > 0.08) & (faixa_central <= 10.0)
        contagem_colunas = np.sum(coluna_valida, axis=0)
        profundidades_ordenadas = np.sort(np.where(coluna_valida, faixa_central, np.inf), axis=0)
        indices_p10 = np.maximum(np.ceil(contagem_colunas * 0.1).astype(int) - 1, 0)
        perfil = profundidades_ordenadas[indices_p10, np.arange(larg)]
        perfil[contagem_colunas == 0] = 0.0

        # 3. Leitura instantânea e assíncrona das detecções YOLO (zero latência)
        with self.yolo_lock:
            deteccoes_raw = list(self.ultimas_deteccoes)

        objetos = self.extrair_profundidade_deteccoes(deteccoes_raw, depth_array)
        self.deteccoes_atuais = objetos

        # Aplica desvio semântico no perfil
        perfil_semantico, obstaculos_criticos = self.aplicar_inflacao_semantica(perfil, objetos, larg)

        # Diagnóstico periódico no terminal
        if tempo_atual - self.ultimo_diagnostico > 1.0:
            dist_str = f"{dist_frontal:.2f}m" if dist_frontal is not None else "N/A"
            yolo_str = f"YOLO: {len(objetos)} objs" if self.detector.ativo else "YOLO: Desativado"
            self.get_logger().info(
                f"[STATUS] Estado={self.estado} | Frontal P5={dist_str} | {yolo_str}"
            )
            self.ultimo_diagnostico = tempo_atual

        # ==============================================================
        # MÁQUINA DE ESTADOS (TOTALMENTE REATIVA E RESILIENTE)
        # ==============================================================
        if self.estado == 'NAVEGANDO':
            # Se a leitura frontal for None ou extremamente distante, ainda assim navega pelo perfil
            frontal_critico = (dist_frontal is not None and dist_frontal < self.DIST_GATILHO_RECUO)

            if frontal_critico:
                # Obstáculo físico imediato na frente (< 0.60m)
                # Procura se há gap lateral na imagem para desviar diretamente
                gaps_disponiveis = self.detectar_gaps(perfil_semantico, larg)
                gaps_laterais = [
                    g for g in gaps_disponiveis
                    if ((g[0] + g[1]) / 2.0 < larg * 0.35 or (g[0] + g[1]) / 2.0 > larg * 0.65)
                ]

                if gaps_laterais:
                    # Escolhe o gap lateral com melhor score e vira diretamente para ele
                    melhor_lateral = max(gaps_laterais, key=lambda g: self._score_gap(g[3], g[2], larg))
                    centro_lat = (melhor_lateral[0] + melhor_lateral[1]) / 2.0
                    erro_lat = (centro_x - centro_lat) / (larg / 2.0)
                    wz = math.copysign(min(abs(erro_lat) * 1.2, self.WZ_GIRO_MAX), erro_lat)
                    self.publicar_cmd(0.05, 0.0, wz)
                    self.get_logger().warn(
                        f"[DESVIO RÁPIDO] Frontal={dist_frontal:.2f}m. Contornando lateralmente (Wz={wz:.2f}rad/s)."
                    )
                else:
                    # Sem gap lateral visível: recua para abrir ângulo
                    self.estado = 'RECUANDO'
                    self.tempo_inicio_estado = tempo_atual
                    self.tentativas_recuo = 1
                    self.publicar_cmd(-0.16, 0.0, 0.0)
                    self.get_logger().warn(
                        f"[RECUO] Frontal={dist_frontal:.2f}m sem gap lateral visível. Recuando com segurança..."
                    )
                return

            # Navegação normal: o desvio semântico do YOLO já inflacionou os obstáculos no perfil!
            pontuacao, melhor_gap = self.avaliar_gaps(perfil_semantico, larg, obstaculos_criticos)

            if melhor_gap:
                self.tentativas_recuo = 0
                g_inicio, g_fim, z_medio, g_larg = melhor_gap
                g_centro = (g_inicio + g_fim) / 2.0

                erro = (centro_x - g_centro) / (larg / 2.0)
                vx   = max(self.VX_MIN, self.VX_MAX * (1.0 - 0.6 * abs(erro)))
                wz   = 0.65 * erro
                self.publicar_cmd(vx, 0.0, wz)

                if tempo_atual - self.ultimo_print > 0.5:
                    sem_info = f" | YOLO={len(objetos)} objs" if objetos else ""
                    self.get_logger().info(
                        f"[NAVEGANDO] Vx={vx:.2f}m/s | Wz={wz:.2f}rad/s | Gap={g_larg}px | Z={z_medio:.2f}m{sem_info}"
                    )
                    self.ultimo_print = tempo_atual
            else:
                # Sem gap frontal: gira suavemente procurando abertura
                self.publicar_cmd(0.0, 0.0, 0.45)
                if tempo_atual - self.ultimo_print > 1.0:
                    self.get_logger().warn("[PROCURANDO GAP] Girando suavemente para encontrar corredor navegável...")
                    self.ultimo_print = tempo_atual

        elif self.estado == 'RECUANDO':
            duracao_recuo = min(
                self.TEMPO_RECUO_INICIAL + 0.3 * (self.tentativas_recuo - 1),
                2.2,
            )
            if tempo_atual - self.tempo_inicio_estado < duracao_recuo:
                self.publicar_cmd(-0.16, 0.0, 0.0)
            else:
                self.parar()
                self.melhor_gap_score = -1.0
                self.alvo_yaw = None
                # Se pose estiver disponível usa varredura 360, senão gira por tempo
                if not self.iniciar_varredura('VARRENDO_360', 1.0):
                    self.estado = 'VARRENDO_TEMPO'
                    self.tempo_inicio_estado = tempo_atual

        elif self.estado == 'VARRENDO_TEMPO':
            # Escape rotacional por tempo (não depende de tópicos externos de pose)
            self.publicar_cmd(0.0, 0.0, self.WZ_VARREDURA)
            if tempo_atual - self.tempo_inicio_estado > 2.5:
                self.parar()
                self.estado = 'NAVEGANDO'

        elif self.estado == 'VARRENDO_360':
            self.registrar_gap_varredura(perfil_semantico, larg)

            if self.progresso_varredura < 2.0 * math.pi:
                self.publicar_cmd(0.0, 0.0, self.WZ_VARREDURA)
            else:
                self.parar()
                if self.alvo_yaw is None:
                    if self.tentativas_recuo < self.MAX_TENTATIVAS_RECUO:
                        self.tentativas_recuo += 1
                        self.get_logger().warn(
                            f"[SEM GAP] Novo recuo ({self.tentativas_recuo}/{self.MAX_TENTATIVAS_RECUO})..."
                        )
                        self.estado = 'RECUANDO'
                        self.tempo_inicio_estado = tempo_atual
                    else:
                        self.get_logger().error("[SEM GAP] Limite de tentativas atingido. Robô parado.")
                        self.estado = 'BLOQUEADO'
                else:
                    self.get_logger().info(
                        f"Melhor gap encontrado (score={self.melhor_gap_score:.3f}). Alinhando..."
                    )
                    self.estado = 'ALINHANDO_GAP'

        elif self.estado == 'ALINHANDO_GAP':
            if self.alvo_yaw is None or self.yaw_atual is None:
                self.estado = 'NAVEGANDO'
                return

            diferenca = self.diferenca_angular(self.alvo_yaw, self.yaw_atual)

            if abs(diferenca) > 0.05:
                wz = math.copysign(min(abs(diferenca) * 1.5, self.WZ_GIRO_MAX), diferenca)
                self.publicar_cmd(0.0, 0.0, wz)
            else:
                self.parar()
                self.tentativas_recuo = 0
                self.get_logger().info("Alinhamento concluído. Retomando navegação.")
                self.estado = 'NAVEGANDO'

        elif self.estado == 'BLOQUEADO':
            self.parar()

        # Visualização Gráfica Opcional (limitada a 20 FPS para manter fluidez total)
        if self.show_gui and (tempo_atual - self.ultimo_gui_draw > 0.05):
            self.desenhar_gui(depth_array, objetos, dist_frontal)
            self.ultimo_gui_draw = tempo_atual

    def desenhar_gui(self, depth_array, objetos, dist_frontal):
        try:
            alt, larg = depth_array.shape
            if self.latest_rgb_cv is not None:
                canvas = cv2.resize(self.latest_rgb_cv, (larg, alt))
            else:
                d_norm = np.clip(depth_array / 6.0 * 255.0, 0, 255).astype(np.uint8)
                canvas = cv2.applyColorMap(d_norm, cv2.COLORMAP_JET)

            for obj in objetos:
                x1, y1, x2, y2 = obj['box']
                z = obj['dist_z']
                cor = (0, 0, 255) if z < self.DIST_CRITICA_SEMANTICA else (0, 255, 0)
                cv2.rectangle(canvas, (x1, y1), (x2, y2), cor, 2)
                cv2.putText(
                    canvas, f"{obj['class']}: {z:.2f}m",
                    (x1, max(15, y1 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, cor, 1
                )

            status = f"Estado: {self.estado}"
            if dist_frontal:
                status += f" | Frontal: {dist_frontal:.2f}m"
            cv2.putText(canvas, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.imshow("TIGO Semantico", canvas)
            cv2.waitKey(1)
        except Exception as e:
            self.show_gui = False
            self.get_logger().warn(f"[GUI] Erro ao renderizar janela gráfica: {e}. Desativando GUI.")

    def destroy_node(self):
        self.thread_yolo_ativa = False
        self.novo_rgb_evento.set()
        self.parar()
        if self.show_gui:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    parser = argparse.ArgumentParser(description="TIGO - YOLO + Profundidade + GAP")
    parser.add_argument('--model', type=str, default='yolov8n.pt', help='Modelo YOLO')
    parser.add_argument('--conf', type=float, default=0.35, help='Confiança mínima')
    parser.add_argument('--gui', action='store_true', help='Ativa interface gráfica OpenCV')
    cli_args, ros_args = parser.parse_known_args(sys.argv[1:])

    rclpy.init(args=ros_args)
    node = YoloDepthGap(
        model_name=cli_args.model,
        conf_thresh=cli_args.conf,
        show_gui=cli_args.gui
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.parar()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
