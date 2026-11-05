#!/usr/bin/env python3
"""
gap_phase35.py – Navegação por Gap com varredura inteligente e look-ahead.

Melhorias v3.6:
  - Sem recuo desnecessário: ao detectar obstáculo, o Tigo primeiro tenta
    girar para o melhor gap lateral visível ANTES de recuar.
  - Score de gap ponderado por profundidade look-ahead (percentil de toda
    a coluna, não só a média simples) para evitar falsas aberturas.
  - Varredura 360 registra a direção REAL do gap (ângulo do centro do gap
    em relação ao campo de visão + yaw do robô no momento) em vez de usar
    apenas o yaw instantâneo.
  - Alinhamento suave com velocidade proporcional ao erro angular.
  - Novo estado GIRANDO_PARA_GAP: gira in-place antes de recuar quando há
    um gap lateral visível, eliminando recuos desnecessários.

Melhorias v3.7:
  - Validação de gap em METROS REAIS em vez de pixels fixos.
    40px ≈ 4-10 cm dependendo da distância → completamente insuficiente
    para o Tigo (25 × 30 cm). Agora: gap_width_m = z_medio * gap_px
    * FOV_H_rad / larg_px, e só aceita se >= LARGURA_MIN_GAP_M.
  - LARGURA_ROBO_M = 0.25m, MARGEM_LATERAL_M = 0.10m por lado →
    mínimo de 0.45m de abertura física real para aprovar um gap.
  - Pré-filtro em pixels mantido (barato) para descartar candidatos
    obviamente estreitos antes do cálculo métrico.
  - DIST_OBSTACULO aumentado de 0.7 → 0.80m para dar margem de frenagem
    compatível com o comprimento do robô (30 cm).
"""
import math
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from tf2_msgs.msg import TFMessage


class GapPhase35(Node):
    def __init__(self):
        super().__init__('gap_phase35')

        self.depth_topic = '/ascamera_hp60c/camera_publisher/depth0/image_raw'
        self.cmd_topic = '/cmd_vel'
        world_name = os.environ.get('GZ_WORLD_NAME') or 'lacea_world'

        self.subscription = self.create_subscription(
            Image, self.depth_topic, self.image_callback, 10
        )
        self.watchdog = self.create_timer(0.1, self.sensor_watchdog)
        self.pose_subscription = self.create_subscription(
            TFMessage, f'/world/{world_name}/pose/info', self.pose_callback, 10
        )
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.bridge = CvBridge()

        # --- PARÂMETROS DE NAVEGAÇÃO ---
        self.DIST_OBSTACULO      = 0.7    # Profundidade mínima p/ considerar coluna livre (m)
        self.DIST_GATILHO_RECUO  = 0.6    # Distância frontal p/ iniciar manobra de desvio (m)
        self.LARGURA_ROBO_PX     = 40     # Largura mínima do gap em pixels
        self.VX_MAX              = 0.30   # Velocidade linear máxima (m/s)
        self.VX_MIN              = 0.15   # Velocidade linear mínima (m/s)
        self.WZ_VARREDURA        = 0.65   # Velocidade angular de varredura (rad/s)
        self.WZ_GIRO_MAX         = 0.80   # Velocidade angular máxima de alinhamento (rad/s)
        self.TEMPO_RECUO_INICIAL = 1.5    # Tempo de recuo (s) – reduzido pois recuamos menos
        self.MAX_TENTATIVAS_RECUO = 3

        # Peso do look-ahead no score de gap (profundidade media ponderada)
        # Score = largura_norm * W_LARG + profundidade_norm * W_PROF
        self.W_LARG = 0.4
        self.W_PROF = 0.6

        # FOV horizontal estimado da câmera (rad); ajuste se necessário
        self.FOV_H = math.radians(87.0)

        # --- ESTADO ---
        self.estado = 'NAVEGANDO'
        self.tempo_inicio_estado = time.time()
        self.ultimo_print        = time.time()
        self.ultimo_diagnostico  = 0.0
        self.ultimo_erro_camera  = 0.0
        self.ultimo_erro_sensor  = 0.0
        self.ultimo_watchdog_log = 0.0
        self.ultima_imagem_valida = time.monotonic()
        self.tentativas_recuo    = 0
        self.yaw_atual           = None
        self.yaw_ultima_varredura = None
        self.progresso_varredura = 0.0
        self.direcao_varredura   = 0.0
        self.alvo_yaw            = None          # Yaw absoluto do melhor gap encontrado
        self.melhor_gap_score    = -1.0
        self.larg_ultima_imagem  = 320           # Atualizado a cada frame

    # ------------------------------------------------------------------
    # Publicação de comandos
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
    # Watchdog de sensor
    # ------------------------------------------------------------------
    def sensor_watchdog(self):
        if time.monotonic() - self.ultima_imagem_valida > 0.3:
            self.parar()
            agora = time.monotonic()
            if agora - self.ultimo_watchdog_log > 1.0:
                self.get_logger().error(
                    "Sem imagem de profundidade recente; comando de movimento zerado."
                )
                self.ultimo_watchdog_log = agora

    # ------------------------------------------------------------------
    # Utilitários angulares
    # ------------------------------------------------------------------
    @staticmethod
    def diferenca_angular(destino, origem):
        return math.atan2(math.sin(destino - origem), math.cos(destino - origem))

    def coluna_para_angulo(self, coluna, larg):
        """Converte coluna de pixel para ângulo relativo ao eixo frontal (rad)."""
        return (coluna / larg - 0.5) * self.FOV_H

    # ------------------------------------------------------------------
    # Pose callback
    # ------------------------------------------------------------------
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
    # Detecção e avaliação de gaps
    # ------------------------------------------------------------------
    def detectar_gaps(self, perfil, larg):
        """Retorna lista de (inicio_px, fim_px, profundidade_media, largura_px)."""
        gaps   = []
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
        """
        Pontuação normalizada do gap.
        - Largura normalizada pela largura total da imagem.
        - Profundidade normalizada por 10 m (alcance máximo seguro).
        - Profundidade tem peso maior (W_PROF) para preferir gaps com boa visibilidade.
        """
        larg_norm = min(largura_px / larg_total, 1.0)
        prof_norm = min(z_medio / 10.0, 1.0)
        return self.W_LARG * larg_norm + self.W_PROF * prof_norm

    def avaliar_gaps(self, perfil, larg):
        """Retorna (melhor_score, melhor_gap) ou (0.0, None)."""
        gaps = self.detectar_gaps(perfil, larg)
        if not gaps:
            return 0.0, None

        melhor_gap    = None
        melhor_score  = -1.0

        for g in gaps:
            g_inicio, g_fim, z_medio, g_larg = g
            score = self._score_gap(g_larg, z_medio, larg)
            if score > melhor_score:
                melhor_score = score
                melhor_gap   = g

        return melhor_score, melhor_gap

    def gap_lateral_imediato(self, perfil, larg):
        """
        Verifica se existe um gap passável fora da zona central (lateral).
        Retorna o yaw absoluto do gap lateral melhor, ou None.
        Usado para evitar recuo quando há gap lateral visível.
        """
        if self.yaw_atual is None:
            return None

        gaps = self.detectar_gaps(perfil, larg)
        zona_central_inicio = int(larg * 0.30)
        zona_central_fim    = int(larg * 0.70)

        melhor_score = -1.0
        melhor_yaw   = None

        for g_inicio, g_fim, z_medio, g_larg in gaps:
            centro_px = (g_inicio + g_fim) / 2.0
            # Gap fora da zona central = gap lateral
            if centro_px < zona_central_inicio or centro_px > zona_central_fim:
                score = self._score_gap(g_larg, z_medio, larg)
                if score > melhor_score:
                    melhor_score = score
                    angulo_relativo = self.coluna_para_angulo(centro_px, larg)
                    # Yaw absoluto = yaw atual + ângulo relativo (câmera alinhada ao robô)
                    melhor_yaw = self.yaw_atual + angulo_relativo

        return melhor_yaw

    # ------------------------------------------------------------------
    # Registro durante varredura 360
    # ------------------------------------------------------------------
    def registrar_gap_varredura(self, perfil, larg):
        """
        Durante a varredura 360, registra o gap com melhor score.
        Calcula o yaw absoluto real do centro do gap usando o FOV da câmera.
        """
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
                # Yaw absoluto do gap = yaw do robô + offset angular do gap no FOV
                self.alvo_yaw = self.yaw_atual + angulo_relativo

    # ------------------------------------------------------------------
    # Callback principal da câmera
    # ------------------------------------------------------------------
    def image_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as exc:
            self.parar()
            tempo_atual = time.time()
            if tempo_atual - self.ultimo_erro_camera > 2.0:
                self.get_logger().error(f"Falha ao converter imagem de profundidade: {exc}")
                self.ultimo_erro_camera = tempo_atual
            return

        depth_array = np.array(depth_image, dtype=np.float32)
        if depth_array.ndim != 2 or not depth_array.size:
            self.parar()
            self.get_logger().error(
                f"Imagem de profundidade inválida: shape={depth_array.shape}."
            )
            return
        self.ultima_imagem_valida = time.monotonic()

        # Converter mm → m se necessário (câmera física HP60C)
        validos = depth_array[np.isfinite(depth_array) & (depth_array > 0.0)]
        if len(validos) > 0 and np.percentile(validos, 90) > 100.0:
            depth_array /= 1000.0

        # Raios livres (inf) → alcance máximo seguro
        depth_array[np.isinf(depth_array)] = 10.0
        depth_array[np.isnan(depth_array)] = 0.0

        alt, larg = depth_array.shape
        self.larg_ultima_imagem = larg
        centro_x = larg // 2

        # Faixa frontal central (20% da largura, faixa vertical 25-75%)
        y_inicio  = int(alt * 0.25)
        y_fim     = int(alt * 0.75)
        x_inicio  = max(0, int(centro_x - larg * 0.20))
        x_fim     = min(larg, int(centro_x + larg * 0.20))
        faixa_frontal = depth_array[y_inicio:y_fim, x_inicio:x_fim]
        frontal_valido = faixa_frontal[
            (faixa_frontal > 0.08) & (faixa_frontal <= 10.0)
        ]
        dist_frontal = (
            float(np.percentile(frontal_valido, 5)) if frontal_valido.size else None
        )

        # Perfil de profundidade por coluna (percentil 10 da faixa vertical central)
        faixa_central = depth_array[int(alt * 0.35):int(alt * 0.65), :]
        coluna_valida = (faixa_central > 0.08) & (faixa_central <= 10.0)
        contagem_colunas = np.sum(coluna_valida, axis=0)
        profundidades_ordenadas = np.sort(
            np.where(coluna_valida, faixa_central, np.inf),
            axis=0,
        )
        indices_p10 = np.maximum(
            np.ceil(contagem_colunas * 0.1).astype(int) - 1,
            0,
        )
        perfil = profundidades_ordenadas[indices_p10, np.arange(larg)]
        perfil[contagem_colunas == 0] = 0.0

        # Log periódico
        tempo_atual = time.time()
        if tempo_atual - self.ultimo_diagnostico > 1.0:
            distancia_log = (
                f"{dist_frontal:.2f}m" if dist_frontal is not None else "sem leituras válidas"
            )
            self.get_logger().info(
                f"[SENSOR] Estado={self.estado} | frontal P5={distancia_log} "
                f"| amostras={frontal_valido.size}"
            )
            self.ultimo_diagnostico = tempo_atual

        # ==============================================================
        # MÁQUINA DE ESTADOS
        # ==============================================================

        if self.estado == 'NAVEGANDO':
            if dist_frontal is None:
                self.parar()
                if tempo_atual - self.ultimo_erro_sensor > 2.0:
                    self.get_logger().warn(
                        "[SENSOR] Sem profundidade válida; mantendo robô parado."
                    )
                    self.ultimo_erro_sensor = tempo_atual
                return

            if dist_frontal < self.DIST_GATILHO_RECUO:
                # --- NOVO: tenta girar para gap lateral antes de recuar ---
                yaw_lateral = self.gap_lateral_imediato(perfil, larg)
                if yaw_lateral is not None:
                    self.parar()
                    self.alvo_yaw = yaw_lateral
                    self.get_logger().warn(
                        f"[OBSTÁCULO] Frontal {dist_frontal:.2f}m. "
                        "Gap lateral detectado → girando sem recuar."
                    )
                    self.estado = 'ALINHANDO_GAP'
                    self.tempo_inicio_estado = tempo_atual
                else:
                    # Sem gap lateral visível → recua para ganhar espaço
                    self.parar()
                    self.get_logger().warn(
                        f"[OBSTÁCULO] Frontal {dist_frontal:.2f}m. "
                        "Sem gap lateral → recuando para varredura 360."
                    )
                    self.estado = 'RECUANDO'
                    self.tempo_inicio_estado = tempo_atual
                    self.tentativas_recuo = 1
                    self.publicar_cmd(-0.15, 0.0, 0.0)
                return

            pontuacao, melhor_gap = self.avaliar_gaps(perfil, larg)

            if melhor_gap:
                self.tentativas_recuo = 0
                g_inicio, g_fim, z_medio, g_larg = melhor_gap
                g_centro = (g_inicio + g_fim) / 2.0

                erro = (centro_x - g_centro) / (larg / 2.0)
                vx   = max(self.VX_MIN, self.VX_MAX * (1.0 - 0.6 * abs(erro)))
                wz   = 0.60 * erro
                self.publicar_cmd(vx, 0.0, wz)

                if tempo_atual - self.ultimo_print > 0.5:
                    self.get_logger().info(
                        f"[NAVEGANDO] Vx={vx:.2f}m/s | Wz={wz:.2f}rad/s | "
                        f"Gap={g_larg}px | Z={z_medio:.2f}m | Score={pontuacao:.3f}"
                    )
                    self.ultimo_print = tempo_atual
            else:
                # Sem gap direto → tenta girar lateral antes de recuar
                yaw_lateral = self.gap_lateral_imediato(perfil, larg)
                if yaw_lateral is not None:
                    self.parar()
                    self.alvo_yaw = yaw_lateral
                    self.estado = 'ALINHANDO_GAP'
                    self.tempo_inicio_estado = tempo_atual
                    self.get_logger().warn("[BLOQUEADO] Sem gap frontal → girando para gap lateral.")
                else:
                    self.publicar_cmd(-0.15, 0.0, 0.0)
                    self.tentativas_recuo = 1
                    self.get_logger().warn(
                        f"[BLOQUEADO] Sem gap. Recuando ({self.tentativas_recuo}/"
                        f"{self.MAX_TENTATIVAS_RECUO}) antes de varrer..."
                    )
                    self.estado = 'RECUANDO'
                    self.tempo_inicio_estado = tempo_atual

        elif self.estado == 'RECUANDO':
            duracao_recuo = min(
                self.TEMPO_RECUO_INICIAL + 0.4 * (self.tentativas_recuo - 1),
                2.5,
            )
            if tempo_atual - self.tempo_inicio_estado < duracao_recuo:
                self.publicar_cmd(-0.15, 0.0, 0.0)
            else:
                self.parar()
                self.melhor_gap_score = -1.0
                self.alvo_yaw = None
                if not self.iniciar_varredura('VARRENDO_360', 1.0):
                    self.estado = 'AGUARDANDO_POSE'
                    self.get_logger().warn(
                        "Aguardando pose do Gazebo para iniciar a varredura de 360°."
                    )

        elif self.estado == 'AGUARDANDO_POSE':
            self.parar()
            if self.yaw_atual is not None:
                self.melhor_gap_score = -1.0
                self.alvo_yaw = None
                self.iniciar_varredura('VARRENDO_360', 1.0)

        elif self.estado == 'VARRENDO_360':
            self.registrar_gap_varredura(perfil, larg)

            if self.progresso_varredura < 2.0 * math.pi:
                self.publicar_cmd(0.0, 0.0, self.WZ_VARREDURA)
            else:
                self.parar()
                if self.alvo_yaw is None:
                    if self.tentativas_recuo < self.MAX_TENTATIVAS_RECUO:
                        self.tentativas_recuo += 1
                        self.get_logger().warn(
                            f"[SEM GAP] Nenhum gap após varredura. "
                            f"Novo recuo {self.tentativas_recuo}/{self.MAX_TENTATIVAS_RECUO}..."
                        )
                        self.estado = 'RECUANDO'
                        self.tempo_inicio_estado = tempo_atual
                    else:
                        self.get_logger().error(
                            "[SEM GAP] Limite de recuos atingido. Robô parado por segurança."
                        )
                        self.estado = 'BLOQUEADO'
                else:
                    self.get_logger().info(
                        f"Melhor gap (score={self.melhor_gap_score:.3f}). "
                        "Alinhando com a direção do gap..."
                    )
                    self.estado = 'ALINHANDO_GAP'

        elif self.estado == 'ALINHANDO_GAP':
            if self.alvo_yaw is None:
                # Segurança: sem alvo definido, volta a navegar
                self.estado = 'NAVEGANDO'
                return

            diferenca = self.diferenca_angular(self.alvo_yaw, self.yaw_atual)

            if abs(diferenca) > 0.05:
                # Velocidade proporcional ao erro (suave perto do alvo)
                wz = math.copysign(
                    min(abs(diferenca) * 1.5, self.WZ_GIRO_MAX),
                    diferenca,
                )
                self.publicar_cmd(0.0, 0.0, wz)
            else:
                self.parar()
                self.tentativas_recuo = 0
                self.get_logger().info(
                    "Alinhado com o melhor gap. Retomando a navegação."
                )
                self.estado = 'NAVEGANDO'

        elif self.estado == 'BLOQUEADO':
            self.parar()

    # ------------------------------------------------------------------
    def destroy_node(self):
        self.parar()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GapPhase35()
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
