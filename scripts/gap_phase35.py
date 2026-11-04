#!/usr/bin/env python3
import math
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

        # --- CORREÇÃO 1: Nome exato do tópico publicado pela câmera HP60C ---
        self.depth_topic = '/ascamera_hp60c/camera_publisher/depth0/image_raw'
        self.cmd_topic = '/cmd_vel'

        self.subscription = self.create_subscription(
            Image, self.depth_topic, self.image_callback, 10
        )
        self.watchdog = self.create_timer(0.1, self.sensor_watchdog)
        self.pose_subscription = self.create_subscription(
            TFMessage, '/world/lacea_world/pose/info', self.pose_callback, 10
        )
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.bridge = CvBridge()

        # --- PARÂMETROS DE NAVEGAÇÃO AJUSTADOS ---
        self.DIST_OBSTACULO = 0.7     # Profundidade mínima por coluna para considerar um gap (m)
        self.DIST_GATILHO_RECUO = 0.6 # Distância frontal para iniciar recuo (m)
        self.LARGURA_ROBO_PX = 40     # Largura mínima do gap em pixels
        self.VX_MAX = 0.30            # Velocidade linear máxima reduzida p/ maior controle
        self.VX_MIN = 0.15            
        self.WZ_VARREDURA = 0.65
        self.TEMPO_RECUO_INICIAL = 1.8
        self.MAX_TENTATIVAS_RECUO = 3
        
        self.estado = 'NAVEGANDO'
        self.tempo_inicio_estado = time.time()
        self.ultimo_print = time.time()
        self.ultimo_diagnostico = 0.0
        self.ultimo_erro_camera = 0.0
        self.ultimo_erro_sensor = 0.0
        self.ultimo_watchdog_log = 0.0
        self.ultima_imagem_valida = time.monotonic()
        self.tentativas_recuo = 0
        self.yaw_atual = None
        self.yaw_ultima_varredura = None
        self.progresso_varredura = 0.0
        self.direcao_varredura = 0.0
        self.maior_gap_largura = 0
        self.alvo_yaw = None

    def publicar_cmd(self, vx, vy, wz):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        if rclpy.ok():
            self.cmd_pub.publish(msg)

    def parar(self):
        self.publicar_cmd(0.0, 0.0, 0.0)

    def sensor_watchdog(self):
        if time.monotonic() - self.ultima_imagem_valida > 0.3:
            self.parar()
            agora = time.monotonic()
            if agora - self.ultimo_watchdog_log > 1.0:
                self.get_logger().error(
                    "Sem imagem de profundidade recente; comando de movimento zerado."
                )
                self.ultimo_watchdog_log = agora

    @staticmethod
    def diferenca_angular(destino, origem):
        return math.atan2(math.sin(destino - origem), math.cos(destino - origem))

    def pose_callback(self, msg):
        tigo_pose = next(
            (transform for transform in msg.transforms if transform.child_frame_id == 'tigo'),
            None,
        )
        if tigo_pose is None:
            return

        q = tigo_pose.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        if (
            self.estado == 'VARRENDO_360'
            and self.yaw_ultima_varredura is not None
        ):
            delta = self.diferenca_angular(yaw, self.yaw_ultima_varredura)
            self.progresso_varredura += self.direcao_varredura * delta

        self.yaw_atual = yaw
        if self.estado == 'VARRENDO_360':
            self.yaw_ultima_varredura = yaw

    def iniciar_varredura(self, estado, direcao):
        if self.yaw_atual is None:
            return False
        self.estado = estado
        self.direcao_varredura = direcao
        self.progresso_varredura = 0.0
        self.yaw_ultima_varredura = self.yaw_atual
        self.tempo_inicio_estado = time.time()
        return True

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
                    gaps.append((inicio, i, np.mean(perfil[inicio:i]), i - inicio))

        if em_gap and (larg - inicio) >= self.LARGURA_ROBO_PX:
            gaps.append((inicio, larg, np.mean(perfil[inicio:larg]), larg - inicio))
        return gaps

    def avaliar_gaps(self, perfil, larg):
        gaps = self.detectar_gaps(perfil, larg)
        if not gaps:
            return 0.0, None

        melhor_gap = None
        maior_pontuacao = -1.0

        for g_inicio, g_fim, z_medio, g_largura in gaps:
            pontuacao = (g_largura * 0.7) + (z_medio * 25.0)

            if pontuacao > maior_pontuacao:
                maior_pontuacao = pontuacao
                melhor_gap = (g_inicio, g_fim, z_medio, g_largura)

        return maior_pontuacao, melhor_gap

    def registrar_maior_gap(self, perfil, larg):
        gaps = self.detectar_gaps(perfil, larg)
        if not gaps or self.yaw_atual is None:
            return

        gap = max(gaps, key=lambda candidato: (candidato[3], candidato[2]))
        if gap[3] > self.maior_gap_largura:
            self.maior_gap_largura = gap[3]
            self.alvo_yaw = self.yaw_atual

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

        # Checa se a imagem está em milímetros (câmera física HP60C) considerando apenas valores finitos
        validos = depth_array[np.isfinite(depth_array) & (depth_array > 0.0)]
        if len(validos) > 0 and np.percentile(validos, 90) > 100.0:
            depth_array /= 1000.0

        # No simulador Gazebo, raios livres retornam inf; convertemos para alcance máximo seguro (10.0m)
        depth_array[np.isinf(depth_array)] = 10.0
        depth_array[np.isnan(depth_array)] = 0.0

        alt, larg = depth_array.shape
        centro_x = larg // 2

        y_inicio = int(alt * 0.25)
        y_fim = int(alt * 0.75)
        x_inicio = max(0, int(centro_x - larg * 0.20))
        x_fim = min(larg, int(centro_x + larg * 0.20))
        faixa_frontal = depth_array[y_inicio:y_fim, x_inicio:x_fim]
        frontal_valido = faixa_frontal[
            (faixa_frontal > 0.08) & (faixa_frontal <= 10.0)
        ]
        dist_frontal = (
            float(np.percentile(frontal_valido, 5)) if frontal_valido.size else None
        )

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

        tempo_atual = time.time()
        if tempo_atual - self.ultimo_diagnostico > 1.0:
            distancia_log = (
                f"{dist_frontal:.2f}m" if dist_frontal is not None else "sem leituras válidas"
            )
            self.get_logger().info(
                f"[SENSOR] Estado={self.estado} | distância frontal P5={distancia_log} "
                f"| amostras={frontal_valido.size}"
            )
            self.ultimo_diagnostico = tempo_atual

        # --- MÁQUINA DE ESTADOS REVISADA ---
        if self.estado == 'NAVEGANDO':
            if dist_frontal is None:
                self.parar()
                if tempo_atual - self.ultimo_erro_sensor > 2.0:
                    self.get_logger().warn(
                        "[SENSOR] Sem profundidade válida na faixa frontal; "
                        "mantendo o robô parado."
                    )
                    self.ultimo_erro_sensor = tempo_atual
                return

            if dist_frontal < self.DIST_GATILHO_RECUO:
                self.parar()
                self.get_logger().warn(
                    f"[OBSTÁCULO] Distância frontal P5: {dist_frontal:.2f}m. "
                    "Recuando antes de procurar o maior gap..."
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
                vx = max(self.VX_MIN, self.VX_MAX * (1.0 - 0.6 * abs(erro)))
                wz = 0.60 * erro  # Ganho proporcional aumentado
                self.publicar_cmd(vx, 0.0, wz)

                if tempo_atual - self.ultimo_print > 0.5:
                    self.get_logger().info(
                        f"[NAVEGANDO] Vx={vx:.2f}m/s | Wz={wz:.2f}rad/s | Gap={g_larg}px | Z={z_medio:.2f}m"
                    )
                    self.ultimo_print = tempo_atual
            else:
                self.publicar_cmd(-0.15, 0.0, 0.0)
                self.tentativas_recuo = 1
                self.get_logger().warn(
                    f"[BLOQUEADO] Nenhum gap direto. Recuo {self.tentativas_recuo}/"
                    f"{self.MAX_TENTATIVAS_RECUO} antes de varrer..."
                )
                self.estado = 'RECUANDO'
                self.tempo_inicio_estado = tempo_atual

        elif self.estado == 'RECUANDO':
            duracao_recuo = min(
                self.TEMPO_RECUO_INICIAL + 0.5 * (self.tentativas_recuo - 1),
                2.8,
            )
            if tempo_atual - self.tempo_inicio_estado < duracao_recuo:
                self.publicar_cmd(-0.15, 0.0, 0.0)
            else:
                self.parar()
                self.maior_gap_largura = 0
                self.alvo_yaw = None
                if not self.iniciar_varredura('VARRENDO_360', 1.0):
                    self.estado = 'AGUARDANDO_POSE'
                    self.get_logger().warn(
                        "Aguardando pose do Gazebo para iniciar a varredura de 360 graus."
                    )

        elif self.estado == 'AGUARDANDO_POSE':
            self.parar()
            if self.yaw_atual is not None:
                self.iniciar_varredura('VARRENDO_360', 1.0)

        elif self.estado == 'VARRENDO_360':
            self.registrar_maior_gap(perfil, larg)

            if self.progresso_varredura < 2.0 * math.pi:
                self.publicar_cmd(0.0, 0.0, self.WZ_VARREDURA)
            else:
                self.parar()
                if self.alvo_yaw is None:
                    if self.tentativas_recuo < self.MAX_TENTATIVAS_RECUO:
                        self.tentativas_recuo += 1
                        self.get_logger().warn(
                            f"[SEM GAP] Nenhum gap após a varredura. "
                            f"Novo recuo {self.tentativas_recuo}/"
                            f"{self.MAX_TENTATIVAS_RECUO}..."
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
                        f"Maior gap encontrado: {self.maior_gap_largura}px. "
                        "Alinhando com a direção observada..."
                    )
                    self.estado = 'ALINHANDO_GAP'

        elif self.estado == 'ALINHANDO_GAP':
            diferenca = self.diferenca_angular(self.alvo_yaw, self.yaw_atual)
            if abs(diferenca) > 0.05:
                wz = math.copysign(
                    min(abs(diferenca) * 1.2, self.WZ_VARREDURA),
                    diferenca,
                )
                self.publicar_cmd(0.0, 0.0, wz)
            else:
                self.parar()
                self.tentativas_recuo = 0
                self.get_logger().info("Alinhado com o maior gap. Retomando a navegação.")
                self.estado = 'NAVEGANDO'

        elif self.estado == 'BLOQUEADO':
            self.parar()

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
