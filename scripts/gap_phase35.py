#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge
import numpy as np
import time

class GapPhase35(Node):
    def __init__(self):
        super().__init__('gap_phase35')

        # --- CORREÇÃO 1: Nome exato do tópico publicado pela câmera HP60C ---
        self.depth_topic = '/ascamera_hp60c/camera_publisher/depth0/image_raw'
        self.cmd_topic = '/cmd_vel'

        self.subscription = self.create_subscription(
            Image, self.depth_topic, self.image_callback, 10
        )
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.bridge = CvBridge()

        # --- PARÂMETROS DE NAVEGAÇÃO AJUSTADOS ---
        self.DIST_OBSTACULO = 0.75    # Distância para acionar desvio (m)
        self.DIST_CRITICA = 0.45      # Distância muito próxima -> força RECUO (m)
        self.LARGURA_ROBO_PX = 70     # Aumentado para garantir espaço físico
        self.VX_MAX = 0.30            # Velocidade linear máxima reduzida p/ maior controle
        self.VX_MIN = 0.15            
        self.WZ_VARREDURA = 0.65      # Velocidade angular aumentada para varrer mais área
        
        self.estado = 'NAVEGANDO'
        self.tempo_inicio_estado = time.time()
        self.ultimo_print = time.time()
        
        self.gap_esquerda_max = 0.0
        self.gap_direita_max = 0.0
        self.direcao_escolhida = None

    def publicar_cmd(self, vx, vy, wz):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        if rclpy.ok():
            try:
                self.cmd_pub.publish(msg)
            except Exception:
                pass

    def parar(self):
        self.publicar_cmd(0.0, 0.0, 0.0)

    def avaliar_gaps(self, perfil, larg):
        gaps = []
        em_gap = False
        inicio = 0

        for i in range(larg):
            if perfil[i] > self.DIST_OBSTACULO:
                if not em_gap:
                    em_gap = True
                    inicio = i
            else:
                if em_gap:
                    em_gap = False
                    if (i - inicio) >= self.LARGURA_ROBO_PX:
                        gaps.append((inicio, i))
        if em_gap and (larg - inicio) >= self.LARGURA_ROBO_PX:
            gaps.append((inicio, larg))

        if not gaps:
            return 0.0, None

        melhor_gap = None
        maior_pontuacao = -1.0

        for g_inicio, g_fim in gaps:
            g_largura = g_fim - g_inicio
            z_medio = np.mean(perfil[g_inicio:g_fim])
            pontuacao = (g_largura * 0.7) + (z_medio * 25.0)

            if pontuacao > maior_pontuacao:
                maior_pontuacao = pontuacao
                melhor_gap = (g_inicio, g_fim, z_medio, g_largura)

        return maior_pontuacao, melhor_gap

    def image_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception:
            return

        depth_array = np.array(depth_image, dtype=np.float32)

        # Checa se a imagem está em milímetros (câmera física HP60C) considerando apenas valores finitos
        validos = depth_array[np.isfinite(depth_array) & (depth_array > 0.0)]
        if len(validos) > 0 and np.percentile(validos, 90) > 100.0:
            depth_array /= 1000.0

        # No simulador Gazebo, raios livres retornam inf; convertemos para alcance máximo seguro (10.0m)
        depth_array[np.isinf(depth_array)] = 10.0
        depth_array[np.isnan(depth_array)] = 0.0

        alt, larg = depth_array.shape
        centro_x = larg // 2

        faixa_central = depth_array[int(alt * 0.35):int(alt * 0.65), :]
        
        with np.errstate(all='ignore'):
            faixa_mascarada = np.where(faixa_central <= 0.05, np.nan, faixa_central)
            perfil = np.nanmean(faixa_mascarada, axis=0)
        
        perfil = np.nan_to_num(perfil, nan=0.0)

        # Região frontal estendida
        bloco_centro = faixa_central[:, max(0, centro_x - 40):min(larg, centro_x + 40)]
        with np.errstate(all='ignore'):
            bloco_valido = bloco_centro[(bloco_centro > 0.08) & (bloco_centro < 9.9)]
            # --- CORREÇÃO 2: Garante valor seguro alto se não houver bloco válido ---
            dist_min_centro = np.min(bloco_valido) if bloco_valido.size > 0 else 99.0

        tempo_atual = time.time()

        # --- MÁQUINA DE ESTADOS REVISADA ---
        if self.estado == 'NAVEGANDO':
            # Se colou demais no obstáculo, recua antes de varrer
            if 0.08 < dist_min_centro < self.DIST_CRITICA:
                self.parar()
                self.get_logger().warn(f"[CRÍTICO] Obstáculo a {dist_min_centro:.2f}m! Iniciando RECUO...")
                self.estado = 'RECUANDO'
                self.tempo_inicio_estado = tempo_atual
                return

            if 0.08 < dist_min_centro < self.DIST_OBSTACULO:
                self.parar()
                self.get_logger().warn(f"[OBSTÁCULO] Distância: {dist_min_centro:.2f}m. Varrendo ESQUERDA...")
                self.estado = 'VARRENDO_ESQ'
                self.tempo_inicio_estado = tempo_atual
                self.gap_esquerda_max = 0.0
                self.gap_direita_max = 0.0
                return

            pontuacao, melhor_gap = self.avaliar_gaps(perfil, larg)

            if melhor_gap:
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
                self.parar()
                self.get_logger().warn("[BLOQUEADO] Nenhum gap direto. Recuando...")
                self.estado = 'RECUANDO'
                self.tempo_inicio_estado = tempo_atual

        elif self.estado == 'RECUANDO':
            if tempo_atual - self.tempo_inicio_estado < 1.2:
                self.publicar_cmd(-0.15, 0.0, 0.0)
            else:
                self.parar()
                self.estado = 'VARRENDO_ESQ'
                self.tempo_inicio_estado = tempo_atual
                self.gap_esquerda_max = 0.0
                self.gap_direita_max = 0.0

        elif self.estado == 'VARRENDO_ESQ':
            pontuacao, _ = self.avaliar_gaps(perfil, larg)
            self.gap_esquerda_max = max(self.gap_esquerda_max, pontuacao)
            
            if tempo_atual - self.tempo_inicio_estado < 1.3:
                self.publicar_cmd(0.0, 0.0, self.WZ_VARREDURA)
            else:
                self.parar()
                self.get_logger().info("Varrendo DIREITA...")
                self.estado = 'VARRENDO_DIR'
                self.tempo_inicio_estado = tempo_atual

        elif self.estado == 'VARRENDO_DIR':
            pontuacao, _ = self.avaliar_gaps(perfil, larg)
            self.gap_direita_max = max(self.gap_direita_max, pontuacao)

            if tempo_atual - self.tempo_inicio_estado < 2.6:
                self.publicar_cmd(0.0, 0.0, -self.WZ_VARREDURA)
            else:
                self.parar()
                self.direcao_escolhida = 'ESQUERDA' if self.gap_esquerda_max >= self.gap_direita_max else 'DIREITA'
                self.get_logger().info(
                    f"==================================================\n"
                    f" DECISÃO: Desviar para a {self.direcao_escolhida}\n"
                    f" Score Esq: {self.gap_esquerda_max:.1f} | Score Dir: {self.gap_direita_max:.1f}\n"
                    f"=================================================="
                )
                self.estado = 'CONTORNANDO'
                self.tempo_inicio_estado = tempo_atual

        elif self.estado == 'CONTORNANDO':
            wz = self.WZ_VARREDURA * 1.2 if self.direcao_escolhida == 'ESQUERDA' else -self.WZ_VARREDURA * 1.2
            duracao = 2.0 if self.direcao_escolhida == 'ESQUERDA' else 1.6
            
            if tempo_atual - self.tempo_inicio_estado < duracao:
                self.publicar_cmd(0.18, 0.0, wz)
            else:
                self.parar()
                self.get_logger().info("Contorno concluído. Voltando à navegação normal.")
                self.estado = 'NAVEGANDO'

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
