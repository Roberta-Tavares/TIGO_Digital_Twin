#!/usr/bin/env python3
"""
tigo_mapper.py – Mapeamento exploratório por varredura de gaps.

Objetivo:
  Em vez de navegar apenas pelo MELHOR gap (como faz o gap_phase35.py),
  o Tigo visita TODOS os gaps passáveis de forma sistemática, construindo
  um registro do ambiente explorado.

Estratégia:
  1. VARRENDO: Rotação 360° no lugar. Detecta e registra TODOS os gaps
     passáveis com seus ângulos absolutos e profundidades.
  2. ESCOLHENDO_ALVO: Seleciona o próximo gap ainda não visitado
     (prioridade: gaps não explorados, depois os mais distantes para
     ampliar a área coberta).
  3. ALINHANDO_GAP: Gira in-place em direção ao gap alvo.
  4. AVANCANDO: Avança pelo gap até detectar obstáculo ou cumprir distância.
  5. Ao chegar em nova posição → volta a VARRENDO para redescobrir gaps.
  6. RECUANDO: Recua se frontal muito próximo antes de girar.
  7. CONCLUIDO: Sem mais gaps novos para explorar → robô para.

O mapa é salvo em:
  - /tmp/tigo_map_gaps.json  (lista de posições e gaps visitados)
  - Log periódico no terminal.
"""
import json
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


# ---------------------------------------------------------------------------
# Utilitários
# ---------------------------------------------------------------------------

def normalizar_angulo(a: float) -> float:
    """Mantém o ângulo no intervalo (-π, π]."""
    return math.atan2(math.sin(a), math.cos(a))


def dist_2d(p1, p2) -> float:
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


# ---------------------------------------------------------------------------
# Nó principal
# ---------------------------------------------------------------------------

class TigoMapper(Node):

    # ------------------------------------------------------------------
    # Inicialização
    # ------------------------------------------------------------------
    def __init__(self):
        super().__init__('tigo_mapper')

        self.depth_topic = '/ascamera_hp60c/camera_publisher/depth0/image_raw'
        self.cmd_topic   = '/cmd_vel'
        world_name = os.environ.get('GZ_WORLD_NAME') or 'lacea_world'

        self.subscription = self.create_subscription(
            Image, self.depth_topic, self.image_callback, 10
        )
        self.watchdog_timer = self.create_timer(0.1, self.sensor_watchdog)
        self.pose_subscription = self.create_subscription(
            TFMessage,
            f'/world/{world_name}/pose/info',
            self.pose_callback,
            10,
        )
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.bridge  = CvBridge()

        # --- PARÂMETROS DE NAVEGAÇÃO ---
        self.DIST_OBSTACULO      = 0.70   # Coluna livre se profundidade > este valor (m)
        self.DIST_GATILHO_RECUO  = 0.55   # Frontal mínimo antes de recuar (m)
        self.DIST_AVANCO_PADRAO  = 1.20   # Quanto avançar por gap (m)
        self.LARGURA_ROBO_PX     = 40     # Largura mínima de gap em pixels
        self.LARGURA_ROBO_M      = 0.25   # Largura física do Tigo (m)
        self.MARGEM_LATERAL_M    = 0.10   # Margem de segurança por lado (m)
        self.VX_AVANCO           = 0.22   # Velocidade ao avançar (m/s)
        self.WZ_VARREDURA        = 0.60   # Velocidade angular de varredura (rad/s)
        self.WZ_GIRO_MAX         = 0.75   # Velocidade máxima de alinhamento (rad/s)
        self.FOV_H               = math.radians(87.0)

        # Dois raios com responsabilidades diferentes:
        #   RAIO_NOVA_POSICAO: quão perto uma nova chegada precisa estar para
        #   NÃO ser registrada como nova posição (evita duplicatas de posição).
        #   Deve ser bem menor que DIST_AVANCO_PADRAO.
        self.RAIO_NOVA_POSICAO   = 0.40   # m

        #   RAIO_EXPLORADO: raio usado para decidir se o DESTINO de um gap
        #   já é território conhecido. Deve ser ligeiramente menor que
        #   DIST_AVANCO_PADRAO para não bloquear gaps legítimos para adiante,
        #   mas grande o suficiente para rejeitar gaps que voltam para trás.
        self.RAIO_EXPLORADO      = 0.90   # m

        # Tolerância angular para deduplicar gap da MESMA origem (rad)
        self.TOL_ANGULO_GAP      = math.radians(12)
        # Tempo máximo avançando sem recuo (s)
        self.TEMPO_MAX_AVANCO    = 8.0
        # Largura mínima real exigida para o gap (robô + margem dos dois lados)
        self.LARGURA_MIN_REAL_M  = self.LARGURA_ROBO_M + 2 * self.MARGEM_LATERAL_M  # 0.45m

        # --- ESTADO ---
        self.estado = 'VARRENDO'
        self.tempo_inicio_estado = time.time()
        self.ultimo_print        = time.time()
        self.ultimo_diagnostico  = 0.0
        self.ultimo_erro_camera  = 0.0
        self.ultimo_erro_sensor  = 0.0
        self.ultimo_watchdog_log = 0.0
        self.ultima_imagem_valida = time.monotonic()

        # Pose
        self.yaw_atual           = None
        self.pos_atual           = (0.0, 0.0)   # (x, y) do Tigo
        self.yaw_ultima_varredura = None
        self.progresso_varredura = 0.0
        self.direcao_varredura   = 1.0

        # Alvo de navegação
        self.alvo_yaw            = None
        self.alvo_dist           = self.DIST_AVANCO_PADRAO
        self.pos_inicio_avanco   = (0.0, 0.0)

        # Mapa de gaps candidatos:  lista de dicts com chaves:
        #   yaw_abs, profundidade, largura_m, visitado, pos_origem
        self.gaps_candidatos: list[dict] = []

        # Histórico de posições visitadas
        self.posicoes_visitadas: list[dict] = []

        # Largura da última imagem
        self.larg_ultima_imagem = 320

        self.get_logger().info("TigoMapper iniciado. Iniciando varredura 360°...")

    # ------------------------------------------------------------------
    # Publicação de comandos
    # ------------------------------------------------------------------
    def publicar_cmd(self, vx: float, vy: float, wz: float):
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
    def dif_ang(destino: float, origem: float) -> float:
        return math.atan2(math.sin(destino - origem), math.cos(destino - origem))

    def coluna_para_angulo(self, coluna: float, larg: int) -> float:
        """Converte coluna de pixel para ângulo relativo ao eixo frontal (rad)."""
        return (coluna / larg - 0.5) * self.FOV_H

    # ------------------------------------------------------------------
    # Pose callback
    # ------------------------------------------------------------------
    def pose_callback(self, msg: TFMessage):
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

        x = tigo_pose.transform.translation.x
        y = tigo_pose.transform.translation.y
        self.pos_atual = (x, y)

        if self.estado == 'VARRENDO' and self.yaw_ultima_varredura is not None:
            delta = self.dif_ang(yaw, self.yaw_ultima_varredura)
            self.progresso_varredura += self.direcao_varredura * delta

        self.yaw_atual = yaw
        if self.estado == 'VARRENDO':
            self.yaw_ultima_varredura = yaw

    # ------------------------------------------------------------------
    # Iniciar varredura
    # ------------------------------------------------------------------
    def iniciar_varredura(self) -> bool:
        if self.yaw_atual is None:
            return False
        self.estado = 'VARRENDO'
        self.direcao_varredura   = 1.0
        self.progresso_varredura = 0.0
        self.yaw_ultima_varredura = self.yaw_atual
        self.tempo_inicio_estado = time.time()
        self.get_logger().info(
            f"[VARREDURA] Iniciando nova varredura em pos=({self.pos_atual[0]:.2f}, "
            f"{self.pos_atual[1]:.2f})."
        )
        return True

    # ------------------------------------------------------------------
    # Detecção de gaps
    # ------------------------------------------------------------------
    def detectar_gaps(self, perfil: np.ndarray, larg: int) -> list[tuple]:
        """
        Retorna lista de (inicio_px, fim_px, profundidade_media, largura_px).
        Pré-filtra por pixels e depois valida largura em metros.
        """
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
                larg_px = i - inicio
                if larg_px >= self.LARGURA_ROBO_PX:
                    z_medio = float(np.mean(perfil[inicio:i]))
                    if self._gap_passavel_metros(larg_px, z_medio, larg):
                        gaps.append((inicio, i, z_medio, larg_px))

        if em_gap:
            larg_px = larg - inicio
            if larg_px >= self.LARGURA_ROBO_PX:
                z_medio = float(np.mean(perfil[inicio:larg]))
                if self._gap_passavel_metros(larg_px, z_medio, larg):
                    gaps.append((inicio, larg, z_medio, larg_px))

        return gaps

    def _gap_passavel_metros(self, larg_px: int, z_medio: float, larg_total: int) -> bool:
        """Valida a largura real do gap em metros com margem de segurança rigorosa."""
        gap_m = z_medio * larg_px * self.FOV_H / larg_total
        return gap_m >= self.LARGURA_MIN_REAL_M

    # ------------------------------------------------------------------
    # Lógica de mapeamento / memória espacial
    # ------------------------------------------------------------------

    def _destino_explorado(self, yaw_abs: float, pos_origem: tuple, profundidade: float) -> bool:
        """
        Verifica se o destino estimado deste gap já é território conhecido.

        Usa RAIO_EXPLORADO (maior), que cobre a área ao redor de cada posição
        visitada. Assim gaps que levam de volta a regiões já varridas são
        rejeitados na hora do registro — o Tigo não perde tempo indo para
        onde já esteve.
        """
        d = min(profundidade * 0.75, self.DIST_AVANCO_PADRAO)
        destino = (
            pos_origem[0] + d * math.cos(yaw_abs),
            pos_origem[1] + d * math.sin(yaw_abs),
        )
        for p in self.posicoes_visitadas:
            if dist_2d(p['pos'], destino) < self.RAIO_EXPLORADO:
                return True
        return False

    def _gap_ja_registrado(self, yaw_abs: float, pos_origem: tuple) -> bool:
        """
        Evita duplicatas: gap com ângulo parecido da mesma posição de origem.
        """
        for g in self.gaps_candidatos:
            mesma_origem = dist_2d(g['pos_origem'], pos_origem) < self.RAIO_NOVA_POSICAO * 2
            mesmo_angulo = abs(normalizar_angulo(g['yaw_abs'] - yaw_abs)) < self.TOL_ANGULO_GAP
            if mesma_origem and mesmo_angulo:
                return True
        return False

    def _e_nova_posicao(self, pos) -> bool:
        """Retorna True se esta posição está longe o suficiente das já registradas."""
        for p in self.posicoes_visitadas:
            if dist_2d(p['pos'], pos) < self.RAIO_NOVA_POSICAO:
                return False
        return True

    def registrar_gaps_varredura(self, perfil: np.ndarray, larg: int):
        """
        Durante a varredura, registra gaps passáveis como candidatos.

        FILTRO PRINCIPAL: se o destino estimado do gap já está dentro do raio
        de uma posição visitada (RAIO_EXPLORADO), o gap é descartado — o Tigo
        já conhece aquela área e não precisa voltar.
        """
        if self.yaw_atual is None:
            return

        gaps = self.detectar_gaps(perfil, larg)
        novos = ignorados = 0
        for g_inicio, g_fim, z_medio, g_larg in gaps:
            centro_px = (g_inicio + g_fim) / 2.0
            ang_rel   = self.coluna_para_angulo(centro_px, larg)
            yaw_abs   = normalizar_angulo(self.yaw_atual + ang_rel)

            # 1) Rejeita se o destino já é território conhecido
            if self._destino_explorado(yaw_abs, self.pos_atual, z_medio):
                ignorados += 1
                continue

            # 2) Evita duplicata do mesmo gap já na fila
            if self._gap_ja_registrado(yaw_abs, self.pos_atual):
                continue

            larg_m = z_medio * g_larg * self.FOV_H / larg
            self.gaps_candidatos.append({
                'yaw_abs':      yaw_abs,
                'profundidade': z_medio,
                'largura_m':    larg_m,
                'visitado':     False,
                'pos_origem':   list(self.pos_atual),
            })
            novos += 1

        if novos > 0 or ignorados > 0:
            self.get_logger().info(
                f"[VARREDURA] +{novos} novo(s), {ignorados} ignorado(s) "
                f"(já explorado). Fila: {len(self.gaps_candidatos)}"
            )

    def escolher_proximo_gap(self) -> dict | None:
        """
        Escolhe o próximo gap não visitado cuja direção ainda leva
        a território desconhecido.
        Prioridade: maior profundidade (explora mais longe primeiro).
        """
        candidatos = []
        for g in self.gaps_candidatos:
            if g['visitado']:
                continue
            # Rejeição tardia: pode ter sido registrado antes de uma posição
            # próxima ser visitada → verifica novamente com o mapa atual
            if self._destino_explorado(g['yaw_abs'], g['pos_origem'], g['profundidade']):
                g['visitado'] = True  # descarta silenciosamente
                continue
            candidatos.append(g)

        if not candidatos:
            return None
        candidatos.sort(key=lambda g: g['profundidade'], reverse=True)
        return candidatos[0]

    def _salvar_mapa(self):
        """Salva o mapa em JSON para análise posterior."""
        mapa = {
            'posicoes_visitadas': self.posicoes_visitadas,
            'gaps_registrados':   self.gaps_candidatos,
            'timestamp':          time.time(),
        }
        path = '/tmp/tigo_map_gaps.json'
        try:
            with open(path, 'w') as f:
                json.dump(mapa, f, indent=2, default=float)
        except Exception as e:
            self.get_logger().warn(f"Erro ao salvar mapa: {e}")

    # ------------------------------------------------------------------
    # Callback principal da câmera
    # ------------------------------------------------------------------
    def image_callback(self, msg: Image):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as exc:
            self.parar()
            tempo_atual = time.time()
            if tempo_atual - self.ultimo_erro_camera > 2.0:
                self.get_logger().error(f"Falha ao converter imagem: {exc}")
                self.ultimo_erro_camera = tempo_atual
            return

        depth_array = np.array(depth_image, dtype=np.float32)
        if depth_array.ndim != 2 or not depth_array.size:
            self.parar()
            self.get_logger().error(f"Imagem inválida: shape={depth_array.shape}.")
            return
        self.ultima_imagem_valida = time.monotonic()

        # Converter mm → m se necessário
        validos = depth_array[np.isfinite(depth_array) & (depth_array > 0.0)]
        if len(validos) > 0 and np.percentile(validos, 90) > 100.0:
            depth_array /= 1000.0

        depth_array[np.isinf(depth_array)] = 10.0
        depth_array[np.isnan(depth_array)] = 0.0

        alt, larg = depth_array.shape
        self.larg_ultima_imagem = larg
        centro_x = larg // 2

        # Faixa frontal central (20% da largura, 25-75% vertical)
        y0 = int(alt * 0.25); y1 = int(alt * 0.75)
        x0 = max(0, int(centro_x - larg * 0.20))
        x1 = min(larg, int(centro_x + larg * 0.20))
        faixa_frontal = depth_array[y0:y1, x0:x1]
        frontal_valido = faixa_frontal[
            (faixa_frontal > 0.08) & (faixa_frontal <= 10.0)
        ]
        dist_frontal = (
            float(np.percentile(frontal_valido, 5)) if frontal_valido.size else None
        )

        # Perfil por coluna (percentil 10, faixa vertical 35-65%)
        faixa_central = depth_array[int(alt * 0.35):int(alt * 0.65), :]
        col_valida = (faixa_central > 0.08) & (faixa_central <= 10.0)
        contagem = np.sum(col_valida, axis=0)
        ordenadas = np.sort(np.where(col_valida, faixa_central, np.inf), axis=0)
        idx_p10 = np.maximum(np.ceil(contagem * 0.1).astype(int) - 1, 0)
        perfil = ordenadas[idx_p10, np.arange(larg)]
        perfil[contagem == 0] = 0.0

        # Log periódico
        tempo_atual = time.time()
        if tempo_atual - self.ultimo_diagnostico > 2.0:
            nao_visit = sum(1 for g in self.gaps_candidatos if not g['visitado'])
            dist_log = f"{dist_frontal:.2f}m" if dist_frontal is not None else "N/A"
            self.get_logger().info(
                f"[STATUS] Estado={self.estado} | Frontal={dist_log} | "
                f"Gaps pendentes={nao_visit} | "
                f"Posições visitadas={len(self.posicoes_visitadas)}"
            )
            self.ultimo_diagnostico = tempo_atual

        # ==============================================================
        # MÁQUINA DE ESTADOS
        # ==============================================================

        if self.estado == 'VARRENDO':
            self.registrar_gaps_varredura(perfil, larg)

            if self.progresso_varredura < 2.0 * math.pi:
                self.publicar_cmd(0.0, 0.0, self.WZ_VARREDURA)
            else:
                # Varredura completa → escolhe próximo gap
                self.parar()
                self.estado = 'ESCOLHENDO_ALVO'
                self.get_logger().info(
                    f"[VARREDURA] Completa. {len(self.gaps_candidatos)} gaps "
                    f"registrados no total."
                )

        elif self.estado == 'ESCOLHENDO_ALVO':
            proximo = self.escolher_proximo_gap()
            if proximo is None:
                self.get_logger().info(
                    "[MAPEAMENTO] Todos os gaps foram explorados! "
                    "Mapa salvo em /tmp/tigo_map_gaps.json"
                )
                self._salvar_mapa()
                self.estado = 'CONCLUIDO'
            else:
                proximo['visitado'] = True
                self.alvo_yaw  = proximo['yaw_abs']
                # Avança a profundidade real do gap, limitado a DIST_AVANCO_PADRAO
                self.alvo_dist = min(proximo['profundidade'] * 0.75, self.DIST_AVANCO_PADRAO)
                self.get_logger().info(
                    f"[ALVO] Gap selecionado: yaw={math.degrees(self.alvo_yaw):.1f}° | "
                    f"prof={proximo['profundidade']:.2f}m | "
                    f"largura={proximo['largura_m']:.2f}m | "
                    f"avançar={self.alvo_dist:.2f}m"
                )
                self.estado = 'ALINHANDO_GAP'
                self.tempo_inicio_estado = tempo_atual

        elif self.estado == 'ALINHANDO_GAP':
            if self.yaw_atual is None or self.alvo_yaw is None:
                self.estado = 'VARRENDO'
                self.iniciar_varredura()
                return

            diferenca = self.dif_ang(self.alvo_yaw, self.yaw_atual)

            if abs(diferenca) > 0.06:
                wz = math.copysign(
                    min(abs(diferenca) * 1.5, self.WZ_GIRO_MAX),
                    diferenca,
                )
                self.publicar_cmd(0.0, 0.0, wz)
            else:
                self.parar()
                self.pos_inicio_avanco = self.pos_atual
                self.estado = 'AVANCANDO'
                self.tempo_inicio_estado = tempo_atual
                self.get_logger().info(
                    f"[AVANÇANDO] Alinhado. Avançando {self.alvo_dist:.2f}m pelo gap."
                )

        elif self.estado == 'AVANCANDO':
            # Verifica se chegou à distância alvo ou se há obstáculo
            percorrido = dist_2d(self.pos_inicio_avanco, self.pos_atual)
            tempo_avanco = tempo_atual - self.tempo_inicio_estado

            if dist_frontal is not None and dist_frontal < self.DIST_GATILHO_RECUO:
                self.parar()
                self.get_logger().warn(
                    f"[AVANÇANDO] Obstáculo a {dist_frontal:.2f}m. Recuando."
                )
                self.estado = 'RECUANDO'
                self.tempo_inicio_estado = tempo_atual
            elif percorrido >= self.alvo_dist or tempo_avanco > self.TEMPO_MAX_AVANCO:
                self.parar()
                self.get_logger().info(
                    f"[AVANÇANDO] Chegou ao destino ({percorrido:.2f}m). "
                    "Iniciando nova varredura."
                )
                self._registrar_posicao_atual()   # registra onde chegou
                self._salvar_mapa()
                if not self.iniciar_varredura():
                    self.estado = 'AGUARDANDO_POSE'
            else:
                # Ajuste de curso: mantém alinhamento com alvo_yaw
                if self.yaw_atual is not None:
                    erro_yaw = self.dif_ang(self.alvo_yaw, self.yaw_atual)
                    wz = math.copysign(min(abs(erro_yaw) * 1.0, 0.4), erro_yaw)
                else:
                    wz = 0.0
                self.publicar_cmd(self.VX_AVANCO, 0.0, wz)

        elif self.estado == 'RECUANDO':
            if tempo_atual - self.tempo_inicio_estado < 1.5:
                self.publicar_cmd(-0.15, 0.0, 0.0)
            else:
                self.parar()
                self._registrar_posicao_atual()
                if not self.iniciar_varredura():
                    self.estado = 'AGUARDANDO_POSE'

        elif self.estado == 'AGUARDANDO_POSE':
            self.parar()
            if self.yaw_atual is not None:
                self.iniciar_varredura()

        elif self.estado == 'CONCLUIDO':
            self.parar()

    # ------------------------------------------------------------------
    def _registrar_posicao_atual(self):
        """
        Registra a posição atual como visitada se ainda não estiver registrada.
        Usa RAIO_NOVA_POSICAO (pequeno) para identificar duplicatas de posição.
        """
        if self._e_nova_posicao(self.pos_atual):
            self.posicoes_visitadas.append({
                'pos':       list(self.pos_atual),
                'yaw':       self.yaw_atual,
                'timestamp': time.time(),
            })
            self.get_logger().info(
                f"[MEMÓRIA] Posição registrada: ({self.pos_atual[0]:.2f}, "
                f"{self.pos_atual[1]:.2f}). Total visitadas: {len(self.posicoes_visitadas)}"
            )

    # ------------------------------------------------------------------
    def destroy_node(self):
        self.parar()
        self._salvar_mapa()
        cv2.destroyAllWindows()
        super().destroy_node()


# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = TigoMapper()
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
