# TIGO Digital Twin

Digital Twin do robô móvel omnidirecional TIGO desenvolvido no Laboratório de Controle e Automação (LACEA) utilizando ROS 2 Humble e Ignition Gazebo 6.

---

## 1. O que quero fazer (Objetivo do Projeto)

O objetivo principal deste projeto é criar um **Gêmeo Digital (Digital Twin)** fidedigno do robô TIGO para pesquisa e validação de algoritmos de **navegação autônoma reativa** e **percepção semântica com Deep Learning**, sem risco de colisão física e sem depender do acesso presencial contínuo ao laboratório.

### Foco: Navegação Reativa Sem Mapeamento
Diferente de abordagens pesadas baseadas em mapeamento prévio global (como SLAM e Nav2 clássico), este projeto adota uma arquitetura **puramente reativa**:
- O robô não constrói nem necessita de mapa prévio do ambiente para navegar.
- As decisões de controle são tomadas em tempo real a partir de dados instantâneos dos sensores a bordo (câmera RGB-D).
- O robô detecta espaços transitáveis (**Gap Following**) e reconhece semanticamente a classe dos obstáculos à sua frente (**YOLOv8**), desviando de forma inteligente e adaptando margens de segurança conforme o tipo de objeto (ex.: pessoas, cadeiras, caixas).

---

## 2. Como fiz (Metodologia e Arquitetura)

O sistema foi estruturado integrando simulação física, middleware robótico e modelos de inteligência artificial em tempo real:

```text
       Ignition Gazebo (TIGO Virtual no LACEA)
        ├── Câmera RGB (ascamera_hp60c_rgb)
        ├── Câmera Profundidade (ascamera_hp60c)
        └── Rodas Mecanum (Plugin Mecanum Drive)
                       │
                       ▼ (ros_gz_bridge)
                  ROS 2 Humble
                       │
         ┌─────────────┴─────────────┐
         ▼                           ▼
   [/rgb0/image_raw]          [/depth0/image_raw]
         │                           │
         ▼ (Assíncrono)              │
  ┌──────────────┐                   │
  │ YOLOv8n (CPU)│                   │
  │ Worker Thread│                   │
  └──────┬───────┘                   │
         │ Bounding Boxes            │
         ▼                           ▼
  ┌──────────────────────────────────────────────┐
  │   Fusão Semântico-Espacial (3D Matching)     │
  │   - Amostragem central de profundidade (Z)   │
  │   - Inflação de margem de segurança por classe│
  └──────────────────────┬───────────────────────┘
                         ▼
  ┌──────────────────────────────────────────────┐
  │       Algoritmo GAP Following Semântico      │
  │       - Detecção de corredores livres        │
  │       - Seleção do melhor gap transitável    │
  └──────────────────────┬───────────────────────┘
                         ▼
                  [/cmd_vel] (Twist)
                         │
                         ▼
                Movimentação do TIGO
```

### Componentes Implementados:
1. **Modelagem do Robô e Cenário (`urdf/tigo.sdf`, `worlds/lacea.sdf` e `worlds/gap_challenge.sdf`):**
   - Chassi baseado no modelo CAD real (`models/meshes/Tigo_para_gazebo.stl`).
   - Movimentação omnidirecional com rodas Mecanum via plugin `ignition-gazebo-mecanum-drive-system`.
   - Câmera virtual RGB-D sincronizada (sensores óptico colorido e de profundidade métrica).
   - Cenário tridimensional reconstruído do laboratório LACEA.
2. **Ponte de Comunicação Gazebo ↔ ROS 2 (`scripts/run_bridge.sh`):**
   - Encaminha tópicos de velocidade (`/cmd_vel`), imagens de profundidade e RGB, odometria e pose.
3. **Navegação Reativa Clássica (`scripts/gap_phase35.py`):**
   - Segmentação horizontal de profundidade, extração de gaps navegáveis, seleção por score ponderado e máquina de estados com recuo incremental e varredura 360° em caso de bloqueio frontal.
4. **Navegação Semântica com Deep Learning (`scripts/yolo_depth_gap.py`):**
   - Arquitetura multi-thread desacoplada: Worker Thread para inferência YOLOv8n a 45 FPS e thread principal mantendo o loop de controle reativo a 20 Hz.
   - Inflação métrica e projeção angular de margens de segurança personalizadas por classe de objeto.
   - Interface visual interativa OpenCV (`--gui`) com caixas 3D, distâncias métricas e telemetria.

---

## 3. Problemas Enfrentados e Soluções Implementadas

| # | Problema Enfrentado | Causa Técnica | Solução Implementada |
|---|---|---|---|
| 1 | **Crash por ABI do NumPy 2.x com `cv_bridge`** | `pip install ultralytics` atualizou o NumPy para a versão 2.x, incompatível com o `cv_bridge` do ROS 2 Humble (compilado para NumPy 1.x), gerando `Segmentation fault`. | Remoção total da dependência do `cv_bridge`. Implementou-se conversão nativa direta da memória de bytes ROS via `np.frombuffer()`, 100% agnóstica à versão do NumPy. |
| 2 | **Engasgos do robô e disparo do Watchdog** | Inferência sequencial do YOLOv8 na CPU demorava 200–450 ms, estourando o limite de 400 ms do watchdog e acionando o freio do robô repetidamente. | Separação do YOLO em uma **Worker Thread assíncrona** dedicada e otimização da resolução de inferência (`imgsz=320`). O loop de controle roda contínuo a 20 Hz e a inferência opera a 45 FPS (~22 ms). |
| 3 | **Desalinhamento visual e de eixos do STL** | A malha CAD do robô tinha o comprimento orientado no eixo `Y`, enquanto a cinemática do Gazebo e a câmera usam `+X` como frente. | Rotação de `-90°` aplicada à malha visual no SDF e ajuste na posição da câmera virtual na dianteira e acima do chassi. |
| 4 | **Filtro métrico rejeitando todos os gaps** | O cálculo inicial exigia largura métrica fixa de 0,45 m. Para obstáculos próximos (~0,8 m), isso exigia mais de 300 pixels livres na imagem, zerando os gaps candidatos. | Calibração do limiar mínimo para 40 pixels com pontuação mista (largura em pixels + profundidade média), garantindo a travessia segura em corredores estreitos. |
| 5 | **Travamento de interface gráfica no Wayland** | Chamadas frequentes de `cv2.imshow`/`waitKey` na mesma thread concorriam com o subsistema gráfico do Linux, gerando congelamentos. | Limitação da taxa de renderização da GUI a 20 FPS, desacoplamento por variáveis atômicas e tratamento de exceção com fallback automático para modo headless caso não haja display. |
| 6 | **Valores infinitos e ruídos no sensor de profundidade** | Leituras sem reflexão retornavam `inf` ou `nan` no Gazebo, quebrando cálculos matemáticos de distância. | Filtragem estatística por percentil nas colunas e substituição de raios infinitos por valor seguro teto (10 m). |
| 7 | **Bloqueio frontal e deadlocks em cantos** | O robô se aproximava de obstáculos frontais e parava sem conseguir enxergar saídas laterais imediatas. | Implementação de máquina de estados de escape: recuo progressivo temporal (1,8 a 2,8 s) seguido de giro para varredura e alinhamento com o maior gap livre disponível. |

---

## 4. Estrutura do Projeto

```text
TIGO_Digital_Twin
│
├── worlds
│   ├── lacea.sdf               # Cenário virtual do laboratório LACEA
│   └── gap_challenge.sdf       # Cenário de teste com corredor e obstáculos
│
├── urdf
│   └── tigo.sdf                # Modelo físico e cinemático do robô TIGO
│
├── models
│   ├── LACEA_prototipo_sem_chao_v2.dae
│   ├── chao_lacea_linha.dae
│   └── meshes
│       └── Tigo_para_gazebo.stl # Malha CAD real do chassi
│
├── scripts
│   ├── run_simulation.sh       # Script de inicialização do Gazebo
│   ├── run_bridge.sh           # Script unificado da ponte ROS 2 ↔ Gazebo
│   ├── gap_phase35.py          # Nó de Gap Following clássico (puramente geométrico)
│   └── yolo_depth_gap.py       # Nó de Navegação Semântica (YOLOv8 + Depth + GAP)
│
├── docs
│   ├── YOLO_DEPTH_GAP.md       # Relatório técnico completo de percepção semântica
│   ├── GAP_INTEGRATION.md      # Integração e arquitetura do algoritmo Gap Following
│   ├── GAP_FOLLOWING_CONCEPT.md# Conceitos teóricos e comparação Gap Following vs SLAM
│   ├── DEVELOPMENT_HISTORY.md  # Histórico cronológico de decisões e problemas resolvidos
│   ├── GAZEBO_SETUP.md         # Documentação da física e plugins do simulador
│   ├── ROS2_BRIDGE.md          # Configuração e tópicos da ponte de comunicação
│   ├── INSTALLATION.md         # Passo a passo de instalação das dependências
│   └── imagens
│
├── yolov8n.pt                  # Pesos da rede neural YOLOv8 nano
└── README.md                   # Apresentação geral do projeto
```

---

## 5. Como Executar o Projeto

### 1. Iniciar a Simulação no Gazebo

Em um primeiro terminal:

```bash
cd /home/betinha/TIGO_Digital_Twin
./scripts/run_simulation.sh
```

*(Opcional: para executar o cenário com corredor de obstáculos)*:
```bash
./scripts/run_simulation.sh worlds/gap_challenge.sdf
```

### 2. Iniciar a Ponte ROS 2 ↔ Gazebo (Bridge)

Em um segundo terminal:

```bash
cd /home/betinha/TIGO_Digital_Twin
export GZ_WORLD_NAME=lacea_world
./scripts/run_bridge.sh
```

*(Caso use o mundo `gap_challenge.sdf`, exporte `export GZ_WORLD_NAME=lacea_gap_challenge`)*.

### 3. Opção A: Executar a Navegação Clássica (Gap Following Geométrico)

Em um terceiro terminal:

```bash
cd /home/betinha/TIGO_Digital_Twin
source /opt/ros/humble/setup.bash
export GZ_WORLD_NAME=lacea_world
python3 scripts/gap_phase35.py
```

### 4. Opção B: Executar a Navegação Semântica (YOLOv8 + Profundidade + GAP)

Para navegar com reconhecimento semântico de obstáculos e **janela gráfica interativa OpenCV**:

```bash
cd /home/betinha/TIGO_Digital_Twin
source /opt/ros/humble/setup.bash
export GZ_WORLD_NAME=lacea_world
python3 scripts/yolo_depth_gap.py --gui
```

> **Parâmetros úteis do `yolo_depth_gap.py`:**
> - `--gui`: Exibe a janela com bounding boxes coloridas, distância métrica e vetores de direção. *(Omitir para execução headless sem interface)*.
> - `--conf 0.40`: Ajusta o limiar de confiança das detecções.

---

## 6. Próximas Etapas

- [x] Criação do ambiente virtual do laboratório LACEA
- [x] Inserção e calibração dimensional do modelo STL do TIGO
- [x] Configuração da câmera de profundidade virtual
- [x] Ponte ROS 2 ↔ Gazebo bidirecional configurada
- [x] Implementação do algoritmo Gap Following reativo com recuo e varredura
- [x] Inserção da câmera RGB virtual sincronizada com a profundidade
- [x] Navegação Semântica com YOLOv8n + Profundidade + Inflação de margem por classe
- [x] Resolução de incompatibilidades de ABI do NumPy e latência de CPU
- [ ] Inserção de pedestres móveis e obstáculos dinâmicos no cenário de simulação
- [ ] Ensaios experimentais comparativos entre o modelo simulado e o robô físico no LACEA

---

## Orientadora

Milena Faria Pinto

## Autora

Roberta Tavares  
Laboratório de Controle e Automação (LACEA)  
UFCI/RJ
