# TIGO Digital Twin

Digital Twin do robô omnidirecional TIGO desenvolvido no Laboratório de Controle e Automação (LACEA) utilizando ROS 2 Humble e Ignition Gazebo 6.

## Objetivo

Este projeto tem como objetivo criar um ambiente virtual capaz de reproduzir o comportamento do robô TIGO para:

- Desenvolvimento remoto
- Testes de navegação autônoma
- Validação do algoritmo Gap Following
- Simulação de sensores
- Testes sem risco de colisão física
- Desenvolvimento colaborativo

---

## Tecnologias Utilizadas

- Ubuntu 22.04 LTS
- ROS 2 Humble
- Ignition Gazebo 6
- OpenCV
- Python 3
- STL Mesh
- SDF

---

## Estrutura do Projeto

```text
TIGO_Digital_Twin
│
├── worlds
│   └── lacea.sdf
│
├── urdf
│   └── tigo.sdf
│
├── models
│   ├── LACEA_prototipo_sem_chao_v2.dae
│   ├── chao_lacea_linha.dae
│   └── meshes
│       └── Tigo_para_gazebo.stl
│
├── scripts
│   ├── run_simulation.sh
│   ├── run_bridge.sh
│   ├── gap_phase35.py
│   ├── tigo_mapper.py
│   └── yolo_depth_gap.py
│
├── docs
│   ├── YOLO_DEPTH_GAP.md
│   ├── GAP_INTEGRATION.md
│   ├── GAP_FOLLOWING_CONCEPT.md
│   ├── DEVELOPMENT_HISTORY.md
│   ├── GAZEBO_SETUP.md
│   ├── ROS2_BRIDGE.md
│   └── imagens
│
└── README.md
```

---

## Ambiente Virtual

O ambiente é baseado no laboratório LACEA e utiliza modelos tridimensionais em formato DAE para representar:

- Estrutura do laboratório
- Paredes
- Piso
- Obstáculos

---

## Modelo do Robô

O modelo do TIGO utiliza:

- Malha STL real do chassi
- Escala convertida para metros
- Rodas Mecanum omnidirecionais com atrito direcional
- Câmera de profundidade virtual HP60C
- Plugin `ignition-gazebo-mecanum-drive-system` para movimentação omnidirecional
- Base para integração ROS 2

Arquivo:

```text
models/meshes/Tigo_para_gazebo.stl
```

---

## Executando o Projeto

O projeto pode ser executado facilmente através dos scripts disponíveis na pasta `scripts/`:

### 1. Iniciar a Simulação no Gazebo

Em um terminal:

```bash
cd /home/betinha/TIGO_Digital_Twin
./scripts/run_simulation.sh
```

```

### 2. Iniciar a Ponte ROS 2 ↔ Gazebo (Bridge)

Em um segundo terminal:

```bash
cd /home/betinha/TIGO_Digital_Twin
export GZ_WORLD_NAME=lacea_gap_challenge
./scripts/run_bridge.sh
```

A ponte conecta os tópicos:
- `/cmd_vel` (`ROS 2 -> Gazebo`)
- `/ascamera_hp60c/camera_publisher/depth0/image_raw` (`Gazebo -> ROS 2`)
- `/ascamera_hp60c/camera_publisher/depth0/camera_info` (`Gazebo -> ROS 2`)
- `/ascamera_hp60c/camera_publisher/rgb0/image_raw` (`Gazebo -> ROS 2`)
- `/ascamera_hp60c/camera_publisher/rgb0/camera_info` (`Gazebo -> ROS 2`)
- `/model/tigo/odometry` (`Gazebo -> ROS 2`)
- `/world/$GZ_WORLD_NAME/pose/info` (`Gazebo -> ROS 2`, usado para medir a rotação do TIGO)

### 3. Executar a Navegação Autônoma (Gap Following)

Em um terceiro terminal:

```bash
cd /home/betinha/TIGO_Digital_Twin
source /opt/ros/humble/setup.bash
export GZ_WORLD_NAME=lacea_gap_challenge
python3 scripts/gap_phase35.py
```

### 4. Executar a Navegação Semântica (YOLO + Profundidade + GAP)

Para executar o algoritmo com percepção semântica e **abrir a câmera com interface gráfica OpenCV**:

```bash
cd /home/betinha/TIGO_Digital_Twin
source /opt/ros/humble/setup.bash
export GZ_WORLD_NAME=lacea_gap_challenge
python3 scripts/yolo_depth_gap.py --gui
```

> **Destaques:**
> - A flag `--gui` abre a janela visual do OpenCV exibindo caixas delimitadoras coloridas por classe, medição métrica da distância dos obstáculos em metros ($Z$) e os vetores de navegação em tempo real.
> - O processamento do YOLOv8 roda de forma assíncrona em uma *worker thread* paralela, garantindo controle reativo a 20 Hz e sem travamentos.
> - Documentação técnica completa e relatório de problemas/soluções disponíveis em: [`docs/YOLO_DEPTH_GAP.md`](docs/YOLO_DEPTH_GAP.md).


---

## Próximas Etapas

- [x] Criação do ambiente virtual
- [x] Inserção do modelo STL do TIGO
- [x] Organização do repositório
- [x] Implementação da câmera de profundidade
- [x] Ponte ROS ↔ Gazebo
- [x] Integração do algoritmo Gap Following
- [x] Navegação autônoma em simulação
- [x] Sensor RGB virtual alinhado à profundidade
- [x] Navegação Semântica com YOLOv8 + Profundidade + GAP
- [ ] Inserção de pedestres e obstáculos dinâmicos adicionais
- [ ] Validação experimental comparativa (simulado vs real)

---
## Orientadora

Milena Faria Pinto

## Autora

Roberta Tavares

Laboratório de Controle e Automação (LACEA)

UFCI/RJ
