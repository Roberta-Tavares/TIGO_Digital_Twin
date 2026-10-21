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
├── docs
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
- Estrutura preparada para sensores virtuais
- Base para integração ROS 2

Arquivo:

```text
models/meshes/Tigo_para_gazebo.stl
```

---

## Executando a Simulação

Abrir o Ignition Gazebo:

```bash
ign gazebo worlds/lacea.sdf
```

---

## Integração ROS 2

A comunicação entre o Gazebo e o ROS 2 é realizada utilizando:

```bash
ros_gz_bridge
```

Exemplo:

```bash
ros2 run ros_gz_bridge parameter_bridge \
/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist
```

---

## Próximas Etapas

- [x] Criação do ambiente virtual
- [x] Inserção do modelo STL do TIGO
- [x] Organização do repositório
- [ ] Implementação da câmera de profundidade
- [ ] Ponte ROS ↔ Gazebo
- [ ] Integração do algoritmo Gap Following
- [ ] Navegação autônoma completa
- [ ] Validação experimental

---

## Autora

Roberta Tavares

Laboratório de Controle e Automação (LACEA)

CEFET/RJ
