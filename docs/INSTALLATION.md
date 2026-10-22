# Instalação do Ambiente

Este documento descreve o processo de instalação necessário para executar o projeto TIGO Digital Twin utilizando ROS 2 Humble e Ignition Gazebo 6.

---

# Requisitos

Sistema operacional:

```text
Ubuntu 22.04 LTS
```

---

# Instalação do ROS 2 Humble

Atualize o sistema:

```bash
sudo apt update
sudo apt upgrade -y
```

Instale o ROS 2 Humble seguindo a documentação oficial:

```bash
sudo apt install ros-humble-desktop -y
```

Adicione o ROS ao terminal:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

source ~/.bashrc
```

Verifique:

```bash
ros2 --version
```

---

# Instalação do Ignition Gazebo

Instale o simulador:

```bash
sudo apt update

sudo apt install ignition-fortress -y
```

Verifique:

```bash
ign gazebo --version
```

---

# Instalação da Bridge ROS ↔ Gazebo

A comunicação entre ROS 2 e Ignition Gazebo é realizada através do pacote:

```text
ros_gz_bridge
```

Instalação:

```bash
sudo apt update

sudo apt install ros-humble-ros-gz -y
```

Verifique:

```bash
ros2 pkg list | grep ros_gz
```

---

# Instalação do GitHub CLI

Utilizado para autenticação e publicação do projeto.

Instalação:

```bash
sudo apt update

sudo apt install gh -y
```

Autenticação:

```bash
gh auth login
```

Configuração utilizada:

```text
GitHub.com
HTTPS
Login with a web browser
```

Verificação:

```bash
gh auth status
```

---

# Configuração do Git

Configurar usuário:

```bash
git config --global user.name "Seu Nome"

git config --global user.email "seu_email@exemplo.com"
```

Verificar:

```bash
git config --global --list
```

---

# Clonando o Repositório

```bash
git clone https://github.com/Roberta-Tavares/TIGO_Digital_Twin.git

cd TIGO_Digital_Twin
```

---

# Estrutura Esperada

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
└── docs
```

---

# Executando a Simulação

Inicie o Ignition Gazebo:

```bash
ign gazebo worlds/lacea.sdf
```

Resultado esperado:

- Ambiente do LACEA carregado
- Modelo do TIGO carregado
- Piso renderizado corretamente

---

# Ponte ROS ↔ Gazebo

Em outro terminal:

```bash
ros2 run ros_gz_bridge parameter_bridge \
/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist \
/ascamera_hp60c/camera_publisher/depth0/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image
```

---

# Validação

Verifique os tópicos disponíveis:

```bash
ros2 topic list
```

Verifique os tópicos do Gazebo:

```bash
ign topic -l
```

---

# Próximos Passos

- Implementação da câmera de profundidade virtual
- Integração completa com o algoritmo Gap Following
- Inserção de obstáculos virtuais
- Navegação autônoma do TIGO
