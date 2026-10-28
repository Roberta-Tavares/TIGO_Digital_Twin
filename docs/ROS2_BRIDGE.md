# Comunicação entre ROS 2 e Ignition Gazebo

## Objetivo

Este documento descreve o processo de integração entre o ROS 2 Humble e o Ignition Gazebo 6 através do pacote `ros_gz_bridge`.

Essa comunicação é fundamental para permitir que nós ROS 2 interajam com sensores e atuadores simulados dentro do ambiente virtual do TIGO.

---

# Arquitetura

O fluxo de comunicação é:

```text
Ignition Gazebo
       │
       ▼
 ros_gz_bridge
       │
       ▼
     ROS 2
       │
       ▼
gap_phase35.py
       │
       ▼
    /cmd_vel
       │
       ▼
 TIGO Virtual
```

---

# Instalação

Instalar os pacotes necessários:

```bash
sudo apt update

sudo apt install ros-humble-ros-gz -y
```

Verificar instalação:

```bash
ros2 pkg list | grep ros_gz
```

Resultado esperado:

```text
ros_gz
ros_gz_bridge
ros_gz_image
ros_gz_sim
```

---

# Motivação

O Ignition Gazebo e o ROS 2 utilizam sistemas de mensagens diferentes.

Exemplo:

| Ignition Gazebo | ROS 2 |
|---------------|--------|
| ignition.msgs.Twist | geometry_msgs/msg/Twist |
| ignition.msgs.Image | sensor_msgs/msg/Image |

A bridge é responsável por converter automaticamente essas mensagens.

---

# Bridge Utilizada

Durante o desenvolvimento foi utilizada a seguinte configuração:

```bash
ros2 run ros_gz_bridge parameter_bridge \
/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist \
/ascamera_hp60c/camera_publisher/depth0/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image
```

---

# Comunicação do /cmd_vel

Responsabilidade:

```text
ROS 2 → Gazebo
```

Objetivo:

Permitir que o algoritmo de navegação publique comandos de velocidade.

Tipo ROS:

```text
geometry_msgs/msg/Twist
```

Tipo Gazebo:

```text
ignition.msgs.Twist
```

---

# Comunicação da Câmera

Responsabilidade:

```text
Gazebo → ROS 2
```

Objetivo:

Enviar imagens da câmera virtual para os nós ROS.

Tipo Gazebo:

```text
ignition.msgs.Image
```

Tipo ROS:

```text
sensor_msgs/msg/Image
```

Tópico:

```text
/ascamera_hp60c/camera_publisher/depth0/image_raw
```

---

# Verificação da Bridge

Após iniciar a bridge:

```bash
ros2 run ros_gz_bridge parameter_bridge ...
```

Foi observado:

```text
Creating ROS->GZ Bridge
Creating GZ->ROS Bridge
```

Indicando que a ponte foi criada com sucesso.

---

# Verificação dos Tópicos ROS

Listar tópicos:

```bash
ros2 topic list
```

Verificar frequência:

```bash
ros2 topic hz /cmd_vel
```

ou

```bash
ros2 topic hz \
/ascamera_hp60c/camera_publisher/depth0/image_raw
```

---

# Verificação dos Tópicos Gazebo

Listar tópicos do simulador:

```bash
ign topic -l
```

Inspecionar um tópico:

```bash
ign topic -i \
-t /ascamera_hp60c/camera_publisher/depth0/image_raw
```

---

# Problemas Encontrados

## Pacote não encontrado

Erro:

```text
Package 'ros_gz_bridge' not found
```

Causa:

```text
Pacote ros_gz não instalado.
```

Solução:

```bash
sudo apt install ros-humble-ros-gz -y
```

---

## Erro de Sintaxe na Bridge

Algumas tentativas iniciais utilizaram uma sintaxe incorreta.

Exemplo:

```text
Twist]ignition.msgs.Twist
```

A sintaxe correta é:

```text
Twist@ignition.msgs.Twist
```

---

## Tópicos Existentes sem Dados

Em alguns momentos a bridge era criada corretamente, porém o robô não se movia.

Possíveis causas:

- câmera virtual não publicando
- plugin de movimentação não carregado
- tópico ROS sem mensagens
- ausência de obstáculos válidos

---

# Integração com Gap Following

Foi executado:

```bash
python3 gap_phase35.py
```

Logs observados:

```text
[BLOQUEADO] Nenhum gap direto

Varrendo DIREITA

DECISÃO: Desviar para a ESQUERDA
```

Conclusão:

- O algoritmo estava executando
- O nó ROS estava ativo
- Havia publicação de Twist
- A validação completa da câmera virtual permanece como etapa futura

---

# Estado Atual

Concluído:

- Bridge instalada e unificada via `scripts/run_bridge.sh`
- Comunicação bidirecional ROS 2 ↔ Gazebo validada
- Publicação de `/cmd_vel` com controle das 4 rodas Mecanum validada
- Publicação da câmera de profundidade em 15 Hz validada
- Algoritmo Gap Following adaptado para tratar dados virtuais (metros e infinitos) mantendo compatibilidade física
- Robô navegando de forma autônoma no cenário virtual do LACEA

---

# Próximos Passos

- Inserir obstáculos dinâmicos no cenário
- Validar desvios com diferentes geometrias de obstáculo
- Integrar mapeamento SLAM e Nav2
