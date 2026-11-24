# Integração do Algoritmo Gap Following

## 1. O que quero fazer (Objetivo)

Este documento descreve a integração entre o algoritmo de navegação autônoma reativa do robô TIGO e o ambiente virtual desenvolvido no Ignition Gazebo 6.

O objetivo é executar na simulação um nó de navegação ROS 2 que recebe imagens de profundidade em tempo real da câmera virtual HP60C, processa instantaneamente o perfil de obstáculos e publica comandos de velocidade (`cmd_vel`) para as rodas Mecanum omnidirecionais.

### Foco: Navegação Reativa Sem Mapeamento Prévio
O Gap Following opera exclusivamente com leituras sensoriais instantâneas:
- Não constrói nem requer um mapa prévio do ambiente (sem SLAM).
- Opera com baixíssimo custo computacional, ideal para controle em tempo real (20 Hz).
- Encontra corredores livres ("gaps") e comanda o robô através da passagem mais segura.

---

## 2. Como fiz (Arquitetura e Implementação)

### Fluxo de Dados

```text
Ignition Gazebo (Mundo LACEA)
       │
       ▼
Depth Camera Virtual (ascamera_hp60c)
       │
       ▼ (/ascamera_hp60c/camera_publisher/depth0/image_raw)
ros_gz_bridge
       │
       ▼
ROS 2 Humble
       │
       ▼
scripts/gap_phase35.py (Nó Gap Following)
       │
       ▼ (/cmd_vel)
ros_gz_bridge
       │
       ▼
TIGO Virtual (Plugin ignition-gazebo-mecanum-drive-system)
```

### Componentes Principais:

1. **Sensor de Profundidade:**
   - Tipo: Câmera de profundidade virtual (equivalente ao sensor real HP60C da Angstrong).
   - Tópico: `/ascamera_hp60c/camera_publisher/depth0/image_raw` (formato `32FC1` em metros).
   - Frequência: 15 Hz.

2. **Região de Interesse (ROI):**
   - Para maximizar a taxa de atualização e focar no espaço de circulação do robô, seleciona-se os 50% centrais verticais da matriz de profundidade.

3. **Segmentação de Espaço Livre:**
   - Cada coluna da ROI é avaliada: o percentil 10 das leituras válidas de cada coluna é calculado.
   - Uma coluna é considerada "livre" se sua distância for superior a `DIST_OBSTACULO` (0,7 m).

4. **Identificação e Ranqueamento de Gaps:**
   - O algoritmo identifica sequências contínuas de colunas livres com largura mínima de `LARGURA_ROBO_PX` (40 pixels).
   - Cada gap candidato recebe uma pontuação calculada por:
     $$\text{Score} = \text{Largura}_{\text{px}} \times 0{,}7 + \text{Profundidade}_{\text{média}} \times 25$$
   - O melhor gap é selecionado e a correção angular ($W_z$) é proporcional ao desvio do centro do gap em relação ao centro da imagem.

5. **Máquina de Estados de Recuperação:**
   - Quando a distância frontal fica abaixo de `DIST_GATILHO_RECUO` (0,6 m) ou nenhum gap direto é encontrado:
     1. O robô desacelera e recua progressivamente (tempo inicial de 1,8 s, incrementando 0,5 s a cada tentativa, até 2,8 s).
     2. Executa uma varredura giratória de 360° para a esquerda monitorando a pose publicada pelo Gazebo.
     3. Registra o gap mais largo observado durante a volta completa.
     4. Ao final da varredura, reorienta-se na direção desse melhor gap e retoma a navegação.

---

## 3. Problemas Enfrentados e Soluções Implementadas

### 3.1. Desalinhamento Visual e de Eixos de Movimento
- **Problema:** O modelo tridimensional exportado do CAD (`Tigo_para_gazebo.stl`) tinha seu comprimento posicionado ao longo do eixo `Y`. O plugin Mecanum do Gazebo e os sensores usam a convenção do ROS, onde `+X` é a frente. O robô se movia lateralmente ao receber comandos de avanço frontal.
- **Solução Implementada:** Aplicou-se uma rotação visual de `-90°` no link visual dentro de `urdf/tigo.sdf`, alinhando a dianteira física do chassi perfeitamente com o eixo `+X` e a câmera frontal.

### 3.2. Ruídos, NaNs e Raios Infinitos na Câmera Simulada
- **Problema:** Quando o sensor do Gazebo não encontra superfícies dentro do range (ex.: portas abertas ou corredores longos), a imagem retorna valores `inf` ou `nan`. Cálculos aritméticos padrão falhavam ou geravam exceções numéricas.
- **Solução Implementada:** Desenvolveu-se um tratamento numérico onde valores infinitos e NaNs são convertidos em um valor de segurança fixo de 10 m (espaço livre garantido). Além disso, a segmentação utiliza o percentil 10 em vez do mínimo absoluto, tornando o algoritmo imune a pixels ruidosos isolados.

### 3.3. Travamentos em Cantos e Obstáculos Frontais (Deadlock)
- **Problema:** Ao se aproximar de paredes perpendiculares ou cantos fechados, todos os pixels à frente ficavam abaixo do limiar de obstáculo. Sem saídas imediatas, o robô ficava parado indefinidamente.
- **Solução Implementada:** Criação de uma rotina de escape robusta com recuo temporal progressivo (1,8 s até 2,8 s) associado a uma volta de 360° com memória angular. O robô mede sua rotação por `/world/$GZ_WORLD_NAME/pose/info`, encontra a orientação do gap mais amplo e se realinha automaticamente.

### 3.4. Queda de Dados do Sensor e Segurança do Robô
- **Problema:** Caso a ponte de comunicação oscile ou a simulação congele, comandos antigos de velocidade poderiam continuar ativos, colidindo o robô.
- **Solução Implementada:** Implementação de um temporizador *Watchdog* de 0,3 s. Caso o callback de imagem não receba novas leituras dentro da janela, o robô para imediatamente e publica velocidade zero por segurança.

---

## 4. Como Executar

### 1. Iniciar Simulação e Bridge
```bash
# Terminal 1 - Simulação
cd /home/betinha/TIGO_Digital_Twin
./scripts/run_simulation.sh

# Terminal 2 - Bridge
cd /home/betinha/TIGO_Digital_Twin
export GZ_WORLD_NAME=lacea_world
./scripts/run_bridge.sh
```

### 2. Iniciar o Nó de Gap Following
```bash
# Terminal 3 - Nó de Navegação
cd /home/betinha/TIGO_Digital_Twin
source /opt/ros/humble/setup.bash
export GZ_WORLD_NAME=lacea_world
python3 scripts/gap_phase35.py
```

---

## 5. Estado Atual e Próximas Etapas

### Estado Atual:
- ✅ Câmera de profundidade virtual validada a 15 Hz em formato métrico (32FC1).
- ✅ Algoritmo Gap Following funcional com recuo incremental e varredura 360°.
- ✅ Comunicação bidirecional Gazebo ↔ ROS 2 via `run_bridge.sh`.
- ✅ Robô navegando de forma autônoma e reativa no ambiente do laboratório LACEA.

### Próximas Etapas:
- [ ] Inserir obstáculos dinâmicos no ambiente virtual para testar evasão em movimento.
- [ ] Conduzir testes comparativos de tempo e suavidade entre o Gap clássico e o Gap Semântico (`yolo_depth_gap.py`).
- [ ] Validar o algoritmo calibrado na plataforma robótica física no laboratório LACEA.
