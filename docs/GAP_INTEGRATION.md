# Integração do Algoritmo Gap Following

## Objetivo

Este documento descreve a integração entre o algoritmo de navegação autônoma do TIGO e o ambiente virtual desenvolvido no Ignition Gazebo.

O objetivo é permitir que o mesmo algoritmo utilizado no robô físico seja executado sem modificações dentro da simulação.

---

# Arquitetura Geral

Fluxo de dados:

```text
Ignition Gazebo
       │
       ▼
Depth Camera Virtual
       │
       ▼
ros_gz_bridge
       │
       ▼
ROS 2 Humble
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

# Algoritmo Utilizado

Arquivo:

```text
gap_phase35.py
```

Responsabilidade:

- detectar espaços livres
- identificar gaps navegáveis
- calcular direção segura
- publicar comandos de velocidade

---

# Sensor Utilizado

No robô real:

```text
HP60C (Angstrong)
```

Tipo:

```text
Câmera de Profundidade
```

Dados utilizados:

```text
Mapa de profundidade
```

---

# Tópico de Profundidade

O algoritmo espera receber imagens através de:

```text
/ascamera_hp60c/camera_publisher/depth0/image_raw
```

Tipo:

```text
sensor_msgs/msg/Image
```

---

# Região de Interesse (ROI)

Para reduzir processamento, apenas uma faixa horizontal da imagem é utilizada.

Objetivos:

- reduzir carga computacional
- aumentar FPS
- focar na área de navegação

---

# Segmentação do Espaço Livre

Cada coluna da ROI é analisada.

Uma coluna é considerada livre quando:

```text
profundidade > limiar configurado
```

---

# Detecção de Gaps

Após a segmentação:

```text
livre
ocupado
livre
livre
livre
ocupado
```

o algoritmo procura sequências contínuas de espaço livre.

Essas sequências são classificadas como:

```text
Gaps candidatos
```

---

# Validação Física

Nem todo gap encontrado é navegável.

É necessário verificar sua largura física.

Modelo utilizado:

```text
Modelo Pinhole
```

Equação:

```text
Largura Real = (Pixels × Profundidade) / fx
```

onde:

```text
fx = 566.83
```

---

# Critério de Aceitação

Um gap somente é aceito quando:

```text
largura_real >= largura_segura
```

A largura segura considera:

- dimensões do TIGO
- margem de segurança

---

# Seleção do Gap

Quando múltiplos gaps existem:

```text
Gap mais próximo do centro
```

ou

```text
Gap com melhor pontuação
```

é escolhido.

---

# Controle do Movimento

O algoritmo publica mensagens:

```text
geometry_msgs/msg/Twist
```

no tópico:

```text
/ cmd_vel
```

---

# Modos de Operação

## Navegação Normal

Condição:

```text
Gap válido encontrado
```

Comportamento:

- avanço frontal
- correção lateral
- alinhamento gradual

---

## Modo Escape

Condição:

```text
Nenhum gap válido encontrado
```

Comportamento:

```text
Recuar
↓
Varrer ambiente
↓
Escolher direção
↓
Retornar à navegação
```

---

# Filtro EMA

Foi implementado:

```text
Exponential Moving Average
```

Objetivo:

Suavizar oscilações.

Equação:

```text
erro_filtrado =
α × erro_atual +
(1 − α) × erro_anterior
```

Parâmetro utilizado:

```text
α = 0.3
```

---

# Integração com Gazebo

A comunicação ocorre através do:

```text
ros_gz_bridge
```

Bridge utilizada:

```bash
ros2 run ros_gz_bridge parameter_bridge \
/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist \
/ascamera_hp60c/camera_publisher/depth0/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image
```

---

# Testes Realizados

Durante os testes foram observados logs como:

```text
[BLOQUEADO] Nenhum gap direto

Varrendo DIREITA

DECISÃO: Desviar para a ESQUERDA

Contorno concluído
```

Isso confirmou:

- execução correta do algoritmo
- publicação de Twist
- funcionamento da máquina de estados

---

# Limitações Atuais

Ainda precisam ser validados:

- publicação da câmera virtual
- profundidade simulada
- resposta dinâmica do robô
- obstáculos virtuais

---

# Próximas Etapas

## Curto Prazo

- validar câmera virtual
- validar tópicos de profundidade

## Médio Prazo

- inserir obstáculos
- validar desvio autônomo

## Longo Prazo

- integração com SLAM
- integração com Nav2
- navegação autônoma completa

---

# Estado Atual

Concluído:

✅ Gap Following funcional

✅ Integração ROS 2

✅ Bridge configurada

✅ Simulação criada

✅ Estrutura pronta para validação completa

Em desenvolvimento:

🔄 Profundidade virtual

🔄 Navegação totalmente autônoma

🔄 Ambiente de testes com obstáculos
