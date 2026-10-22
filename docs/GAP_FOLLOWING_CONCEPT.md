# Gap Following

## O que é Gap Following?

Gap Following é uma estratégia de navegação reativa utilizada em robôs móveis para evitar obstáculos e encontrar caminhos livres em tempo real.

A ideia central é simples:

> O robô identifica espaços livres (gaps) entre obstáculos e navega através do gap mais seguro disponível.

Diferentemente de algoritmos baseados em mapeamento global, o Gap Following toma decisões utilizando apenas informações instantâneas do sensor.

---

# Motivação

Em ambientes desconhecidos ou dinâmicos, nem sempre é possível possuir um mapa completo.

Nesses casos, uma abordagem reativa apresenta vantagens:

- Baixo custo computacional
- Resposta rápida
- Implementação simples
- Operação em tempo real

Por esse motivo o método foi escolhido para o desenvolvimento inicial da navegação autônoma do TIGO.

---

# Conceito Básico

Imagine a seguinte situação:

```text
Parede        Espaço Livre        Parede

███████       _________       ███████
███████       _________       ███████
███████       _________       ███████
```

O espaço livre entre os obstáculos é chamado de:

```text
Gap
```

O algoritmo tenta conduzir o robô através do centro desse espaço.

---

# Como o Gap é Detectado

O sensor fornece informações do ambiente.

Exemplo:

```text
Livre Livre Livre Livre
Obstáculo Obstáculo
Livre Livre Livre Livre Livre
```

O algoritmo procura sequências contínuas de espaço livre.

Exemplo:

```text
LLLLLL OOO LLLLLLLLL OOO LLLL
```

onde:

```text
L = Livre

O = Obstáculo
```

Nesse exemplo existem três gaps.

---

# Seleção do Gap

Após identificar todos os gaps, o algoritmo determina qual deles é mais adequado.

Critérios comuns:

- Gap mais largo
- Gap mais próximo do centro
- Gap mais seguro
- Gap mais próximo do objetivo

---

# Limitação dos Métodos Clássicos

Em muitas implementações tradicionais o gap é calculado apenas em pixels.

Problema:

```text
Um gap aparentemente grande na imagem
pode ser pequeno na realidade.
```

Por isso foi necessária uma abordagem baseada em profundidade.

---

# Gap Following Baseado em Profundidade

O TIGO utiliza uma câmera de profundidade.

Sensor:

```text
HP60C (Angstrong)
```

Em vez de utilizar apenas a geometria da imagem, o algoritmo utiliza a distância real medida pelo sensor.

Isso permite estimar:

```text
Largura física do gap
```

---

# Modelo Pinhole

A largura real do espaço livre é calculada através do modelo de câmera Pinhole.

Equação:

```text
Largura Real = (Pixels × Profundidade) / fx
```

onde:

```text
fx = distância focal da câmera
```

No TIGO:

```text
fx = 566.83
```

---

# Vantagens da Abordagem

A utilização da profundidade permite:

- reduzir falsas detecções
- estimar larguras reais
- aumentar a segurança
- melhorar a tomada de decisão

---

# Aplicação no TIGO

O algoritmo desenvolvido para o TIGO executa:

```text
Mapa de Profundidade
        ↓
Região de Interesse (ROI)
        ↓
Segmentação do Espaço Livre
        ↓
Detecção de Gaps
        ↓
Validação Física
        ↓
Seleção do Gap
        ↓
Publicação de /cmd_vel
```

---

# Navegação Normal

Quando um gap válido é encontrado:

```text
Gap Encontrado
      ↓
Centralização
      ↓
Movimento
```

O robô continua avançando.

---

# Modo Escape

Quando nenhum gap válido é encontrado:

```text
Nenhum Gap
      ↓
Recuar
      ↓
Varredura
      ↓
Escolher direção
      ↓
Retomar navegação
```

---

# Gap Following x SLAM

## Gap Following

Vantagens:

- simples
- rápido
- leve computacionalmente

Desvantagens:

- não cria mapas
- não realiza planejamento global

---

## SLAM

Vantagens:

- cria mapas
- localiza o robô
- planeja trajetórias

Desvantagens:

- exige maior capacidade computacional

---

# Abordagem Utilizada no Projeto

O método desenvolvido para o TIGO pode ser classificado como:

```text
Depth-Based Gap Following
```

ou

```text
Gap Following Baseado em Profundidade
```

pois utiliza informações tridimensionais obtidas pela câmera HP60C para validar fisicamente cada espaço navegável.

---

# Próximos Passos

- Integração completa com o gêmeo digital
- Validação da câmera virtual
- Testes com obstáculos simulados
- Comparação com métodos baseados em SLAM

