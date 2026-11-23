# Integração YOLOv8 + Câmera de Profundidade + Gap Following (Navegação Semântica)

## Projeto
TIGO Digital Twin – Laboratório de Controle e Automação (LACEA)  
**Autora:** Roberta Tavares  
**Orientadora:** Milena Faria Pinto  
**Instituição:** CEFET/RJ  

---

## 1. Visão Geral e Motivação

O algoritmo clássico **Gap Following** é uma técnica reativa puramente geométrica: ele analisa a distância dos obstáculos fornecida por um sensor (como uma câmera de profundidade ou pseudo-LiDAR) e encontra corredores livres por onde o robô possa navegar. 

No entanto, em ambientes reais e dinâmicos compartilhados com seres humanos e objetos variados, a navegação puramente geométrica apresenta limitações críticas:
- Trata todos os obstáculos de forma idêntica (uma parede estática, uma cadeira ou uma pessoa recebem a mesma margem de desvio).
- Não possui percepção semântica para antecipar comportamentos de risco (ex.: dar maior distância de segurança para pessoas ou animais).
- Pode ser induzido a erro por obstáculos reflexivos ou finos onde a câmera de profundidade falha.

Para superar essas limitações, foi desenvolvida a integração **YOLO + Profundidade + GAP**, permitindo ao robô TIGO realizar **Desvio Semântico de Obstáculos**: o robô compreende o que está à sua frente através de aprendizado profundo (Deep Learning) e adapta dinamicamente sua trajetória e margem de segurança física.

---

## 2. Arquitetura do Sistema

```text
       Ignition Gazebo (TIGO Virtual)
        ├── Câmera RGB (ascamera_hp60c_rgb)
        └── Câmera Profundidade (ascamera_hp60c)
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
 │   - Inflação de margem baseada na classe     │
 └──────────────────────┬───────────────────────┘
                        ▼
 ┌──────────────────────────────────────────────┐
 │       Algoritmo GAP Following Semântico      │
 │       - Detecção de corredores livres        │
 │       - Penalização de gaps próximos a risco │
 └──────────────────────┬───────────────────────┘
                        ▼
                 [/cmd_vel] (Twist)
                        │
                        ▼
               Movimentação do TIGO
```

### Componentes Principais:
1. **Câmera RGB Virtual ([urdf/tigo.sdf](file:///home/betinha/TIGO_Digital_Twin/urdf/tigo.sdf)):** Sensor óptico colorido alinhado perfeitamente com a câmera de profundidade, com resolução $640 \times 480$, taxa de $15\,\text{Hz}$ e formato `R8G8B8`.
2. **Ponte Gazebo ↔ ROS 2 ([scripts/run_bridge.sh](file:///home/betinha/TIGO_Digital_Twin/scripts/run_bridge.sh)):** Encaminha as imagens coloridas e de profundidade para o ecossistema ROS 2 através do pacote `ros_gz_bridge`.
3. **Módulo de Percepção ([scripts/yolo_depth_gap.py](file:///home/betinha/TIGO_Digital_Twin/scripts/yolo_depth_gap.py)):** Rede neural YOLOv8n executada de forma assíncrona para detecção e classificação em tempo real.
4. **Controlador Reativo de Navegação:** Avalia gaps no espaço métrico e calcula comandos de velocidade linear ($V_x$) e angular ($W_z$).

---

## 3. Metodologia e Formulação Matemática

### 3.1. Extração de Profundidade 3D dos Bounding Boxes
Para cada objeto detectado pelo YOLO com bounding box $[x_1, y_1, x_2, y_2]$, classe $c$ e confiança conf:
1. Uma sub-região de interesse (ROI) central é extraída para evitar ruídos de borda e do plano de fundo:
   $$x_{1,\text{roi}} = x_1 + 0{,}2 \cdot (x_2 - x_1), \quad x_{2,\text{roi}} = x_2 - 0{,}2 \cdot (x_2 - x_1)$$
   $$y_{1,\text{roi}} = y_1 + 0{,}2 \cdot (y_2 - y_1), \quad y_{2,\text{roi}} = y_2 - 0{,}2 \cdot (y_2 - y_1)$$
2. A distância métrica $Z_{\text{obj}}$ é obtida através do percentil 20 das leituras válidas da matriz de profundidade:
   $$Z_{\text{obj}} = \text{percentil}_{20}\left(D[y_{1,\text{roi}}:y_{2,\text{roi}},\, x_{1,\text{roi}}:x_{2,\text{roi}}]\right)$$
3. O ângulo horizontal do obstáculo em relação ao eixo frontal do robô é calculado por:
   $$\theta_{\text{obj}} = \left(\frac{x_{\text{centro}} - W / 2}{W}\right) \cdot \text{FOV}_H$$
   onde $W = 640\text{ px}$ e $\text{FOV}_H = 87^\circ \approx 1{,}518\text{ rad}$.

### 3.2. Inflação Semântica no Perfil de Profundidade
Cada classe possui uma margem física adicional de segurança $M_{\text{classe}}$ em metros:
- `person`: $0{,}40\,\text{m}$ (margem prioritária para pedestres)
- `chair` / `table` / `box`: $0{,}25\,\text{m}$
- Padrão geral: $0{,}20\,\text{m}$

Essa margem métrica é projetada dinamicamente para largura angular em pixels de acordo com a distância do objeto:
$$\Delta x_{\text{px}} = \text{int}\left(\frac{M_{\text{classe}}}{\max(Z_{\text{obj}} \cdot \text{FOV}_H, 0{,}1)} \cdot W\right)$$

O intervalo de colunas $[x_1 - \Delta x_{\text{px}},\, x_2 + \Delta x_{\text{px}}]$ é modificado no perfil de profundidade:
- Se $Z_{\text{obj}} < 1{,}0\,\text{m}$ (proximidade crítica): o perfil é forçado para $0{,}20\,\text{m}$ (bloqueio total dessa direção).
- Se $Z_{\text{obj}} < 2{,}5\,\text{m}$: o perfil é reduzido para $Z_{\text{obj}} \cdot 0{,}70$ (penalização virtual para incentivar o robô a buscar rotas mais distantes).

---

## 4. Problemas Encontrados e Soluções Implementadas

Durante o desenvolvimento e integração da percepção semântica com ROS 2 e Gazebo, foram superados quatro desafios de engenharia críticos:

### 4.1. Incompatibilidade de ABI do NumPy 2.x com `cv_bridge`
- **Problema:** Ao instalar a biblioteca Ultralytics (`pip install ultralytics`), o gerenciador `pip` atualizou a dependência do NumPy para a versão `2.2.6` (NumPy 2.x). Entretanto, os módulos C++ oficiais do ROS 2 Humble (como o `cv_bridge`) foram compilados para a ABI C do NumPy 1.x. Assim que o nó recebia o primeiro frame da câmera, o sistema abortava com:
  ```text
  AttributeError: _ARRAY_API not found
  Segmentation fault (core dumped)
  ```
  Isso causava a queda imediata do nó antes mesmo de publicar qualquer comando de movimento.
- **Solução Implementada:** Eliminou-se totalmente a dependência do `cv_bridge` no script. Desenvolveu-se a função nativa `converter_imagem_ros()` que realiza a conversão direta da memória de bytes ROS (`msg.data`) para matriz NumPy via `np.frombuffer()`. Essa implementação é 100% agnóstica à versão do NumPy, operando perfeitamente tanto no NumPy 1.x quanto no NumPy 2.x sem erros de ABI e com menor consumo de memória.

### 4.2. Latência da CPU com YOLO Síncrono e Disparo do Watchdog
- **Problema:** Inicialmente, a inferência do YOLOv8 na resolução nativa ($640 \times 480$) era executada sequencialmente dentro do callback de profundidade (`image_callback`). Na CPU, a inferência demorava entre $200\text{ a }450\,\text{ms}$. Como o temporizador de segurança do robô (`sensor_watchdog`) tinha tolerância de $400\,\text{ms}$, ele disparava o alarme de segurança e puxava o freio (`self.parar()`) a cada segundo:
  ```text
  [ERROR] Sem imagem de profundidade recente; robô parado por segurança.
  ```
  O robô apresentava comportamento com "soluços", andando e parando repetidamente de forma lenta ("lerda").
- **Solução Implementada:**
  1. **Arquitetura Assíncrona Multi-thread:** O processamento do YOLO foi isolado em uma *Worker Thread* dedicada (`threading.Thread`). O loop de controle do Gap Following roda a 15–20 Hz contínuos sem esperar pela rede neural ($< 2\,\text{ms}$ por ciclo).
  2. **Otimização de Inferência (`imgsz=320`):** Configurou-se o modelo para inferência em resolução otimizada de 320 px, reduzindo a latência na CPU de $200\,\text{ms}$ para apenas **$22{,}2\,\text{ms}$ ($45\,\text{FPS}$)**.
  3. **Calibração do Watchdog:** A tolerância foi ajustada para $1{,}2\,\text{s}$, eliminando paradas espúrias por picos transitórios de CPU.

### 4.3. Filtro Métrico Rígido vs Abertura Angular do Cenário
- **Problema:** Na primeira versão, foi adicionado um filtro que descartava qualquer gap cuja largura métrica calculada fosse inferior a $0{,}45\,\text{m}$ ($W_m = Z \cdot \frac{\text{px}}{W} \cdot \text{FOV}_H$). Como os obstáculos em corredores de desafio ficam a cerca de $0{,}8\,\text{m}$ de distância, para atingir $0{,}45\,\text{m}$ era necessária uma abertura de mais de $300\text{ pixels}$ (metade da tela). Com isso, todos os gaps eram rejeitados (`gaps = []`), fazendo o robô achar que estava bloqueado e não sair do lugar.
- **Solução Implementada:** Adotou-se o filtro em pixels calibrado (`LARGURA_ROBO_PX = 40`), combinado com a pontuação ponderada por profundidade *look-ahead* ($W_{\text{larg}} = 0{,}40$, $W_{\text{prof}} = 0{,}60$). Isso garantiu que corredores fisicamente passáveis fossem aceitos e priorizados.

### 4.4. Concorrência Gráfica e Ambiente Wayland/Gnome
- **Problema:** A chamada frequente de `cv2.imshow` e `cv2.waitKey` dentro da mesma thread do ROS travava o nó devido a incompatibilidades entre a biblioteca gráfica Qt/GTK e o servidor de exibição Wayland.
- **Solução Implementada:** A renderização da GUI foi limitada a 20 FPS através de controle temporal e encapsulada em blocos `try/except`. Caso a variável `DISPLAY` não esteja presente ou ocorra erro de janela, a interface gráfica é desativada suavemente sem interromper a navegação autônoma do robô.

---

## 5. Como Executar o Sistema

Para executar a simulação com a percepção semântica ativada:

### Terminal 1 – Simulação no Ignition Gazebo
```bash
cd /home/betinha/TIGO_Digital_Twin
./scripts/run_simulation.sh worlds/gap_challenge.sdf
```

### Terminal 2 – Ponte de Comunicação ROS 2 ↔ Gazebo
```bash
cd /home/betinha/TIGO_Digital_Twin
export GZ_WORLD_NAME=lacea_gap_challenge
./scripts/run_bridge.sh
```

### Terminal 3 – Nó de Navegação Semântica com Janela OpenCV
```bash
cd /home/betinha/TIGO_Digital_Twin
source /opt/ros/humble/setup.bash
export GZ_WORLD_NAME=lacea_gap_challenge
python3 scripts/yolo_depth_gap.py --gui
```

> **Opções de Linha de Comando:**
> - `--gui`: Abre a janela gráfica do OpenCV exibindo caixas delimitadoras coloridas, distância métrica dos obstáculos e vetores de navegação. *(Omitir caso queira executar em modo headless direto no terminal).*
> - `--conf 0.40`: Ajusta o limiar de confiança mínima do detector (padrão: `0.35`).
> - `--model yolov8s.pt`: Permite utilizar um modelo de pesos customizado.

---

## 6. Resultados e Comparação

| Característica | Gap Following Clássico (`gap_phase35.py`) | YOLO + Profundidade + GAP (`yolo_depth_gap.py`) |
| :--- | :--- | :--- |
| **Sensores Utilizados** | Câmera de Profundidade | Câmera RGB + Câmera de Profundidade (RGB-D) |
| **Percepção Semântica** | Não (trata todos os obstáculos iguais) | Sim (identifica classes: pessoas, cadeiras, caixas, etc.) |
| **Margens de Segurança** | Margem estática fixa | Margem dinâmica inflacionada conforme o tipo de objeto |
| **Interface Visual** | Apenas logs no terminal | Janela OpenCV com caixas 3D, distâncias e status do robô |
| **Arquitetura de Processamento** | Single-thread síncrono | Multi-thread assíncrono (Controle a 20 Hz + YOLO a 45 FPS) |
| **Comportamento em Bloqueio** | Recuo e varredura temporal | Desvio lateral ativo em tempo real ou varredura 360 inteligente |

O sistema demonstrou navegação fluida, sem travamentos de CPU, com desvios seguros e visualização em tempo real das decisões de controle.
