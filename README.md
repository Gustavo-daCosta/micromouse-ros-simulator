## Vídeo explicativo da atividade


https://github.com/user-attachments/assets/b82a4ad8-cc13-476d-9339-8d7636ed1512




### Solution (Parte 1) - Floodfill com Mapa Completo

Implementação que usa o serviço `/get_map` para obter o labirinto completo e aplicar o algoritmo Floodfill para encontrar o caminho ótimo.

**Algoritmo:** Floodfill (baseado em BFS)
- Obtém o mapa completo no início via serviço `/get_map`
- Calcula distâncias de todas as células até o target
- Segue o gradiente descendente para chegar ao destino

**Funcionalidades ROS Utilizadas:**
- **Serviço Cliente:** `rclcpp::Client<cg_interfaces::srv::GetMap>` - Requisita mapa completo
- **Serviço Cliente:** `rclcpp::Client<cg_interfaces::srv::MoveCmd>` - Envia comandos de movimento
- **Chamadas Síncronas:** `client->async_send_request()` + `rclcpp::spin_until_future_complete()` - Aguarda resposta dos serviços

**Como Executar:**
1. Abra um terminal e inicie o jogo:
   ```bash
   ros2 run cg maze
   ```

2. Em outro terminal, compile o workspace (se ainda não compilou):
   ```bash
   cd ~/Documents/Projects/micromouse-ros-simulator
   colcon build --packages-select solution
   source install/setup.bash
   ```

3. Execute a solução:
   ```bash
   ros2 run solution maze_solver
   ```

### Solution Part 2 - DFS com Sensores

Implementação que usa apenas dados dos sensores (`/culling_games/robot_sensors`) para explorar o labirinto dinamicamente usando DFS com backtracking.

**Algoritmo:** Depth-First Search (DFS) puro
- Explora o labirinto usando apenas sensores
- Não entra no target durante exploração (detecta via sensores)
- Usa backtracking recursivo até encontrar o target
- Calcula tamanho do labirinto dinamicamente

**Funcionalidades ROS Utilizadas:**
- **Subscriber:** `rclcpp::Subscription<cg_interfaces::msg::RobotSensors>` - Recebe dados dos sensores continuamente
- **Serviço Cliente:** `rclcpp::Client<cg_interfaces::srv::MoveCmd>` - Envia comandos de movimento
- **QoS Profile:** `rclcpp::QoS(10)` - Configuração de qualidade de serviço para o subscriber
- **Callback:** Processa mensagens de sensores de forma assíncrona

**Como Executar:**
1. Abra um terminal e inicie o jogo:
   ```bash
   ros2 run cg maze
   ```

2. Em outro terminal, compile o workspace (se ainda não compilou):
   ```bash
   cd ~/Documents/Projects/micromouse-ros-simulator
   colcon build --packages-select solution_part2
   source install/setup.bash
   ```

3. Execute a solução:
   ```bash
   ros2 run solution_part2 maze_solver
   ```

