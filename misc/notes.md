# Anotações aleatórias

## Pacotes instalados

... e outras alterações a desfazer depois

- terminator
- ros* (um monte de coisa)
- registry / repository
- ros-foxy-gazebo-ros-pkgs
- entradas no bashrc
- verificar regras de firewall para Xming e vcXsrv (temp. desab.)

## Obs

Config X server

```bash
export DISPLAY=$(grep nameserver /etc/resolv.conf | awk '{print $2}'):0.0
export LIBGL_ALWAYS_INDIRECT=0
```

Para iniciar o gazebo com mapa e robô:

```bash
cd submodule
colcon build --symlink-install
. install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo maze_world.launch.py
```

Depois disso, podemos executar o comando para mover o Burguer com:
```bash
cd submodule
ros2 run custom_turtle turtle_movement 
```

Para criar o mapa:
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_cartographer cartographer.launch.py
```


ros2 run turtlebot3_teleop teleop_keyboard

## TODO

- [x] Carregar mapa e robô no gazebo
- [x] Comunicar do ROS2 para o robo gazebo
- [x] Fazer robô andar para frente
- [x] Fazer robô virar 90º para a direita
- [ ] Teste do "bogosort" -- andar aleatoriamente até achar a saída enquanto evita paredes

- [ ] Estimar posição do robô
- [ ] Algoritmo para encontrar saída

PS. acho que vai ser complicado fazer esses dois últimos, mas vamos ver