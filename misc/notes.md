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
export LIBGL_ALWAYS_INDIRECT=1 <---------- tem que testar com o 0 tb, mas o 1 deu certo
```

Para carregar o mapa no gazebo:

```bash
gz model -f Labirinto/model.sdf -m my_mesh
```

Para carregar o robô no gazebo:

```bash
ros2 launch turtlebot3_gazebo empty_world.launch.py
```