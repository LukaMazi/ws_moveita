## Terminal 1 jojoo fjhdsajdqs  gfdgfg
to je spreme,mba
```zsh
cd ~/fr3_ws
source install/setup.zsh
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=192.168.1.11
```

## Terminal 2

```zsh
cd ~/ws_moveita
source ~/fr3_ws/install/setup.zsh
source install/setup.zsh
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=192.168.1.11

ros2 launch hello_moveit hello_moveit.launch.py 
```


tenks dela !