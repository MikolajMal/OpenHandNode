# Dynamixel Servo Control Node

### Instalacja niezbędnych pakietów dla systemu ROS-Kinetic:
```
sudo apt-get install ros-kinetic-dynamixel-sdk
sudo apt-get install ros-kinetic-qt-build
```

### Pobranie repozytorium z Githuba:

```
git clone https://github.com/LRMPUT/OpenHandNode
```

### Skopiowanie folderu dynamixel_servos do katalogu catkin_ws

```
cp ./OpenHandNode/dynamixel_servos  ../catkin_ws
```

### Kompilacja węzła dynamixel_servos

```
cd ../catkin_ws
catkin_make
source ../catkin_ws/devel/setup.bash
```

### Uruchomienie węzła dynamixel_servos

```
roscore
rosrun dynamixel_servos servo_control 
```

### Subskrypcja topic'u:

```
rostopic echo /servo_control_info 
```

### Wysłanie wiadomości do topic'u - przykładowo zadanie pozycji serwa o id = 21 :

```
rostopic pub /servo_control_commands dynamixel_servos/CommandMessage "command: 1
servo_id: 21
value: 0" 

rostopic pub /servo_control_commands dynamixel_servos/CommandMessage "command: 2
servo_id: 21
value: 2000" 
 
```
gdzie **command** przyjmuje wartości:

```
disableTorque = 0,
enableTorque = 1,
writeGoalPosition = 2
```
