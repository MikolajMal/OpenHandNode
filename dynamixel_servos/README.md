# Dynamixel Servo Control Node

### Instalacja niezbędnych pakietów dla systemu ROS-Kinetic:
```
$ sudo apt-get install ros-kinetic-dynamixel-sdk
$ sudo apt-get install ros-kinetic-qt-build
```

### Pobranie repozytorium Git:

```
$ git clone https://github.com/LRMPUT/OpenHandNode
```

### Skopiowanie folderu dynamixel_servos do katalogu catkin_ws:

```
$ cp ./OpenHandNode/dynamixel_servos  ../catkin_ws
```

### Kompilacja węzła dynamixel_servos:

```
$ cd ../catkin_ws
$ catkin_make
$ source ../catkin_ws/devel/setup.bash
```

### Uruchomienie węzła dynamixel_servos:

```
$ roscore
$ rosrun dynamixel_servos servo_control 
```

### Subskrypcja topic'u:

```
$ rostopic echo /servo_control_info 

servo_id: 21
present_current: 0
present_velocity: 0
present_position: 2000

```

### Wysłanie wiadomości do topic'u

##### Włączenie momentu:
```
$ rostopic pub /servo_control_commands dynamixel_servos/CommandMessage "servo_id: 21
register_address: 64
bytes_number: 1
value: 1" 
```

###### Zadanie pozycji serwa:
```
$ rostopic pub /servo_control_commands dynamixel_servos/CommandMessage "servo_id: 21
register_address: 116
bytes_number: 4
value: 2000" 
```

gdzie: 
**servo_id** - ID serwa
**register_address** - Adres rejestru
**bytes_number** - Rozmiar rejestru
**value** - Wartość do zapisania

[Tabela zawierająca adresy oraz rozmiary rejestrów](http://support.robotis.com/en/product/actuator/dynamixel_x/xm_series/xm430-w210.htm#bookmark23) 

