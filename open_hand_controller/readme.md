# Open Hand Controller Node

### Instalacja niezbędnych pakietów
Nie jest wymagana instalacji dodatkowych zewnętrznych pakietów

### Instalacja pakietu Open Hand Controller
W celu instalacji, należy folder open_hand_controller umieścić w katalogu 
```
(adres workspace ros'a)/src
```

Następnie w terminalu przejść do katalogu z naszym workspace'em i wpisać:
```
$ source devel/setup.bash
$ catkin_make
```


### Zawartość
Głównym plikiem wykonywalnym jest wezel open_hand_controller, który komunikuje się z węzłem dynamixel_servos za pomocą topic'ów:
```
/servo_control_commands
/servo_control_info
```
oraz z wysyła i obiera wiadomości od systemu ROS za pomocą topic'ów:
```
/contr_to_ros
/ros_to_contr
```

Dodatkowo w pakiecie znajdują się dwa węzły ros test_talker i test_listener. Ich przeznaczeniem jest testowanie węzła pod kątem poprawności danych, które przez niego przechodzą na lini ROS - open_hand_controller

### open_hand_controller
Węzeł w komunikacji z końcówką manipulatora wykorzystuje przygotowane w osobnym pakiecie wiadomości oraz topic'i. Ta część działania programu została dopasowana do działającego projektu. W momencie odebrania wiadomości od elementu wykonawczego następuje aktualizacja danych przechowywanych na temat stanu serwomechanizmów.

W celu uruchomienia kontrolera należy użyć polecenia:
```
$ rosrun open_hand_controller open_hand_controller
```

Do komunikacji z systemem ROS zostały zaprojektowane dwie osobne wiadomości ros_to_contr oraz contr_to_ros, posiadające strukturę adekwatną do realizowanych zadań. Dodatkowo z chwytakiem można komunikować się za pomocą wiadomości close_hand.
```
ros_to_contr

float64 Finger1Position
float64 Finger2Position
float64 Finger3Position
float64 FingersRotationPosition

float64 Finger1Torque
float64 Finger2Torque
float64 Finger3Torque
float64 FingersRotationTorque

bool Finger1Enable
bool Finger2Enable
bool Finger3Enable
bool FingersRotationEnable
```
Chcąc zmienić pozycję zadaną należy w polu Finger_x_Position (gdzie _x_ to nr serwomechanizmu) podać odpowiednią wartość w radianach oraz aktywować odpowiedni serwomechanizm w polu Finger_x_Enable.

Ponieważ serwomechanizmy pracują w trybie sterowania pozycją i momentem, należy podać również wartość momentu maksymalnego jaki będzie mógł zostać użyty do wykonania danego ruchu. Wartośc ta powinna zawierać się w przedziale od 0 do 1.


Dane do serwomechanizmów należy wysyłać w momencie, kiedy zależy nam na zmianie któregoś z parametrów. Program jest zabezpieczony przed niektórymi przypadkami złego sformułowania wiadomości. 

```
contr_to_ros

float64 Finger1Position
float64 Finger2Position
float64 Finger3Position
float64 FingersRotationPosition

float64 Finger1Velocity
float64 Finger2Velocity
float64 Finger3Velocity
float64 FingersRotationVelocity

float64 Finger1Torque
float64 Finger2Torque
float64 Finger3Torque
float64 FingersRotationTorque
```

Wiadomości contr_to_ros są wysyłane z częstotliwością 10Hz. W przypadku z jakiegoś powodu przerwanie z serwomechanizmem zostanie przerwane, w wiadomościach będą przesyłane ostatnio zaktualizowane dane na temat stanu mechanizmu.

Wartości Position zawierają kąt w radianach, Velocity aktualną prędkość w poszczegónych napędach, a Torque aktualny moment na wale wyskalowany do wartości od 0 do 1.

```
close_hand

bool FingersClose
float64 FingersTorque
```

Wiadomość close_hand jest wysyłana do kontrolera w celu zamknięcia (true) lub otwarcia (false) chwytaka za pomocą jednej komendy. Dodatkowym parametrem jest maksymalny moment jaki może zostać użyty do tej operacji. Przykład użycia poprzez wywołanie w konsoli (zamknięcie szczęk z momentem o wartości 10% maksymalnego):
```
$ rostopic pub /close_hand open_hand_controller/close_hand "FingersClose: true FingersTorque: 0.1"
```

### test_talker oraz test_listener
Oba wezły działają na podstawie topic'ów łączących węzeł open_hand_controller z systemem ROS.

test_listener po uruchomieniu wyświetla ostatnio przesłaną wiadomość do systemu ROS w konsoli.

test_talker posiada intefejs pozwalający na bezpośrednie wprowadzanie danych jakie mają zostać wysłane do serwomechanizmu. W celu obserwacji zmian zaleca się otwarcie nowego okna w konsoli i użycie komendy:
```
$ rostopic echo /servo_control_commands 
```

Aby uruchomić węzły testowe należy użyć komendy:
W celu uruchomienia kontrolera należy użyć polecenia:
```
$ rosrun open_hand_controller test_talker
```
w osobnym oknie:
```
$ rosrun open_hand_controller test_listener
```

