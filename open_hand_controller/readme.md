# Open Hand Controller Node

### Instalacja niezbędnych pakietów
Nie jest wymagana instalacji dodatkowych zewnętrznych pakietów

### Instalacja pakietu Open Hand Controller
W celu instalacji, należy folder open_hand_controller umieścić w katalogu 
...
(adres workspace ros'a)/src
...

Następnie w terminalu przejść do katalogu z naszym workspace'em i wpisać:
...
$ source devel/setup.bash
$ catkin_make
...


### Zawartość
Głównym plikiem wykonywalnym jest wezel open_hand_controller, który komunikuje się z węzłem dynamixel_servos za pomocą topic'ów:
...
/servo_control_commands
/servo_control_info
...
oraz z wysyła i obiera wiadomości od systemu ROS za pomocą topic'ów:
...
/contr_to_ros
/ros_to_contr
...

Dodatkowo w pakiecie znajdują się dwa węzły ros test_talker i test_listener. Ich przeznaczeniem jest testowanie węzła pod kątem poprawności danych, które przez niego przechodzą na lini ROS - open_hand_controller

### open_hand_controller
Węzeł w komunikacji z końcówką manipulatora wykorzystuje przygotowane w osobnym pakiecie wiadomości oraz topic'i. Ta część działania programu została dopasowana do działającego projektu. W momencie odebrania wiadomości od elementu wykonawczego następuje aktualizacja danych przechowywanych na temat stanu serwomechanizmów.

Do komunikacji z systemem ROS zostały zaprojektowane dwie osobne wiadomości ros_to_contr oraz contr_to_ros, posiadające strukturę adekwatną do realizowanych zadań
...
ros_to_contr

float64 Position1
float64 Position2
float64 Position3
float64 Position4

float64 Torque1
float64 Torque2
float64 Torque3
float64 Torque4

bool enable1
bool enable2
bool enable3
bool enable4
...

...
contr_to_ros

float64 Position1
float64 Position2
float64 Position3
float64 Position4

float64 Velocity1
float64 Velocity2
float64 Velocity3
float64 Velocity4

float64 Torque1
float64 Torque2
float64 Torque3
float64 Torque4
...

Dane z serwomechanizmów są wysyłane przez węzeł z częstotliwością 10Hz.

Dane do serwomechanizmów należy wysyłać w momencie, kiedy zależy nam na zmianie któregoś z parametrów. Program jest zabezpieczony przed niektówymi przypadkami złego sformułowania wiadomości. Chcąc zmienić pozycję zadaną należy w polu Positionx (gdzie x to nr serwomechanizmu) podać odpowiednią wartość w radianach oraz aktywować odpowiedni serwomechanizm w polu enablex. 

Pole Positionx jest jedynym polem, którego edycja musi być przeprowadzona łącznie z polem enablex, aby uzyskać pożądany efekt. Wynika to z specyfiki działania układu.

### test_talker oraz test_listener
Oba wezły działają na podstawie topic'ów łączących węzeł open_hand_controller z systemem ROS.

test_listener po uruchomieniu wyświetla ostatnio przesłaną wiadomość do systemu ROS w kpnsoli.

test_talker posiada intefejs pozwalający na bezpośrednie wprowadzanie danych jakie mają zostać wysłane do serwomechanizmu. W celu obserwacji zmian zaleca się otwarcie nowego okna w konsoli i użycie komendy:
...
$ rostopic echo /servo_control_commands 
...