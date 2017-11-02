#include "dynamixel_servos/servo_control.h"


ServoControl::ServoControl()
{
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
}

ServoControl::~ServoControl()
{
	
}

bool ServoControl::open_port()
{
  // Open port
  if (portHandler->openPort())
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    printf("Press any key to continue...\n");
    getch();
    return false;
  }
}

bool ServoControl::close_port()
{
  // Close port
  portHandler->closePort();
}

bool ServoControl::set_baudrate()
{	
  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    printf("Succeeded to change the baudrate!\n");
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to continue...\n");
    getch();
    return false;
  }
}


void ServoControl::write_goal_position(uint8_t servo_id, uint32_t goal_position)
{
	// Write goal position
	uint8_t dxl_error = 0;
    int dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, servo_id, ADDR_PRO_goal_position, goal_position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
		packetHandler->printTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
		packetHandler->printRxPacketError(dxl_error);
    }
	
}

uint32_t ServoControl::read_present_position(uint8_t servo_id)
{
	// Read present position
	uint32_t present_position;
	uint8_t dxl_error;
    int dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, servo_id, ADDR_PRO_present_position, (uint32_t*)& present_position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
		packetHandler->printTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
		packetHandler->printRxPacketError(dxl_error);
    }
    
    return present_position;

}

uint16_t ServoControl::read_present_current(uint8_t servo_id)
{
	// Read present current
	uint16_t present_current;
	uint8_t dxl_error;
	
    int dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, servo_id, ADDR_PRO_present_current, (uint16_t*)& present_current, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
		packetHandler->printTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
		packetHandler->printRxPacketError(dxl_error);
    }
    
    return present_current;
}

uint32_t ServoControl::read_present_velocity(uint8_t servo_id)
{
	// Read present velocity
	uint32_t present_velocity;
	uint8_t dxl_error;
	
    int dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, servo_id, ADDR_PRO_present_velocity, (uint32_t*)& present_velocity, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
		packetHandler->printTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
		packetHandler->printRxPacketError(dxl_error);
    }
    
    return  present_velocity;
}

void ServoControl::enable_torque(uint8_t servo_id)
{
	// Enable Dynamixel Torque
	uint8_t dxl_error = 0;
	int dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, servo_id, ADDR_PRO_torque_enable, TORQUE_ENABLE, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
    {
		packetHandler->printTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
		packetHandler->printRxPacketError(dxl_error);
    }
}

void ServoControl::disable_torque(uint8_t servo_id)
{
	// Disable Dynamixel Torque
	uint8_t  dxl_error = 0;
	int dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, servo_id, ADDR_PRO_torque_enable, TORQUE_DISABLE, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
    {
		packetHandler->printTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
		packetHandler->printRxPacketError(dxl_error);
    }
}

void ServoControl::write_register(uint8_t servo_id, uint8_t address, uint8_t bytes_number, uint32_t value)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  if (bytes_number   == 1)
  {
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, servo_id, address, (uint8_t)value, &dxl_error);
  }
  else if (bytes_number == 2)
  {
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, servo_id, address, (uint16_t)value, &dxl_error);
  }
  else if (bytes_number == 4)
  {
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, servo_id, address, (uint32_t)value, &dxl_error);
  }
  
  if (dxl_comm_result != COMM_SUCCESS)
  {
	packetHandler->printTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
	packetHandler->printRxPacketError(dxl_error);
  }
  
}

int ServoControl::getch()
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

int ServoControl::kbhit(void)
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();
#endif
}

void ServoControl::command_callback(const dynamixel_servos::CommandMessage::ConstPtr& msg)
{
	
	this->write_register(msg->servo_id, msg->register_address, msg->bytes_number, msg->value);
	
	//ROS_INFO("Register address [msg->register_address] changed to: %d", msg->register_address, msg->value);
}

void sigintHandler(int sig)
{
	ServoControl servos;
	
	// Disable Torque
	for (int i = FIRST_ID; i < FIRST_ID + SERVOS_NUMBER; i++)
	servos.disable_torque(i);
	
	// Close serial port
	servos.close_port();
	
	ROS_INFO("Shutting down");
	ros::shutdown();
}

int main(int argc, char **argv)
{
  ServoControl servos;
//  servos.open_port();
 // servos.set_baudrate();
  
  ros::init(argc, argv, "servo_control");
  ros::NodeHandle n;
  ros::Publisher servo_publisher = n.advertise<dynamixel_servos::InfoMessage>("servo_control_info", 1000);
  ros::Subscriber servo_subscriber = n.subscribe("servo_control_commands",1000, &ServoControl::command_callback, &servos);
  ros::Rate loop_rate(10);
  
  // Override the default ros sigint (CTRL + C) handler.
  signal(SIGINT, sigintHandler);

  //for (int i = FIRST_ID; i < FIRST_ID + SERVOS_NUMBER; i++)
	//servos.enable_torque(i);
  
  while(ros::ok())
  {    
	  
	for (int id = FIRST_ID; id < FIRST_ID + SERVOS_NUMBER; id++)
	{
		dynamixel_servos::InfoMessage msg;
		msg.servo_id = id;		
		msg.present_current = servos.read_present_current(id);
		msg.present_velocity = servos.read_present_velocity(id);
		msg.present_position = servos.read_present_position(id);
			
		servo_publisher.publish(msg); 	
	}
	   
	ros::spinOnce(); 	
	loop_rate.sleep();
  }
  
  return 0;
}
