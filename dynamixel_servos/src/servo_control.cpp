#include "dynamixel_servos/servo_control.h"


ServoControl::ServoControl()
{
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  open_port();
  set_baudrate();
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
    printf("Press any key to terminate...\n");
    getch();
    return false;
  }
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
    printf("Press any key to terminate...\n");
    getch();
    return false;
  }
}

uint32_t ServoControl::read_present_position(uint8_t servo_id)
{
	// Read present position
	uint32_t present_position;
	uint8_t dxl_error;
    int dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, servo_id, ADDR_PRO_PRESENT_POSITION, (uint32_t*)& present_position, &dxl_error);
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

void ServoControl::write_goal_position(uint8_t servo_id, uint32_t goal_position)
{
	// Write goal position
	uint8_t dxl_error = 0;
    int dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, servo_id, ADDR_PRO_GOAL_POSITION, goal_position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
		packetHandler->printTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
		packetHandler->printRxPacketError(dxl_error);
    }
	
}

uint16_t ServoControl::read_present_current_value(uint8_t servo_id)
{
	// Read present current
	uint32_t present_current;
	uint8_t dxl_error;
    int dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, servo_id, ADDR_PRO_CURRENT_VALUE, (uint16_t*)& present_current, &dxl_error);
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

void ServoControl::enable_torque(uint8_t servo_id)
{
	// Enable Dynamixel Torque
	uint8_t dxl_error = 0;
	int dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, servo_id, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
    {
		packetHandler->printTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
		packetHandler->printRxPacketError(dxl_error);
    }
	else
	{
		printf("Dynamixel has been successfully connected \n");
	}
}

void ServoControl::test()
{
	printf("Dynamixel has been successfully connected \n");
}

void ServoControl::disable_torque(uint8_t servo_id)
{
	// Disable Dynamixel Torque
	uint8_t  dxl_error = 0;
	int dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, servo_id, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
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
	switch(msg->command)
	{
		case Commands(disableTorque) :
			this->disable_torque(msg->servo_id);
		break;
		
		case Commands(enableTorque) :
			this->enable_torque(msg->servo_id);
		break;
		
		case Commands(writeGoalPosition):
			this->write_goal_position(msg->servo_id, msg->value);
		
		break;
	}
	ROS_INFO("I heard: [%d]", msg->servo_id);
}


int main(int argc, char **argv)
{
  ServoControl servos;
    
  ros::init(argc, argv, "servo_control");
  ros::NodeHandle n;
  ros::Publisher servo_publisher = n.advertise<dynamixel_servos::InfoMessage>("servo_control_info", 1000);
  ros::Subscriber servo_subscriber = n.subscribe("servo_control_commands",1000, &ServoControl::command_callback, &servos);
  ros::Rate loop_rate(10);
 
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  int dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE};         // Goal position

  uint8_t dxl_error = 0;                          // Dynamixel error
  int32_t dxl_present_position = 0;               // Present position
   
 // servo.test();
 servos.enable_torque(23);
 servos.disable_torque(22);
  servos.disable_torque(21);
  
  while(1)
  {    
	  
	 // for (uint8_t i=0; i < 3; i++)
	  //{
	//	  my_dynamixel::InfoMessage msg;
	//	  msg.servo_id = 21+i;
	//	  msg.present_position = servos.read_present_position(21+i);
	//	  servo_publisher.publish(msg); 	
	 // }
	 
	 dynamixel_servos::InfoMessage msg;
	 msg.servo_id = 23;
	 msg.present_position = servos.read_present_position(23);
	 msg.present_current = servos.read_present_current_value(23);
	 servo_publisher.publish(msg); 
	  
    ros::spinOnce(); 	
    loop_rate.sleep();
  }
  // Nie wchodzi
  servos.disable_torque(21);
  servos.disable_torque(22);
  servos.disable_torque(23);
  return 0;
}
