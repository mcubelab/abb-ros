#include <SerialStream.h>
#include <iostream>

using namespace LibSerial;

int main(int argc, char** argv)
{
  SerialStream serial_port("/dev/ttyACM0", SerialStreamBuf::BAUD_9600, SerialStreamBuf::CHAR_SIZE_8, SerialStreamBuf::PARITY_NONE, 1, SerialStreamBuf::FLOW_CONTROL_NONE);
  if (!serial_port.good())
  {
    std::cout << "couldn't open!\n";
  }

  char c;
  do
  {
    std::cin >> c;
    std::cout << c << "\n";
    serial_port << c << std::endl;
  }while(c != 'q');

  serial_port.Close();

  if (!serial_port.IsOpen())
  {
    std::cout << "It's closed!\n";
  }

  return 0;
}
