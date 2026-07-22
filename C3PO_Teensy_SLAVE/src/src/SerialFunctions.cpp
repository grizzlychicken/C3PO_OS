// RS485 Serial Port Functions
// 485 used Serial Port 4
// TX4 - D17
// RX4 - D16
// RTS4 - D41
// RTS sets the Direction for half duplex operation.  Low - Receive Mode, High - Transmit
#include <src/SerialFunctions.hpp>
// #include <SBUSFunctions.hpp>
#include <src/MotionControl.hpp>

// #include <Audio.hpp>
#include "Definitions.h"

// create object

// struct SEND_DATA_STRUCTURE
// {
// 	// Send Data Structure - Must match Receive Structure
// 	uint16_t address = 99;	// Device address
// 	uint16_t command = 'z'; // Command to execute
// 	uint16_t Data1 = 0;
// 	uint16_t Data2 = 0;
// };

// // give a name to the group of data
// SEND_DATA_STRUCTURE mysenddata;

#define RS485SERIAL Serial4

// char rx_byte = 0;
// int gVisorPosition = 85;
// uint16_t gAudioLevel = 0;

elapsedMillis SerialElapsed;

void SetupSerial()
{
	RS485SERIAL.begin(115200);
	RS485SERIAL.transmitterEnable(41);

	Serial.println("Setup 485");
}

void SerialLoop()
{

	CheckSerialCommands();
}

void TxServoPosition(uint8_t Addr, uint8_t ServoNum, int ServoPositionuS)
{
	RS485SERIAL.print("<");
	RS485SERIAL.print(Addr);
	RS485SERIAL.print("SP");
	RS485SERIAL.print(ServoNum);
	RS485SERIAL.printf("%04d", ServoPositionuS);
	RS485SERIAL.print(">");

#if 1
	Serial.print("SP: ");
	Serial.print(Addr);
	Serial.print(" ");
	Serial.print(ServoNum);
	Serial.print(" P: ");
	Serial.println(ServoPositionuS);
#endif
}

// Command to attach (enable) a servo.  ServoNum 0 - attach all
void TxServoAttach(uint8_t Addr, uint8_t ServoNum)
{
	RS485SERIAL.print("<");
	RS485SERIAL.print(Addr);
	RS485SERIAL.print("SA");
	RS485SERIAL.print(ServoNum);
	RS485SERIAL.print(">");

#if 1
	Serial.print("Attach: ");
	Serial.print(Addr);
	Serial.print(ServoNum);
#endif
}

// Command to detach (disable) a servo.  ServoNum 0 - detach all
void TxServoDetach(uint8_t Addr, uint8_t ServoNum)
{
	RS485SERIAL.print("<");
	RS485SERIAL.print(Addr);
	RS485SERIAL.print("SD");
	RS485SERIAL.print(ServoNum);
	RS485SERIAL.print(">");

#if 1
	Serial.print("Detach: ");
	Serial.print(Addr);
	Serial.print(ServoNum);

#endif
}

void CheckSerialCommands()
{
	char rx_byte = 0;
	// int count = 0;
	if (SerialElapsed > 25)
	{

		// digitalWrite(9, !digitalRead(9));
		//  Serial Port Command Processing
		if (Serial.available() > 0)
		{							 // is a character available?
			rx_byte = Serial.read(); // get the character
			Serial.println(rx_byte);
			// check if a number was received

			if (rx_byte == '0')
			{
				Serial.println("Rx 0");
				// TxServoPosition(TORSO_ID, 1, 1800);
				// TxServoPosition(TORSO_ID, 2, 800);
			}
			if (rx_byte == '9')
			{
				Serial.println("Rx 9");
				// TxServoPosition(TORSO_ID, 1, 800);
				// TxServoPosition(TORSO_ID, 2, 1800);
			}
			if (rx_byte == '8')
			{
				Serial.println("Rx 8");

				// TxServoPosition(HEAD_ID, 1, 800);
				// TxServoPosition(HEAD_ID, 2, 1800);
			}
			if (rx_byte == '7')
			{
				Serial.println("Rx 7");

				// TxServoPosition(HEAD_ID, 1, 1800);
				// TxServoPosition(HEAD_ID, 2, 800);
			}

			if (rx_byte == 'T') //	Test all servos
			{
				Serial.println("Test all Servos");
				//TestAllServos();
		
			}

			if (rx_byte == 'A')
			{
				Serial.println("Rx A");
                ServoAttach(0);
				// TxServoAttach(HEAD_ID, 0);
				// TxServoAttach(SHOULDER_ID, 0);
				// TxServoAttach(ARM_ID, 0);
				// TxServoAttach(TORSO_ID, 0);
				// TxServoAttach(HEAD_ID, 0);
			}
			if (rx_byte == 'D')
			{
				Serial.println("Rx D");

				// TxServoDetach(HEAD_ID, 0);
				// TxServoDetach(SHOULDER_ID, 0);
				// TxServoDetach(ARM_ID, 0);
				// TxServoDetach(TORSO_ID, 0);
				// ServoDetach(0);
			}

			if (rx_byte == 'S')
			{
				Serial.println("Rx S");
				ToggleSBUSDebug();
			}

		} // end: if (Serial.available() > 0)
		SerialElapsed = 0;
	}
}
