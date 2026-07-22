// RS485 Serial Port Functions
// 485 used Serial Port 4
// TX4 - D17
// RX4 - D16
// RTS4 - D41
// RTS sets the Direction for half duplex operation.  Low - Receive Mode, High - Transmit
#include <SerialFunctions.hpp>
#include "Definitions.h"
#include <ServoFunctions.hpp>

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

// char rx_byte = 0;

unsigned long SerialElapsed = millis();
unsigned long SerialElapsed485 = millis();

// RS485 Serial Port Functions
// Serial
char g485_rx_byte = 0;
const byte maxDataLength = 30; // maxDataLength is the maximum length allowed for received data.
char g485RxChars[64];
String RxString = "";
bool newData = false; // newData is used to determine if there is a new command
bool recvInProgress = false;
byte g485ndx = 0;
const char startMarker = '<';
const char endMarker = '>';

void SetupSerial()
{
	// HeadSerial.begin(115200);

	Serial.println("Setup Serial");
}

void SerialLoop()
{

	CheckSerialCommands();
	// UpdateHead();
}

void CheckSerialCommands()
{
	char rx_byte = 0;
	if (millis() - SerialElapsed > 25)
	{
		// digitalWrite(9, !digitalRead(9));
		//  Serial Port Command Processing
		if (Serial.available() > 0)
		{							 // is a character available?
			rx_byte = Serial.read(); // get the character

			// check if a number was received

			if (rx_byte == '0')
			{
				Serial.println("Received 0");
				//	StopAllMotors();
				// digitalWrite(5, HIGH);
				// digitalWrite(18, HIGH);
				// // delay(200);
				// Serial2.println("Received 0");
				// digitalWrite(5, LOW);
				// digitalWrite(18, LOW);
			}

		} // end: if (Serial.available() > 0)
		SerialElapsed = millis();
	}
}

void Check485Serial()
{
	if (millis() - SerialElapsed485 > 5)
	{

		// is a character available?
		for (int x = 0; x < min(RS485.available(), 24); x++)
		{
			g485_rx_byte = RS485.read(); // get the character
										   //  Serial.print(" x");
										   // Serial.print(x);
			//Serial.print(g485_rx_byte);
			if (recvInProgress == true)
			{
				if (g485_rx_byte != endMarker)
				{
					if (g485ndx < maxDataLength)
					{
						g485RxChars[g485ndx] = g485_rx_byte;
						g485ndx++;
						RxString += g485_rx_byte;
					}
				}
				else
				{
					g485RxChars[g485ndx] = '\0'; // terminate the string
					recvInProgress = false;
					g485ndx = 0;
					newData = true;
				}
			}
			else if (g485_rx_byte == startMarker)
			{
				recvInProgress = true;
			}
		} // end: if (Serial.available() > 0)
		if (newData)
		{
			ParseData(g485RxChars);
			// ParseDataString(RxString);
			// Serial.println(receivedChars);
			// Serial.println(RxString);
			RxString = "";
			newData = false;
		}
		SerialElapsed485 = millis();
	}
}

void ParseData(char RxChar[64])
{
	char Command;
	int Addr;
	//Serial.println(RxChar);
	Addr = charToInt(RxChar[0]);
	Command = RxChar[1];
	// Serial.println(Addr);
	// Serial.println(Command);
	if (Addr == BOARD_ID)
	{
		switch (Command)
		{
		case 'S':				  // Servo
			if (RxChar[2] == 'P') // Position
			{
				Serial.print("SP ");
				Serial.print(charToInt(RxChar[3])); // servo number
				Serial.println(atoi(RxChar + 4));	  // Servo position
				SetServoTarget(charToInt(RxChar[3]), atoi(RxChar + 4));
				break;
			}

			if (RxChar[2] == 'A') // Attach
			{
				Serial.print("SA ");
				Serial.println(charToInt(RxChar[3])); // servo number

				SetServoAttach(charToInt(RxChar[3]));
				break;
			}
			if (RxChar[2] == 'D') // Detach
			{
				Serial.print("SD ");
				Serial.println(charToInt(RxChar[3])); // servo number

				SetServoDetach(charToInt(RxChar[3]));
				break;
			}
			break;
		default:
			break;
		}
	}
}

int charToInt(char c)
{
	int arr[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
	return arr[c - '0'];
}
