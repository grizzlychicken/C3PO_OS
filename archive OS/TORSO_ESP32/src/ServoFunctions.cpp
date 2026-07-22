// Servo Functions
#include <ServoFunctions.hpp>
#include "Definitions.h"
#include <ESP32Servo.h>

// Create an array of Servo objects
Servo servos[NUM_SERVOS];
int currentPositions[NUM_SERVOS]; // Current positions of the servos
int targetPositions[NUM_SERVOS];  // Target positions of the servos
int servoSpeeds[NUM_SERVOS];	  // Speeds for the servos

unsigned long ServoMillis = millis();

void SetupServo()
{
	Serial.println("Setup Servo");
	// Attach servos to pins and initialize positions
}

void ServoLoop()
{
	if (millis() - ServoMillis > 25)
	{
		// Update servo positions gradually to achieve the desired speed
		for (uint8_t i = 0; i < NUM_SERVOS; i++)
		{
			// if (currentPositions[i] < targetPositions[i])
			// {
			// 	currentPositions[i] += servoSpeeds[i];
			// 	if (currentPositions[i] > targetPositions[i])
			// 	{
			// 		currentPositions[i] = targetPositions[i];
			// 	}
			// }
			// else if (currentPositions[i] > targetPositions[i])
			// {
			// 	currentPositions[i] -= servoSpeeds[i];
			// 	if (currentPositions[i] < targetPositions[i])
			// 	{
			// 		currentPositions[i] = targetPositions[i];
			// 	}
			// }

			currentPositions[i] = targetPositions[i];

			// if (currentPositions[i] >= 150)
			// {
			//     targetPositions[i] = 0;
			// }
			// if (currentPositions[i] <= 30)
			// {
			//     targetPositions[i] = 180;
			// }
			// Serial.print(" i: ");
			// Serial.print(i);
			// Serial.print(" CP: ");
			// Serial.print(currentPositions[i]);
			// Serial.print(" TP: ");
			// Serial.print(targetPositions[i]);
			servos[i].write(currentPositions[i]);

			ServoMillis = millis();
		}
		// Serial.println("");
	}
}

void SetServoTarget(uint8_t ServoNum, int TargetPos)
{
	targetPositions[ServoNum-1] = TargetPos;
}

void SetServoAttach(uint8_t ServoNum)
{
	if (ServoNum == 0)
	{ // attach all
		for (int i = 0; i < NUM_SERVOS; i++)
		{
			servos[i].attach(torsoServoPins[i]);
			// currentPositions[i] = 90; // Assuming initial position is 90
			// targetPositions[i] = 0;
			// servoSpeeds[i] = 5;
		}
	}
	else
	{
		servos[ServoNum-1].attach(torsoServoPins[ServoNum-1]);
	}
}

void SetServoDetach(uint8_t ServoNum)
{
	if (ServoNum == 0)
	{ // attach all
		for (int i = 0; i < NUM_SERVOS; i++)
		{
			servos[i].detach();
			// currentPositions[i] = 90; // Assuming initial position is 90
			// targetPositions[i] = 0;
			// servoSpeeds[i] = 5;
		}
	}
	else
	{
		servos[ServoNum-1].detach();
	}
}