#include "src/CommandStreamProvider.h"
#include "BottangoArduinoConfig.h"
#include "src/BottangoCore.h"

#ifdef ENABLE_STATUS_LIGHTS
#include "StatusLights.h"
#endif

CommandStreamProvider::CommandStreamProvider()
{
}

void CommandStreamProvider::runSetup()
{
    if (streamIsInProgress())
    {
        stop();
    }

#ifdef USE_CODE_COMMAND_STREAM
    commandStream = GeneratedCodeAnimations::GenerateSetupCommandStream();
#elif defined(USE_SD_CARD_COMMAND_STREAM)
    SDCardCommandStreamDataSource *dataSource = new SDCardCommandStreamDataSource();
    if (dataSource->isValid)
    {
        commandStream = new CommandStream(dataSource);
    }
    else
    {
        delete dataSource;
        commandStream = nullptr;
    }

#endif

    // run setup
    runInProgressCommand();

    // parse configs
    // will also run start animation if any
#if defined(USE_SD_CARD_COMMAND_STREAM) || defined(USE_CODE_COMMAND_STREAM)
    ExportedAnimationPlaybackControl::initialize();
#endif
}

void CommandStreamProvider::startCommandStream(byte animationIndex, bool loop)
{
    if (streamIsInProgress())
    {
        stop();
        Serial.println("Stopping existing animation...");
    }
    Serial.print("Starting CommandStream for animationIndex: ");
    Serial.println(animationIndex);
#ifdef ENABLE_STATUS_LIGHTS
    StatusLights::setDesiredColor(SIGNAL_STATUS_LIGHT, STATUS_COLOR_SIGNAL_OFFLINEPLAY);
#endif

#ifdef USE_CODE_COMMAND_STREAM
    commandStream = GeneratedCodeAnimations::GenerateCommandStreamByIndex(animationIndex);
    Serial.println("Generated command stream from code.");
#elif defined(USE_SD_CARD_COMMAND_STREAM)
    commandStream = new CommandStream(new SDCardCommandStreamDataSource(streamID, loop));
#endif

    if (commandStream != nullptr)
    {   Serial.println("CommandStream successfully created!");
        if (loop)
        {
            commandStream->setShouldLoop();
        }
        Time::syncTime(0);
    
            // New debug check: Is the animation actually running?
    if (streamIsInProgress())
            {
                Serial.println("✅ Animation is playing!");
            }
            else
            {
                Serial.println("⚠️ Animation did NOT start!");
            }
        }
        else
        {
            Serial.println("❌ Error: commandStream is NULL!");
        }
}

void CommandStreamProvider::runInProgressCommand()
{
    if (streamIsInProgress())
    {
        while (commandStream->readyForNextCommand())
        {
            char commandBuffer[MAX_COMMAND_LENGTH];
            commandBuffer[0] = '\0';

            commandStream->getNextCommand(commandBuffer);
            if (commandBuffer[0] != '\0')
            {
                BottangoCore::executeCommand(commandBuffer);
            }
        }

        if (commandStream->complete())
        {
            stop();
        }
    }
}

void CommandStreamProvider::updateOnLoop()
{
    runInProgressCommand();
#if defined(USE_CODE_COMMAND_STREAM) || defined(USE_SD_CARD_COMMAND_STREAM)
    ExportedAnimationPlaybackControl::updatePlaybackStatus();
#endif
}

void CommandStreamProvider::stop()
{
    delete commandStream;
    commandStream = nullptr;
    BottangoCore::effectorPool.clearAllCurves();

#ifdef ENABLE_STATUS_LIGHTS
    StatusLights::setDesiredColor(SIGNAL_STATUS_LIGHT, STATUS_COLOR_SIGNAL_OFFLINEREADY);
#endif
}

bool CommandStreamProvider::streamIsInProgress()
{
    return commandStream != nullptr;
}