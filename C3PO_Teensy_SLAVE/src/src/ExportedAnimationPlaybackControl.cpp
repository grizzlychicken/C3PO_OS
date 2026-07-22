#include "src/ExportedAnimationPlaybackControl.h"
#include "src/BottangoCore.h"
#include "src/SDCardUtil.h"

#if defined(USE_SD_CARD_COMMAND_STREAM) || defined(USE_CODE_COMMAND_STREAM)
namespace ExportedAnimationPlaybackControl
{
    CircularArray<AnimationConfiguration> animationConfigs = CircularArray<AnimationConfiguration>(MAX_EXPORTED_ANIMATIONS);
    int currentPlayingIndex = -1;
    int defaultIndex = -1;

    void initialize()
    {
#ifdef USE_SD_CARD_COMMAND_STREAM
        SDCardUtil::initialize();
        if (!SDCardUtil::sdCardAvailable)
        {
            return;
        }

        // parse and build config files
        for (int i = 0; i < MAX_EXPORTED_ANIMATIONS; i++)
        {
            char path[MAX_FILE_PATH_SIZE];

            SDCardUtil::getAnimationFilePath(i, path, false, false); // check if data file exists
            if (!SDCardUtil::fileExists(path))
            {
                continue;
            }

            SDCardUtil::getAnimationFilePath(i, path, false, true); // check if config file exists
            if (!SDCardUtil::fileExists(path))
            {
                continue;
            }

            File configFile = SDCardUtil::openFile(path);
            if (!configFile)
            {
                continue;
            }

            AnimationConfiguration *config = parseConfiguration(configFile);

            // set up pins if needed
            if (config->playOnPin > 0)
            {
                pinMode(config->playOnPin, INPUT);
            }

            animationConfigs.pushBack(config);
        }
#elif defined(USE_CODE_COMMAND_STREAM)
        for (int i = 0; i < GeneratedCodeAnimations::getAnimationCount(); i++)
        {
            const uint16_t *configValues = GeneratedCodeAnimations::getConfigValues(i);
            AnimationConfiguration *config = parseConfiguration(configValues);
            animationConfigs.pushBack(config);
        }
#endif
        Serial.println(animationConfigs.size());
        // look for starting animation to play and idle animation
        for (int i = 0; i < animationConfigs.size(); i++)
        {
            AnimationConfiguration *checkConfig = animationConfigs.get(i);
            if (checkConfig != nullptr)
            {
                if (checkConfig->playOnStart == 1 && currentPlayingIndex == -1)
                {
                    BottangoCore::commandStreamProvider->startCommandStream(i, checkConfig->loopOnStart == 1);
                    currentPlayingIndex = i;
                }
                if (checkConfig->idleAnim == 1 && defaultIndex == -1)
                {
                    defaultIndex = i;
                }
            }
        }
    }

    void updatePlaybackStatus()
    {
        // something already playing
        if (BottangoCore::commandStreamProvider->streamIsInProgress())
        {
            // find an animation that wants to trigger
            int index = getIndexOfAnimationToTrigger(true);
            if (index >= 0 && index != currentPlayingIndex) // that isn't also the animation playing currently
            {
                AnimationConfiguration *nextConfig = animationConfigs.get(index);
                if (nextConfig != nullptr)
                {
                    BottangoCore::commandStreamProvider->startCommandStream(index, nextConfig->loop > 0);
                    currentPlayingIndex = index;
                }
            }
        }
        else
        {
            // nothing is playing
            if (currentPlayingIndex >= 0)
            {
                currentPlayingIndex = -1;
            }

            // something to trigger?
            int index = getIndexOfAnimationToTrigger(false);
            if (index >= 0)
            {
                AnimationConfiguration *nextConfig = animationConfigs.get(index);
                if (nextConfig != nullptr)
                {
                    BottangoCore::commandStreamProvider->startCommandStream(index, nextConfig->loop > 0);
                    currentPlayingIndex = index;
                }
            }
            // default to play when nothing should play?
            else if (defaultIndex >= 0 && currentPlayingIndex != defaultIndex)
            {
                AnimationConfiguration *nextConfig = animationConfigs.get(defaultIndex);
                if (nextConfig != nullptr)
                {
                    BottangoCore::commandStreamProvider->startCommandStream(defaultIndex, true);
                    currentPlayingIndex = defaultIndex;
                }
            }
        }
    }

#ifdef USE_SD_CARD_COMMAND_STREAM
    AnimationConfiguration *parseConfiguration(File configFile)
    {
        AnimationConfiguration *config = new AnimationConfiguration();
        byte lineIndex = 0;

        while (configFile.available())
        {
            // start of the line
            char c = configFile.read();

            // skip this line, it's a comment
            if (c == '/')
            {
                while (configFile.available())
                {
                    c = configFile.read();
                    if (c == '\n' || c == '\r')
                    {
                        break;
                    }
                }
            }

            // skip this line, it's blank
            if (c == '\n' || c == '\r')
            {
                continue;
            }

            // valid line
            char value[10];
            value[0] = c;
            for (int i = 1; i < 10; i++)
            {
                if (configFile.available())
                {
                    char cNext = configFile.read();
                    if (cNext == '\n' || cNext == '\r')
                    {
                        value[i] = '\0';
                        break;
                    }
                    else
                    {
                        value[i] = cNext;
                        if (i == 9)
                        {
                            value[10] = '\0';
                        }
                    }
                }
            }

            int parsedValue = atoi(value);

            switch (lineIndex)
            {
            case 0:
                config->playOnStart = parsedValue;
                break;
            case 1:
                config->loopOnStart = parsedValue;
                break;
            case 2:
                config->idleAnim = parsedValue;
                break;
            case 3:
                config->playOnPin = parsedValue;
                break;
            case 4:
                config->loop = parsedValue;
                break;
            case 5:
                config->playOnPinHigh = parsedValue;
                break;
            case 6:
                config->buttonLadderMin = parsedValue;
                break;
            case 7:
                config->buttonLadderMax = parsedValue;
                break;
            }

            lineIndex++;
        }
        return config;
    }
#endif
#ifdef USE_CODE_COMMAND_STREAM
    AnimationConfiguration *parseConfiguration(const uint16_t *configValues)
    {
        AnimationConfiguration *config = new AnimationConfiguration();
        config->playOnStart = configValues[0];
        config->loopOnStart = configValues[1];
        config->idleAnim = configValues[2];
        config->playOnPin = configValues[3];
        config->loop = configValues[4];
        config->playOnPinHigh = configValues[5];
        config->buttonLadderMin = configValues[6];
        config->buttonLadderMax = configValues[7];
        return config;
    }
#endif

    int getIndexOfAnimationToTrigger(bool interruptingAnimationsOnly)
    {
        for (int i = 0; i < MAX_EXPORTED_ANIMATIONS; i++)
        {
            AnimationConfiguration *checkConfig = animationConfigs.get(i);
            if (checkConfig != nullptr && checkConfig->playOnPin > 0 && i != currentPlayingIndex)
            {
                if (checkConfig->playOnPinHigh == 0)
                {
                    if (digitalRead(checkConfig->playOnPin) == LOW)
                    {
                        return i;
                    }
                }
                else if (checkConfig->playOnPinHigh == 1)
                {
                    if (digitalRead(checkConfig->playOnPin) == HIGH)
                    {
                        return i;
                    }
                }
                else
                {
                    uint16_t pinV = analogRead(checkConfig->playOnPin);
                    if (pinV >= checkConfig->buttonLadderMin && pinV <= checkConfig->buttonLadderMax)
                    {
                        return i;
                    }
                }
            }
        }
        return -1;
    }
}
#endif