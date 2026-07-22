#ifndef SRC_ACTUATORPRIVATE
#define SRC_ACTUATORPRIVATE

#include <MyActuator/ActuatorModel.h>

namespace MyActuator {
    class ActuatorPrivate {
        public:
        bool isRegistered = false;
        float output_ratio;

        ActuatorPrivate(ActuatorModel model)
        {
            switch (model) {
                case ActuatorModel::X4:
                    output_ratio = (1.0 / 6.0);
                    break;
                case ActuatorModel::RMD_X8:
                    output_ratio = (1.0 / 6.0);
                    break;
            }
        }
    };
};

#endif // SRC_ACTUATORPRIVATE
