#ifndef PHASECURRENTSTATUS_H
#define PHASECURRENTSTATUS_H

#include <stddef.h>
#include <stdint.h>

namespace MyActuator {
    struct PhaseCurrentStatus {
        uint8_t temp;

        // TODO: Normalize these values like Angle, Velocity
        int16_t phase_a_current;
        int16_t phase_b_current;
        int16_t phase_c_current;
    };
};

#endif /* PHASECURRENTSTATUS_H */
