// #include <Arduino.h>
// #include <EEPROM.h>  // for persistent breadcrumb storage

// // ---- Project Includes ----
// #include <src/SerialFunctions.hpp>
// #include <src/SBUSFunctions.hpp>
// #include <src/Audio.hpp>
// #include <src/Events.hpp>
// #include <src/MotionControl.hpp>
// #include "Definitions.h"
// #include <src/BottangoCore.h>

// // ---------------- Debug breadcrumbs & health ----------------
// volatile uint32_t g_breadcrumb = 0;          // write small codes like 1001, 2002, etc.
// elapsedMicros     g_loopDurationUs = 0;

// // Optional: LED heartbeat (pin 13 is onboard LED on Teensy 4.1)
// constexpr uint8_t LED_PIN = 13;
// elapsedMillis g_heartbeatMs = 0;
// bool g_ledOn = false;

// // Slow-loop logger
// uint32_t g_slowLoopCount = 0;
// uint32_t g_lastSlowLoopReportMs = 0;
// constexpr uint32_t SLOW_LOOP_US = 4000;  // warn if loop() > 4 ms

// // EEPROM address to store last breadcrumb
// constexpr int EEPROM_ADDR_BREADCRUMB = 0;

// // ---------------- Helpers ----------------
// static inline void mark(uint32_t code) {
//     g_breadcrumb = code;
//     EEPROM.put(EEPROM_ADDR_BREADCRUMB, g_breadcrumb); // writes immediately
// }

// // ---------------- Visual Breadcrumb Blink ----------------
// // void blinkCode(uint32_t code) {
// //     // Split code into hundreds, tens, units
// //     uint8_t hundreds = (code / 100) % 10;
// //     uint8_t tens     = (code / 10) % 10;
// //     uint8_t units    = code % 10;

//     // auto doBlink = [](uint8_t n) {
//     //     for (uint8_t i = 0; i < n; ++i) {
//     //         digitalWrite(LED_PIN, HIGH);
//     //         delay(200);
//     //         digitalWrite(LED_PIN, LOW);
//     //         delay(200);
//     //     }
//     // };

//     // doBlink(hundreds); delay(500); // pause between groups
//     // doBlink(tens);     delay(500);
//     // doBlink(units);    delay(500);
// // }

// // Print last breadcrumb from previous boot
// void printLastBreadcrumb() {
//     uint32_t lastBreadcrumb = 0;
//     EEPROM.get(EEPROM_ADDR_BREADCRUMB, lastBreadcrumb);
//     Serial.printf("Last known breadcrumb (EEPROM): %lu\n", (unsigned long)lastBreadcrumb);

//     // Blink LED to indicate last breadcrumb visually
//     // blinkCode(lastBreadcrumb);
// }

// /*
// ---------------- Blink Code Reference ----------------
// Each digit represents a blink group: hundreds / tens / units
// Example: 2030 -> 2 blinks, pause, 0 blinks, pause, 3 blinks

// 100  -> Setup started
// 110  -> SBUS setup complete
// 120  -> Serial setup complete
// 130  -> Audio setup complete
// 140  -> Motion setup complete
// 150  -> BottangoCore setup complete
// 199  -> Setup complete, ready
// 2000 -> Loop start
// 2010 -> SBUS loop processed
// 2020 -> Serial loop processed
// 2030 -> Audio loop processed
// 2040 -> BottangoCore loop processed
// 2050 -> RC/Bottango mode check
// 2060 -> Motion loop processed
// 2070 -> Event processing complete
// */

// void setup() {
//     Serial.begin(115200);
//     EEPROM.begin(); // Teensy 4.x EEPROM does not take size argument
//     pinMode(LED_PIN, OUTPUT);
//     digitalWrite(LED_PIN, LOW);

//     printLastBreadcrumb(); // print last breadcrumb and blink

//     mark(100); SBUSsetup();        mark(110);
//     SetupSerial();      mark(120);
//     AudioSetup();       mark(130);
//     SetupMotion();      mark(140);
//     BottangoCore::bottangoSetup(); mark(150);

//     digitalWrite(LED_PIN, HIGH); // indicate setup done
//     PlayWavA(4055);  // boot audio cue
//     mark(199);

//     Serial.println("Boot complete: C-3PO Teensy ready.");
// }

// void loop() {
//     g_loopDurationUs = 0; // reset timer
//     mark(2000); // loop start

//     SBUSloop();                 mark(2010);
//     SerialLoop();               mark(2020);
//     AudioLoop();                mark(2030);
//     BottangoCore::bottangoLoop(); mark(2040);
//     CheckRCBottangoMode();      mark(2050);
//     MotionLoop();               mark(2060);
//     ProcessEventsLoop();        mark(2070);

//     if (g_heartbeatMs >= 250) { // blink every 250 ms
//         g_heartbeatMs = 0;
//         g_ledOn = !g_ledOn;
//         digitalWrite(LED_PIN, g_ledOn);
//     }

//     if (g_loopDurationUs > SLOW_LOOP_US) {
//         g_slowLoopCount++;
//         const uint32_t nowMs = millis();
//         if (nowMs - g_lastSlowLoopReportMs > 500) {
//             g_lastSlowLoopReportMs = nowMs;
//             Serial.printf("WARN slow loop: %lu us (count=%lu) | breadcrumb=%lu\n",
//                           (unsigned long)g_loopDurationUs,
//                           (unsigned long)g_slowLoopCount,
//                           (unsigned long)g_breadcrumb);
//         }
//     }
// }





// #include <Arduino.h>
// #include <CrashReport.h>

// // ---- Project Includes ----
// #include <src/SerialFunctions.hpp>
// #include <src/SBUSFunctions.hpp>
// #include <src/Audio.hpp>
// #include <src/Events.hpp>
// #include <src/MotionControl.hpp>
// #include "Definitions.h"
// #include <src/BottangoCore.h>

// // ---------------- Debug breadcrumbs & health ----------------
// volatile uint32_t g_breadcrumb = 0;          // write small codes like 1001, 2002, etc.
// elapsedMicros     g_loopDurationUs = 0;

// // Optional: LED heartbeat (pin 13 is onboard LED on Teensy 4.1)
// constexpr uint8_t LED_PIN = 13;
// elapsedMillis g_heartbeatMs = 0;
// bool g_ledOn = false;

// // Slow-loop logger
// uint32_t g_slowLoopCount = 0;
// uint32_t g_lastSlowLoopReportMs = 0;
// constexpr uint32_t SLOW_LOOP_US = 4000;  // warn if loop() > 4 ms

// // ---------------- Helpers ----------------
// static inline void mark(uint32_t code) { g_breadcrumb = code; }

// // void printCrashReportIfAny() {
// //   while (!Serial && millis() < 4000) {} // allow USB to enumerate
// // if (CrashReport) {
// //     Serial.println("\n=== CrashReport from previous reset ===");
// //     Serial.print(CrashReport);
// //     Serial.println("=== End CrashReport ===\n");
// // }
// // Serial.printf("Last breadcrumb (RAM): %lu\n", (unsigned long)g_breadcrumb);
// // CrashReport.clear();  // <-- always clear, even if no report
// // }
// //   if (CrashReport) {
// //     Serial.println("\n=== CrashReport from previous reset ===");
// //     Serial.print(CrashReport);   // includes fault type, PC/LR, stack info
// //     Serial.printf("Last breadcrumb (RAM): %lu\n", (unsigned long)g_breadcrumb);
// //     Serial.println("=== End CrashReport ===\n");
// //     CrashReport.clear(); // important: clears fault flag so Teensy won’t get stuck
// //   }
// // }

// // ---------------- Setup ----------------
// void setup() {
//   Serial.begin(115200);
//   // printCrashReportIfAny();

//   pinMode(LED_PIN, OUTPUT);
//   digitalWrite(LED_PIN, LOW);

//   mark(100); // setup start

//   SBUSsetup();        mark(110);
//   SetupSerial();      mark(120);
//   AudioSetup();       mark(130);
//   SetupMotion();      mark(140);
//   BottangoCore::bottangoSetup(); mark(150);

//   digitalWrite(LED_PIN, HIGH); // setup done

//   PlayWavA(4055);  // boot audio cue
//   mark(199);

//   Serial.println("Boot complete: C-3PO Teensy ready.");
// }

// // ---------------- Loop ----------------
// void loop() {
//   g_loopDurationUs = 0; // reset timer
//   mark(2000); // loop start

//   // === Core project loops ===
//   SBUSloop();                 mark(2010);
//   SerialLoop();               mark(2020);
//   AudioLoop();                mark(2030);
//   BottangoCore::bottangoLoop(); mark(2040);
//   CheckRCBottangoMode();      mark(2050);
//   MotionLoop();               mark(2060);
//   ProcessEventsLoop();        mark(2070);

//   // --------- Health: heartbeat & slow-loop detector ----------
//   if (g_heartbeatMs >= 250) {              // blink every 250 ms
//     g_heartbeatMs = 0;
//     g_ledOn = !g_ledOn;
//     digitalWrite(LED_PIN, g_ledOn);
//   }

//   if (g_loopDurationUs > SLOW_LOOP_US) {
//     g_slowLoopCount++;
//     const uint32_t nowMs = millis();
//     if (nowMs - g_lastSlowLoopReportMs > 500) {
//       g_lastSlowLoopReportMs = nowMs;
//       Serial.printf("WARN slow loop: %lu us (count=%lu) | breadcrumb=%lu\n",
//                     (unsigned long)g_loopDurationUs,
//                     (unsigned long)g_slowLoopCount,
//                     (unsigned long)g_breadcrumb);
//     }
//   }
// }



// #include <Arduino.h>
// #include <src/SerialFunctions.hpp>
// #include <src/SBUSFunctions.hpp>
// #include <src/Audio.hpp>
// //#include <LEDFunctions.hpp>
// #include <src/Events.hpp>
// #include <src/MotionControl.hpp>
// #include "Definitions.h"
// #include <src/BottangoCore.h>


// void setup()
// {

//     // put your setup code here, to run once:
//     Serial.begin(115200);
//     Serial.println("Booting C-3PO");
//     SBUSsetup();
//     SetupSerial();
//     AudioSetup();
//         // SetupLEDs();
//     SetupMotion();
//     // BottangoCore::bottangoSetup();
//     pinMode(13, OUTPUT);
//     digitalWrite(13, HIGH); // Turn on LED to indicate setup done.
//     delay (25);
//     PlayWavA(4055);

// }

// void loop()
// {
//     SBUSloop();
//     SerialLoop();
//     AudioLoop();
//     // LEDLoop();
//     CheckRCBottangoMode();
//     MotionLoop();
//     // BottangoCore::bottangoLoop();
//     ProcessEventsLoop();
//     // loopSwitches();
//     // toggleControlMode();
// }

// #include <Arduino.h>
// #include <CrashReport.h>

// #include <src/SerialFunctions.hpp>
// #include <src/SBUSFunctions.hpp>
// #include <src/Audio.hpp>
// #include <src/Events.hpp>
// #include <src/MotionControl.hpp>
// #include "Definitions.h"
// #include <src/BottangoCore.h>

// // ---------------- Debug breadcrumbs & health ----------------
// volatile uint32_t g_breadcrumb = 0;          // write small codes like 1001, 2002, etc.
// elapsedMillis     g_sinceLoopMs = 0;
// elapsedMicros     g_loopDurationUs = 0;

// static inline void mark(uint32_t code) { g_breadcrumb = code; }

// // Optional: LED heartbeat (pin 13 is on-board LED on Teensy 4.1)
// constexpr uint8_t LED_PIN = 13;
// elapsedMillis g_heartbeatMs = 0;
// bool g_ledOn = false;

// // Slow-loop logger (prints once per violation window)
// uint32_t g_slowLoopCount = 0;
// uint32_t g_lastSlowLoopReportMs = 0;
// constexpr uint32_t SLOW_LOOP_US = 4000;  // warn if loop() ever exceeds 4 ms

// void printCrashReportIfAny() {
//   while (!Serial && millis() < 4000) {} // allow USB to enumerate for CrashReport print
//   if (CrashReport) {
//     Serial.println("\n=== CrashReport from previous reset ===");
//     Serial.print(CrashReport);   // includes fault type, PC/LR, stack info
//     Serial.println("=== End CrashReport ===\n");
//   }
//   // Also surface last breadcrumb from RAM (if not power-cycled)
//   Serial.printf("Last breadcrumb (RAM): %lu\n", (unsigned long)g_breadcrumb);
// }

// // --------------- Setup ---------------
// void setup() {
//   // Keep Serial very early for CrashReport visibility.
//   Serial.begin(115200);
//   printCrashReportIfAny();

//   pinMode(LED_PIN, OUTPUT);
//   digitalWrite(LED_PIN, LOW);

//   mark(100); // setup start
//   SBUSsetup();        mark(110);
//   SetupSerial();      mark(120);
//   AudioSetup();       mark(130);
//   SetupMotion();      mark(140);
//   // BottangoCore::bottangoSetup(); // uncomment when ready
//   mark(150);

//   // Indicate setup done
//   digitalWrite(LED_PIN, HIGH);

//   // Avoid long delays here—short pulse is fine
//   delay(10);

//   // Kick an audio cue last, once everything is ready
//   PlayWavA(4055);
//   mark(199); // setup complete
//   Serial.println("Booting C-3PO: setup complete.");
// }

// // --------------- Loop ---------------
// void loop() {
//   g_loopDurationUs = 0; // reset timing
//   mark(2000); // loop start sentinel

//   // Highest priority: keep S.Bus serviced first to avoid buffer overruns
//   SBUSloop();                 mark(2010);

//   // Keep comms flowing; avoid long prints inside these functions
//   SerialLoop();               mark(2020);

//   // Audio processing can be time-sensitive; keep it early but after S.Bus
//   AudioLoop();                mark(2030);

//   // If Bottango can starve other loops, gate it carefully (currently off here)
//   // BottangoCore::bottangoLoop(); mark(2040);

//   // Your mode checks and motion control
//   CheckRCBottangoMode();      mark(2050);
//   MotionLoop();               mark(2060);

//   // Events last so earlier subsystems stay fed
//   ProcessEventsLoop();        mark(2070);

//   // --------- Health: heartbeat & slow-loop detector ----------
//   if (g_heartbeatMs >= 250) {              // blink every 250 ms
//     g_heartbeatMs = 0;
//     g_ledOn = !g_ledOn;
//     digitalWrite(LED_PIN, g_ledOn);
//   }

//   // If the loop took too long, log occasionally so it doesn't spam
//   if (g_loopDurationUs > SLOW_LOOP_US) {
//     g_slowLoopCount++;
//     const uint32_t nowMs = millis();
//     if (nowMs - g_lastSlowLoopReportMs > 500) {
//       g_lastSlowLoopReportMs = nowMs;
//       Serial.printf("WARN slow loop: %lu us (count=%lu) | breadcrumb=%lu\n",
//                     (unsigned long)g_loopDurationUs,
//                     (unsigned long)g_slowLoopCount,
//                     (unsigned long)g_breadcrumb);
//     }
//   }
// }








// ****Curent Main code to reinsert after debug */
#include <Arduino.h>
#include <src/SerialFunctions.hpp>
#include <src/SBUSFunctions.hpp>
#include <src/Audio.hpp>
//#include <LEDFunctions.hpp>
#include <src/Events.hpp>
#include <src/MotionControl.hpp>
#include "Definitions.h"
#include <src/BottangoCore.h>


void setup()
{

    // put your setup code here, to run once:
    Serial.begin(115200);
    Serial.println("Booting C-3PO");
    SBUSsetup();
    SetupSerial();
    AudioSetup();
        // SetupLEDs();
    SetupMotion();
    // BottangoCore::bottangoSetup();
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH); // Turn on LED to indicate setup done.
    delay (25);
    PlayWavA(4055);

}

void loop()
{
    SBUSloop();
    SerialLoop();
    AudioLoop();
    // LEDLoop();
    CheckRCBottangoMode();
    MotionLoop();
    // BottangoCore::bottangoLoop();
    ProcessEventsLoop();
    // loopSwitches();
    // toggleControlMode();
}
