////////////////////////////////////////////////////////////////////////////////////////////////
// includes
#include "include\joint_defs.h"

#include "src\mcu_util\mcu_util.h"
#include "src\state_estimation\state_estimation.h"
#include "src\motor_control\motor_control.h"
#include "src\locomotion\locomotion.h"

////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  z_body_nominal = 230;
  leg_swing_percent = 0.6;

  initTeensy();
  initActuators();
  
  homeLeggedRobot();
  zeroIMUReading();
  standUp();

  Serial.println("Standing test");

  elapsedMillis timer_1;
  while(timer_1 < 500) {
    updateFunctions();
  }

  for (auto i = 0; i < 2; ++i) {
    // touchdown
    actuation_phase = ActuationPhases::kTouchDown;
    updateLegMotorsForTouchdown();
    while (!isInContact[gait_phase * 2] && !isInContact[gait_phase * 2 + 1]) {
      updateFunctions();
    }

    // swing
    actuation_phase = ActuationPhases::kRetractLeg;
    updateSwingLegSetpoints();
    updateLegMotorsForSwing();
    resetSwingLegContactState();
    while (!motors[gait_phase * 2].states_.holding && !motors[gait_phase * 2 + 1].states_.holding) {
      updateFunctions();
    }

    elapsedMillis timer_2;
    while(timer_2 < 400) {
      updateFunctions();
    }
  }

  stopActuators();
  closeDataFile();
}

void loop() {
}

void updateFunctions() {
  handleODriveCANMsg();
  parseTeensySerial();
  updateStates();
  updateMotorCommands();
  sendTelemetry();

  if (stop_signal) {
    stopActuators();    // only use if the legs are actually non-backdrviable; if not, the robot will drop when the motors turn off
    closeDataFile();
    delay(10000000);
  }
}
