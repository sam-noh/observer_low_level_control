#include "locomotion.h"
#include "../../include/joint_defs.h"
#include "../mcu_util/mcu_util.h"
#include "../motor_control/motor_control.h"
#include "../state_estimation/state_estimation.h"

std::vector<std::vector<float>> touchdown_torque = {
  {0.25, 0.18, 0.14}, // 0.18 and 0.12
  {0.25, 0.18, 0.14},
  {0.25, 0.18, 0.14},
  {0.25, 0.18, 0.14}
};

// gait variables
std::vector<float> cmd_vector = {0, 0};                 // command vector: {forward-back, yaw angle}
uint8_t gait_phase = GaitPhases::kLateralSwing;         // current gait phase/swing body; (0 medial swing/lateral stance) -> (1 medial stance/lateral swing); double support is omitted
uint8_t actuation_phase = ActuationPhases::kRetractLeg; // current actuation phase of the swing legs; 0 retract -> 1 translate -> 2 touchdown
uint32_t gait_cycles = 0;                               // number of completed gait cycles
bool isBlocking = false;                                // true if any motion primitive outside of the standard gait cycle is in progress
std::queue<MotionPrimitives> mp_queue;

// nominal leg trajectory parameters; can be updated by a high-level planner
// exact trajectory is determined by the motor controller's trapezoidal trajectory generation: acceleration, deceleration, max velocity
float z_body_nominal = 180;               // nominal body height over local terrain in mm; currently taken as avg of stance leg motors joint position
float leg_swing_percent = 0.9;            // swing leg stroke as a percentage of its stroke at last stance phase

// actuation phase transition parameters
// these are currently fixed and not exposed for easier teleop
float swing_percent_at_translate = 0.5;   // percentage of swing leg retraction after which translation begins; small values can cause swing legs to collide with rough terrains
float trans_percent_at_touchdown = 0.4;   // percentage of translatonal displacement from midpoint after which leg touchdown begins; small values can result in leg touchdown before the translation completes, resulting in some backward motion after stance switch
float yaw_percent_at_touchdown = 0.6;     // percentage of yaw command from midpoint after which leg touchdown begins; small values can result in leg touchdown before the turning completes, resulting in some backward motion after stance switch

std::vector<float> q_leg_contact = {kQLegMax, kQLegMax};    // position of the swing leg actuators when they were last in contact
std::vector<float> q_leg_swing = {kQLegMin, kQLegMin};      // position setpoint of swing leg actuators during leg retraction

void homeLeggedRobot() {
  snprintf(sent_data, sizeof(sent_data), "Homing all legs...\n\n");
  writeToSerial();

  // leg actuator homing
  for (uint8_t idx_motor = 0; idx_motor < kNumOfLegs/2; ++idx_motor) {
    motors[idx_motor].states_.current_limit = kCurrentLegMaxHoming;
    motors[idx_motor].states_.ctrl_mode = ODriveTeensyCAN::ControlMode_t::kVelocityControl;
    motors[idx_motor].states_.q_dot_d = kQdotLegHoming;
  }

  bool moving[4] = {true, true, true, true};
  uint32_t t_current = millis();

  // wait a minimum time to allow motors to start moving
  while (moving[0] || moving[1] || moving[2] || moving[3]) {
    handleODriveCANMsg();
    updateStates();
    updateMotorCommands();

    for (uint8_t idx_motor = 0; idx_motor < kNumOfLegs/2; ++idx_motor) {
      // if the motor has slowed down, it's at the joint limit
      if (moving[idx_motor] && millis() - t_current > 1000 && fabs(motors[idx_motor].states_.q_dot) < fabs(kQdotLegHomingStop)) {
        moving[idx_motor] = false;
        
        motors[idx_motor].states_.q_dot_d = 0;
        motors[idx_motor].states_.ctrl_mode = ODriveTeensyCAN::ControlMode_t::kPositionControl;
        motors[idx_motor].states_.trap_traj_vel_limit = kVelLegTrajStandup;
        motors[idx_motor].states_.current_limit = kCurrentLegMax;
                
        motors[idx_motor].states_.homed = true;
        motors[idx_motor].states_.pos_home = motors[idx_motor].states_.pos_abs;
        motors[idx_motor].states_.pos_rel = motors[idx_motor].states_.pos_abs - motors[idx_motor].states_.pos_home;
        encoders[idx_motor*2].write(0);
        encoders[idx_motor*2 + 1].write(0);
        
        snprintf(sent_data, sizeof(sent_data), "Actuator %d homed.\n******************************************************\n", idx_motor + 1);
        writeToSerial();
      }
    }
    
    if (stop_signal) {
      stopActuators();
      while (1) {}
    }
  }

  snprintf(sent_data, sizeof(sent_data), "Finished homing the legs.\n\n");
  writeToSerial();

  // lift the body slightly to allow locomotion mechanism homing
  // depends on which body starts in stance
  snprintf(sent_data, sizeof(sent_data), "Lifting body off the ground...\n\n");
  writeToSerial();

  uint8_t stance = (gait_phase + 1) % kNumOfGaitPhases;
  isInContact[stance*2] = true;
  isInContact[stance*2 + 1] = true;
  updateMotorsClimb(stance, kDqLegLift);
  
  t_current = millis();
  while (millis() - t_current < 500 || (fabs(motors[stance*2].states_.q_dot) > fabs(kQdotLegHomingStop)
                                        && fabs(motors[stance*2 + 1].states_.q_dot) > fabs(kQdotLegHomingStop))) {
    handleODriveCANMsg();
    updateStates();
    updateMotorCommands();
    if (stop_signal) {
      stopActuators();
      while (1) {}
    }
  }

  // yaw actuator homing
  snprintf(sent_data, sizeof(sent_data), "Homing the yaw mechanism...\n");
  writeToSerial();

  motors[MotorID::kMotorYaw].states_.ctrl_mode = ODriveTeensyCAN::ControlMode_t::kPositionControl;
  motors[MotorID::kMotorYaw].states_.q_d = 0;   // yaw mechanism's range of motion is < 1 rev so the absolute encoder can be used directly
                                                // if the ODrive firmware changes, check the API to ensure this is the correct command for absolute position control

  t_current = millis();
  while (millis() - t_current < 500 || fabs(motors[MotorID::kMotorYaw].states_.q_dot) > fabs(kQdotYawHomingStop)) {
    handleODriveCANMsg();
    updateStates();
    updateMotorCommands();
    if (stop_signal) {
      stopActuators();
      while (1) {}
    }
  }

  motors[MotorID::kMotorYaw].states_.homed = true;
  motors[MotorID::kMotorYaw].states_.pos_home = motors[MotorID::kMotorYaw].states_.pos_abs;
  motors[MotorID::kMotorYaw].states_.pos_rel = motors[MotorID::kMotorYaw].states_.pos_abs - motors[MotorID::kMotorYaw].states_.pos_home;

  snprintf(sent_data, sizeof(sent_data), "Finished homing the yaw mechanism.\n\n");
  writeToSerial();

  // translation actuator homing
  snprintf(sent_data, sizeof(sent_data), "Homing the translation mechanism...\n");
  writeToSerial();

  motors[MotorID::kMotorTranslate].states_.ctrl_mode = ODriveTeensyCAN::ControlMode_t::kVelocityControl;
  motors[MotorID::kMotorTranslate].states_.current_limit = kCurrentTransMaxHoming;
  motors[MotorID::kMotorTranslate].states_.q_dot_d = kQdotTransHoming;

  bool moving2 = true;
  t_current = millis();

  while (moving2) {
    handleODriveCANMsg();
    updateStates();
    updateMotorCommands();
    if (stop_signal) {
      stopActuators();
      while (1) {}
    }
    
    if (millis() - t_current > 500 && fabs(motors[MotorID::kMotorTranslate].states_.q_dot) < fabs(kQdotTransHomingStop)) {
      moving2 = false;

      snprintf(sent_data, sizeof(sent_data), "Reached translation joint limit.\n");
      writeToSerial();

      motors[MotorID::kMotorTranslate].states_.q_dot_d = 0;
      motors[MotorID::kMotorTranslate].states_.ctrl_mode = ODriveTeensyCAN::ControlMode_t::kPositionControl;
      motors[MotorID::kMotorTranslate].states_.current_limit = kCurrentTransMax;
      motors[MotorID::kMotorTranslate].states_.q_d = motors[MotorID::kMotorTranslate].params_.direction*
                                                     motors[MotorID::kMotorTranslate].states_.pos_abs*
                                                     motors[MotorID::kMotorTranslate].params_.T + kQTransHomingOffset; // move to the zero position
    }
  }

  t_current = millis();
  while (millis() - t_current < 500 || fabs(motors[MotorID::kMotorTranslate].states_.q_dot) > fabs(kQdotTransHomingStop)) {
    handleODriveCANMsg();
    updateStates();
    updateMotorCommands();
    if (stop_signal) {
      stopActuators();
      while (1) {}
    }
  }

  motors[MotorID::kMotorTranslate].states_.homed = true;
  motors[MotorID::kMotorTranslate].states_.pos_home = motors[MotorID::kMotorTranslate].states_.pos_abs;
  motors[MotorID::kMotorTranslate].states_.pos_rel = motors[MotorID::kMotorTranslate].states_.pos_abs - motors[MotorID::kMotorTranslate].states_.pos_home;
  motors[MotorID::kMotorTranslate].states_.q_d = 0;

  snprintf(sent_data, sizeof(sent_data), "Finished homing the translation mechanism.\n\n");
  writeToSerial();

  snprintf(sent_data, sizeof(sent_data), "Homing finished.\n---------------------------------------------\n\n");
  writeToSerial();
}

void standUp() {
  snprintf(sent_data, sizeof(sent_data), "Standing up...\n");
  writeToSerial();
  
  uint8_t stance = (gait_phase + 1) % kNumOfGaitPhases;
  updateMotorsClimb(stance, z_body_nominal - kDqLegLift);

  // update states while standing up
  while (!motors[stance * 2].states_.holding && !motors[stance * 2 + 1].states_.holding) {
    handleODriveCANMsg();
    updateStates();
    updateMotorCommands();
  }

  updateMotorsStance(stance);
  
  snprintf(sent_data, sizeof(sent_data), "Starting\n");
  writeToSerial();
}

void updateGait() {
  uint32_t t_current = millis();
  if (t_current - t_last_setpoint_update >= k_dtSetpointUpdate) {
    t_last_setpoint_update = t_current;

    // calculate stability metrics here
    regulateBodyPose(); // if this function executes motions, updateSetpoint() is bypassed during the process
    updateSetpoints();  // update setpoints for the motors in the swing phase
  }
}

// update desired motion vector, body height, trajectories, etc. based on higher-level input/planner
void updateTrajectory() {
  // apply deadzone to left-right joystick input to prevent undesired turning during translation
  float input_y_filtered = max(abs(input_y) - kGUIJoystickYDeadZone, 0);
  int dir_y = (input_y > 0) - (input_y < 0);
  input_y_filtered *= dir_y/kGUIJoystickYDeadZone;

  // apply deadzone to front-back joystick input to prevent undesired translation during in-place turning
  float input_x_filtered = max(abs(input_x) - kGUIJoystickXDeadZone, 0);
  int dir_x = (input_x > 0) - (input_x < 0);
  input_x_filtered *= dir_x/kGUIJoystickXDeadZone;

  cmd_vector[0] = input_x_filtered;                                     // use deadzone/scaled input
  cmd_vector[1] = input_y_filtered;                                     // use deadzone/scaled input

  leg_swing_percent = max(min(input_swing, kLegSwingPercentMax), kLegSwingPercentMin);  // bound the leg swing percentage with min/max
  z_body_nominal = (k_zBodyMax - k_zBodyMin)*input_height + k_zBodyMin;                 // for now, nominal body height is equal to the leg actuator setpoint in stance
  
}

// executes body orientation and height regulation that blocks normal gait cycle
void regulateBodyPose() {

  if (mp_queue.size() == 0) {    // only allow motion primitives when there is none at the moment

    uint8_t stance = (gait_phase + 1) % kNumOfGaitPhases;
    float z_error = z_body_local - z_body_nominal;
    float dq = stance_width[gait_phase] * tan(rpy_lateral[gait_phase] * DEG2RAD);

    // body height regulation
    if (fabs(z_error) > k_zErrorSoftMax                                                   // if there is height error
        && fabs(rpy_lateral[0]) < kTiltNominal && fabs(rpy_lateral[1]) < kTiltNominal) {  // AND the body tilt is small

      // non-blocking motion: only performed during swing phase and for small height error
      if (actuation_phase == ActuationPhases::kLocomote                                     // if leg retraction is complete
          && fabs(z_error) < k_zErrorHardMax                                                // AND height error is small
          && fabs(z_error) + kDqSwingLegClearance < (q_leg_contact[0] - q_leg_swing[0])     // AND there is sufficient swing leg clearance (assumes flat terrain)
          && fabs(motors[MotorID::kMotorTranslate].states_.q) < kQTransCentered) {          // AND the translational joint is near the midpoint
            
        // move stance legs
        updateMotorsClimb(stance, -z_error);

        // add this motion primitive to the queue
        mp_queue.push(std::make_tuple(stance, ReactiveBehaviors::kStancePosition, updateMotorsStance));
      }

      // blocking motion: only performed during double stance and for large height error
      if (actuation_phase == ActuationPhases::kTouchDown                                    // if currently touching down
          && fabs(z_error) > k_zErrorHardMax                                                // AND height error is large
          && isInContact[gait_phase * 2] && isInContact[gait_phase * 2 + 1]) {              // AND the swing legs are now also on the ground (double stance)

        // move double stance legs
        updateMotorsClimb(GaitPhases::kMedialSwing, -z_error);
        updateMotorsClimb(GaitPhases::kLateralSwing, -z_error);

        // add this motion primitive to the queue
        mp_queue.push(std::make_tuple(0, ReactiveBehaviors::kStancePosition, updateMotorsStance));
        mp_queue.push(std::make_tuple(1, ReactiveBehaviors::kStancePosition, updateMotorsStance));
        isBlocking = true;
      }

    // body tilt regulation
    } else if (actuation_phase == ActuationPhases::kLocomote                            // if currently translating or turning
               && fabs(rpy_lateral[gait_phase]) > kTiltNominal                          // AND the body tilt is not within nominal range
               && fabs(dq) + kDqSwingLegClearance < (q_leg_contact[0] - q_leg_swing[0]) // AND there is sufficient swing leg clearance (assumes flat terrain)
               && fabs(motors[MotorID::kMotorTranslate].states_.q) < kQTransCentered) { // AND the translational joint is near the midpoint

      if (dq > kDqLegMaxTilt) dq = kDqLegMaxTilt;
      if (dq < -kDqLegMaxTilt) dq = -kDqLegMaxTilt;

      for (uint8_t i = 0; i < 2; ++i) {
        // put leg motors in position control for the maneuver
        motors[stance * 2 + i].states_.ctrl_mode = ODriveTeensyCAN::ControlMode_t::kPositionControl;

        // adjust velocity, acceleration, deceleration limits for trapezoidal trajectory during this maneuver
        motors[stance * 2 + i].states_.trap_traj_vel_limit = kVelLegTrajTilt;
        motors[stance * 2 + i].states_.trap_traj_accel_limit = kAccelLegTrajTilt;
        motors[stance * 2 + i].states_.trap_traj_decel_limit = kDecelLegTrajTilt;

        // move opposite leg motors
        motors[stance * 2 + i].states_.q_d = motors[stance * 2 + i].states_.q + pow(-1, i)*dq / 2;
      }

      mp_queue.push(std::make_tuple(stance, ReactiveBehaviors::kStancePosition, updateMotorsStance));
     }


  } else {
    MotionPrimitives mp = mp_queue.front();
    while (mp_queue.size() > 0 && isFinishedReacting(std::get<0>(mp), std::get<1>(mp))) {
      std::function<void(uint8_t)> callback = std::get<2>(mp);
      callback(std::get<0>(mp));
      mp_queue.pop();
      if (actuation_phase == ActuationPhases::kTouchDown) isBlocking = false;
      if (mp_queue.size() > 0) {
        mp = mp_queue.front();
      }
    }
  }
}

bool isFinishedReacting(uint8_t motor_group, uint8_t behavior) {
  if (behavior == ReactiveBehaviors::kStancePosition) {
    return motors[motor_group*2].states_.holding && motors[motor_group*2 + 1].states_.holding;

  } else {
    return false;
  }
}

// returns true if ready for transition to the next actuation phase
bool isReadyForTransition(uint8_t phase) {
  
  if (phase == ActuationPhases::kRetractLeg) {  // if currently retracting leg
    float q_leg_transition_1 = q_leg_swing[0] + swing_percent_at_translate*(q_leg_contact[0] - q_leg_swing[0]);
    float q_leg_transition_2 = q_leg_swing[1] + swing_percent_at_translate*(q_leg_contact[1] - q_leg_swing[1]);

    return motors[gait_phase*2].states_.q < q_leg_transition_1 && motors[gait_phase*2 + 1].states_.q < q_leg_transition_2;

  } else if (phase == ActuationPhases::kLocomote) { // if currently translating or turning
    bool isTranslated = false, isTurned = false;

    // if there is a translation command
    if (fabs(cmd_vector[0]) > EPS) {
      int dir = (cmd_vector[0] > 0) - (cmd_vector[0] < 0);                                              // direction of translation command
      float q_trans_transition = fabs(trans_percent_at_touchdown*kQTransMax*cmd_vector[0]);             // this value is always positive since it represents forward motion, regardless of direction
      isTranslated = dir * pow(-1, gait_phase + 1) * q[JointID::kJointTranslate] > q_trans_transition;  // the translational joint has reached the transition point
                                                                                                        // don't check yaw; assume that any concurrent yaw motion will be completed in time

    // if there is an in-place turn command
    } else if (fabs(cmd_vector[1]) > EPS) {
      int dir = (cmd_vector[1] > 0) - (cmd_vector[1] < 0);                              // direction of yaw command
      float q_yaw_transition = fabs(yaw_percent_at_touchdown*kQYawMax*cmd_vector[1]);   // this value is always positive since it represents forward motion, regardless of direction
      isTurned = dir * pow(-1, gait_phase) * q[JointID::kJointYaw] > q_yaw_transition;  // the yaw joint has reached the transition point
    }

    return isTranslated || isTurned;

  } else if (phase == ActuationPhases::kTouchDown) {  // if currently touching down
        
    return isInContact[gait_phase * 2] && isInContact[gait_phase * 2 + 1];

  } else {
    return false;
  }
}

// touchdown: torque control
// translate: position control
// leg retract: position control
void updateSetpoints() {
  // update setpoints at gait phase transitions here
  if (!isBlocking && isReadyForTransition(actuation_phase)) {

    if (actuation_phase == ActuationPhases::kRetractLeg) {  // if currently retracting leg

      if (fabs(cmd_vector[0]) < EPS && fabs(cmd_vector[1]) < EPS) {   // if no translation or yaw command
        holdLocomotionMechanism();

      } else {  // else, move according to the current command
        moveLocomotionMechanism();
      }

    } else if (actuation_phase == ActuationPhases::kLocomote) { // if currently translating or turning
      updateMotorsTouchdown();
      moveLocomotionMechanism();      // reapply locomotion mechanism setpoints in the case of resuming locomotion from standstill
      
    } else if (actuation_phase == ActuationPhases::kTouchDown) {        // if currently touching down
      updateMotorsStance(gait_phase);

      // advance the gait phase
      gait_phase = (gait_phase + 1) % kNumOfGaitPhases;
      if (gait_phase == 0) {
        gait_cycles++; // if the gait phase is back to 0, increment the number of completed gait cycles
      }

      updateMotorsSwing();
      resetSwingLegContactState();
    }

    actuation_phase = (actuation_phase + 1) % kNumOfActuationPhases;

    // update setpoints that do not involve an actuation phase transition here
  } else if (!isBlocking) {
    if (actuation_phase == ActuationPhases::kRetractLeg) {          // if currently retracting leg
      
    } else if (actuation_phase == ActuationPhases::kLocomote) {     // if currently translating or turning
      
      if (fabs(cmd_vector[0]) < EPS && fabs(cmd_vector[1]) < EPS) { // if no translation or yaw command
        holdLocomotionMechanism();

      } else {                                                      // else, move according to the current command
        moveLocomotionMechanism();
      }
      
    } else if (actuation_phase == ActuationPhases::kTouchDown) {    // if currently touching down
      updateTouchdownTorque();
      updateMotorsStance(gait_phase);
    }
  }
}

void moveLocomotionMechanism() {
  float q_trans = pow(-1, gait_phase + 1) * kQTransMax*cmd_vector[0];
  float q_yaw = pow(-1, gait_phase)*kQYawMax*cmd_vector[1];
  motors[MotorID::kMotorTranslate].states_.q_d = q_trans;
  motors[MotorID::kMotorYaw].states_.q_d = q_yaw;
}

void holdLocomotionMechanism() {
  motors[MotorID::kMotorTranslate].states_.q_d = motors[MotorID::kMotorTranslate].states_.q;  // stop at current translation and yaw
  motors[MotorID::kMotorYaw].states_.q_d = motors[MotorID::kMotorYaw].states_.q;
}

void updateMotorsTouchdown() {
  for (uint8_t idx_motor = gait_phase*2; idx_motor < gait_phase*2 + 2; ++idx_motor) {
    motors[idx_motor].states_.ctrl_mode = ODriveTeensyCAN::ControlMode_t::kTorqueControl; // change to torque control for leg touchdown
    motors[idx_motor].states_.velocity_limit = kVelLegMaxContact;                         // limit velocity in torque control during touchdown
    motors[idx_motor].states_.tau_d = touchdown_torque[idx_motor][0];                       // set torque command for leg touchdown
  }
}

void updateTouchdownTorque() {
  for (uint8_t i = 0; i < 2; ++i) {
    uint8_t idx_motor = gait_phase*2 + i;

    // if the motor is not in contact
    if (!isInContact[idx_motor]) {
      if ((motors[idx_motor].states_.q  - q_leg_swing[i]) > kDqLegStartup) {  // if past the startup displacement
        motors[idx_motor].states_.tau_d = touchdown_torque[idx_motor][1];       // lower the leg torque
      }

      if ((motors[idx_motor].states_.q - q_leg_swing[i]) > kDqLegRamp) {      // if past the ramp up displacement
        motors[idx_motor].states_.tau_d = touchdown_torque[idx_motor][2];       // lower the leg torque
      }
    }
  }
}

// updates the specified stance body's leg motors for zero torque stance, leveraging the non-backdrivable legs
void updateMotorsStance(uint8_t stance) {
  for (uint8_t idx_motor = stance*2; idx_motor < stance*2 + 2; ++idx_motor) {
    if(isInContact[idx_motor]) {                                                            // safety check; cannot be in stance without being in contact
      motors[idx_motor].states_.ctrl_mode = ODriveTeensyCAN::ControlMode_t::kTorqueControl; // torque control
      motors[idx_motor].states_.tau_d = 0;                                                  // zero torque command
      motors[idx_motor].states_.q_d = motors[idx_motor].states_.q;                          // set desired position for telemetry purposes and possible control changes
    }
  }
}

// update the specified stance body's leg motors to move by dz
void updateMotorsClimb(uint8_t stance, float dz) {
  for (uint8_t idx_motor = stance*2; idx_motor < stance*2 + 2; ++idx_motor) {
    motors[idx_motor].states_.ctrl_mode = ODriveTeensyCAN::ControlMode_t::kPositionControl;
    motors[idx_motor].states_.trap_traj_vel_limit = kVelLegTrajStandup;
    motors[idx_motor].states_.trap_traj_accel_limit = kAccelLegTrajStandup;
    motors[idx_motor].states_.trap_traj_decel_limit = kDecelLegTrajStandup;
    motors[idx_motor].states_.q_d = motors[idx_motor].states_.q + dz;
    motors[idx_motor].states_.holding = false;
  }
}

// determine swing leg setpoints based on contact conditions and update the motor control mode and limits for swing phase
void updateMotorsSwing() {
  for (uint8_t i = 0; i < 2; ++i) {
    uint8_t idx_motor = gait_phase*2 + i;

    q_leg_contact[i] = motors[idx_motor].states_.q;             // remember the leg motor position at ground contact
    float q_leg_retract = q_leg_contact[i]*leg_swing_percent;   // nominal swing leg setpoint; NOT necessarily equal to the actual setpoint

    // TODO: change this logic to account for robot kinematics (body tilt)
    if (fabs(q[gait_phase * 4 + i * 2] - q[gait_phase * 4 + i * 2 + 1]) > kDqUnevenTerrain) { // if the previous stance legs are standing on uneven ground
      float q_max = max(q[gait_phase * 4 + i * 2], q[gait_phase * 4 + i * 2 + 1]);            // calculate the additional leg stroke due to uneven terrain
      float dq = q_max - motors[idx_motor].states_.q;
      
      motors[idx_motor].states_.q_d = max(kQLegMin + 5, q_leg_retract - dq); // retract more by dq to ensure proper ground clearance

    } else {  
      motors[idx_motor].states_.q_d = q_leg_retract;  // if even terrain, use the nominal swing leg setpoint
    }

    // remember the swing leg position setpoint
    q_leg_swing[i] = motors[idx_motor].states_.q_d;

    // update the motor control mode and limits for swing phase
    motors[idx_motor].states_.ctrl_mode = ODriveTeensyCAN::ControlMode_t::kPositionControl;
    motors[idx_motor].states_.holding = false;
    motors[idx_motor].states_.velocity_limit = kVelLegMax;
    motors[idx_motor].states_.trap_traj_vel_limit = kVelLegTrajSwing;
    motors[idx_motor].states_.trap_traj_accel_limit = kAccelLegTrajSwing;
    motors[idx_motor].states_.trap_traj_decel_limit = kDecelLegTrajSwing;
  }
}