#pragma once

#include "opendbc/safety/declarations.h"

static bool tesla_external_panda = false;
static bool tesla_hw1 = false;
static bool tesla_hw2 = false;
static bool tesla_hw3 = false;

static int chassis_bus = 0U;
static int das_control_msg = 0x2bfU;
static int di_torque1_msg = 0x106U;

static bool tesla_legacy_stock_aeb = false;

// Whether openpilot owns longitudinal. When it does not, the factory ACC module keeps driving
// DAS_control and openpilot must neither send that message nor block the stock one -- see the
// fwd hook. tesla.h has always gated its DAS_control blocking on this; the legacy modes did not,
// which left the car with no DAS_control at all whenever openpilot longitudinal was turned off.
static bool tesla_legacy_longitudinal = false;

// Only rising edges while controls are not allowed are considered for these systems:
// TODO: Only LKAS (non-emergency) is currently supported since we've only seen it
static bool tesla_legacy_stock_lkas = false;
static bool tesla_legacy_stock_lkas_prev = false;

// Stock autopark (HW1). Off unless the car port opts in, since letting the stock module steer
// means opening the forwarding gate on DAS_steeringControl for a system we can't bound.
static bool tesla_legacy_allow_stock_autopark = false;
static bool tesla_legacy_cars_as_trucks = false;
static bool tesla_legacy_sync_cluster_speed = false;
static bool tesla_legacy_stock_autopark = false;
// The maneuver is announced late: on a recorded attempt DAS_steeringControl went to
// ANGLE_CONTROL a full 2.0s before DAS_accState ever reached an APC state, so waiting for the
// APC state would swallow that entire opening. The steering command itself is the early signal,
// and unlike AutopilotStatus it is already an rx-checked message -- only whitelisted addresses
// reach this hook, and making a 2Hz status message required to engage is not worth it.
static bool tesla_legacy_autopark_steering = false;  // stock ANGLE_CONTROL on the camera bus
static bool tesla_legacy_autopark_active = false;    // DAS_accState in the APC range
// Held for a whole maneuver rather than decided per frame. A per-frame gate let 2 of the stock
// module's 6 APC_BACKWARD frames through and openpilot's own DAS_control fill the gaps, which
// put two counter sequences on one arbitration id and got the maneuver aborted. The stock
// module needs the channel continuously, so once it starts it keeps it until it stops asking.
// Sized off the recorded maneuver: the stock module asks at roughly 30Hz and the largest gap
// between two of its requests was 44ms, so this is ~4.5x margin. It was 1s at first, which was
// 23x more than needed -- and every millisecond of it is a millisecond where neither module is
// feeding DAS_control, which the car expects continuously.
#define TESLA_AUTOPARK_TIMEOUT 200000U  // us of stock silence before handing the bus back
static uint32_t tesla_legacy_autopark_ts = 0;
static bool tesla_legacy_autopark_ts_valid = false;

static void tesla_legacy_rx_hook(const CANPacket_t *msg) {

  // Steering angle: (0.1 * val) - 819.2 in deg.
  if (!tesla_external_panda && (msg->bus == 0U) && (msg->addr == 0x370U)) {
    // Store it 1/10 deg to match steering request
    const int angle_meas_new = (((msg->data[4] & 0x3FU) << 8) | msg->data[5]) - 8192U;
    update_sample(&angle_meas, angle_meas_new);

    const int hands_on_level = msg->data[4] >> 6;  // handsOnLevel
    const int eac_status = msg->data[6] >> 5;      // eacStatus
    const int eac_error_code = msg->data[2] >> 4;  // eacErrorCode

    // Disengage on normal user override, or if high angle rate fault from user overriding extremely quickly
    steering_disengage = (hands_on_level >= 3) || ((eac_status == 0) && (eac_error_code == 9));
  }

  // Vehicle speed (ESP_B: ESP_vehicleSpeed)
  if ((!tesla_external_panda) && (msg->bus == chassis_bus) && (msg->addr == 0x155U)) {
    // Vehicle speed: (0.01 * val) * KPH_TO_MS
    float speed = ((msg->data[6] | (msg->data[5] << 8)) * 0.01) * KPH_TO_MS;
    UPDATE_VEHICLE_SPEED(speed);
  }

  // Gas pressed
  if ((tesla_external_panda || tesla_hw1) && (msg->bus == 0U) && (msg->addr == di_torque1_msg)) {
    gas_pressed = msg->data[6] != 0U;
  }

  if (((tesla_external_panda) && (msg->bus == 0U) && (msg->addr == 0x1f8U)) ||
     ((!tesla_external_panda) && (msg->bus == chassis_bus) && (msg->addr == 0x20aU))) {
    brake_pressed = (((msg->data[0] & 0x0CU) >> 2) != 1U);
  }

  // Cruise
  if (((tesla_external_panda) && (msg->bus == 0U) && (msg->addr == 0x256U)) ||
     ((!tesla_external_panda) && (msg->bus == chassis_bus) && (msg->addr == 0x368U))) {
      // Cruise state
      int cruise_state = (msg->data[1] >> 4) & 0x07U;
      bool cruise_engaged = (cruise_state == 2) ||  // ENABLED
                            (cruise_state == 3) ||  // STANDSTILL
                            (cruise_state == 4) ||  // OVERRIDE
                            (cruise_state == 6) ||  // PRE_FAULT
                            (cruise_state == 7);    // PRE_CANCEL
      vehicle_moving = cruise_state != 3; // STANDSTILL
      // The park module drives through the ACC channel, so the car reports cruise engaged for
      // its own manoeuvre. Reading that as openpilot engagement is what shut the gate above.
      cruise_engaged = cruise_engaged && !tesla_legacy_stock_autopark;
      pcm_cruise_check(cruise_engaged);
   }

  if (msg->bus == 2U) {
    // DAS_control
    if ((tesla_external_panda || tesla_hw1) && msg->addr == das_control_msg) {
      // "AEB_ACTIVE"
      tesla_legacy_stock_aeb = (msg->data[2] & 0x03U) == 1U;

      // DAS_accState, APC_BACKWARD through APC_SELFPARK_START
      const int das_acc_state = (msg->data[1] >> 4) & 0x0FU;
      tesla_legacy_autopark_active = (das_acc_state >= 5) && (das_acc_state <= 11);
    }


    // DAS_steeringControl
    if (!tesla_external_panda && msg->addr == 0x488U) {
      int steering_control_type = msg->data[2] >> 6;
      bool tesla_legacy_stock_lkas_now = steering_control_type == 2;  // "LANE_KEEP_ASSIST"

      // Only consider rising edges while controls are not allowed
      if (tesla_legacy_stock_lkas_now && !tesla_legacy_stock_lkas_prev && !controls_allowed) {
        tesla_legacy_stock_lkas = true;
      }
      if (!tesla_legacy_stock_lkas_now) {
        tesla_legacy_stock_lkas = false;
      }
      tesla_legacy_stock_lkas_prev = tesla_legacy_stock_lkas_now;

      tesla_legacy_autopark_steering = steering_control_type == 1;  // "ANGLE_CONTROL"
    }

    // Never hand the car over mid-drive: openpilot wins while it is engaged, and the stock
    // module only gets the bus back once controls are no longer allowed. This stays a continuous
    // check rather than a rising-edge latch -- the manoeuvre used to close this gate on itself,
    // but that came from cruise, and the cruise handler above no longer reports engagement
    // while the manoeuvre runs. Fixing the source leaves this guarantee intact.
    const bool asking = tesla_legacy_autopark_steering || tesla_legacy_autopark_active;
    const uint32_t now = microsecond_timer_get();
    if (asking) {
      tesla_legacy_autopark_ts = now;
      tesla_legacy_autopark_ts_valid = true;
    }
    const bool recent = tesla_legacy_autopark_ts_valid &&
                        (safety_get_ts_elapsed(now, tesla_legacy_autopark_ts) < TESLA_AUTOPARK_TIMEOUT);
    if (controls_allowed || !recent) {
      tesla_legacy_autopark_ts_valid = false;
    }
    tesla_legacy_stock_autopark = tesla_legacy_allow_stock_autopark && !controls_allowed &&
                                  tesla_legacy_autopark_ts_valid;
  }
}


static bool tesla_legacy_tx_hook(const CANPacket_t *msg) {
  const AngleSteeringLimits TESLA_STEERING_LIMITS = {
    .max_angle = 3600,  // 360 deg, EPAS faults above this
    .angle_deg_to_can = 10,
    .frequency = 50U,
  };

  // NOTE: based off TESLA_MODEL_S_HW3 to match openpilot
  const AngleSteeringParams TESLA_LEGACY_STEERING_PARAMS = {
    .slip_factor = -0.0005666493436310427,  // calc_slip_factor(VM)
    .steer_ratio = 15.,
    .wheelbase = 2.96,
  };

  // DAS_accelMin/Max are 0.04 m/s^2 per step with a -15 offset, so 375 is 0 and each step is
  // 0.04. min_accel was 288 (-3.48): ISO 15622:2018's ACC deceleration ceiling rounded to a whole
  // step, inherited from the Model 3/Y port rather than measured on this car.
  //
  // This is a safety envelope, so it is sized off what the factory system asks for on this
  // channel, not off the operating point. Across six logged drives a Model X HW1's own ACC went
  // to raw 262 (-4.52) with the driver's foot off the pedal; it reached raw 249 only in a frame
  // where the driver was already braking, which says nothing about what it asks for unprompted.
  // The port asks for -4.2 (raw 270), inside that with room to spare.
  //
  // max_accel is measured the same way and was the same kind of inherited number: 425 (+2.0) is
  // openpilot's own generic ACCEL_MAX, not anything about this car. The factory ACC authorises
  // raw 440 (+2.60) pulling away from a stop -- 47 separate runs across six drives with the
  // driver's feet off, 37 of them held longer than 0.3s and the longest 7.9s, so it is a real
  // request and not a seam artifact. The port's own ceiling (A_CRUISE_MAX_VALS, 1.6 at rest) is
  // a ride-comfort curve and stays well inside this; the envelope just stops being the thing
  // that decides.
  //
  // Only the legacy mode moves -- Model 3/Y stay at the generic limits, since nothing has been
  // measured there.
  const LongitudinalLimits TESLA_LONG_LIMITS = {
    .max_accel = 440,       // +2.60 m/s^2, the most the factory ACC asked for unprompted
    .min_accel = 262,       // -4.52 m/s^2, the deepest the factory ACC asked for unprompted
    .inactive_accel = 375,  // 0. m/s^2
  };

  bool tx = true;
  bool violation = false;

  // Steering control: (0.1 * val) - 1638.35 in deg.
  if (!tesla_external_panda && (msg->addr == 0x488U)) {
    // We use 1/10 deg as a unit here
    int raw_angle_can = ((msg->data[0] & 0x7FU) << 8) | msg->data[1];
    int desired_angle = raw_angle_can - 16384;
    int steer_control_type = msg->data[2] >> 6;
    bool steer_control_enabled = steer_control_type == 1;  // ANGLE_CONTROL

    if (steer_angle_cmd_checks_vm(desired_angle, steer_control_enabled, TESLA_STEERING_LIMITS, TESLA_LEGACY_STEERING_PARAMS)) {
      violation = true;
    }

    bool valid_steer_control_type = (steer_control_type == 0) ||  // NONE
                                    (steer_control_type == 1);    // ANGLE_CONTROL
    if (!valid_steer_control_type) {
      violation = true;
    }

    if (tesla_legacy_stock_lkas) {
      // Don't allow any steering commands when stock LKAS is active
      violation = true;
    }

    if (tesla_legacy_stock_autopark) {
      // The stock module is steering the car through the maneuver; two angle sources on the
      // bus at once is worse than neither
      violation = true;
    }
  }

  // STW_ACTN_RQ: the cruise stalk, driven to move the cluster's MAX speed.
  //
  // The whole point of the check is the value. On Model S/X this stalk is also the gear selector,
  // and SpdCtrlLvr_Stat carries FWD (1) and RWD (2) in the same field as the cruise steps. Only
  // the four speed steps and IDLE may ever leave, so no combination of a bug upstream can turn
  // into a gear request. Blocked outright unless the feature is on.
  if (msg->addr == 0x45U) {
    if (!tesla_legacy_sync_cluster_speed) {
      violation = true;
    } else {
      int lever = msg->data[0] & 0x3FU;
      bool allowed_lever = (lever == 0) ||    // IDLE
                           (lever == 4U) ||   // UP_2ND   +5
                           (lever == 8U) ||   // DN_2ND   -5
                           (lever == 16U) ||  // UP_1ST   +1
                           (lever == 32U);    // DN_1ST   -1
      if (!allowed_lever) {
        violation = true;
      }
    }
  }

  // DAS_control: longitudinal control message
  if ((tesla_external_panda || tesla_hw1) && (msg->addr == das_control_msg)) {
    // The factory ACC owns this message when openpilot longitudinal is off. Two masters on one
    // arbitration id with independent counters is what the car reads as a fault, and it takes
    // TACC and Autopilot down together. HW1 only, matching the fwd hook -- external panda never
    // carries the flag.
    if (tesla_hw1 && !tesla_legacy_longitudinal) {
      violation = true;
    }

    // No AEB events may be sent by openpilot
    int aeb_event = msg->data[2] & 0x03U;
    if (aeb_event != 0) {
      violation = true;
    }

    // Don't send long/cancel messages when the stock AEB system is active
    if (tesla_legacy_stock_aeb) {
      violation = true;
    }

    // Same for autopark, which drives the car longitudinally through the maneuver
    if (tesla_legacy_stock_autopark) {
      violation = true;
    }

    int raw_accel_max = ((msg->data[6] & 0x1FU) << 4) | (msg->data[5] >> 4);
    int raw_accel_min = ((msg->data[5] & 0x0FU) << 5) | (msg->data[4] >> 3);

    // Prevent both acceleration from being negative, as this could cause the car to reverse after coming to standstill
    if ((raw_accel_max < TESLA_LONG_LIMITS.inactive_accel) && (raw_accel_min < TESLA_LONG_LIMITS.inactive_accel)) {
      violation = true;
    }

    // Don't allow any acceleration limits above the safety limits
    violation |= longitudinal_accel_checks(raw_accel_max, TESLA_LONG_LIMITS);
    violation |= longitudinal_accel_checks(raw_accel_min, TESLA_LONG_LIMITS);
  }

  if (violation) {
    tx = false;
  }

  return tx;
}

static bool tesla_legacy_fwd_hook(int bus_num, int addr) {
  bool block_msg = false;

  // While the stock module has the manoeuvre, none of the blocking below applies: it needs
  // steering and the ACC channel continuously, and a single dropped frame is enough for the car
  // to fault out of the manoeuvre. Structured as an early skip rather than an extra term on each
  // condition, the way the Model 3/Y port does it -- as separate AND terms, one momentary dip in
  // the flag turns straight back into blocking, which is exactly the failure this is fixing.
  if ((bus_num == 2) && !tesla_legacy_stock_autopark) {
    // APS_eacMonitor
    if (!tesla_external_panda && !tesla_hw1 && (addr == 0x27dU)) {
      block_msg = true;
    }

    // DAS_steeringControl
    if (!tesla_external_panda && (addr == 0x488U) && !tesla_legacy_stock_lkas) {
      block_msg = true;
    }

    // DAS_control. Only openpilot longitudinal replaces this message; otherwise the factory ACC
    // module's frames must reach the car, or it loses TACC and Autopilot along with it. Scoped to
    // HW1: the external-panda configs drive longitudinal through the second panda without ever
    // carrying TESLA_FLAG_LONG_CONTROL, so they must keep blocking unconditionally.
    const bool op_owns_das_control = tesla_external_panda || (tesla_hw1 && tesla_legacy_longitudinal);
    if (op_owns_das_control && (addr == das_control_msg) && !tesla_legacy_stock_aeb) {
      block_msg = true;
    }

    // DAS_object. openpilot re-sends the factory's own object list with the vehicle type
    // relabelled, because the cluster stopped drawing CAR. Forwarding the original as well would
    // put two frames for the same group on the bus a few ms apart, one saying CAR and one saying
    // TRUCK, and which one the cluster ends up drawing is not ours to decide. Display only: this
    // message reaches nothing that steers or brakes.
    if (tesla_legacy_cars_as_trucks && (addr == 0x309U)) {
      block_msg = true;
    }
  }

  return block_msg;
}

static safety_config tesla_legacy_init(uint16_t param) {
  const int TESLA_FLAG_LONG_CONTROL = 1;
  const int TESLA_FLAG_EXTERNAL_PANDA = 4;
  const int TESLA_FLAG_HW1 = 8;
  const int TESLA_FLAG_HW2 = 16;
  const int TESLA_FLAG_HW3 = 32;
  const int TESLA_FLAG_STOCK_AUTOPARK = 64;
  const int TESLA_FLAG_CARS_AS_TRUCKS = 128;
  const int TESLA_FLAG_SYNC_CLUSTER_SPEED = 256;

  // Extract flags
  tesla_legacy_longitudinal = GET_FLAG(param, TESLA_FLAG_LONG_CONTROL);
  tesla_external_panda = GET_FLAG(param, TESLA_FLAG_EXTERNAL_PANDA);
  tesla_hw1 = GET_FLAG(param, TESLA_FLAG_HW1);
  tesla_hw2 = GET_FLAG(param, TESLA_FLAG_HW2);
  tesla_hw3 = GET_FLAG(param, TESLA_FLAG_HW3);
  tesla_legacy_allow_stock_autopark = GET_FLAG(param, TESLA_FLAG_STOCK_AUTOPARK) && tesla_hw1;
  tesla_legacy_cars_as_trucks = GET_FLAG(param, TESLA_FLAG_CARS_AS_TRUCKS) && tesla_hw1;
  tesla_legacy_sync_cluster_speed = GET_FLAG(param, TESLA_FLAG_SYNC_CLUSTER_SPEED) && tesla_hw1;

  // Initialize state variables
  tesla_legacy_stock_aeb = false;
  tesla_legacy_stock_lkas = false;
  tesla_legacy_stock_lkas_prev = false;
  tesla_legacy_stock_autopark = false;
  tesla_legacy_autopark_steering = false;
  tesla_legacy_autopark_active = false;
  tesla_legacy_autopark_ts = 0;
  tesla_legacy_autopark_ts_valid = false;
  chassis_bus = 0U;
  di_torque1_msg = 0x106U;

  // Set DAS control message address
  das_control_msg = tesla_external_panda ? 0x2bfU : 0x2b9U;

  // Define message arrays (keeping them as is)
  static const CanMsg TESLA_TX_LEGACY_MSGS[] = {
    {0x488, 0, 4, .check_relay = true, .disable_static_blocking = true},  // DAS_steeringControl
    {0x27D, 0, 3, .check_relay = true, .disable_static_blocking = true},  // APS_eacMonitor
  };

  static const CanMsg TESLA_LEGACY_PT_MSGS[] = {
    {0x2bf, 0, 8, .check_relay = true, .disable_static_blocking = true},  // DAS_control
  };

  // DAS_object is display only -- the cluster draws from it and nothing acts on it -- and unlike
  // the messages around it, openpilot adds to the factory's stream rather than replacing it. That
  // is why check_relay is off here: relay detection exists to catch an ECU still talking on a
  // channel openpilot has taken over, and this channel is deliberately shared. Leaving it on would
  // read the factory's own frames as a stuck relay and cut steering.
  static const CanMsg TESLA_TX_LEGACY_HW1_MSGS[] = {
    {0x488, 0, 4, .check_relay = true, .disable_static_blocking = true},  // DAS_steeringControl
    {0x2b9, 0, 8, .check_relay = true, .disable_static_blocking = true},  // DAS_control
    {0x309, 0, 8, .check_relay = false, .disable_static_blocking = true},  // DAS_object
    {0x45, 0, 8, .check_relay = false, .disable_static_blocking = true},   // STW_ACTN_RQ
  };

  // Stock-ACC mode: lateral only. DAS_control is absent, so openpilot cannot transmit it at all
  // and the factory module's frames pass through untouched.
  static const CanMsg TESLA_TX_LEGACY_HW1_STOCK_LONG_MSGS[] = {
    {0x488, 0, 4, .check_relay = true, .disable_static_blocking = true},  // DAS_steeringControl
    {0x309, 0, 8, .check_relay = false, .disable_static_blocking = true},  // DAS_object
    {0x45, 0, 8, .check_relay = false, .disable_static_blocking = true},   // STW_ACTN_RQ
  };

  // Define RX check arrays (keeping them as is)
  static RxCheck tesla_legacy_pt_rx_checks[] = {
    {.msg = {{0x106, 0, 8, 100U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},  // DI_torque1
    {.msg = {{0x1f8, 0, 8, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // BrakeMessage
    {.msg = {{0x2bf, 2, 8, 25U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // DAS_control
    {.msg = {{0x256, 0, 8, 10U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // DI_state
  };

  static RxCheck tesla_legacy_hw1_rx_checks[] = {
    {.msg = {{0x108, 0, 8, 100U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},  // DI_torque1
    {.msg = {{0x2b9, 2, 8, 25U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // DAS_control
    {.msg = {{0x370, 0, 8, 25U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // EPAS_sysStatus (25hz)
    {.msg = {{0x155, 0, 8, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // ESP_private1
    {.msg = {{0x20a, 0, 8, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // BrakeMessage
    {.msg = {{0x368, 0, 8, 10U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // DI_state
    {.msg = {{0x488, 2, 4, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // DAS_steeringControl
  };

  static RxCheck tesla_legacy_hw2_rx_checks[] = {
    {.msg = {{0x370, 0, 8, 25U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // EPAS_sysStatus (25hz)
    {.msg = {{0x155, 0, 8, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // ESP_private1
    {.msg = {{0x20a, 0, 8, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // BrakeMessage
    {.msg = {{0x368, 0, 8, 10U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // DI_state
    {.msg = {{0x488, 2, 4, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // DAS_steeringControl
  };

  static RxCheck tesla_legacy_hw3_rx_checks[] = {
    {.msg = {{0x370, 0, 8, 100U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // EPAS_sysStatus (100hz)
    {.msg = {{0x155, 1, 8, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // ESP_private1
    {.msg = {{0x20a, 1, 8, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // BrakeMessage
    {.msg = {{0x368, 1, 8, 10U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // DI_state
    {.msg = {{0x488, 2, 4, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // DAS_steeringControl
  };

  // Determine configuration based on hardware type
  if (tesla_external_panda && (tesla_hw3 || tesla_hw2)) {
    return BUILD_SAFETY_CFG(tesla_legacy_pt_rx_checks, TESLA_LEGACY_PT_MSGS);
  }

  if (tesla_hw3) {
    chassis_bus = 1U;
    return BUILD_SAFETY_CFG(tesla_legacy_hw3_rx_checks, TESLA_TX_LEGACY_MSGS);
  }

  if (tesla_hw1) {
    di_torque1_msg = 0x108U;
    if (!tesla_legacy_longitudinal) {
      return BUILD_SAFETY_CFG(tesla_legacy_hw1_rx_checks, TESLA_TX_LEGACY_HW1_STOCK_LONG_MSGS);
    }
    return BUILD_SAFETY_CFG(tesla_legacy_hw1_rx_checks, TESLA_TX_LEGACY_HW1_MSGS);
  }

  // Default case: HW2
  return BUILD_SAFETY_CFG(tesla_legacy_hw2_rx_checks, TESLA_TX_LEGACY_MSGS);
}

const safety_hooks tesla_legacy_hooks = {
  .init = tesla_legacy_init,
  .rx = tesla_legacy_rx_hook,
  .tx = tesla_legacy_tx_hook,
  .fwd = tesla_legacy_fwd_hook,
};
