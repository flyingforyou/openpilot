#pragma once

#include <string>
#include <unordered_map>

#include "openpilot/cereal/gen/cpp/log.capnp.h"

inline static std::unordered_map<std::string, ParamKeyAttributes> keys = {
    {"AccessToken", {CLEAR_ON_MANAGER_START | DONT_LOG, STRING}},
    {"AdbEnabled", {PERSISTENT, BOOL}},
    {"AlwaysOnDM", {PERSISTENT, BOOL}},
    {"ApiCache_Device", {PERSISTENT, STRING}},
    {"ApiCache_FirehoseStats", {PERSISTENT, JSON}},
    {"AssistNowToken", {PERSISTENT, STRING}},
    {"AthenadPid", {PERSISTENT, INT}},
    {"AthenadUploadQueue", {PERSISTENT, JSON}},
    {"AthenadRecentlyViewedRoutes", {PERSISTENT, STRING}},
    {"BootCount", {PERSISTENT, INT}},
    {"CalibrationParams", {PERSISTENT, BYTES}},
    {"CameraDebugExpGain", {CLEAR_ON_MANAGER_START, STRING}},
    {"CameraDebugExpTime", {CLEAR_ON_MANAGER_START, STRING}},
    {"CarBatteryCapacity", {PERSISTENT, INT}},
    {"CarParams", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, BYTES}},
    {"CarParamsCache", {CLEAR_ON_MANAGER_START, BYTES}},
    {"CarParamsPersistent", {PERSISTENT, BYTES}},
    {"CarParamsPrevRoute", {PERSISTENT, BYTES}},
    {"CompletedTrainingVersion", {PERSISTENT, STRING, "0"}},
    {"ControlsReady", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, BOOL}},
    {"CurrentBootlog", {PERSISTENT, STRING}},
    {"CurrentRoute", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, STRING}},
    {"DisableLogging", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, BOOL}},
    {"DisablePowerDown", {PERSISTENT, BOOL}},
    {"DisableUpdates", {PERSISTENT, BOOL}},
    {"DisengageOnAccelerator", {PERSISTENT, BOOL, "0"}},
    {"DongleId", {PERSISTENT, STRING}},
    {"DoReboot", {CLEAR_ON_MANAGER_START, BOOL}},
    {"DoShutdown", {CLEAR_ON_MANAGER_START, BOOL}},
    {"DoUninstall", {CLEAR_ON_MANAGER_START, BOOL}},
    // Which driving model the WebUI selector asked for. It is a request for the next modeld
    // start, not a statement about now, so what actually got loaded is reported separately --
    // an external artifact that fails to load leaves this alone and runs stock.
    {"DrivingModel", {PERSISTENT, STRING, "stock"}},
    {"RunningDrivingModel", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, STRING}},
    {"DriverTooDistracted", {CLEAR_ON_MANAGER_START | CLEAR_ON_IGNITION_ON, BOOL}},
    {"DriverLockoutCount", {CLEAR_ON_MANAGER_START | CLEAR_ON_IGNITION_ON, INT, "0"}},
    {"AlphaLongitudinalEnabled", {PERSISTENT | DEVELOPMENT_ONLY, BOOL}},
    {"ExperimentalMode", {PERSISTENT, BOOL}},
    {"ExperimentalModeConfirmed", {PERSISTENT, BOOL}},
    {"FirmwareQueryDone", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, BOOL}},
    {"ForcePowerDown", {PERSISTENT, BOOL}},
    {"GitBranch", {PERSISTENT, STRING}},
    {"GitCommit", {PERSISTENT, STRING}},
    {"GitCommitDate", {PERSISTENT, STRING}},
    {"GitDiff", {PERSISTENT, STRING}},
    {"GithubSshKeys", {PERSISTENT, STRING}},
    {"GithubUsername", {PERSISTENT, STRING}},
    {"GitRemote", {PERSISTENT, STRING}},
    {"GsmApn", {PERSISTENT, STRING}},
    {"GsmMetered", {PERSISTENT, BOOL, "1"}},
    {"GsmRoaming", {PERSISTENT, BOOL}},
    {"HardwareSerial", {PERSISTENT, STRING}},
    {"HasAcceptedTerms", {PERSISTENT, STRING, "0"}},
    {"InstallDate", {PERSISTENT, TIME}},
    {"IsDriverViewEnabled", {CLEAR_ON_MANAGER_START, BOOL}},
    {"IsEngaged", {PERSISTENT, BOOL}},
    {"IsLdwEnabled", {PERSISTENT, BOOL}},
    {"IsLiveStreaming", {CLEAR_ON_MANAGER_START, BOOL}},
    {"IsMetric", {PERSISTENT, BOOL}},
    {"IsOffroad", {CLEAR_ON_MANAGER_START, BOOL}},
    {"IsRhdDetected", {PERSISTENT, BOOL}},
    {"IsReleaseBranch", {CLEAR_ON_MANAGER_START, BOOL}},
    {"IsTestedBranch", {CLEAR_ON_MANAGER_START, BOOL}},
    {"JoystickDebugMode", {CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION, BOOL}},
    {"LanguageSetting", {PERSISTENT, STRING, "en"}},
    {"LastAthenaPingTime", {CLEAR_ON_MANAGER_START, INT}},
    {"LastGPSPosition", {PERSISTENT, STRING}},
    {"LastManagerExitReason", {CLEAR_ON_MANAGER_START, STRING}},
    {"LastOffroadStatusPacket", {CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION, JSON}},
    {"LastAgnosPowerMonitorShutdown", {CLEAR_ON_MANAGER_START, STRING}},
    {"LastPowerDropDetected", {CLEAR_ON_MANAGER_START, STRING}},
    {"LastUpdateException", {CLEAR_ON_MANAGER_START, STRING}},
    {"LastUpdateRouteCount", {PERSISTENT, INT, "0"}},
    {"LastUpdateTime", {PERSISTENT, TIME}},
    {"LastUpdateUptimeOnroad", {PERSISTENT, FLOAT, "0.0"}},
    // TODO: rename the Live* learner cache keys to match their Cereal services, with migration for persisted values.
    {"LiveDelay", {PERSISTENT, BYTES}},
    {"LiveParametersV2", {PERSISTENT, BYTES}},
    {"LivestreamEncoderBitrate", {CLEAR_ON_MANAGER_START | DONT_LOG, INT}},
    {"LivestreamRequestKeyframe", {CLEAR_ON_MANAGER_START | DONT_LOG, BOOL}},
    {"LiveTorqueParameters", {PERSISTENT | DONT_LOG, BYTES}},
    {"LocationFilterInitialState", {PERSISTENT, BYTES}},
    {"LateralManeuverMode", {CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION, BOOL}},
    {"LongitudinalManeuverMode", {CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION, BOOL}},
    {"LongitudinalPersonality", {PERSISTENT, INT, std::to_string(static_cast<int>(cereal::LongitudinalPersonality::STANDARD))}},
    {"NetworkMetered", {PERSISTENT, BOOL}},
    {"ObdMultiplexingChanged", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, BOOL}},
    {"ObdMultiplexingEnabled", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, BOOL}},
    {"Offroad_CarUnrecognized", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, JSON}},
    {"Offroad_ConnectivityNeeded", {CLEAR_ON_MANAGER_START, JSON}},
    {"Offroad_ConnectivityNeededPrompt", {CLEAR_ON_MANAGER_START, JSON}},
    {"Offroad_ExcessiveActuation", {PERSISTENT, JSON}},
    {"Offroad_NeosUpdate", {CLEAR_ON_MANAGER_START, JSON}},
    {"Offroad_NoFirmware", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, JSON}},
    {"Offroad_Recalibration", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, JSON}},
    {"Offroad_TemperatureTooHigh", {CLEAR_ON_MANAGER_START, JSON}},
    {"Offroad_UnregisteredHardware", {CLEAR_ON_MANAGER_START, JSON}},
    {"Offroad_UpdateFailed", {CLEAR_ON_MANAGER_START, JSON}},
    {"Offroad_DriverMonitoringUncertain", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, JSON}},
    {"OnroadCycleRequested", {CLEAR_ON_MANAGER_START, BOOL}},
    {"OpenpilotEnabledToggle", {PERSISTENT, BOOL, "1"}},
    {"PandaHeartbeatLost", {CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION, BOOL}},
    {"PrimeType", {PERSISTENT, INT}},
    {"RecordAudio", {PERSISTENT, BOOL}},
    {"RecordFront", {PERSISTENT, BOOL}},
    {"RecordFrontLock", {PERSISTENT, BOOL}},  // for the internal fleet
    {"SecOCKey", {PERSISTENT | DONT_LOG, STRING}},
    {"ShowDebugInfo", {PERSISTENT, BOOL}},
    {"RouteCount", {PERSISTENT, INT, "0"}},
    {"SnoozeUpdate", {CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION, BOOL}},
    {"SshEnabled", {PERSISTENT, BOOL}},
    {"UbloxAvailable", {PERSISTENT, BOOL}},
    {"UpdateAvailable", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, BOOL}},
    {"UpdateFailedCount", {CLEAR_ON_MANAGER_START, INT}},
    {"UpdaterAvailableBranches", {PERSISTENT, STRING}},
    {"UpdaterCurrentDescription", {CLEAR_ON_MANAGER_START, STRING}},
    {"UpdaterCurrentReleaseNotes", {CLEAR_ON_MANAGER_START, BYTES}},
    {"UpdaterFetchAvailable", {CLEAR_ON_MANAGER_START, BOOL}},
    {"UpdaterNewDescription", {CLEAR_ON_MANAGER_START, STRING}},
    {"UpdaterNewReleaseNotes", {CLEAR_ON_MANAGER_START, BYTES}},
    {"UpdaterState", {CLEAR_ON_MANAGER_START, STRING}},
    {"UpdaterTargetBranch", {CLEAR_ON_MANAGER_START, STRING}},
    {"UpdaterLastFetchTime", {PERSISTENT, TIME}},
    {"UptimeOffroad", {PERSISTENT, FLOAT, "0.0"}},
    {"UptimeOnroad", {PERSISTENT, FLOAT, "0.0"}},
    {"UsbGpuActive", {CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION | CLEAR_ON_IGNITION_ON, BOOL}},
    {"UsbGpuLoading", {CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION | CLEAR_ON_IGNITION_ON, BOOL}},
    {"Version", {PERSISTENT, STRING}},

    // Tesla HW1 port and the CarrotPilot longitudinal planner.
    {"DriverMonitorBypass", {PERSISTENT, BOOL, "0"}},
    {"GapProfile", {PERSISTENT, INT, "0"}},
    {"RadarLeadHoldCm", {PERSISTENT, INT, "0"}},
    {"RadarLeadHoldMs", {PERSISTENT, INT, "1000"}},
    {"StopDistanceCm", {PERSISTENT, INT, "600"}},
    {"TFollowRiseRatePct", {PERSISTENT, INT, "35"}},

    // CarrotPilot longitudinal, behind CarrotLongEnabled. Defaults are carrot's own -- the
    // point of the port is to run what it runs, so these are not retuned here. Stored as
    // integers in hundredths where the code divides by 100, matching carrot's own settings.
    // Defaults on (2026-08): the old plain-openpilot planner is retired -- CarrotPilot and the
    // car's stock ACC (TeslaStockLong) are the only two paths anyone runs going forward.
    {"CarrotLongEnabled", {PERSISTENT, BOOL, "1"}},
    // Trims how far off the lane centre the model is allowed to sit. Off by default: it
    // changes steering, so it is opted into rather than inherited. Fractions are integers
    // here like the rest of this block -- authority is a percent, offset is centimetres.
    // Tenths of a second the blinker must be held before a lane change starts without the
    // driver's nudge. 0 keeps stock behaviour, where only the nudge starts it.
    {"AutoLaneChange", {PERSISTENT, INT, "0"}},
    {"LaneCentering", {PERSISTENT, BOOL, "0"}},
    {"LaneCenteringPauseOnSignal", {PERSISTENT, BOOL, "1"}},
    // 0 corrects every deviation, 100 leaves the big confident ones to the model. StarPilot
    // ships 100; half is the point where a wide line through a bend still gets pulled in.
    {"LaneCenteringE2EAuthority", {PERSISTENT, INT, "50"}},
    {"LaneCenterOffset", {PERSISTENT, INT, "0"}},
    // Centiseconds of low-pass on the model's desired curvature; 0 = off (upstream's default).
    // Carrot ships 13. Off here so it stays an opt-in option -- see modeld.py's note on why the
    // low-speed wheel shake is what it is for.
    {"LatSmoothSec", {PERSISTENT, INT, "0"}},
    // These two carrot reads from params where this tree takes them from the car port. The
    // defaults are the port's values for this car, not carrot's generic 20/50: vEgoStopping is
    // set to 0.1 in the Tesla interface specifically, and 0.5 would have the car decide it has
    // stopped while still rolling at walking pace.
    {"LongActuatorDelay", {PERSISTENT, INT, "15"}},
    {"VEgoStopping", {PERSISTENT, INT, "10"}},
    {"MyDrivingMode", {PERSISTENT, INT, "3"}},
    {"MyDrivingModeAuto", {PERSISTENT, INT, "0"}},
    // Seconds of unbroken green before openpilot will pull away from a light stop. The launch
    // is otherwise taken on the first green frame, and this port has no second traffic-light
    // source to cross-check the camera against. 0 restores the old immediate behaviour.
    {"TrafficLightGreenHold", {PERSISTENT, INT, "5"}},  // 0.1s units
    {"TrafficLightDetectMode", {PERSISTENT, INT, "2"}},
    // Seven entries, one per position of this car's gap stalk. carrot ships four because the
    // cars it target have a four-position button, which openpilot maps onto the four
    // personality levels; this car reports seven. Same curve, subdivided: carrot's own
    // 1.10/1.20/1.40/1.60 land on gaps 1/3/5/7 and the even positions are interpolated between
    // them, so the endpoints and the shape are carrot's and only the resolution is ours.
    {"TFollowGap1", {PERSISTENT, INT, "110"}},
    {"TFollowGap2", {PERSISTENT, INT, "115"}},
    {"TFollowGap3", {PERSISTENT, INT, "120"}},
    {"TFollowGap4", {PERSISTENT, INT, "130"}},
    {"TFollowGap5", {PERSISTENT, INT, "140"}},
    {"TFollowGap6", {PERSISTENT, INT, "150"}},
    {"TFollowGap7", {PERSISTENT, INT, "160"}},
    {"DynamicTFollow", {PERSISTENT, INT, "0"}},
    {"DynamicTFollowLC", {PERSISTENT, INT, "100"}},
    {"EnableSpeedTF", {PERSISTENT, INT, "0"}},
    {"CruiseMaxVals0", {PERSISTENT, INT, "160"}},
    {"CruiseMaxVals1", {PERSISTENT, INT, "200"}},
    {"CruiseMaxVals2", {PERSISTENT, INT, "160"}},
    {"CruiseMaxVals3", {PERSISTENT, INT, "130"}},
    {"CruiseMaxVals4", {PERSISTENT, INT, "110"}},
    {"CruiseMaxVals5", {PERSISTENT, INT, "95"}},
    {"CruiseMaxVals6", {PERSISTENT, INT, "80"}},
    // The deceleration the planner assumes it can comfortably use, in 0.01 m/s^2. It sets how
    // much room the follow distance reserves: desired = (v_ego^2 - v_lead^2)/(2*this) + ...
    // A lower value assumes the car can only brake gently, so it demands more room and brakes
    // earlier and harder for the same closing speed. Measured over a drive, this term was 60%
    // of the whole follow distance.
    {"ComfortBrake", {PERSISTENT, INT, "240"}},
    {"ComfortBrake2", {PERSISTENT, INT, "250"}},
    {"StopDistanceCarrot", {PERSISTENT, INT, "550"}},
    // Stopping accel: the car must already be braking at least this hard before the controller
    // commits to its stopping ramp. 0 means "use the port's own stopAccel", which is what the
    // stock planner does. Negative hundredths, matching carrot.
    {"StoppingAccel", {PERSISTENT, INT, "-50"}},
    // How long the lead's measured acceleration is assumed to last, as a percentage. Lower
    // reacts sooner and holds it longer; higher assumes it fades and responds more gently.
    {"RadarReactionFactor", {PERSISTENT, INT, "100"}},
    // Longitudinal PID, exposed the way carrot exposes it. This port leaves kpV and kiV at
    // [0.], so the loop is feedforward-only until these are set -- Kp 100 is 1.00, Ki is in
    // thousandths, Kf 100 leaves a_target passing through unchanged.
    {"LongTuningKpV", {PERSISTENT, INT, "100"}},
    {"LongTuningKiV", {PERSISTENT, INT, "0"}},
    {"LongTuningKf", {PERSISTENT, INT, "100"}},
    // How far ahead a radar track is projected when judging whether it is moving into our lane,
    // in hundredths of a second. carrot ships 0, which turns the projection off entirely; 60
    // keeps what this tree has been doing since the radard port.
    {"RadarLatFactor", {PERSISTENT, INT, "60"}},
    {"JLeadFactor3", {PERSISTENT, INT, "0"}},
    {"AutoNaviSpeedDecelRate", {PERSISTENT, INT, "120"}},
    {"AChangeCostStarting", {PERSISTENT, INT, "10"}},
    {"TrafficStopDistanceAdjust", {PERSISTENT, INT, "-150"}},
    // Auto cruise speed from the car's own navigation map (selfdrive/controls/lib/map_cruise.py).
    // Off by default: it lowers the cruise setpoint on its own, which is not something to turn on
    // behind a driver's back.
    {"TeslaMapAutoSpeed", {PERSISTENT, BOOL, "0"}},
    // Percent of the posted limit to target, before the car's own offset is added. 100 is the
    // sign as posted.
    {"TeslaMapAutoSpeedRatio", {PERSISTENT, INT, "100"}},
    // Ceiling for the auto-set speed, kph. This is the number the stalk cannot be, because on
    // this car the stalk *is* v_cruise (pcmCruise) -- using it as the ceiling would pin the map
    // to whatever was dialled in on the last street. Set once; the map moves freely below it.
    // 129 kph = 80 mph.
    {"TeslaMapAutoSpeedMax", {PERSISTENT, INT, "129"}},
    // Whether to add UI_userSpeedOffset, the "limit + n" the driver already configured in the
    // car's own menu, on top of that ratio.
    // Let the fleet speed for this stretch of road cap the target on ordinary roads, not just
    // on ramps. This is the curve slowdown: the gateway's fleetSplineSpeed leads a curve by a
    // median 4s, so it arrives in time to be braked for. Only ever lowers the target.
    {"TeslaMapAutoSpeedCurve", {PERSISTENT, BOOL, "1"}},
    {"TeslaMapCurveLatAccel", {PERSISTENT, INT, "300"}},
    // Let the gateway's own road-ahead cubic (UI_roadCurvature) cap the curve speed past 60m,
    // where the model has stopped seeing bends -- at 100m it calls 15% of the real ones against
    // the map's 72%. Tighter of the two wins, so this can only ever slow the car down.
    {"TeslaMapCurveUseMap", {PERSISTENT, BOOL, "1"}},
    // Hand a car that is merging in front to the planner as a second obstacle, before the gap
    // has collapsed. Replayed over three drives it warns a median 2.1s early; a wrong call costs
    // a little room given to a real car in the next lane.
    // Braking jerk floor handed to the DI, in 0.1 m/s^3 steps; 0 = off (full JERK_LIMIT_MIN,
    // the shipped behaviour). See CarControllerParams.JERK_BRAKE_GAIN for why.
    {"TeslaBrakeJerk", {PERSISTENT, INT, "0"}},
    {"TeslaCutInLead", {PERSISTENT, BOOL, "1"}},
    {"TeslaCoopLatAccelCms", {PERSISTENT, INT, "150"}},
    {"TeslaCoopMaxTorqueCNm", {PERSISTENT, INT, "250"}},
    // Drive the cluster's MAX number to match what openpilot is actually targeting, by
    // emulating the cruise stalk. DAS_setSpeed does not reach that display at all -- the DI
    // owns it and only STW_ACTN_RQ moves it. Off by default: it writes the stalk.
    {"TeslaSyncClusterSpeed", {PERSISTENT, BOOL, "0"}},
    // Tesla Unity-style AP1 instrument-cluster integration (HW1 only). While engaged, openpilot
    // takes over the factory 0x239 DAS_lanes and 0x399 AutopilotStatus frames -- keeping their
    // rolling counters -- so the cluster draws openpilot's path in the AP-active view, and adds
    // its radar leads to the 0x309 object stream. Off by default: it changes cluster-side CAN.
    {"TeslaICIntegration", {PERSISTENT, BOOL, "0"}},
    {"TeslaCoopSteer", {PERSISTENT, BOOL, "0"}},
    {"TeslaLastGapAdjust", {PERSISTENT, INT, "0"}},
    {"TeslaStockAutopark", {PERSISTENT, BOOL, "0"}},
    {"TeslaStockLong", {PERSISTENT, BOOL, "0"}},
    // Blocks openpilot from ever actuating -- same mechanism OpenpilotEnabledToggle/dashcamOnly
    // already use (CarParams.passive: panda safetyModel forced to noOutput, card.py skips
    // controls_update), so it is enforced at the panda level, not just the app. Camera, CAN
    // logging, modeld and radard keep running exactly as normal; only the actuation path is cut.
    // For driving with the car's own stock Autopilot engaged instead, to log what its real
    // DAS_lanes/AutopilotStatus frames look like. Set at init, so a restart is required.
    {"TeslaLogOnly", {PERSISTENT, BOOL, "0"}},
    // Stops a double-stroke of the cruise stalk (the gesture that turns on the factory's own
    // Autosteer) from blocking openpilot's own engagement. Safe to do because panda already
    // blocks the factory's DAS_steeringControl (0x488) from ever reaching EPAS unconditionally
    // -- see tesla_legacy.h's fwd_hook, the block has no controls_allowed term at all -- so the
    // factory's steering commands never actually reach the car regardless of this toggle; only
    // whether openpilot's software-side invalidLkasSetting NO_ENTRY gate honours the factory's
    // own "I'm autosteering" signal or ignores it. Off by default. Set at init, so a restart is
    // required.
    {"TeslaDoubleStrokeOverride", {PERSISTENT, BOOL, "0"}},

    // Experiment: hold openpilot's own steering correction at zero (angle command frozen, no
    // lateral torque) until the genuine bus2 autopilotStatus reports Active_nominal(3) on its
    // own, then start actuating. Tests whether the AP1 computer's DAS_activationFailureStatus
    // rejection is an entry-time check or an ongoing one. Meant to be combined with
    // TeslaDoubleStrokeOverride, which keeps openpilot from faulting on the double stroke itself.
    // Off by default. Set at init, so a restart is required.
    {"TeslaWaitForStockAP", {PERSISTENT, BOOL, "0"}},
};
