"""Braking jerk authority (TeslaBrakeJerk) on the legacy DAS_control path.

DAS_jerkMin tells the DI how fast it may change deceleration. Pinned at the fault limit it grabs
harder than openpilot asks for, which is the juddering heard when it brakes at speed. These lock
down that the feature is inert when off, that it never hands out more than the fault limit, and
that authority opens on demand rather than lagging it.
"""
from opendbc.can import CANPacker
from opendbc.car.tesla.teslacan_legacy import TeslaCANRaven
from opendbc.car.tesla.values import CANBUS, CarControllerParams

BASE = 2.0
DT = CarControllerParams.JERK_BRAKE_DT


def _can():
    packers = {CANBUS.party: CANPacker("tesla_can"), CANBUS.powertrain: CANPacker("tesla_powertrain")}
    return TeslaCANRaven(packers)


def _limit(can, accels, base, active=True):
    """Run a sequence of commanded accels through and return the final jerk floor."""
    out = None
    for a in accels:
        out = can._brake_jerk_limit(a, active, base)
    return out


class TestOff:
    def test_zero_base_is_the_old_behaviour(self):
        can = _can()
        # even a violently changing command must not tighten anything when the feature is off
        assert _limit(can, [0.0, -1.0, -3.0, -0.5], 0.0) == CarControllerParams.JERK_LIMIT_MIN

    def test_inactive_gives_full_authority(self):
        can = _can()
        assert _limit(can, [0.0, -2.0], BASE, active=False) == CarControllerParams.JERK_LIMIT_MIN


class TestAuthorityTracksDemand:
    def test_steady_command_sits_at_the_base(self):
        can = _can()
        # a constant accel is zero commanded jerk -> only the base
        assert _limit(can, [-1.0] * 50, BASE) == -BASE

    def test_gentle_command_stays_near_the_base(self):
        can = _can()
        # 0.41 m/s^3 is the measured median commanded jerk; 2x that is under the base
        step = 0.41 * DT
        limit = _limit(can, [-(i * step) for i in range(30)], BASE)
        assert limit == -BASE

    def test_hard_demand_opens_authority(self):
        can = _can()
        # a 4 m/s^3 command needs more than the base and must get it
        step = 4.0 * DT
        limit = _limit(can, [-(i * step) for i in range(10)], BASE)
        assert limit < -BASE

    def test_never_exceeds_the_fault_limit(self):
        can = _can()
        # an absurd step change must still be clamped to what the ACC tolerates
        limit = _limit(can, [0.0, -3.5], BASE)
        assert limit >= CarControllerParams.JERK_LIMIT_MIN

    def test_opens_immediately_but_relaxes_slowly(self):
        can = _can()
        _limit(can, [0.0, -3.5], BASE)          # one big step opens it
        opened = can.cmd_jerk
        for _ in range(3):                       # a few quiet frames
            can._brake_jerk_limit(-3.5, True, BASE)
        assert can.cmd_jerk < opened, "authority must relax"
        assert can.cmd_jerk > 0.5 * opened, "but not collapse in a few frames"


class TestOnlyBrakingDemandCounts:
    """Demand is directional. Taking abs() of the command delta let an acceleration transient hand
    out BRAKING authority, so the cap could already be wide open when a hard stop began -- which is
    what left some grab audible with the feature on."""

    def test_easing_off_the_brakes_does_not_open_authority(self):
        can = _can()
        # command relaxing from hard braking back toward zero: big |delta|, but no braking demand
        limit = _limit(can, [-3.0 + 4.0 * DT * i for i in range(20)], BASE)
        assert limit == -BASE

    def test_accelerating_does_not_open_authority(self):
        can = _can()
        limit = _limit(can, [4.0 * DT * i for i in range(20)], BASE)
        assert limit == -BASE

    def test_braking_demand_still_opens_it(self):
        can = _can()
        limit = _limit(can, [-4.0 * DT * i for i in range(20)], BASE)
        assert limit < -BASE

    def test_acceleration_leaves_no_carried_over_demand(self):
        """A stretch of acceleration must leave the remembered demand empty, so it contributes
        nothing to whatever braking comes next. (The step INTO braking is real demand and does
        open the cap -- spreading that is the open-ramp's job, not this one's.)"""
        can = _can()
        _limit(can, [4.0 * DT * i for i in range(25)], BASE)
        assert can.cmd_jerk < 0.05, "acceleration was remembered as braking demand"


class TestStoppedCarRamp:
    """The case this exists for: a stopped car seen late, so the planner steps the accel command.
    Opening the ceiling in one frame there just reproduces the grab, so it has to be spread out."""

    def _run(self, base):
        can = _can()
        can._brake_jerk_limit(0.0, False, base)      # pre-engage settles accel_last
        can.jerk_lower = CarControllerParams.JERK_LIMIT_MIN
        out = []
        for a in [0.0] * 50 + [-3.0] * 75:           # 2 s cruise, then a step to -3 m/s^2
            can.create_longitudinal_command(4, a, 0, 20.0, True, False, 0.0, base)
            out.append(can.jerk_brake_cap if base > 0 else can.jerk_lower)
        return out

    def test_off_jumps_straight_to_the_limit(self):
        v = self._run(0.0)
        assert v[49] == CarControllerParams.JERK_LIMIT_MIN
        assert v[51] == CarControllerParams.JERK_LIMIT_MIN

    def test_on_ramps_instead_of_jumping(self):
        v = self._run(BASE)
        assert v[49] == -BASE, "sitting at the floor before the step"
        assert -BASE > v[51] > CarControllerParams.JERK_LIMIT_MIN, "opened, but not all the way"
        assert v[51] > v[53], "still opening a few frames later"

    def test_on_relaxes_back_to_the_floor(self):
        v = self._run(BASE)
        assert v[-1] == -BASE, "returns to the floor once the command settles"

    def test_sustained_hard_demand_still_reaches_full_authority(self):
        can = _can()
        can._brake_jerk_limit(0.0, False, BASE)
        limit = None
        for i in range(60):                          # a sustained 4 m/s^3 request
            can.create_longitudinal_command(4, -4.0 * DT * i, 0, 20.0, True, False, 0.0, BASE)
            limit = can.jerk_brake_cap
        assert abs(limit - CarControllerParams.JERK_LIMIT_MIN) < 0.2, "must not cap real demand"


class TestMessageStillValid:
    def test_frame_packs_across_the_range(self):
        # DAS_jerkMin is [-15.232, 0.098]; an out-of-range value would corrupt the frame
        can = _can()
        for base in (0.0, BASE, 4.0):
            for accel in (0.0, -1.0, -3.5):
                msg = can.create_longitudinal_command(4, accel, 0, 20.0, True, False, 0.0, base)
                assert len(msg[1]) == 8, "DAS_control is 8 bytes"

    def test_jerk_lower_is_rate_limited(self):
        can = _can()
        can.jerk_lower = 0.0
        can.create_longitudinal_command(4, -2.0, 0, 20.0, True, False, 0.0, BASE)
        moved = abs(can.jerk_lower - 0.0)
        assert moved <= CarControllerParams.JERK_RAMP_RATE + 1e-9, "must not step in one frame"


class TestCeiling:
    """The floor governs ordinary braking; the ceiling is the only thing that governs a hard stop.
    Measured at floor 0.8, hard braking still opened to -4.37 at p10 because demand lifted it."""

    def test_off_reaches_the_full_limit(self):
        can = _can()
        limit = _limit(can, [-4.0 * DT * i for i in range(30)], BASE)
        assert abs(limit - CarControllerParams.JERK_LIMIT_MIN) < 0.2

    def test_ceiling_caps_hard_demand(self):
        can = _can()
        can._brake_jerk_limit(0.0, False, BASE, 3.0)
        limit = None
        for i in range(30):
            limit = can._brake_jerk_limit(-4.0 * DT * i, True, BASE, 3.0)
        assert limit == -3.0

    def test_ceiling_does_not_raise_the_floor(self):
        can = _can()
        can._brake_jerk_limit(-1.0, False, BASE, 3.0)
        limit = None
        for _ in range(50):
            limit = can._brake_jerk_limit(-1.0, True, BASE, 3.0)
        assert limit == -BASE, "steady command must still sit on the floor"

    def test_ceiling_never_exceeds_the_fault_limit(self):
        can = _can()
        can._brake_jerk_limit(0.0, False, BASE, 99.0)
        limit = None
        for i in range(30):
            limit = can._brake_jerk_limit(-4.0 * DT * i, True, BASE, 99.0)
        assert limit >= CarControllerParams.JERK_LIMIT_MIN
