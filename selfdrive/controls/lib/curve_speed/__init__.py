from openpilot.common.constants import CV

MIN_V = 20 * CV.KPH_TO_MS  # do not operate under 20 km/h

# Sentinel for "this controller is not constraining speed right now". Compared against v_cruise in
# m/s, so it has to lose every min() it takes part in.
V_UNSET = float("inf")
