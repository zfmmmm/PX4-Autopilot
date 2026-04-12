# Depth sensor data
#
# Provides measured water depth and ambient temperature from a depth sensor.
# Used by estimators, controllers, and for telemetry reporting.

uint32 MESSAGE_VERSION = 1

uint64 timestamp        # [us] Time since system start

float32 depth_m         # [m]    [@invalid NaN] Measured water depth
float32 temperature_c  # [degC] [@invalid NaN] Ambient water temperature
