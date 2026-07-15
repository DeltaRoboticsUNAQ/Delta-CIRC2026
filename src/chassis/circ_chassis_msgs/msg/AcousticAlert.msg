# Progressive acoustic alert command.
# Consumers can translate this to beeps, buzzer PWM, or sound playback.

builtin_interfaces/Time stamp
uint8 level          # 0 safe, 1 caution, 2 danger
float32 beep_hz      # suggested beep repetition frequency
float32 volume_0_1   # 0..1
float32 min_distance_m
