time stamp

# sensor flags
bool rain_detected
bool emergency_left_wheel_lifted
bool emergency_right_wheel_lifted
bool emergency_tilt_accel_triggered
bool emergency_tilt_mech_triggered
bool emergency_stopbutton_triggered

# charging
float32 v_charge
float32 v_battery
float32 i_charge
uint16 charge_pwm
bool is_charging

# motor controllers
bool blade_motor_ctrl_enabled
bool drive_motor_ctrl_enabled

# left encoder ticks
uint32 left_encoder_ticks
uint32 right_encoder_ticks

# onboard imu temp
float32 imu_temp

