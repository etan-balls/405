import pyb
from pyb import Pin, Timer, UART
from gc import collect, mem_free

collect()
from task_share import Share
collect()
from cotask import Task, task_list
collect()
from task_user import task_user
collect()
from motor_driver import motor_driver
collect()
from encoder_driver import encoder
collect()
from Line_sensor_driver import L_sensor
collect()
from multi_sensor_read import multiple_ir_readings
collect()
from task_motor import task_motor
collect()
from control_task import task_control
collect()
from task_sensor import task_line_sensor
collect()
from task_imu import task_imu
collect()
from task_state_estimator import task_state_estimator
collect()
from bump_sensor import BumpSensors
collect()
print("RAM free after imports:", mem_free())


# -----------------
# Bluetooth UART1 setup (HC-05 on PB6=TX, PB7=RX)
# -----------------
bt = UART(1, baudrate=9600, timeout=10)  # change to 115200 if bt_config.py was run

# Redirect the MicroPython REPL (and therefore all print/input) to UART3.
# PuTTY connects to the HC-05 COM port at 115200 to see all output.
pyb.repl_uart(bt)


# -----------------
# Hardware setup
# -----------------
tim3 = Timer(3, freq=20000)
leftMotor  = motor_driver(Pin.cpu.B5,  Pin.cpu.B4,  Pin.cpu.C8, tim3, 3)
rightMotor = motor_driver(Pin.cpu.B15, Pin.cpu.B14, Pin.cpu.C9, tim3, 4)

leftEncoder  = encoder(1, pyb.Pin.cpu.A8, pyb.Pin.cpu.A9)
rightEncoder = encoder(2, pyb.Pin.cpu.A0, pyb.Pin.cpu.A1)

# Bumper boards (see lab pinout sheet)
# LEFT bumper board:
#   GND   -> CN7-20
#   BUMP1 -> PB12
#   BUMP2 -> PB11
#   BUMP3 -> PC7
# RIGHT bumper board:
#   BUMP1 -> PA15
#   BUMP2 -> PH0
#   BUMP3 -> PH1
left_bump_pins  = [Pin.cpu.B12, Pin.cpu.B11, Pin.cpu.C7]
right_bump_pins = [Pin.cpu.A15, Pin.cpu.H0,  Pin.cpu.H1]
bump_sensors = BumpSensors(left_pins=left_bump_pins, right_pins=right_bump_pins)

pins = [
    # Leftmost two sensors (C0, C1) disabled; we only use 7 sensors:
    # C2, C3, C4, C5, B0, B1, A4  (center is C5/B0 pair, i.e. original sensor 7)
    Pin.cpu.C2, Pin.cpu.C3, Pin.cpu.C4, Pin.cpu.C5,
    Pin.cpu.B0, Pin.cpu.B1, Pin.cpu.A4,
]
sensor_fun = multiple_ir_readings(*pins)

# ADC calibration — per-sensor black/white arrays (left -> right).
# Updated black values from your latest measurements.
black_adc = [1627, 1566, 1500, 1046, 1114, 1294, 852]
white_adc = [256, 268, 260, 243, 247, 256, 235]
line = L_sensor(
    sensor_fun,
    black=black_adc,
    white=white_adc,
    bias=0.0,
    sensor_count=7,
)

# IMU — BNO055 on I2C1 (PB8=SCL, PB9=SDA)
# rst_pin=None: PC8 is already used by the left motor PWM, so we skip HW reset
i2c = pyb.I2C(1, pyb.I2C.CONTROLLER, baudrate=400000)


# -----------------
# Tuning parameters — edit these before flashing
# -----------------
BASE_EFFORT = 20.0   # raised — right motor needs >20% to overcome friction          20
KP_LINE     = 6.0    # proportional gain on line error (raw value, no hidden scaling)
KI_LINE     = .010    # integral gain on line error
                     # start at 0, raise slowly (e.g. 0.001) if robot drifts on long straights
MAX_EFFORT  = 90.0   # hard ceiling on any single wheel PWM %
                     # must be > BASE_EFFORT or steering has no headroom
RIGHT_OFFSET = 2.0   # additive PWM % added to right wheel to correct straight-line drift   2
                     # if robot veers LEFT  -> increase (try 2, 3, 4...)
                     # if robot veers RIGHT -> decrease or go negative
                     # tune with 's' key until encoder diff stays near 0

# IMU stabilization gains (set to 0.0 to disable each term)
YAW_RATE_GAIN = .010   # damps oscillation using gyro Z (deg/s) — increase if robot wiggles
HEADING_GAIN  = 0.35   # corrects heading drift using absolute Euler heading (deg)
CURVE_THRESH  = 20.0   # deg/s — yaw rate above this suppresses heading hold on curves


# -----------------
# Shares
# -----------------
leftMotorGo  = Share('b', thread_protect=False, name="LM_go")
rightMotorGo = Share('b', thread_protect=False, name="RM_go")
armEnable    = Share('b', thread_protect=False, name='arm_en')

leftEffortCmd  = Share('f', thread_protect=False, name="LM_eff")
rightEffortCmd = Share('f', thread_protect=False, name="RM_eff")

lineError        = Share('f', thread_protect=False, name="line_err")
lineFollowEnable = Share('b', thread_protect=False, name="lf_en")
baseEffort       = Share('f', thread_protect=False, name="base_effort")

# IMU shares — written by task_imu, read by task_control
imu_heading_deg  = Share('f', thread_protect=False, name="imu_head")
imu_yaw_rate_dps = Share('f', thread_protect=False, name="imu_wz")
imu_calib        = Share('b', thread_protect=False, name="imu_cal")

# State estimator output shares
psi_hat  = Share('f', thread_protect=False, name='psi_hat')
s_hat    = Share('f', thread_protect=False, name='s_hat')
omL_hat  = Share('f', thread_protect=False, name='omL_hat')
omR_hat  = Share('f', thread_protect=False, name='omR_hat')

# Initial values
leftMotorGo.put(False)
rightMotorGo.put(False)
armEnable.put(False)
leftEffortCmd.put(0.0)
rightEffortCmd.put(0.0)
lineFollowEnable.put(False)
baseEffort.put(BASE_EFFORT)
imu_heading_deg.put(0.0)
imu_yaw_rate_dps.put(0.0)
imu_calib.put(0)


# -----------------
# Task objects
# -----------------
sensor_obj = task_line_sensor(line, lineError, enable_share=lineFollowEnable)

# IMU task — reads BNO055 heading + yaw rate at 50 Hz
# rst_pin=None because PC8 is already used by left motor PWM (TIM3_CH3)
imu_task_obj = task_imu(i2c, imu_heading_deg, imu_yaw_rate_dps, imu_calib,
                        address=0x28, rst_pin=None)

# Single master control task owns BOTH wheels.
# It reads line error, computes base+steer for left and base-steer for right,
# and publishes both effort commands with coordinated scaling so the
# turn differential is never crushed by the effort ceiling.
ctrl_obj = task_control(
    mot=leftMotor,
    enc=leftEncoder,
    goFlag=leftMotorGo,
    effort_cmd=leftEffortCmd,
    line_sensor=line,
    other_effort_cmd=rightEffortCmd,
    other_goFlag=rightMotorGo,
    base_effort_share=baseEffort,
    imu_heading_share=imu_heading_deg,
    imu_yawrate_share=imu_yaw_rate_dps,
    psi_hat_share=psi_hat,
    omL_hat_share=omL_hat,
    omR_hat_share=omR_hat,
)
ctrl_obj._kp_line    = KP_LINE
ctrl_obj._ki_line    = KI_LINE
ctrl_obj._max_effort = MAX_EFFORT
ctrl_obj._right_offset = RIGHT_OFFSET   # additive right-wheel trim
ctrl_obj._right_enc    = rightEncoder   # needed for odometry
ctrl_obj.set_imu_gains(yawrate_gain=YAW_RATE_GAIN, heading_gain=HEADING_GAIN)
ctrl_obj._curve_threshold_dps = CURVE_THRESH

est_obj = task_state_estimator(
    leftEncoder, rightEncoder,
    imu_heading_deg, imu_yaw_rate_dps,
    leftEffortCmd, rightEffortCmd,
    psi_hat, s_hat, omL_hat, omR_hat,
)

motL_obj = task_motor(leftMotor,  leftMotorGo,  leftEffortCmd,  armEnable, invert=True)
motR_obj = task_motor(rightMotor, rightMotorGo, rightEffortCmd, armEnable, invert=True)
# Note: RIGHT_TRIM is now applied inside ctrl_obj via right_trim_share,
# so no monkey-patch needed here. The trim is also active during motor
# tests via task_user's direct hardware calls (which still use the share value).

user_obj = task_user(
    leftMotorGo, rightMotorGo,
    lineFollowEnable,
    armEnable,
    line,
    lineError=lineError,
    leftEffortCmd=leftEffortCmd,
    rightEffortCmd=rightEffortCmd,
    leftEncoder=leftEncoder,
    rightEncoder=rightEncoder,
    leftMotorDriver=leftMotor,
    rightMotorDriver=rightMotor,
    imu_heading=imu_heading_deg,
    imu_yawrate=imu_yaw_rate_dps,
    imu_calib=imu_calib,
    right_offset=RIGHT_OFFSET,
    ctrl=ctrl_obj,
    s_hat_share=s_hat,
    psi_hat_share=psi_hat,
    omL_hat_share=omL_hat,
    omR_hat_share=omR_hat,
    bump_sensors=bump_sensors,
)


# -----------------
# Scheduler — ctrlR removed, ctrl_obj drives both wheels
# -----------------
task_list.append(Task(sensor_obj.run,   name="Sensor", priority=5, period=20))
task_list.append(Task(imu_task_obj.run, name="IMU",    priority=5, period=20))
task_list.append(Task(ctrl_obj.run,     name="Ctrl",   priority=4, period=20))
task_list.append(Task(motL_obj.run,     name="MotL",   priority=3, period=20))
task_list.append(Task(motR_obj.run,     name="MotR",   priority=3, period=20))
task_list.append(Task(est_obj.run,      name="Est",    priority=2, period=20))  # matrices designed for Ts=20ms
task_list.append(Task(user_obj.run,     name="User",   priority=1, period=50))

collect()
print("Starting scheduler...")
while True:
    try:
        task_list.pri_sched()
    except KeyboardInterrupt:
        print("KeyboardInterrupt: stopping.")
        leftMotor.disable()
        rightMotor.disable()
        break
