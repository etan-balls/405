import pyb
from pyb import Pin, Timer

from motor_driver import motor_driver
from encoder_driver import encoder
from Line_sensor_driver import L_sensor
from multi_sensor_read import multiple_ir_readings

from task_motor import task_motor
from control_task import task_control
from task_sensor import task_line_sensor
from task_user import task_user

from task_share import Share
from cotask import Task, task_list
from gc import collect

# --- Bluetooth console + task (HC-05) ---
from bt_console import BTConsole
from task_bt import task_bt
from task_imu import task_imu



# -----------------
# Hardware setup
# -----------------
tim3 = Timer(3, freq=20000)
leftMotor  = motor_driver(Pin.cpu.B5,  Pin.cpu.B4,  Pin.cpu.C8, tim3, 3)
rightMotor = motor_driver(Pin.cpu.B15, Pin.cpu.B14, Pin.cpu.C9, tim3, 4)

leftEncoder  = encoder(2, pyb.Pin.cpu.A0, pyb.Pin.cpu.A1)
rightEncoder = encoder(1, pyb.Pin.cpu.A8, pyb.Pin.cpu.A9)

pins = [
    Pin.cpu.C0, Pin.cpu.C1, Pin.cpu.C2, Pin.cpu.C3, Pin.cpu.C4,
    Pin.cpu.C5, Pin.cpu.B0, Pin.cpu.B1, Pin.cpu.A4, Pin.cpu.A5, Pin.cpu.A6
]
sensor_fun = multiple_ir_readings(*pins)

# ADC calibration — update these from calibration mode ('c')
black_adc = 2000
white_adc = 350
line = L_sensor(sensor_fun, black=black_adc, white=white_adc, bias=0.0, sensor_count=11)

# -----------------
# IMU (BNO055 over I2C1)
# -----------------
# Pin mapping:
#   PB8  -> I2C1_SCL
#   PB9  -> I2C1_SDA
#   PC8  -> BNO055 RST
i2c = pyb.I2C(1, pyb.I2C.CONTROLLER, baudrate=400000)
imu_rst = Pin(Pin.cpu.C8, mode=Pin.OUT_PP)
imu_rst.value(1)
imu_task_obj = task_imu(i2c, imu_heading_deg, imu_yaw_rate_dps, imu_calib, address=0x28, rst_pin=imu_rst)


# -----------------
# Tuning parameters — edit these before flashing
# -----------------
BASE_EFFORT = 25.0   # base PWM % sent to both wheels (0-100)
                     # start low (~20-25%) and raise until it moves reliably
STEER_GAIN  = 0.08   # recommended starting point from calibration mode
MAX_EFFORT  = 50.0   # hard ceiling on any single wheel PWM %
                     # must be > BASE_EFFORT or steering has no headroom


# -----------------
# Shares
# -----------------
leftMotorGo  = Share('b', thread_protect=False, name="LM_go")
rightMotorGo = Share('b', thread_protect=False, name="RM_go")
armEnable    = Share('b', thread_protect=False, name='arm_en')

leftEffortCmd  = Share('f', thread_protect=False, name="LM_eff")
rightEffortCmd = Share('f', thread_protect=False, name="RM_eff")

lineError        = Share('f', thread_protect=False, name="line_err")
imu_heading_deg  = Share('f', thread_protect=False, name="imu_head")
imu_yaw_rate_dps = Share('f', thread_protect=False, name="imu_wz")
imu_calib        = Share('b', thread_protect=False, name="imu_cal")
lineFollowEnable = Share('b', thread_protect=False, name="lf_en")
baseEffort       = Share('f', thread_protect=False, name="base_effort")
steerGain        = Share('f', thread_protect=False, name="steer_gain")

# Initial values
leftMotorGo.put(False)
rightMotorGo.put(False)
armEnable.put(False)
leftEffortCmd.put(0.0)
rightEffortCmd.put(0.0)
lineFollowEnable.put(False)
baseEffort.put(BASE_EFFORT)
steerGain.put(STEER_GAIN)
imu_heading_deg.put(0.0)
imu_yaw_rate_dps.put(0.0)
imu_calib.put(0)


# -----------------
# Task objects
# -----------------
sensor_obj = task_line_sensor(line, lineError, enable_share=lineFollowEnable)

# Single master control task owns BOTH wheels.
# It reads line error, computes base+steer for left and base-steer for right,
# and publishes both effort commands with coordinated scaling so the
# turn differential is never crushed by the effort ceiling.
ctrl_obj = task_control(
    mot=leftMotor,
    enc=leftEncoder,
    goFlag=leftMotorGo,
    effort_cmd=leftEffortCmd,
    # Effort-based line follow wiring:
    line_sensor=line,
    other_effort_cmd=rightEffortCmd,
    other_goFlag=rightMotorGo,
    base_effort_share=baseEffort,
    imu_heading_share=imu_heading_deg,
    imu_yawrate_share=imu_yaw_rate_dps,
)
# Set gains directly on the object
ctrl_obj._kp_line    = STEER_GAIN * 10   # kp_line scales error -> effort offset
ctrl_obj._kd_line    = 0.0
ctrl_obj._max_effort = MAX_EFFORT
# IMU stabilization defaults (tune these!)
# yawrate_gain adds damping using gyro Z (deg/s)
# heading_gain holds heading relative to start-of-run
ctrl_obj.set_imu_gains(yawrate_gain=0.05, heading_gain=0.02)

motL_obj = task_motor(leftMotor,  leftMotorGo,  leftEffortCmd,  armEnable, invert=True)
motR_obj = task_motor(rightMotor, rightMotorGo, rightEffortCmd, armEnable, invert=True)

user_obj = task_user(
    leftMotorGo, rightMotorGo,
    lineFollowEnable,
    armEnable,
    line,
    lineError,
    imu_heading_deg,
    imu_yaw_rate_dps
)



# -----------------
# Bluetooth (HC-05) debug console (mirrors task_user UI + adds record mode)
# -----------------
bt = BTConsole(uart_num=2, baud=9600, echo=False)

bt_obj = task_bt(
    bt,
    leftEnc=leftEncoder,
    rightEnc=rightEncoder,
    line_sensor=line,
    ctrl_obj=ctrl_obj,
    leftMotorGo=leftMotorGo,
    rightMotorGo=rightMotorGo,
    lineFollowEnable=lineFollowEnable,
    armEnable=armEnable,
    lineError=lineError,
    imu_heading=imu_heading_deg,
    imu_yawrate=imu_yaw_rate_dps,
    imu_calib=imu_calib,
    baseEffort_share=baseEffort
)

bt.send_line("BT READY  f=start  c=calibrate   |  REC START 50 3  |  REC DUMP")

# -----------------
# Scheduler — ctrlR removed, ctrl_obj drives both wheels
# -----------------
task_list.append(Task(sensor_obj.run, name="Sensor", priority=5, period=20))
task_list.append(Task(ctrl_obj.run,   name="Ctrl",   priority=4, period=20))
task_list.append(Task(motL_obj.run,   name="MotL",   priority=3, period=20))
task_list.append(Task(motR_obj.run,   name="MotR",   priority=3, period=20))
task_list.append(Task(user_obj.run,   name="User",   priority=1, period=50))

# IMU task
task_list.append(Task(imu_task_obj.run, name="IMU", priority=5, period=20))

# BT tasks
task_list.append(Task(bt_obj.run_poll, name="BT_RX",  priority=6, period=10))
task_list.append(Task(bt_obj.run_app,  name="BT_APP", priority=2, period=20))

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
