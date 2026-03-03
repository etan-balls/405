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

from bt_console import BTConsole
from task_imu import task_imu


# -----------------
# Hardware setup
# -----------------
tim3 = Timer(3, freq=20000)

# Pinout:
#   Left  motor: NSLP=PB5, DIR=PB4,  PWM=PC8  (TIM3_CH3)
#   Right motor: NSLP=PB15, DIR=PB14, PWM=PC9 (TIM3_CH4)
leftMotor  = motor_driver(Pin.cpu.B5,  Pin.cpu.B4,  Pin.cpu.C8, tim3, 3)
rightMotor = motor_driver(Pin.cpu.B15, Pin.cpu.B14, Pin.cpu.C9, tim3, 4)

# Encoders:
#   Left:  TIM2, PA0 (CH1), PA1 (CH2)
#   Right: TIM1, PA8 (CH1), PA9 (CH2)
leftEncoder  = encoder(2, pyb.Pin.cpu.A0, pyb.Pin.cpu.A1)
rightEncoder = encoder(1, pyb.Pin.cpu.A8, pyb.Pin.cpu.A9)

# Line sensor pins (11 sensors)
pins = [
    Pin.cpu.C0, Pin.cpu.C1, Pin.cpu.C2, Pin.cpu.C3, Pin.cpu.C4,
    Pin.cpu.C5, Pin.cpu.A4, Pin.cpu.A6, Pin.cpu.A7, Pin.cpu.B0, Pin.cpu.B1
]
sensor_fun = multiple_ir_readings(*pins)

# ADC calibration — update from calibration mode ('c')
black_adc = 2000
white_adc = 350
line = L_sensor(sensor_fun, black=black_adc, white=white_adc, bias=0.0, sensor_count=11)

# -----------------
# Tuning parameters
# -----------------
BASE_EFFORT = 25.0
STEER_GAIN  = 0.08
MAX_EFFORT  = 50.0

# -----------------
# Shares (created BEFORE any task that uses them)
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
# IMU (BNO055 on I2C1: PB8=SCL, PB9=SDA)
# PC8 is shared with left motor PWM so do NOT use it as IMU RST.
# Pass rst_pin=None to skip hardware reset.
# -----------------
i2c = pyb.I2C(1, pyb.I2C.CONTROLLER, baudrate=400000)
imu_task_obj = task_imu(i2c, imu_heading_deg, imu_yaw_rate_dps, imu_calib,
                        address=0x28, rst_pin=None)

# -----------------
# Task objects
# -----------------
sensor_obj = task_line_sensor(line, lineError, enable_share=lineFollowEnable)

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
)
ctrl_obj._kp_line    = STEER_GAIN * 10
ctrl_obj._kd_line    = 0.0
ctrl_obj._max_effort = MAX_EFFORT
ctrl_obj.set_imu_gains(yawrate_gain=0.05, heading_gain=0.02)

motL_obj = task_motor(leftMotor,  leftMotorGo,  leftEffortCmd,  armEnable, invert=True)
motR_obj = task_motor(rightMotor, rightMotorGo, rightEffortCmd, armEnable, invert=True)

# Bluetooth — HC-06 on UART3 (PC10=TX, PC11=RX)
bt = BTConsole(uart_num=3, baud=9600, echo=False)

# Single merged UI task
ui_obj = task_user(
    bt,
    leftMotorGo,
    rightMotorGo,
    lineFollowEnable,
    armEnable,
    line,
    lineError        = lineError,
    imu_heading      = imu_heading_deg,
    imu_yawrate      = imu_yaw_rate_dps,
    imu_calib        = imu_calib,
    ctrl_obj         = ctrl_obj,
    baseEffort_share = baseEffort,
)

# -----------------
# Scheduler
# -----------------
task_list.append(Task(sensor_obj.run,   name="Sensor", priority=5, period=20))
task_list.append(Task(ctrl_obj.run,     name="Ctrl",   priority=4, period=20))
task_list.append(Task(motL_obj.run,     name="MotL",   priority=3, period=20))
task_list.append(Task(motR_obj.run,     name="MotR",   priority=3, period=20))
task_list.append(Task(imu_task_obj.run, name="IMU",    priority=5, period=20))
task_list.append(Task(ui_obj.run,       name="UI",     priority=1, period=20))

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
