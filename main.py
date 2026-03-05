import pyb
from pyb import Pin, Timer, UART

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


# -----------------
# Bluetooth UART1 setup (HC-05 on PB6=TX, PB7=RX)
# -----------------
bt = UART(1, baudrate=9600, timeout=10)

# Redirect the MicroPython REPL (and therefore all print/input) to UART3.
# PuTTY connects to the HC-05 COM port at 115200 to see all output.
pyb.repl_uart(bt)


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
# Tuning parameters — edit these before flashing
# -----------------
BASE_EFFORT = 15.0   # base PWM % sent to both wheels (0-100)
                     # start low (~20-25%) and raise until it moves reliably
STEER_GAIN  = 0.4   # recommended starting point from calibration mode
MAX_EFFORT  = 50.0   # hard ceiling on any single wheel PWM %
                     # must be > BASE_EFFORT or steering has no headroom
RIGHT_TRIM  = 1.1   # scale factor to compensate for slower right motor
                     # > 1.0 boosts right, < 1.0 cuts right
                     # tune until both wheels spin at the same speed with 'r' vs 'l'


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
)
# Set gains directly on the object
ctrl_obj._kp_line    = STEER_GAIN * 10   # kp_line scales error -> effort offset
ctrl_obj._kd_line    = 0.0
ctrl_obj._max_effort = MAX_EFFORT

motL_obj = task_motor(leftMotor,  leftMotorGo,  leftEffortCmd,  armEnable, invert=True)
motR_obj = task_motor(rightMotor, rightMotorGo, rightEffortCmd, armEnable, invert=True)

# Apply right motor trim by monkey-patching set_effort.
# This scales every effort command sent to the right motor by RIGHT_TRIM
# so both wheels spin at the same speed for a given command.
_rightMotor_set_effort_orig = rightMotor.set_effort
def _rightMotor_set_effort_trimmed(effort):
    _rightMotor_set_effort_orig(effort * RIGHT_TRIM)
rightMotor.set_effort = _rightMotor_set_effort_trimmed

user_obj = task_user(
    leftMotorGo, rightMotorGo,
    lineFollowEnable,
    armEnable,
    line,
    leftEffortCmd=leftEffortCmd,
    rightEffortCmd=rightEffortCmd,
    leftEncoder=leftEncoder,
    rightEncoder=rightEncoder,
    leftMotorDriver=leftMotor,
    rightMotorDriver=rightMotor,
)


# -----------------
# Scheduler — ctrlR removed, ctrl_obj drives both wheels
# -----------------
task_list.append(Task(sensor_obj.run, name="Sensor", priority=5, period=20))
task_list.append(Task(ctrl_obj.run,   name="Ctrl",   priority=4, period=20))
task_list.append(Task(motL_obj.run,   name="MotL",   priority=3, period=20))
task_list.append(Task(motR_obj.run,   name="MotR",   priority=3, period=20))
task_list.append(Task(user_obj.run,   name="User",   priority=1, period=50))

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
