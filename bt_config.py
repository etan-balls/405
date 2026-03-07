# Instructions for usage:
# 1) Use a Nucleo with nothing else plugged into it.
# 2) Disconnect the USB cable from your computer but
#    leave it plugged into the Shoe of Brian.
# 3) Connect the HC-05 Bluetooth module to the Nucleo:
#    BT TX  -> PB7   (UART1_RX)
#    BT RX  <- PB6   (UART1_TX)
#    BT VCC <- Nucleo 3.3V or 5V
#    BT GND <- Nucleo GND
# 4) While holding down the enable button on the
#    Bluetooth module plug in the USB cable to your
#    laptop. Continue holding down the enable button
#    until you get a very slow blink on the Bluetooth
#    module's LED (about 2 sec on / 2 sec off).
#    This puts the HC-05 into AT command mode.
# 5) Modify the editable parameters listed below to
#    select desired UART settings, module name, and
#    password. Choose a name and password unique for
#    your lab group.
# 6) Transfer the code to the Nucleo and run the code
#    from PuTTY to send the configuration to the module.
# 7) If everything worked correctly, that should be it!

from pyb import Pin, UART
from time import sleep_ms

# User Editable Parameters
baudrate = 115200
stopbit  = 1
parity   = 0
pswd     = "mecha00"
name     = "mecha00"

# Allowable ranges
al_baudrate = [9600, 19200, 38400, 57600,
               115200, 230400, 460800]
al_stopbit  = [1, 2]
al_parity   = [0, 1, 2]

# Check the selected settings for errors
if baudrate not in al_baudrate:
    raise ValueError(f"Invalid Baudrate Selected; choose from {al_baudrate}")

if stopbit not in al_stopbit:
    raise ValueError(f"Invalid Stopbit Selected; choose from {al_stopbit}")

if parity not in al_parity:
    raise ValueError(f"Invalid Parity Selected; choose from {al_parity}")

# UART1 uses PB6 (TX) and PB7 (RX)
# HC-05 AT command mode runs at 38400 baud regardless of configured baudrate
ser = UART(1, 38400, timeout=1000)

user_in = input("Enter 'R' to factory reset or 'C' to configure the Bluetooth Module\r\nCMD: ")

if user_in in {'R', 'r'}:
    # Factory Reset Device
    s = "AT+ORGL\r\n"
    print("\nApplying Factory Reset to Module\n")
    print(f"Sending command: {repr(s)}")
    ser.write(s)
    sleep_ms(500)
    s = ser.readline()
    print(f"Device Response: {repr(s)}")
    if s is None:
        raise RuntimeError("Command timed out")
    elif b"OK" not in s:
        raise RuntimeError("Reset command not accepted")
    else:
        print("Factory reset successful\n")
    sleep_ms(500)

elif user_in in {'C', 'c'}:
    print("\nApplying Configuration to Module\n")

    # Rename device
    s = f"AT+NAME={name}\r\n"
    print("Renaming Device")
    print(f"Sending command: {repr(s)}")
    ser.write(s)
    sleep_ms(500)
    s = ser.readline()
    print(f"Device Response: {repr(s)}")
    if s is None:
        raise RuntimeError("Command timed out")
    elif b"OK" not in s:
        raise RuntimeError("Rename command not accepted")
    else:
        print("Name set successfully\n")
    sleep_ms(500)

    # Reset password
    s = f"AT+PSWD=\"{pswd}\"\r\n"
    print("Resetting Password")
    print(f"Sending command: {repr(s)}")
    ser.write(s)
    sleep_ms(500)
    s = ser.readline()
    print(f"Device Response: {repr(s)}")
    if s is None:
        raise RuntimeError("Command timed out")
    elif b"OK" not in s:
        raise RuntimeError("Password reset command not accepted")
    else:
        print("Password reset successfully\n")
    sleep_ms(500)

    # Configure UART
    s = f"AT+UART={baudrate},{stopbit},{parity}\r\n"
    print("Configuring UART")
    print(f"Sending command: {repr(s)}")
    ser.write(s)
    sleep_ms(500)
    s = ser.readline()
    print(f"Device Response: {repr(s)}")
    if s is None:
        raise RuntimeError("Command timed out")
    elif b"OK" not in s:
        raise RuntimeError("UART config command not accepted")
    else:
        print("UART configured successfully\n")
    sleep_ms(500)

    # Reset module to apply changes
    s = "AT+RESET\r\n"
    print("Resetting Module")
    print(f"Sending command: {repr(s)}")
    ser.write(s)
    sleep_ms(500)
    s = ser.readline()
    print(f"Device Response: {repr(s)}")
    if s is None:
        raise RuntimeError("Command timed out")
    elif b"OK" not in s:
        raise RuntimeError("Reset failed")
    else:
        print("Device reset successfully\n")

else:
    print("Program Aborted\n")
