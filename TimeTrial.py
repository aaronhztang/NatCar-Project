#*****************************************************************************
#
# Application Name     -   EEC195B Time Trial
# Application Overview -   Natcar Time Trial
# Author               -   Huangze Tang, Eric Qing, Ryan Blanc
#
#*****************************************************************************

import sensor, image, time, math, pyb
import utime
from pyb import LED, UART, Pin, Timer

# Always pass UART 1 for the UART number for your OpenMV Cam.
# The second argument is the UART baud rate. For a more advanced UART control
# example see the BLE-Shield driver.
uart = UART(1, 115200, timeout_char=1000)

# Initialize the command.
command = '0'

# Color Tracking Thresholds (Grayscale Min, Grayscale Max)
# The below grayscale threshold is set to only find extremely bright white areas.
thresholds = (250, 255)

# Each roi is (x, y, w, h).
# This roi spans the entire width of the image and is about 10 pixels high, which is in the top-portion of the image.
roi = (0, 0, 160, 10)

# Initialize the position variable.
angle = 0
# Store the error buffer for the PID caculation.
prev_e_1 = 0
prev_e_2 = 0
prev_e_3 = 0
prev_e_4 = 0
# Initialize the PID term.
Pterm = 0
Dterm = 0
Iterm = 0
prev_I = 0

# Camera setup...
sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.GRAYSCALE) # use grayscale.
sensor.set_framesize(sensor.QQVGA) # use QQVGA for speed. (160*120)
sensor.skip_frames(time = 2000) # Let new settings take affect.
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
clock = time.clock() # Tracks FPS.
led = LED(1) # Initialize the red LED.
led.off() # Turn off the red LED.
i = 0 # Initialize the counter

# Initialize the control signals for the VNH5019A-E.
ina = pyb.Pin("P2", pyb.Pin.OUT_PP) # Initialize the ina.
ina.high() # Turn on the pin.
inb = pyb.Pin("P3", pyb.Pin.OUT_PP) # Initialize the inb.
inb.low() # Turn off the pin.
ena = pyb.Pin("P6", pyb.Pin.OUT_PP) # Initialize the ena.
ena.high() # Turn on the pin.
enb = pyb.Pin("P5", pyb.Pin.OUT_PP) # Initialize the enb.
enb.high() # Turn on the pin.


stop = True # Initialize the stop flag.
stopcounter = 0 # Initialize the final line flag.
finish = LED(2) # Initialize the green LED for finishing.
finish.off() # Turn off the green LED.
start = 0 # Initialize the start time.
delta = 0 # Initialize the program time.
start = utime.ticks_ms() # Record the start time.

# Set up two Timers for servo and motor at different frequencies.
tim1 = Timer(4, freq=300)
# Frequency in Hz for the digital servo.
tim2 = Timer(2, freq=1000)
# Frequency in Hz for the DC Motor.

# Initialize the desired pulse width in seconds (1.5ms).
DPW = 0.0015
Timescaler = tim1.source_freq() / (tim1.prescaler() + 1)
# Calculate the actual pulse width based on the source frequency and the prescale value in OpenMV.
APW = DPW * Timescaler

# Generate a PWM square wave on TIM4 with calculated actual pulse width on channel 1.
ch1 = tim1.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width=int(APW))
# Generate a PWM square wave on TIM2 with 0 pulse width percent on channel 3.
ch3 = tim2.channel(3, Timer.PWM, pin=Pin("P4"), pulse_width_percent=0)

# Set up the PID Value.
Ku = 4
Pu = 10
Kp = Ku/1.7
Kd = Kp*Pu/2
Ki = 0

# Only blobs that with more pixels than "pixel_threshold" and more area than "area_threshold" are
# returned by "find_blobs" below. Change "pixels_threshold" and "area_threshold" if you change the
# camera resolution. "merge=True" merges all overlapping blobs in the image.

while(True):
    clock.tick() # Track elapsed milliseconds between snapshots().
    # BT Control.
    if uart.any() > 0 :
        # Read a single character frm UART1.
        command = chr(uart.readchar())
        # Print the command.
        print (command)
        # "g" for start, "s" for stop.
        if command == 'g' :
            stop = False
            finish.off()
        elif command == 's' :
            stop = True
            finish.on()
            ch3.pulse_width_percent(0)

    # Run the car front when stop is false.
    if stop == False :
        ina.high()
        inb.low()

    img = sensor.snapshot() # Take a picture and return the image.

    # Find blobs in the top-portion in the image.
    blobs = img.find_blobs([thresholds], roi=roi, pixels_threshold=5, area_threshold=10, merge=True)

    # If there is any blob in the top-portion:
    if (len(blobs) >= 1):
        # Find the blob closest to the previous center.
        center_blob = min(blobs, key=lambda b: math.sqrt(pow((b.cx() - 80 + prev_e_1),2) + pow((b.cy() - 5),2)))
        # Draw a rect around the blob.
        img.draw_rectangle(center_blob.rect(), color = 0)
        # Draw a cross through the center's coordinate.
        img.draw_cross(center_blob.cx(), center_blob.cy(), color = 0)
        # Replace the center x position.
        
        # Refresh the error buffer.
        prev_e_4 = prev_e_3
        prev_e_3 = prev_e_2
        prev_e_2 = prev_e_1
        # To decrease the oscillation, refresh the newest error only when 
        # the newest one has enough difference from the previous one.
        if math.fabs(prev_e_1 - 80 + center_blob.cx()) > 10:
            prev_e_1 = 80 - center_blob.cx()

        # Refresh the Iterm.
        prev_I = Iterm

        # Refresh the PID term.
        Pterm = prev_e_1
        Dterm = (prev_e_1 - prev_e_3 + 3*prev_e_1 - 3*prev_e_2)/(6*0.0125)
        Iterm = prev_I + 0.0125 * (prev_e_1 + prev_e_2) / 2

        # Calculate the current error for the correction.
        current_error = Kp * Pterm + Kd * Dterm + Ki * Iterm

        # Neglect the I Term to "unwind".
        if current_error > 100 or current_error < -100:
            current_error = Kp * Pterm + Kd * Dterm
        # Set up the deadband to decrease the sensibility.
        if current_error > 30 or current_error < -30:
        	# Calculate the angle based on the current error.
            angle = math.degrees(math.atan(current_error/120)) + 90
        else :
            angle = 90

        print("Turn Angle: %f" % angle)
        # Calculate the deflection angle away from the straight way.
        Deflection = math.fabs(angle - 90)
        # Calculate the desired pulse width based on the angle the car should go.
        DPW = ((angle/180.0) + 1.0) * 0.001
        # Transform the desired pulse width into the actual one.
        APW = DPW * Timescaler
        # Adjust the pulse width for the servo.
        ch1.pulse_width(int(APW))
        # Calculate the time from starting.
        delta = utime.ticks_diff(utime.ticks_ms(), start)
        # When the program has started 10s and detects three blobs in front.
        if delta > 10000 and len(blobs) == 3 :
        	# To tell from the finish line and the crossover.
            if (blobs[0].w() <= 15 and blobs[1].w() <= 15 and blobs[2].w() <= 15) :
                # Turn on the finish LED.
                finish.on()
                # Detect the line.
                stopcounter = 1

        # If stopcounter or stop flag is high, stop the car.      
        # Adjust the pulse width percent for the motor based on the deflection angle.
        # When the deflection angle is small, the motor should spin fast.
        # But the deflection angle is large, the motor should slow down.
        if stopcounter == 1 :
        	# Reverse the car to stop the car  in the desired range.
            ina.low()
            inb.high()
            ch3.pulse_width_percent(30)
            stopcounter = 0
            stop = True
            time.sleep(400)
        elif stop == True:
            ch3.pulse_width_percent(0)
        elif Deflection < 20 :
            ch3.pulse_width_percent(50)
        elif Deflection < 40 :
            ch3.pulse_width_percent(40)
        elif Deflection < 60 :
            ch3.pulse_width_percent(25)
        else :
            ch3.pulse_width_percent(25)

    # Measure the frame rate and print it.
    print(clock.fps())

    # When the main loops 20 times, toggle the red LED.
    if i == 20 :
        led.toggle() # LED toggle.
        i = 0 # Reset the counter.
    i = i + 1 # Increase the counter.

