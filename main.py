from machine import Pin, PWM, SPI, Timer
import utime
import mcp3008
import ujson

MOTOR_LEFT_IN1_PIN = 9
MOTOR_LEFT_IN2_PIN = 8
MOTOR_RIGHT_IN1_PIN = 11
MOTOR_RIGHT_IN2_PIN = 10
BUZZER_GPIO_PIN = 22
SPI_BUS_ID = 0; SPI_SCK_PIN_ID = 2; SPI_MOSI_PIN_ID = 3
SPI_MISO_PIN_ID = 4; SPI_CS_GPIO_NUM = 5
NUM_SENSORS = 5
SENSOR_MCP_CHANNELS = [5, 4, 3, 2, 1]
KP = 28.0
KI = 0
KD = 48
BASE_SPEED = 80
PWM_FREQ = 1000; MAX_PWM_DUTY = 65535
CALIBRATION_FILENAME = "calibration_data.json"
sensor_min_readings = [0] * NUM_SENSORS
sensor_max_readings = [1023] * NUM_SENSORS
last_error = 0; integral = 0
long_beep_timer = None

pwm_left_in1 = PWM(Pin(MOTOR_LEFT_IN1_PIN)); pwm_left_in2 = PWM(Pin(MOTOR_LEFT_IN2_PIN))
pwm_left_in1.freq(PWM_FREQ); pwm_left_in2.freq(PWM_FREQ)
pwm_right_in1 = PWM(Pin(MOTOR_RIGHT_IN1_PIN)); pwm_right_in2 = PWM(Pin(MOTOR_RIGHT_IN2_PIN))
pwm_right_in1.freq(PWM_FREQ); pwm_right_in2.freq(PWM_FREQ)
spi_bus_obj = SPI(SPI_BUS_ID, baudrate=1000000, polarity=0, phase=0,
                  sck=Pin(SPI_SCK_PIN_ID), mosi=Pin(SPI_MOSI_PIN_ID), miso=Pin(SPI_MISO_PIN_ID))
cs_pin_obj = Pin(SPI_CS_GPIO_NUM, Pin.OUT, value=1)
adc = mcp3008.MCP3008(spi_bus_obj, cs_pin_obj)
buzzer_pwm = PWM(Pin(BUZZER_GPIO_PIN))
buzzer_pwm.freq(2000)
buzzer_pwm.duty_u16(0)

def set_motor_speed(in1_pwm, in2_pwm, speed_percent):
    duty = int(abs(speed_percent / 100.0) * MAX_PWM_DUTY)
    duty = min(duty, MAX_PWM_DUTY)
    duty = max(0, duty)
    if speed_percent > 0:
        in1_pwm.duty_u16(duty)
        in2_pwm.duty_u16(0)
    elif speed_percent < 0:
        in1_pwm.duty_u16(0)
        in2_pwm.duty_u16(duty)
    else:
        in1_pwm.duty_u16(0)
        in2_pwm.duty_u16(0)

def stop_motors(brake=False, brake_duration_ms=50, brake_power_percent=30):
    if brake:
        set_motor_speed(pwm_left_in1, pwm_left_in2, -abs(brake_power_percent))
        set_motor_speed(pwm_right_in1, pwm_right_in2, -abs(brake_power_percent))
        utime.sleep_ms(brake_duration_ms)
    set_motor_speed(pwm_left_in1, pwm_left_in2, 0)
    set_motor_speed(pwm_right_in1, pwm_right_in2, 0)

def read_raw_sensor_values():
    values = [0] * NUM_SENSORS
    for i in range(NUM_SENSORS): values[i] = adc.read(SENSOR_MCP_CHANNELS[i])
    return values

def get_normalized_sensor_values():
    raw_values = read_raw_sensor_values()
    normalized_values = [0.0] * NUM_SENSORS
    for i in range(NUM_SENSORS):
        min_b, max_w = sensor_min_readings[i], sensor_max_readings[i]
        den = max_w - min_b
        if den <= 0:
            norm_val = 1.0 if raw_values[i] <= (min_b + 5) else 0.0
        else:
            val_raw_norm = (raw_values[i] - min_b) / den
            norm_val = 1.0 - val_raw_norm
        normalized_values[i] = max(0.0, min(1.0, norm_val))
    return raw_values, normalized_values

def save_calibration_data(min_b, max_w):
    data = {"min_readings_black": min_b, "max_readings_white": max_w}
    try:
        with open(CALIBRATION_FILENAME, "wt") as f: ujson.dump(data, f)
    except OSError as e: print(f"Error saving calibration data: {e}")

def load_calibration_data():
    global sensor_min_readings, sensor_max_readings
    try:
        with open(CALIBRATION_FILENAME,"rt") as f: data=ujson.load(f)
        if isinstance(data,dict) and "min_readings_black" in data and len(data["min_readings_black"])==NUM_SENSORS and \
           "max_readings_white" in data and len(data["max_readings_white"])==NUM_SENSORS:
            sensor_min_readings, sensor_max_readings = data["min_readings_black"], data["max_readings_white"]
            return True
        else: print(f"Invalid calibration data format in {CALIBRATION_FILENAME}."); return False
    except OSError: print(f"{CALIBRATION_FILENAME} not found. Will need to calibrate."); return False
    except ValueError: print(f"Error parsing JSON from {CALIBRATION_FILENAME}."); return False

def perform_interactive_calibration(duration_sec=20):
    global sensor_min_readings, sensor_max_readings, buzzer_pwm
    min_b_cal = [1023] * NUM_SENSORS
    max_w_cal = [0] * NUM_SENSORS
    t_start = utime.ticks_ms()
    while utime.ticks_diff(utime.ticks_ms(), t_start) < duration_sec * 1000:
        raw_values = read_raw_sensor_values()
        for i in range(NUM_SENSORS):
            if raw_values[i] < min_b_cal[i]:
                min_b_cal[i] = raw_values[i]
            if raw_values[i] > max_w_cal[i]:
                max_w_cal[i] = raw_values[i]
        utime.sleep_ms(20)
    sensor_min_readings = min_b_cal
    sensor_max_readings = max_w_cal
    
    BEEP_FREQUENCY = 2500
    BEEP_DURATION_MS = 150
    PAUSE_DURATION_MS = 100
    NUM_BEEPS = 2
    buzzer_pwm.freq(BEEP_FREQUENCY)
    for _ in range(NUM_BEEPS):
        buzzer_pwm.duty_u16(32767)
        utime.sleep_ms(BEEP_DURATION_MS)
        buzzer_pwm.duty_u16(0)
        if _ < NUM_BEEPS - 1:
            utime.sleep_ms(PAUSE_DURATION_MS)
    for i in range(NUM_SENSORS):
        if sensor_max_readings[i] <= sensor_min_readings[i]:
            print(f"WARNING: Sensor {SENSOR_MCP_CHANNELS[i]} (Idx {i}) Max(W) {sensor_max_readings[i]} <= Min(B) {sensor_min_readings[i]}.")
    save_calibration_data(sensor_min_readings, sensor_max_readings)
    utime.sleep_ms(100)

def calculate_line_position(norm_sensor_values):
    global last_error
    weights = [-2, -1, 0, 1, 2]
    numerator = 0.0
    denominator = 0.0
    line_detected_by_any_sensor = False
    for i in range(NUM_SENSORS):
        val = norm_sensor_values[i]
        if val > 0.5:
            line_detected_by_any_sensor = True
        numerator += weights[i] * val
        denominator += val
    if not line_detected_by_any_sensor or denominator < 0.1:
        if last_error > 0.2:
            return weights[-1] + 0.5
        elif last_error < -0.2:
            return weights[0] - 0.5
        return 0
    return numerator / denominator

def turn_off_long_beep_cb(timer_object_passed_in):
    global buzzer_pwm
    if buzzer_pwm:
        buzzer_pwm.duty_u16(0)

def play_f1_start_sound():
    global buzzer_pwm, long_beep_timer
    COUNTDOWN_BEEP_FREQ = 1500      
    COUNTDOWN_BEEP_DURATION_MS = 200
    COUNTDOWN_PAUSE_MS = 800
    NUM_COUNTDOWN_BEEPS = 5
    GO_BEEP_FREQ = COUNTDOWN_BEEP_FREQ
    GO_BEEP_DURATION_MS = 400
    utime.sleep_ms(500)
    for i in range(NUM_COUNTDOWN_BEEPS):
        buzzer_pwm.freq(COUNTDOWN_BEEP_FREQ)
        buzzer_pwm.duty_u16(32767)
        utime.sleep_ms(COUNTDOWN_BEEP_DURATION_MS)
        buzzer_pwm.duty_u16(0)
        if i < NUM_COUNTDOWN_BEEPS - 1:
            utime.sleep_ms(COUNTDOWN_PAUSE_MS)
    buzzer_pwm.freq(GO_BEEP_FREQ)
    buzzer_pwm.duty_u16(32767)
    if long_beep_timer is None:
        long_beep_timer = Timer()
    long_beep_timer.init(mode=Timer.ONE_SHOT, 
                         period=GO_BEEP_DURATION_MS, 
                         callback=turn_off_long_beep_cb)

def main_loop():
    global last_error, integral, buzzer_pwm, long_beep_timer
    if not load_calibration_data():
        print("No valid cal data. Starting interactive calibration...")
        utime.sleep(1)
        perform_interactive_calibration(7)
    
    play_f1_start_sound()
    utime.sleep_ms(GO_BEEP_DURATION_MS + 50 if 'GO_BEEP_DURATION_MS' in globals() else 450)

    try:
        while True:
            raw_vals, norm_vals = get_normalized_sensor_values()
            all_sensors_on_black = all(val > 0.70 for val in norm_vals) 
            
            if all_sensors_on_black:
                stop_motors(brake=True, brake_duration_ms=75, brake_power_percent=50)
                break
            
            error = calculate_line_position(norm_vals)
            p_term = KP * error
            integral += error
            integral_limit = 200.0 / KI if KI != 0 else float('inf')
            integral = max(-integral_limit, min(integral_limit, integral))
            i_term = KI * integral
            derivative = error - last_error
            d_term = KD * derivative
            last_error = error
            correction = p_term + i_term + d_term
            left_speed_command = BASE_SPEED + correction
            right_speed_command = BASE_SPEED - correction
            left_speed_command = max(-100, min(100, left_speed_command))
            right_speed_command = max(-100, min(100, right_speed_command))
            set_motor_speed(pwm_left_in1, pwm_left_in2, left_speed_command)
            set_motor_speed(pwm_right_in1, pwm_right_in2, right_speed_command)
            
    except KeyboardInterrupt:
        print("Program stopped by user.")
    finally:
        stop_motors(brake=True, brake_duration_ms=50, brake_power_percent=30)
        if buzzer_pwm:
            buzzer_pwm.duty_u16(0)
        if long_beep_timer:
            long_beep_timer.deinit()

if __name__ == "__main__":
    main_loop()