from Jetson import GPIO
import time

# Pin Definitions
input_pin = 12

def main():
    prev_value = None

    # Pin Setup:
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(input_pin, GPIO.IN)  # set pin as an input pin
    print("Starting demo now! Press CTRL+C to exit")
    try:
        while True:
            value = GPIO.input(input_pin)
            if value != prev_value:
                if value == GPIO.HIGH:
                    value_str = "HIGH"
                else:
                    value_str = "LOW"
                print("Value read from pin {} : {}".format(input_pin,
                                                           value_str))
                prev_value = value
            # time.sleep(1)
    finally:
        GPIO.cleanup()

if __name__ == '__main__':
    main()
