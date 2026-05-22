import time
import board
import busio
import adafruit_bno055

# Start I2C
i2c = busio.I2C(board.SCL, board.SDA)

# Initialize sensor
sensor = adafruit_bno055.BNO055_I2C(i2c)

print("BNO055 test started...")

while True:
    print("----------------------")

    # Euler orientation (degrees)
    print("Orientation:")
    print("Heading:", sensor.euler[0])
    print("Roll:", sensor.euler[1])
    print("Pitch:", sensor.euler[2])

    # Acceleration
    print("\nAcceleration (m/s²):")
    print(sensor.acceleration)

    # Gyroscope
    print("\nGyroscope (rad/s):")
    print(sensor.gyro)

    time.sleep(1)