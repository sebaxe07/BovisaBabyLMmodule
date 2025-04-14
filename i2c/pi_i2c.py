from smbus2 import SMBus
import time

I2C_ADDRESS = 0x08  # Arduino I2C address

def send_int(value):
    with SMBus(1) as bus:
        try:
            bus.write_byte(I2C_ADDRESS, value)
            print(f"Raspberry Pi sent: {value} to Arduino at address 0x{I2C_ADDRESS:02X}")
        except Exception as e:
            print(f"Error sending data: {e}")

if __name__ == "__main__":
    try:
        while True:
            send_int(1)
            time.sleep(2)
            send_int(2)
            time.sleep(2)
            send_int(3)
            time.sleep(2)
    except KeyboardInterrupt:
        print("Exiting...")
