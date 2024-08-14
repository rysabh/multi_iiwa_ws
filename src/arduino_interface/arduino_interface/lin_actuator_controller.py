import serial
import time

def main():
    # Replace '/dev/ttyACM0' with your Arduino's serial port
    # For Windows, it might be 'COM3' or similar
    port = '/dev/ttyACM0'
    baud_rate = 115200
    ser = None

    try:
        # Initialize serial connection
        ser = serial.Serial(port, baud_rate, timeout=1)
        time.sleep(2)  # Wait for the serial connection to initialize

        while True:
            # Get user input for the distance in mm
            distance_mm = input("Enter distance in mm (or 'exit' to quit): ")

            if distance_mm.lower() == 'exit':
                print("Moving to 0 position and Exiting program.")
                ser.write("0\n".encode('utf-8'))
                time.sleep(1)
                break

            try:
                # Convert the input to a float and send it to the Arduino
                distance = float(distance_mm)
                ser.write(f"{distance}\n".encode('utf-8'))
                print(f"Sent: {distance} mm")
                
                # Optionally, read response from Arduino
                response = ser.readline().decode('utf-8').strip()
                if response:
                    print(f"Arduino response: {response}")

            except ValueError:
                print("Invalid input. Please enter a valid number.")

    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")

    finally:
        if ser and ser.is_open:
            ser.close()

if __name__ == '__main__':
    main()
