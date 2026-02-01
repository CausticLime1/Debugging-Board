import serial
import serial.tools.list_ports
import time

def find_stm32_port():
    """Helper to list ports so you can find your STM32"""
    ports = serial.tools.list_ports.comports()
    print("\n--- Available Ports ---")
    for port in ports:
        print(f"{port.device}: {port.description} [{port.hwid}]")
    print("-----------------------\n")
    return ports

def main():
    # 1. List ports to help you identify the correct COM number
    find_stm32_port()
    
    # UPDATE THIS to match your board's port (e.g., 'COM3' on Windows, '/dev/ttyACM0' on Linux)
    PORT_NAME = 'COM14' 
    BAUD_RATE = 115200 # Baud rate is virtual for USB CDC, but 115200 is standard
    
    try:
        # 2. Open the Serial Connection
        # timeout=1 means read() will wait 1 second for data before giving up
        ser = serial.Serial(PORT_NAME, BAUD_RATE, timeout=1)
        print(f"Successfully opened {PORT_NAME}")
        
        # Give the device a moment to stabilize if DTR/RTS reset it
        time.sleep(0.1) 

        # 3. Send Data to STM32
        message = "Hello STM32, this is Python!\r\n"
        print(f"Sending: {message.strip()}")
        ser.write(message.encode('utf-8'))

        # 4. Read Response from STM32
        print("Waiting for response...")
        response = ser.readline() # Reads until \n or timeout
        
        if response:
            print(f"Received: {response.decode('utf-8', errors='ignore').strip()}")
        else:
            print("[!] No response received (Timeout).")
            print("    Check: Does your firmware echo data back or send periodic messages?")

        ser.close()

    except serial.SerialException as e:
        print(f"Error opening port {PORT_NAME}: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

if __name__ == "__main__":
    main()