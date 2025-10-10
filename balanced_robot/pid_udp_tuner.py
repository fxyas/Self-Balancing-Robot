import socket

UDP_IP = "192.168.220.6"  # Replace with your ESP32's IP
UDP_PORT = 8888

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(2)  # wait max 2s for response
# Kp=42.0, Ki=200.0, Kd=2.0, SP=-1.0
# Store PID + setpoint
pid = {"Kp": 50.0, "Ki": 300.0, "Kd": 2.1, "SP": -1.2}   
# pid = {"Kp": 42.0, "Ki": 200.0, "Kd": 2.0, "SP": -1.4}   
def send_pid():
    cmd = f"Kp:{pid['Kp']},Ki:{pid['Ki']},Kd:{pid['Kd']},SP:{pid['SP']}"
    sock.sendto(cmd.encode(), (UDP_IP, UDP_PORT))
    try:
        data, _ = sock.recvfrom(1024)
        print("ESP32 Response:", data.decode())
    except socket.timeout:
        print("⚠️ No response from ESP32")

def menu():
    print("\n--- PID Tuner ---")
    print(f"Current Values: Kp={pid['Kp']}, Ki={pid['Ki']}, Kd={pid['Kd']}, SP={pid['SP']}")
    print("1. Tune Kp")
    print("2. Tune Ki")
    print("3. Tune Kd")
    print("4. Tune Setpoint (SP)")
    print("5. Send Current PID Values")
    print("6. Exit")

def main():
    while True:
        menu()
        choice = input("Select option: ")

        if choice == "1":
            try: pid["Kp"] = float(input("Enter new Kp: ")); send_pid()
            except ValueError: print("Invalid input!")
        elif choice == "2":
            try: pid["Ki"] = float(input("Enter new Ki: ")); send_pid()
            except ValueError: print("Invalid input!")
        elif choice == "3":
            try: pid["Kd"] = float(input("Enter new Kd: ")); send_pid()
            except ValueError: print("Invalid input!")
        elif choice == "4":
            try: pid["SP"] = float(input("Enter new Setpoint: ")); send_pid()
            except ValueError: print("Invalid input!")
        elif choice == "5":
            send_pid()
        elif choice == "6":
            print("Exiting...")
            break
        else:
            print("Invalid choice! Please select 1-6.")


if __name__ == "__main__":
    main()
