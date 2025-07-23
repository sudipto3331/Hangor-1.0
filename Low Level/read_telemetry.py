from pymavlink import mavutil
import time
from datetime import datetime

# Connect to MAVLink via UDP
master = mavutil.mavlink_connection('udp:0.0.0.0:14550')
print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Heartbeat received!\n")

print("--- Telemetry Monitor ---")
print("{:<20} {:<12} {:<12} {:<18} {:<18}".format(
    "Time", "Voltage (V)", "Current (A)", "Pressure (hPa)", "Pressure2 (hPa)"
))

# Initialize values
last_voltage = last_current = last_pressure = last_pressure2 = None

voltage_str = current_str = pressure_str = pressure2_str = "N/A"

try:
    while True:
        msg = master.recv_match(blocking=True)
        if not msg:
            continue

        updated = False

        if msg.get_type() == 'BATTERY_STATUS':
            voltage = msg.voltages[0] / 1000.0
            current = msg.current_battery / 100.0

            if 0 < voltage < 60:
                if voltage != last_voltage:
                    voltage_str = f"{voltage:.2f}"
                    last_voltage = voltage
                    updated = True

            if current >= 0:
                if current != last_current:
                    current_str = f"{current:.2f}"
                    last_current = current
                    updated = True

        elif msg.get_type() == 'SCALED_PRESSURE':
            pressure = msg.press_abs
            if pressure != last_pressure:
                pressure_str = f"{pressure:.2f}"
                last_pressure = pressure
                updated = True

        elif msg.get_type() == 'SCALED_PRESSURE2':
            pressure2 = msg.press_abs
            if pressure2 != last_pressure2:
                pressure2_str = f"{pressure2:.2f}"
                last_pressure2 = pressure2
                updated = True

        if updated:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            print("{:<20} {:<12} {:<12} {:<18} {:<18}".format(
                timestamp, voltage_str, current_str, pressure_str, pressure2_str
            ))

except KeyboardInterrupt:
    print("\nTelemetry monitoring stopped.")
