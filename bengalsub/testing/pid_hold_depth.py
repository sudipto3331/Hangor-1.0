
from pymavlink import mavutil
import time

# Configuration
TARGET_DEPTH = 0.3048  # 1 foot in meters
STABILIZE_TIME = 10    # seconds

# Simple PID gains (tune these for your vehicle)
KP = 200  # Proportional gain
KI = 20   # Integral gain  
KD = 50   # Derivative gain

def get_depth_m(master):
    """Get current depth from Bar30 pressure sensor"""
    msg = master.recv_match(type=['SCALED_PRESSURE', 'SCALED_PRESSURE2'], blocking=True, timeout=1)
    if not msg:
        return None
    pressure_mbar = msg.press_abs
    pressure_pa = pressure_mbar * 100
    depth = (pressure_pa - 101325) / (1000 * 9.80665)
    return max(depth, 0)

def calculate_pid(error, prev_error, integral, dt):
    """Calculate PID output"""
    p_term = KP * error
    integral += error * dt
    i_term = KI * integral
    d_term = KD * (error - prev_error) / dt if dt > 0 else 0
    
    output = p_term + i_term + d_term
    return output, integral

def arm_vehicle(master):
    """Arm the vehicle"""
    print("Arming...")
    master.arducopter_arm()
    master.motors_armed_wait()
    print("Armed!")

def disarm_vehicle(master):
    """Disarm the vehicle"""
    print("Disarming...")
    master.arducopter_disarm()
    master.motors_disarmed_wait()
    print("Disarmed!")

def set_manual_mode(master):
    """Set to MANUAL mode"""
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        master.mode_mapping()['MANUAL']
    )
    time.sleep(2)
    print("MANUAL mode set")

def send_throttle(master, pwm):
    """Send throttle command (1100-1900, 1500=neutral)"""
    pwm = max(1100, min(1900, int(pwm)))  # Safety limits
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        0, 0, pwm, 0, 0, 0, 0, 0
    )

def clear_overrides(master):
    """Clear all RC overrides"""
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        0, 0, 0, 0, 0, 0, 0, 0
    )

def dive_and_hold():
    """Main dive function with PID control"""
    # Connect
    print("Connecting...")
    master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    master.wait_heartbeat()
    print("Connected!")
    
    try:
        # Setup
        arm_vehicle(master)
        set_manual_mode(master)
        
        # PID variables
        prev_error = 0
        integral = 0
        prev_time = time.time()
        
        print(f"Diving to {TARGET_DEPTH}m...")
        
        # Dive and stabilize for total time
        start_time = time.time()
        total_time = 30  # 20s to reach + 10s stabilize
        
        while time.time() - start_time < total_time:
            # Get current depth
            current_depth = get_depth_m(master)
            if current_depth is None:
                continue
                
            # Calculate PID
            current_time = time.time()
            dt = current_time - prev_time
            error = TARGET_DEPTH - current_depth
            
            pid_output, integral = calculate_pid(error, prev_error, integral, dt)
            
            # Convert to PWM (1500 + pid_output)
            pwm = 1500 + pid_output
            send_throttle(master, pwm)
            
            # Status
            remaining = total_time - (time.time() - start_time)
            print(f"Depth: {current_depth:.3f}m, Error: {error:+.3f}m, PWM: {int(pwm)}, Time: {remaining:.1f}s")
            
            # Update for next iteration
            prev_error = error
            prev_time = current_time
            
            time.sleep(0.5)  # 2Hz control
        
        print("Mission complete!")
        
    except KeyboardInterrupt:
        print("Stopped by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        clear_overrides(master)
        disarm_vehicle(master)
        master.close()

if __name__ == "__main__":
    dive_and_hold()