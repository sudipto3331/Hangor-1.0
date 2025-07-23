"""Simple Hangor Movement Code."""

import time
import math

class HangorAUV:
    """Simple AUV controling functions."""
    
    def __init__(self):
        """AUV initialization"""
        # Current status
        self.current_direction = "stopped"
        self.current_speed = 0.0
        self.is_moving = False

        # Velocity tracking. (unit: m/s)
        self.velocity_x = 0.0   # Forward and Backward.
        self.velocity_y = 0.0   # Left and Right
        self.velocity_z = 0.0   # Up and Down

    # Default given 50% speed. Tune this according to your need.
    def move_forward(self, speed=50): 
        """Move forward at given speed in the range 0-100%"""
        self.current_direction = "forward"
        self.current_speed = speed
        self.is_moving = True
        self.velocity_x = speed*0.02    # Change the value according to need.
        self.velocity_y = 0.0
        self.velocity_z = 0.0

        print(f"Moving forward at speed {speed}% speed. ({self.velocity_x:0.2f} m/s)")
        self._send_to_thrusters("forward", speed)

    def move_backward(self, speed=50):
        """Move backward at given speed in the range 0-100%"""
        self.current_direction = "backward"
        self.current_speed = speed
        self.is_moving = True
        self.velocity_x = -(speed*0.02)
        self.velocity_y = 0.0
        self.velocity_z = 0.0

        print(f"Moving backward at {speed}% speed. ({self.velocity_x:0.2f} m/s)")
        self._send_to_thrusters("backward", speed)
    
    def move_right(self, speed=50):
        """Move right at given speed in the range 0-100%"""
        self.current_direction = "right"
        self.current_speed = speed
        self.is_moving = True
        self.velocity_x = 0.0
        self.velocity_y = speed*0.02
        self.velocity_z = 0.0

        print(f"Moving right at {speed}% speed. ({self.velocity_y:0.2f} m/s)")
        self._send_to_thrusters("right", speed)

    def move_left(self, speed=50):
        """Move left at given speed in the range 0-100%"""
        self.current_direction = "left"
        self.current_speed = speed
        self.is_moving = True
        self.velocity_x = 0.0
        self.velocity_y = -(speed*0.02)
        self.velocity_z = 0.0

        print(f"Moving left at {speed}% speed. ({self.velocity_y:0.2f} m/s)")
        self._send_to_thrusters("left", speed)
    
    def move_up(self, speed=50):
        """Move up at given speed in the range 0-100%"""
        self.current_direction = "up"
        self.current_speed = speed
        self.is_moving = True
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.velocity_z = speed*0.02

        print(f"Moving up at {speed}% speed. ({self.velocity_z:0.2f} m/s)")
        self._send_to_thrusters("up", speed)

    def move_down(self, speed=50):
        """Move down at given speed in the range 0-100%"""
        self.current_direction = "down"
        self.current_speed = speed
        self.is_moving = True
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.velocity_z = -(speed*0.02)

        print(f"Moving down at {speed}% speed. ({self.velocity_z:0.2f} m/s)")
        self._send_to_thrusters("down", speed)

    def stop(self):
        """Stop all movement"""
        self.current_direction = "stopped"
        self.current_speed = 0.0
        self.is_moving = False
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.velocity_z = 0.0

        print(f"AUV stopped: All thruster OFF")
        self._send_to_thrusters("stop", 0)
    
    def get_speed(self):
        """Get current total speed."""
        total_speed = math.sqrt(self.velocity_x**2+self.velocity_y**2+self.velocity_z**2)
        return total_speed

    def get_direction_angle(self):
        """Get current direction in degrees (0: Forward, 90: Right, 180: Left)"""
        if self.velocity_x==0.0 and self.velocity_y==0.0:
            return 0 # No horizontal movement
        
        angle = math.degrees(math.atan2(self.velocity_y, self.velocity_x))
        return angle%360    # Range (0-360)
    
    def print_status(self):
        """Show the AUV status"""
        total_speed = self.get_speed()
        direction_angle = self.get_direction_angle()

        print("\n" + "="*50)
        print("HANGOR STATUS REPORT")
        print("="*50)
        print(f"Current Action: {self.current_direction.upper()}")
        print(f"Current Power Level: {self.current_speed:0.01f}%")
        print(f"Is Moving: {'YES' if self.is_moving else 'NO'}")
        print(f"")
        print(f"VELOCITY BREAKDOWN:\n")
        print(f"Forward/Back: {self.velocity_x:+.2f} m/s")
        print(f"Left/Right: {self.velocity_y:+.2f} m/s") 
        print(f"Up/Down: {self.velocity_z:+.2f} m/s")
        print(f"Total Speed: {total_speed:.2f} m/s")
        print(f"Direction: {direction_angle:.1f}°")
        print("="*50)

    def _send_to_thrusters(self, direction, speed):
        """This function sends to commands to 8 thrusters."""

        # Converting to PWM values.
        pwm_values = self._convert_to_pwm(direction, speed)

        # Pixhawk code. (How commands will be sent to the Pixhawk via rc_override)

        # Thruster Configuration Data.
        print(f"Command sent to 8-thruster system: {direction} and {speed}%")
        print(f"Vertical Thrusters (0-3): {pwm_values[0:4]}")
        print(f"Horizontal Thrusters (4-7): {pwm_values[4:8]}")

        # Show the active thrusters.
        active_thrusters = []
        for i, pwm in enumerate(pwm_values):
            if pwm != 1500:
                thruster_type = "Vertical" if i<4 else "Horizontal"
                power_direction = "Forward" if pwm>1500 else "Backward"
                power_percent = abs(pwm - 1500)/400 * 100
                active_thrusters.append(f"CH{i} ({thruster_type}): {power_direction} | {power_percent:0.0f}%")

        if active_thrusters:
            print(f"Active: {', '.join(active_thrusters)}")
        else:
            print(f"All thrusters stopped.")

    def print_thruster_layout(self):
        """Print the thruster configuration for reference"""
        print("\n" + "="*60)
        print("THRUSTER CONFIGURATION")
        print("="*60)
        print("VERTICAL THRUSTERS (facing down for up/down movement):")
        print("  Channel 0: Vertical Thruster 1 (front-left)")
        print("  Channel 1: Vertical Thruster 2 (front-right)")
        print("  Channel 2: Vertical Thruster 3 (back-left)")
        print("  Channel 3: Vertical Thruster 4 (back-right)")
        print("")
        print("HORIZONTAL THRUSTERS (45° angles for forward/back/left/right):")
        print("  Channel 4: Front-Left")
        print("  Channel 5: Front-Right")
        print("  Channel 6: Back-Left") 
        print("  Channel 7: Back-Right")
        print("")
        print("MOVEMENT PATTERNS:")
        print("  UP:      All vertical thrusters spin to push water down")
        print("  DOWN:    All vertical thrusters spin to pull water up")
        print("  FORWARD: All horizontal thrusters push forward")
        print("  BACK:    All horizontal thrusters push backward")
        print("  LEFT:    Left thrusters pull, right thrusters push")
        print("  RIGHT:   Left thrusters push, right thrusters pull")
        print("="*60)  

    def _convert_to_pwm(self, direction, speed):
        """
        Convert direction and speed to PWM values for 8 thrusters
        
        Thruster Configuration:
        - Channels 0-3: Vertical thrusters (facing down for up/down movement)
        - Channels 4-7: Horizontal thrusters (at 45° angles for forward/backward/left/right)
        
        Horizontal Thruster Layout:
        CH4: Front-Left     CH5: Front-Right
        CH6: Back-Left     CH7: Back-Right
        """

        neutral_pwm = 1500
        max_offset = 400
        pwm_offset = int((speed/100.0)*max_offset)
        pwm_values = [neutral_pwm]*8

        # VERTICAL MOVEMENT (Channels 0-3)
        if direction == "up":
            # All vertical thrusters push down to move AUV up
            pwm_values[0] = neutral_pwm + pwm_offset  # Vertical thruster 1
            pwm_values[1] = neutral_pwm + pwm_offset  # Vertical thruster 2
            pwm_values[2] = neutral_pwm + pwm_offset  # Vertical thruster 3
            pwm_values[3] = neutral_pwm + pwm_offset  # Vertical thruster 4
            
        elif direction == "down":
            # All vertical thrusters pull up to move AUV down
            pwm_values[0] = neutral_pwm - pwm_offset  # Vertical thruster 1
            pwm_values[1] = neutral_pwm - pwm_offset  # Vertical thruster 2
            pwm_values[2] = neutral_pwm - pwm_offset  # Vertical thruster 3
            pwm_values[3] = neutral_pwm - pwm_offset  # Vertical thruster 4
        
        # HORIZONTAL MOVEMENT (Channels 4-7: Horizontal thrusters at 45° angles)
        elif direction == "forward":
            # Front thrusters push forward, back thrusters push forward
            pwm_values[4] = neutral_pwm + pwm_offset  # Front-Left 
            pwm_values[5] = neutral_pwm + pwm_offset  # Front-Right
            pwm_values[6] = neutral_pwm + pwm_offset  # Back-Left 
            pwm_values[7] = neutral_pwm + pwm_offset  # Back-Right
            
        elif direction == "backward":
            # All horizontal thrusters reverse to move backward
            pwm_values[4] = neutral_pwm - pwm_offset  # Front-Left 
            pwm_values[5] = neutral_pwm - pwm_offset  # Front-Right
            pwm_values[6] = neutral_pwm - pwm_offset  # Back-Left 
            pwm_values[7] = neutral_pwm - pwm_offset  # Back-Right
            
        elif direction == "left":
            # Right-side thrusters push right, left-side thrusters pull left
            pwm_values[4] = neutral_pwm - pwm_offset  # Front-Left (pull)
            pwm_values[5] = neutral_pwm + pwm_offset  # Front-Right (push)
            pwm_values[6] = neutral_pwm - pwm_offset  # Back-Left (pull)
            pwm_values[7] = neutral_pwm + pwm_offset  # Back-Right (push)
            
        elif direction == "right":
            # Left-side thrusters push left, right-side thrusters pull right
            pwm_values[4] = neutral_pwm + pwm_offset  # Front-Left (push)
            pwm_values[5] = neutral_pwm - pwm_offset  # Front-Right (pull)
            pwm_values[6] = neutral_pwm + pwm_offset  # Back-Left (push)
            pwm_values[7] = neutral_pwm - pwm_offset  # Back-Right (pull)
        
        return pwm_values
    
def show_menu():
    """Show the command menu"""
    print("\n" + "="*60)
    print("HANGOR AUV MOVEMENT CONTROLLER")
    print("="*60)
    print("Commands:")
    print("  1. forward [speed]   - Move forward")
    print("  2. backward [speed]  - Move backward") 
    print("  3. left [speed]      - Move left")
    print("  4. right [speed]     - Move right")
    print("  5. up [speed]        - Move up")
    print("  6. down [speed]      - Move down")
    print("  7. stop              - Stop all movement")
    print("  8. status            - Show current status")
    print("  9. layout            - Show thruster configuration")
    print("  10. menu             - Show this menu")
    print("  11. quit             - Exit program")
    print("")
    print("Speed: 0-100% (default is 50% if not specified)")
    print("Examples: 'forward 60', 'up 30', 'stop'")
    print("")
    print("8-THRUSTER SYSTEM:")
    print("  • 4 Vertical thrusters (CH 0-3): Up/Down movement")
    print("  • 4 Horizontal thrusters (CH 4-7): Forward/Back/Left/Right")
    print("="*60)
    
def parse_command(user_input, auv):
    """Parse and execute user commands."""
    parts = user_input.lower().strip().split()
    
    if not parts:
        return True
    
    command = parts[0]
    
    # Get speed if provided, otherwise use default
    speed = 50
    if len(parts) > 1:
        try:
            speed = int(parts[1])
            if not 0 <= speed <= 100:
                print("Speed must be between 0-100%")
                return True
        except ValueError:
            print("Invalid speed. Using default 50%")
            speed = 50
    
    # Execute commands
    if command in ['forward', 'f', '1']:
        auv.move_forward(speed)
    elif command in ['backward', 'back', 'b', '2']:
        auv.move_backward(speed)
    elif command in ['left', 'l', '3']:
        auv.move_left(speed)
    elif command in ['right', 'r', '4']:
        auv.move_right(speed)
    elif command in ['up', 'u', '5']:
        auv.move_up(speed)
    elif command in ['down', 'd', '6']:
        auv.move_down(speed)
    elif command in ['stop', 's', '7']:
        auv.stop()
    elif command in ['status', 'stat', '8']:
        auv.print_status()
    elif command in ['layout', 'thrusters', '9']:
        auv.print_thruster_layout()
    elif command in ['menu', 'help', '10']:
        show_menu()
    elif command in ['quit', 'exit', 'q', '11']:
        print("HASTA LA VISTA, BABY!")
        return False
    else:
        print(f"Unknown command: '{command}'")
        print("Type 'menu' to see available commands")
    
    return True

def main():
    """Main program loop"""
    print("Welcome to BengalSub AUV Controller!")
    print("Starting up...")
    time.sleep(1)
    
    # Create AUV controller
    auv = HangorAUV()
    
    # Show initial menu
    show_menu()
    
    # Main interactive loop
    try:
        while True:
            # Get user input
            user_input = input("\nEnter command: ")
            
            # Parse and execute command
            if not parse_command(user_input, auv):
                break  # User wants to quit
            
    except KeyboardInterrupt:
        print("\n\nProgram interrupted. See you never!")
    
    except Exception as e:
        print(f"\nError: {e}")
    
    finally:
        # Always stop the AUV before exiting
        auv.stop()
        print("AUV bondho hoiya gese.")

if __name__ == "__main__":
    main()