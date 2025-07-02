from enum import Enum
from typing import Dict, Optional
import threading
import time
from controller import BalanceController, Controller, GaitController, TwerkController, _calculate_movement
from config import Config, MovementType, TrajectoryType, LegSide
from motor_handler import MotorHandler
from trajectory_controller import TwerkTrajectoryController, WalkTrajectoryController
from pynput import keyboard
import numpy as np

class RobotMode(Enum):
    IDLE = "idle"
    WALK = "walk" 
    BALANCE = "balance"
    SIT = "sit"
    UP = "up"
    STOP = "stop"
    TWERK = "twerk"
    DOWN = "down"
    TURN_LEFT = "turn_left"
    TURN_RIGHT = "turn_right"
    EXCITED = "excited"
    JUMP = "jump"

class RobotController:
    def __init__(self, motor_handler, imu_controller, config):
        self.motor_handler = motor_handler
        self.imu_controller = imu_controller
        self.config = config
        self.current_mode = RobotMode.IDLE
        self.mode_lock = threading.Lock()
        self.controllers: Dict[RobotMode, Controller] = {}
        self.running = False
        self.mode_changed = False
        self.setup_controllers()
    
    def setup_controllers(self):
        """Initialize all available controllers"""
        # Walk controller
        trajectory_controller = WalkTrajectoryController(
            TrajectoryType.WALK, 
            self.config.gait_config.get_default_gait_params()
        )
        linear_trajectory_controller = TwerkTrajectoryController(
            TrajectoryType.LINEAR, 
            self.config.gait_config.get_default_gait_params()
        )
        self.controllers[RobotMode.WALK] = GaitController(
            trajectory_controller, self.motor_handler, self.imu_controller, self.config
        )
        
        # Balance controller
        self.controllers[RobotMode.BALANCE] = BalanceController(
            self.imu_controller, self.motor_handler, self.config
        )
        
        # Sit controller
        self.controllers[RobotMode.SIT] = SitController(
            self.motor_handler, self.config
        )

        # Excited controller
        self.controllers[RobotMode.EXCITED] = ExcitedController(
            self.motor_handler, self.config
        )
        
        # Stand controller
        self.controllers[RobotMode.UP] = UpController(
            self.motor_handler, self.config
        )

        self.controllers[RobotMode.TWERK] = TwerkController(
            linear_trajectory_controller, self.motor_handler, self.imu_controller, self.config
        )

        self.controllers[RobotMode.DOWN] = DownController(
            self.motor_handler, self.config
        )

        self.controllers[RobotMode.TURN_LEFT] = TurnController(
            self.motor_handler, self.config, "left"
        )
        self.controllers[RobotMode.TURN_RIGHT] = TurnController(
            self.motor_handler, self.config, "right"
        )
        self.controllers[RobotMode.JUMP] = JumpController(
            self.motor_handler, self.config
        )
    
    def set_mode(self, mode: RobotMode):
        """Switch to a different robot mode"""
        with self.mode_lock:
            if mode == self.current_mode:
                return
                
            print(f"🔄 Switching from {self.current_mode.value} to {mode.value}")
            self.current_mode = mode
            self.mode_changed = True
    
    def start_control_loop(self):
        """Start the main control loop"""
        self.running = True
        current_controller = None
        
        print("🤖 Robot control loop started")
        
        while self.running:
            with self.mode_lock:
                mode = self.current_mode
                mode_changed = self.mode_changed
                self.mode_changed = False
            
            # Handle mode changes
            if mode_changed or current_controller is None:
                if mode == RobotMode.STOP:
                    break
                elif mode == RobotMode.IDLE:
                    current_controller = None
                    print("💤 Robot idle")
                else:
                    current_controller = self.controllers.get(mode)
                    if current_controller:
                        print(f"🚀 Initializing {mode.value} mode...")
                        try:
                            current_controller.initial_setup()
                            print(f"✅ {mode.value} mode ready")
                        except Exception as e:
                            print(f"❌ Failed to initialize {mode.value}: {e}")
                            current_controller = None
                            with self.mode_lock:
                                self.current_mode = RobotMode.IDLE
            
            # Execute current controller
            if current_controller and mode != RobotMode.IDLE:
                try:
                    current_controller.tick()
                except Exception as e:
                    print(f"❌ Error in {mode.value} controller: {e}")
                    with self.mode_lock:
                        self.current_mode = RobotMode.IDLE
                    current_controller = None
            
            # Use appropriate sleep time based on mode
            if mode == RobotMode.WALK:
                sleep_time = self.config.sleep_time.get(MovementType.GAIT, 0.1)
            elif mode == RobotMode.BALANCE:
                sleep_time = self.config.sleep_time.get(MovementType.BALANCE, 0.1)
            else:
                sleep_time = 0.1
            
            time.sleep(sleep_time)
    
    def stop_control_loop(self):
        """Stop the control loop"""
        self.running = False
    
    def emergency_stop(self):
        """Emergency stop - immediately stop all motors"""
        print("🚨 EMERGENCY STOP")
        with self.mode_lock:
            self.current_mode = RobotMode.STOP
        self.stop_control_loop()
        
        # Clean up motors
        try:
            motor_ids = [
                id for leg in self.config.legs 
                for id in [leg.upper_id, leg.lower_id, leg.inner_id]
            ]
            self.motor_handler.clean_up(motor_ids)
        except Exception as e:
            print(f"Error during cleanup: {e}")

# Simplified controller classes
class SitController(Controller):
    def __init__(self, motor_handler, config):
        self.motor_handler = motor_handler
        self.config = config
        self.sit_positions_set = False
    
    def tick(self):
        if not self.sit_positions_set:
            print("🪑 Moving to sit position...")
            # Add your sitting logic here
            # For now, just hold position
            self.sit_positions_set = True
        # In sit mode, we just maintain position, so minimal processing needed
        pass
    
    def initial_setup(self):
        print("🪑 Preparing to sit...")
        # Set up motors for sitting
        for leg in self.config.legs:
            for motor_id in [leg.upper_id, leg.lower_id, leg.inner_id]:
                self.motor_handler.set_speed(motor_id, self.config.default_motor_speed // 2)  # Slower for sitting
                self.motor_handler.set_torque(motor_id, 1)
        
        # Define sitting positions (you'll need to adjust these values)
        # This is a placeholder - adjust based on your robot's dimensions
        sit_positions = self.config.gait_config.default_position.copy()
        sit_positions[2, 2:] -= 5  # Lower the robot by 5 units
        
        for pos, leg in zip(sit_positions.T, self.config.legs):
            upper_dxl_angle, lower_dxl_angle, inner_dxl_angle = _calculate_movement(pos, leg)
            self._execute_movement(upper_dxl_angle, lower_dxl_angle, inner_dxl_angle, leg)
        
        time.sleep(1)
        self.sit_positions_set = False
    
    def _execute_movement(self, upper_dxl_angle, lower_dxl_angle, inner_dxl_angle, leg):
        self.motor_handler.move_motor(leg.upper_id, upper_dxl_angle)
        self.motor_handler.move_motor(leg.lower_id, lower_dxl_angle)
        self.motor_handler.move_motor(leg.inner_id, inner_dxl_angle)

# Simplified controller classes
class ExcitedController(Controller):
    def __init__(self, motor_handler, config):
        self.motor_handler = motor_handler
        self.config = config
        self.base_positions = None
        self.tick_counter = 0
        self.shake_amplitude = 1.0  # How high to lift the rear legs
        self.shake_speed = 1  # How fast to alternate (lower = faster)
        self.setup_complete = False
    
    def tick(self):
        if not self.setup_complete:
            return
            
        # Calculate which rear leg to lift based on tick counter
        lift_cycle = (self.tick_counter // self.shake_speed) % 2
        
        # Create working copy of base positions
        excited_positions = self.base_positions.copy()
        
        if lift_cycle == 0:
            # Lift left rear leg (index 2)
            excited_positions[2, 2] += self.shake_amplitude  # LH leg up
            print("🐕 Left rear up!")
        else:
            # Lift right rear leg (index 3)  
            excited_positions[2, 3] += self.shake_amplitude  # RH leg up
            print("🐕 Right rear up!")
        
        # Apply movements to all legs
        for i, leg in enumerate(self.config.legs):
            try:
                pos = excited_positions[:, i]
                upper_dxl_angle, lower_dxl_angle, inner_dxl_angle = _calculate_movement(pos, leg)
                self._execute_movement(upper_dxl_angle, lower_dxl_angle, inner_dxl_angle, leg)
            except Exception as e:
                print(f"Error in excited movement for leg {leg.name}: {e}")
        
        self.tick_counter += 1
    
    def initial_setup(self):
        print("🐕 Getting excited...")
        
        # Set up motors for excited movement
        for leg in self.config.legs:
            for motor_id in [leg.upper_id, leg.lower_id, leg.inner_id]:
                self.motor_handler.set_speed(motor_id, self.config.default_motor_speed)  # Normal speed for excitement
                self.motor_handler.set_torque(motor_id, 1)
        
        # Start from sitting position for more dramatic effect
        self.base_positions = self.config.gait_config.default_position.copy()
        self.base_positions[2, :2] -= 2  # Lower front legs slightly (sitting-like pose)
        
        # Move to initial position
        for pos, leg in zip(self.base_positions.T, self.config.legs):
            upper_dxl_angle, lower_dxl_angle, inner_dxl_angle = _calculate_movement(pos, leg)
            self._execute_movement(upper_dxl_angle, lower_dxl_angle, inner_dxl_angle, leg)
        
        time.sleep(1)
        self.setup_complete = True
        self.tick_counter = 0
        print("🐕 Ready to get excited!")
    
    def _execute_movement(self, upper_dxl_angle, lower_dxl_angle, inner_dxl_angle, leg):
        self.motor_handler.move_motor(leg.upper_id, upper_dxl_angle)
        self.motor_handler.move_motor(leg.lower_id, lower_dxl_angle)
        self.motor_handler.move_motor(leg.inner_id, inner_dxl_angle)

# Simplified controller classes
class DownController(Controller):
    def __init__(self, motor_handler, config):
        self.motor_handler = motor_handler
        self.config = config
        self.sit_positions_set = False
    
    def tick(self):
        if not self.sit_positions_set:
            print("🪑 Moving to sit position...")
            # Add your sitting logic here
            # For now, just hold position
            self.sit_positions_set = True
        # In sit mode, we just maintain position, so minimal processing needed
        pass
    
    def initial_setup(self):
        print("🪑 Preparing to sit...")
        # Set up motors for sitting
        for leg in self.config.legs:
            for motor_id in [leg.upper_id, leg.lower_id, leg.inner_id]:
                self.motor_handler.set_speed(motor_id, self.config.default_motor_speed // 2)  # Slower for sitting
                self.motor_handler.set_torque(motor_id, 1)
        
        # Define sitting positions (you'll need to adjust these values)
        # This is a placeholder - adjust based on your robot's dimensions
        sit_positions = self.config.gait_config.default_position.copy()
        sit_positions[2, :] -= 6  # Lower the robot by 5 units
        
        for pos, leg in zip(sit_positions.T, self.config.legs):
            upper_dxl_angle, lower_dxl_angle, inner_dxl_angle = _calculate_movement(pos, leg)
            self._execute_movement(upper_dxl_angle, lower_dxl_angle, inner_dxl_angle, leg)
        
        time.sleep(1)
        self.sit_positions_set = False
    
    def _execute_movement(self, upper_dxl_angle, lower_dxl_angle, inner_dxl_angle, leg):
        self.motor_handler.move_motor(leg.upper_id, upper_dxl_angle)
        self.motor_handler.move_motor(leg.lower_id, lower_dxl_angle)
        self.motor_handler.move_motor(leg.inner_id, inner_dxl_angle)

class UpController(Controller):
    def __init__(self, motor_handler, config):
        self.motor_handler = motor_handler
        self.config = config
        self.stand_complete = False
    
    def tick(self):
        # In stand mode, just maintain position
        pass
    
    def initial_setup(self):
        print("🧍 Standing up...")
        for leg in self.config.legs:
            for motor_id in [leg.upper_id, leg.lower_id, leg.inner_id]:
                self.motor_handler.set_speed(motor_id, self.config.default_motor_speed)
                self.motor_handler.set_torque(motor_id, 1)
        
        # Move to standing position (default position)
        positions = self.config.gait_config.default_position
        for pos, leg in zip(positions.T, self.config.legs):
            upper_dxl_angle, lower_dxl_angle, inner_dxl_angle = _calculate_movement(pos, leg)
            self._execute_movement(upper_dxl_angle, lower_dxl_angle, inner_dxl_angle, leg)
        
        time.sleep(1)
    
    def _execute_movement(self, upper_dxl_angle, lower_dxl_angle, inner_dxl_angle, leg):
        self.motor_handler.move_motor(leg.upper_id, upper_dxl_angle)
        self.motor_handler.move_motor(leg.lower_id, lower_dxl_angle)
        self.motor_handler.move_motor(leg.inner_id, inner_dxl_angle)

class JumpController(Controller):
    def __init__(self, motor_handler, config):
        self.motor_handler = motor_handler
        self.config = config
        self.sit_positions_set = False
    
    def tick(self):
        if not self.sit_positions_set:
            print("🪑 Moving to down position...")
            # Add your sitting logic here
            # For now, just hold position
            self.sit_positions_set = True
        # In sit mode, we just maintain position, so minimal processing needed
        pass
    
    def initial_setup(self):
        print("🪑 Preparing to lay down...")
        # Set up motors for sitting
        for leg in self.config.legs:
            for motor_id in [leg.upper_id, leg.lower_id, leg.inner_id]:
                self.motor_handler.set_speed(motor_id, self.config.default_motor_speed // 2)  # Slower for sitting
                self.motor_handler.set_torque(motor_id, 1)
        
        # Define sitting positions (you'll need to adjust these values)
        # This is a placeholder - adjust based on your robot's dimensions
        sit_positions = self.config.gait_config.default_position.copy()
        sit_positions[2, :] -= 6  # Lower the robot by 5 units
        
        for pos, leg in zip(sit_positions.T, self.config.legs):
            upper_dxl_angle, lower_dxl_angle, inner_dxl_angle = _calculate_movement(pos, leg)
            self._execute_movement(upper_dxl_angle, lower_dxl_angle, inner_dxl_angle, leg)
        
        time.sleep(1)

        for leg in self.config.legs:
            for motor_id in [leg.upper_id, leg.lower_id, leg.inner_id]:
                self.motor_handler.set_speed(motor_id, self.config.default_motor_speed * 2)
                self.motor_handler.set_torque(motor_id, 1)
        
        # Move to standing position (default position)
        positions = self.config.gait_config.default_position
        for pos, leg in zip(positions.T, self.config.legs):
            upper_dxl_angle, lower_dxl_angle, inner_dxl_angle = _calculate_movement(pos, leg)
            self._execute_movement(upper_dxl_angle, lower_dxl_angle, inner_dxl_angle, leg)
        
        time.sleep(1)
        self.sit_positions_set = False
    
    def _execute_movement(self, upper_dxl_angle, lower_dxl_angle, inner_dxl_angle, leg):
        self.motor_handler.move_motor(leg.upper_id, upper_dxl_angle)
        self.motor_handler.move_motor(leg.lower_id, lower_dxl_angle)
        self.motor_handler.move_motor(leg.inner_id, inner_dxl_angle)



class TurnController(Controller):
    def __init__(self, motor_handler, config, turn_direction="left"):
        self.motor_handler = motor_handler
        self.config = config
        self.turn_direction = turn_direction
        self.base_angles = {}  # Store base DXL angles for each motor
        self.turn_offset = 40 if turn_direction == "left" else -40  # Direct DXL offset
    
    def tick(self):
        # Simply apply offset to inner motors only
        for leg in self.config.legs:
            try:
                # Keep upper and lower at base angles
                upper_base = self.base_angles.get(f"{leg.name}_upper", 2048)
                lower_base = self.base_angles.get(f"{leg.name}_lower", 2048)
                inner_base = self.base_angles.get(f"{leg.name}_inner", 2048)
                
                # Apply turn offset to inner motor
                inner_angle = int(inner_base + self.turn_offset)
                
                self.motor_handler.move_motor(leg.upper_id, int(upper_base))
                self.motor_handler.move_motor(leg.lower_id, int(lower_base))
                self.motor_handler.move_motor(leg.inner_id, inner_angle)
                
            except Exception as e:
                print(f"Error in simple turn for leg {leg.name}: {e}")
    
    def initial_setup(self):
        print(f"🔄 Simple turn {self.turn_direction} setup...")
        
        # Set up motors
        for leg in self.config.legs:
            for motor_id in [leg.upper_id, leg.lower_id, leg.inner_id]:
                self.motor_handler.set_speed(motor_id, self.config.default_motor_speed // 3)
                self.motor_handler.set_torque(motor_id, 1)
        
        # Calculate and store base angles
        positions = self.config.gait_config.default_position
        for pos, leg in zip(positions.T, self.config.legs):
            try:
                upper_dxl_angle, lower_dxl_angle, inner_dxl_angle = _calculate_movement(pos, leg)
                
                # Store base angles
                self.base_angles[f"{leg.name}_upper"] = int(round(upper_dxl_angle))
                self.base_angles[f"{leg.name}_lower"] = int(round(lower_dxl_angle))
                self.base_angles[f"{leg.name}_inner"] = int(round(inner_dxl_angle))
                
            except Exception as e:
                print(f"Error calculating base angles for {leg.name}: {e}")
        
        print(f"✅ Simple turn {self.turn_direction} ready")

# Updated main function
def main():
    #COM7 for windows
    mh = MotorHandler("/dev/tty.usbserial-FT62AJDY", 1000000, 1.0)
    imu_controller = None  # IMUController(sample_freq=15, beta=0.1)
    config = Config()
    
    # Create robot controller
    robot_controller = RobotController(mh, imu_controller, config)
    
    # Set up keyboard controls
    def create_mode_action(mode: RobotMode):
        def action():
            robot_controller.set_mode(mode)
        return action
    
    # Define key mappings
    key_actions = {
        'w': create_mode_action(RobotMode.WALK),
        'b': create_mode_action(RobotMode.BALANCE), 
        's': create_mode_action(RobotMode.SIT),
        'u': create_mode_action(RobotMode.UP),
        'i': create_mode_action(RobotMode.IDLE),
        't': create_mode_action(RobotMode.TWERK),
        'd': create_mode_action(RobotMode.DOWN),
        'e': create_mode_action(RobotMode.EXCITED),
        'j': create_mode_action(RobotMode.JUMP),
        'q': robot_controller.emergency_stop,
    }

    special_key_actions = {
        keyboard.Key.left: create_mode_action(RobotMode.TURN_LEFT),
        keyboard.Key.right: create_mode_action(RobotMode.TURN_RIGHT),
        keyboard.Key.esc: robot_controller.emergency_stop,
    }
    
    def on_press(key):
        try:
            k = key.char
        except AttributeError:
            action = special_key_actions.get(key)
            if action:
                action()
                print(f"Special key '{key}' pressed")
            if key == keyboard.Key.esc:
                robot_controller.emergency_stop()
                return False
            return
        
        action = key_actions.get(k)
        if action:
            action()
            print(f"Key '{k}' pressed")
    
    # Set up keyboard listener in a separate thread
    listener = keyboard.Listener(on_press=on_press)
    listener.start()
    
    print("🤖 Robot Controller Started!")
    print("Controls:")
    print("  W - Walk")
    print("  B - Balance") 
    print("  S - Sit")
    print("  R - Stand")
    print("  I - Idle")
    print("  Q - Emergency Stop")
    print("  ESC - Exit")
    print("\nRobot is ready! Press keys to control...")
    
    try:
        # Run the control loop in the main thread
        robot_controller.start_control_loop()
    except KeyboardInterrupt:
        print("❌ Interrupted by user")
    finally:
        print("🧹 Cleaning up...")
        robot_controller.emergency_stop()
        listener.stop()

if __name__ == "__main__":
    main()