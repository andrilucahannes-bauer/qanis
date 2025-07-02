from motor_handler import MotorHandler
from imu_controller import IMUController
from config import Config, TrajectoryType, MovementType
from trajectory_controller import WalkTrajectoryController, TwerkTrajectoryController
from controller import Controller, GaitController, BalanceController
from pynput import keyboard
import sys
import time


def main():
    # mh = MotorHandler("/dev/ttyUSB0", 1000000, 1.0)
    mh = MotorHandler("COM7", 1000000, 1.0)
    # imu_controller = IMUController(sample_freq=15, beta=0.1)
    imu_controller = None
    config = Config()

    # TODO change from hardcoded to params
    # change to args so that i dont need it with balance
    trajectory_type = TrajectoryType.WALK
    movement_type = MovementType.GAIT

    sleep_time = config.sleep_time[movement_type]

    if not trajectory_type == TrajectoryType.NO_TRAJECTORY:
        trajectory_controller = WalkTrajectoryController(
            trajectory_type, config.gait_config.get_default_gait_params()
        )
    gait_controller = GaitController(trajectory_controller, mh, imu_controller, config)
    gait_controller.initial_setup()

    # balance_controller = BalanceController(imu_controller, mh, config)
    # balance_controller.initial_setup()

    try:
        while True:
            gait_controller.tick()
            time.sleep(sleep_time)
    except KeyboardInterrupt:
        print("❌ Interrupted by user")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        print("Cleaning up motors...")
        mh.clean_up(
            [
                id
                for leg in config.legs
                for id in [leg.upper_id, leg.lower_id, leg.inner_id]
            ]
        )

# tmp funktion um actions in key_action abzubilden
def print_key(k):
    def action():
        print(f"Key {k} pressed")
    return action


# dict welches keys auf roboter funktionen mapped die dann beim keypress aufgerufen werden
key_actions = {
    's': print_key("s"),
    'a': print_key("a")
}

def on_press(key):
    try:
        k = key.char
    except AttributeError:
        # Handle special keys (e.g., Key.enter)
        if key == keyboard.Key.enter:
            print("Not Stop!")
            return
        return

    action = key_actions.get(k)
    if action:
        action()

def test_main():
    listener = keyboard.Listener(on_press=on_press)
    listener.start()
    print("Hello, World!")

    try:
        while True:
            pass
    except KeyboardInterrupt:
        print("\nExiting...")
        
        listener.stop()
        sys.exit(0)


if __name__ == "__main__":
    # TODO add params for trajectory type / movement type
    test_main()
