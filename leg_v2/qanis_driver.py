from motor_handler import MotorHandler
from imu_controller import IMUController
from config import Config, TrajectoryType, MovementType
from trajectory_controller import TrajectoryController
from controller import Controller, GaitController
import time


def main():
    mh = MotorHandler("COM7", 1000000, 1.0)
    #imu_controller = IMUController()
    imu_controller = None  # Placeholder for IMUController, if needed
    config = Config()

    # TODO change from hardcoded to params
    trajectory_type = TrajectoryType.TROT
    movement_type = MovementType.GAIT

    total_ticks = config.gait_config.total_ticks[trajectory_type]
    sleep_time = config.sleep_time[movement_type]

    trajectory_controller = TrajectoryController(
        trajectory_type, total_ticks, config.gait_config.get_default_gait_params()
    )
    gait_controller = GaitController(trajectory_controller, mh, imu_controller, config)

    gait_controller.initial_setup()

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


if __name__ == "__main__":
    # TODO add params for trajectory type / movement type
    main()
