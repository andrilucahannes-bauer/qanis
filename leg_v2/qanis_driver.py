from motor_handler import MotorHandler
from imu_controller import IMUController
from config import Config, TrajectoryType, MovementType
from trajectory_controller import WalkTrajectoryController, LinearTrajectoryController
from controller import Controller, GaitController, BalanceController
import time


def main():
    #mh = MotorHandler("/dev/ttyUSB0", 1000000, 1.0)
    mh = MotorHandler("COM7", 1000000, 1.0)
    #imu_controller = IMUController(sample_freq=15, beta=0.1)
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

    #balance_controller = BalanceController(imu_controller, mh, config)
    #balance_controller.initial_setup()

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
