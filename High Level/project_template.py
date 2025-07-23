import os
from pathlib import Path
import logging

logging.basicConfig(level=logging.INFO, format='[%(asctime)s]: %(message)s:')

project_name = "bengalsub_auv"

list_of_files = [
    f"{project_name}/auv/__init__.py",
    f"{project_name}/auv/sensors/__init__.py",
    f"{project_name}/auv/sensors/vectornav.py",
    f"{project_name}/auv/sensors/oakd_camera.py",
    f"{project_name}/auv/sensors/barometer.py",
    f"{project_name}/auv/sensors/hydrophones.py",
    f"{project_name}/auv/sensors/sensor_manager.py",

    f"{project_name}/auv/vision/__init__.py",
    f"{project_name}/auv/vision/yolo_detector.py",
    f"{project_name}/auv/vision/gate_detector.py",
    f"{project_name}/auv/vision/fish_detector.py",
    f"{project_name}/auv/vision/models/gate_model.pt",
    f"{project_name}/auv/vision/models/fish_model.pt",
    f"{project_name}/auv/vision/utils/__init__.py",
    f"{project_name}/auv/vision/utils/image_utils.py",

    f"{project_name}/auv/control/__init__.py",
    f"{project_name}/auv/control/pixhawk_interface.py",
    f"{project_name}/auv/control/movement_controller.py",
    f"{project_name}/auv/control/depth_controller.py",
    f"{project_name}/auv/control/heading_controller.py",
    f"{project_name}/auv/control/pid_controller.py",

    f"{project_name}/auv/localization/__init__.py",
    f"{project_name}/auv/localization/position_estimator.py",
    f"{project_name}/auv/localization/kalman_filter.py",
    f"{project_name}/auv/localization/navigation.py",

    f"{project_name}/auv/missions/__init__.py",
    f"{project_name}/auv/missions/base_mission.py",
    f"{project_name}/auv/missions/gate_mission.py",
    f"{project_name}/auv/missions/style_mission.py",
    f"{project_name}/auv/missions/stabilize_mission.py",
    f"{project_name}/auv/missions/mission_manager.py",

    f"{project_name}/auv/communication/__init__.py",
    f"{project_name}/auv/communication/pixhawk_comm.py",
    f"{project_name}/auv/communication/raspi_comm.py",
    f"{project_name}/auv/communication/surface_comm.py",

    f"{project_name}/auv/utils/__init__.py",
    f"{project_name}/auv/utils/config_manager.py",
    f"{project_name}/auv/utils/logger.py",
    f"{project_name}/auv/utils/safety.py",
    f"{project_name}/auv/utils/math_utils.py",

    f"{project_name}/config/main_config.yaml",
    f"{project_name}/config/sensor_config.yaml",
    f"{project_name}/config/control_config.yaml",
    f"{project_name}/config/mission_config.yaml",

    f"{project_name}/scripts/setup_environment.sh",
    f"{project_name}/scripts/calibrate_sensors.py",
    f"{project_name}/scripts/test_systems.py",
    f"{project_name}/scripts/run_mission.py",

    f"{project_name}/tests/test_sensors.py",
    f"{project_name}/tests/test_vision.py",
    f"{project_name}/tests/test_control.py",
    f"{project_name}/tests/test_missions.py",

    f"{project_name}/data/logs/.gitkeep",
    f"{project_name}/data/recordings/.gitkeep",
    f"{project_name}/data/calibration/.gitkeep",
    f"{project_name}/data/models/.gitkeep",

    f"{project_name}/requirements.txt",
    f"{project_name}/README.md",
    f"{project_name}/main.py"
]

# Logic to create the directory and files
for filepath in list_of_files:
    filepath = Path(filepath)
    filedir, filename = os.path.split(filepath)

    if filedir:
        os.makedirs(filedir, exist_ok=True)
        logging.info(f"Creating directory: {filedir} for file: {filename}")

    # Create file if not exists or is empty
    if not filepath.exists() or filepath.stat().st_size == 0:
        with open(filepath, "w") as f:
            # For .pt files or models, don't touch content
            if filepath.suffix != ".pt":
                f.write("")
        logging.info(f"Creating empty file: {filepath}")
    else:
        logging.info(f"File already exists: {filepath}")
