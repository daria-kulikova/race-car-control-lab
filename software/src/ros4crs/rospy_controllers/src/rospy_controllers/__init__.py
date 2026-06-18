from rospy_controllers.zero_order_gpmpc import (
    MPCCCPacejkaControllerZOGPMPC,
)
from rospy_controllers.mpcc import MPCCPacejkaController


def resolve_controller(controller_type: str, state_type: str, input_type: str):
    if controller_type == "MPCC_ZOGPMPC":
        if state_type == "pacejka_car" and input_type == "pacejka_car":
            return MPCCCPacejkaControllerZOGPMPC

    if controller_type == "MPCC_PYTHON":
        if state_type == "pacejka_car" and input_type == "pacejka_car":
            return MPCCPacejkaController

    raise ValueError(
        f"No controller found for {controller_type}, {state_type}, {input_type}"
    )
