#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from dynamixel_sdk import PortHandler, PacketHandler, GroupBulkRead
from rclpy.executors import ExternalShutdownException

from ament_index_python.packages import get_package_share_directory
from pathlib import Path

from medlab_dynamixel_hardware_interface.utils.motor_models import (
    MotorModelDatabase,
    OPERATING_MODES,
)
from medlab_dynamixel_hardware_interface.srv import SetMotorTorqueEnable

import math

from dynamixel_sdk import (
    PortHandler,
    PacketHandler,
    GroupBulkRead,
    GroupBulkWrite,
)

class DynamixelHardwareInterface(Node):

    def __init__(self):
        super().__init__("dynamixels")

        self.loadParameters()
        self.setupHardware()
        self.setupIO()

        self.get_logger().info("Dynamixel hardware interface started")
    
    def loadParameters(self):
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 1000000)
        self.declare_parameter("feedback_rate", 50.0)
        self.declare_parameter("motor_names", ["motor_1"])
        self.declare_parameter("motor_ids", [1])
        self.declare_parameter("current_scales", [1.0])
        self.declare_parameter("operating_modes", ["position"])

        self.port_name = self.get_parameter("port").value
        self.baudrate = self.get_parameter("baudrate").value
        self.feedback_rate = float(self.get_parameter("feedback_rate").value)
        self.motor_names = list(self.get_parameter("motor_names").value)
        self.motor_ids = list(self.get_parameter("motor_ids").value)
        self.current_scales = list(self.get_parameter("current_scales").value)
        self.operating_modes = list(self.get_parameter("operating_modes").value)

        if not self.motor_names or not (
            len(self.motor_names)
            == len(self.motor_ids)
            == len(self.current_scales)
            == len(self.operating_modes)
        ):
            raise ValueError("Motor configuration arrays must have equal nonzero lengths")

        if len(set(self.motor_names)) != len(self.motor_names):
            raise ValueError("Motor names must be unique")

        if len(set(self.motor_ids)) != len(self.motor_ids):
            raise ValueError("Motor IDs must be unique")

        if self.feedback_rate <= 0:
            raise ValueError("Feedback rate must be positive")

        if any(scale <= 0 for scale in self.current_scales):
            raise ValueError("Current scales must be positive")

        for mode in self.operating_modes:
            if mode not in OPERATING_MODES:
                raise ValueError(f"Unsupported operating mode: {mode}")
            
    def setupHardware(self):
        models_dir = Path(
            get_package_share_directory("medlab_dynamixel_hardware_interface")
        ) / "models"

        self.database = MotorModelDatabase(models_dir)

        self.port = PortHandler(self.port_name)
        self.packet = PacketHandler(2.0)
        self.bulk_read = None
        self.bulk_write = None
        self.motors = []

        try:
            if not self.port.openPort():
                raise RuntimeError(f"Cannot open {self.port_name}")

            if not self.port.setBaudRate(self.baudrate):
                raise RuntimeError(f"Cannot set baudrate {self.baudrate}")

            self.bulk_read = GroupBulkRead(self.port, self.packet)
            self.bulk_write = GroupBulkWrite(self.port, self.packet)
            
            for name, dxl_id, current_scale, mode_name in zip(
                self.motor_names,
                self.motor_ids,
                self.current_scales,
                self.operating_modes,
            ):
                model_number, result, error = self.packet.ping(self.port, dxl_id)
                self.check_result(result, error)

                model = self.database.get_model(model_number)

                self.configure_operating_mode(dxl_id, model, mode_name)

                registers = [
                    model.get_register(key)
                    for key in ("Present Position", "Present Velocity", "Present Current")
                ]

                start = min(reg.address for reg in registers)
                end = max(reg.address + reg.size for reg in registers)

                if not self.bulk_read.addParam(dxl_id, start, end - start):
                    raise RuntimeError(
                        f"Cannot register motor {dxl_id} for bulk reading"
                    )

                self.motors.append({
                    "name": name,
                    "id": dxl_id,
                    "model": model,
                    "mode": mode_name,
                    "current_scale": float(current_scale),
                })

                self.get_logger().info(
                    f"Connected {name}: ID={dxl_id}, model={model.model_file.stem}"
                )

        except Exception:
            self.close_bus()
            raise
        
    def setupIO(self):
        self.command_sub = self.create_subscription(
            JointState,
            "~/joint_commands",
            self.jointCommandCallback,
            10,
        )
            
        self.joint_state_pub = self.create_publisher(
            JointState, "~/joint_states", 10
        )

        self.feedback_timer = self.create_timer(
            1.0 / self.feedback_rate,
            self.read_feedback
        )
        
        self.torque_service = self.create_service(
            SetMotorTorqueEnable,
            "~/set_motor_torque_enable",
            self.setMotorTorqueEnableCallback,
        )

    
    def jointCommandCallback(self, msg):
        if not msg.name or len(msg.name) != len(msg.position):
            self.get_logger().error("Invalid JointState command dimensions")
            return

        if len(msg.name) != len(set(msg.name)):
            self.get_logger().error("Duplicate motor names in command")
            return

        motor_map = {motor["name"]: motor for motor in self.motors}
        commands = []

        for name, position in zip(msg.name, msg.position):
            if name not in motor_map:
                self.get_logger().error(f"Unknown motor: {name}")
                return

            motor = motor_map[name]

            if motor["mode"] not in (
                "position", "extended_position", "current_based_position"
            ):
                self.get_logger().error(
                    f"{name}: operating mode does not support position commands"
                )
                return

            if not math.isfinite(position):
                self.get_logger().error(f"{name}: non-finite position command")
                return

            model = motor["model"]
            reg = model.get_register("Goal Position")
            unit = model.get_unit("Goal Position")

            # Convert radians to raw Dynamixel position units.
            raw = round((position - unit.offset) / unit.scale)

            # Check that the value fits the register.
            bits = reg.size * 8

            if unit.signed:
                lower = -(1 << (bits - 1))
                upper = (1 << (bits - 1)) - 1
            else:
                lower = 0
                upper = (1 << bits) - 1

            if not lower <= raw <= upper:
                self.get_logger().error(
                    f"{name}: raw goal {raw} outside register range"
                )
                return

            # Convert to little-endian bytes for GroupBulkWrite.
            data = list(
                int(raw).to_bytes(
                    reg.size,
                    byteorder="little",
                    signed=unit.signed,
                )
            )

            commands.append((motor["id"], reg.address, reg.size, data))

        try:
            self.bulk_write.clearParam()

            for dxl_id, address, size, data in commands:
                if not self.bulk_write.addParam(dxl_id, address, size, data):
                    raise RuntimeError(
                        f"Failed to add motor {dxl_id} to bulk write"
                    )

            result = self.bulk_write.txPacket()
            self.check_result(result)

        except Exception as exc:
            self.get_logger().error(f"Position command failed: {exc}")

        finally:
            self.bulk_write.clearParam()

    
    def setMotorTorqueEnableCallback(self, request, response):
        motor_map = {motor["name"]: motor for motor in self.motors}

        # Validate all requested motors before making changes
        if not request.names:
            response.success = False
            response.message = "No motor names provided"
            return response

        if len(request.names) != len(set(request.names)):
            response.success = False
            response.message = "Duplicate motor names provided"
            return response

        invalid_names = [
            name for name in request.names if name not in motor_map
        ]

        if invalid_names:
            response.success = False
            response.message = f"Unknown motors: {invalid_names}"
            return response

        succeeded = []
        failed = []

        for name in request.names:
            motor = motor_map[name]
            reg = motor["model"].get_register("Torque Enable")

            try:
                result, error = self.packet.write1ByteTxRx(
                    self.port,
                    motor["id"],
                    reg.address,
                    1 if request.enable else 0
                )
                self.check_result(result, error)

                succeeded.append(name)

            except Exception as exc:
                failed.append(f"{name}: {exc}")

        action = "enabled" if request.enable else "disabled"

        response.success = len(failed) == 0
        response.message = (
            f"Torque {action}: {succeeded}. "
            f"Failures: {failed}"
        )

        if response.success:
            self.get_logger().info(response.message)
        else:
            self.get_logger().error(response.message)

        return response

    def configure_operating_mode(self, dxl_id, model, mode_name):
        mode_reg = model.get_register("Operating Mode")
        torque_reg = model.get_register("Torque Enable")

        requested_mode = OPERATING_MODES[mode_name]

        current_mode, result, error = self.packet.read1ByteTxRx(
            self.port, dxl_id, mode_reg.address
        )
        self.check_result(result, error)

        if current_mode == requested_mode:
            self.get_logger().info(
                f"Motor {dxl_id}: operating mode already {mode_name}"
            )
            return

        self.get_logger().info(
            f"Motor {dxl_id}: changing operating mode "
            f"{current_mode} -> {requested_mode} ({mode_name})"
        )

        # Torque must be disabled before changing operating mode.
        result, error = self.packet.write1ByteTxRx(
            self.port, dxl_id, torque_reg.address, 0
        )
        self.check_result(result, error)

        result, error = self.packet.write1ByteTxRx(
            self.port, dxl_id, mode_reg.address, requested_mode
        )
        self.check_result(result, error)

        actual_mode, result, error = self.packet.read1ByteTxRx(
            self.port, dxl_id, mode_reg.address
        )
        self.check_result(result, error)

        if actual_mode != requested_mode:
            raise RuntimeError(
                f"Motor {dxl_id}: failed to set operating mode "
                f"to {mode_name}"
            )

        self.get_logger().info(
            f"Motor {dxl_id}: operating mode set to {mode_name}; "
            "torque remains disabled"
        )

    def check_result(self, result, error=0):
        if result != 0:
            raise RuntimeError(self.packet.getTxRxResult(result))
        if error != 0:
            raise RuntimeError(self.packet.getRxPacketError(error))

    def read_register(self, motor, name):
        reg = motor["model"].get_register(name)
        dxl_id = motor["id"]

        if not self.bulk_read.isAvailable(dxl_id, reg.address, reg.size):
            raise RuntimeError(f"{motor['name']}: {name} unavailable")

        return self.bulk_read.getData(dxl_id, reg.address, reg.size)

    @staticmethod
    def signed(raw, bits):
        mask = (1 << bits) - 1
        raw &= mask
        sign = 1 << (bits - 1)
        return raw - (1 << bits) if raw & sign else raw

    def read_feedback(self):
        try:
            result = self.bulk_read.txRxPacket()
            self.check_result(result)

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()

            for motor in self.motors:
                model = motor["model"]

                raw_position = self.read_register(motor, "Present Position")
                raw_velocity = self.read_register(motor, "Present Velocity")
                raw_current = self.read_register(motor, "Present Current")

                position = model.convert("Present Position", raw_position)
                velocity = model.convert("Present Velocity", raw_velocity)

                current_reg = model.get_register("Present Current")
                current = self.signed(
                    raw_current, current_reg.size * 8
                ) * motor["current_scale"]

                msg.name.append(motor["name"])
                msg.position.append(position)
                msg.velocity.append(velocity)
                msg.effort.append(current)

            self.joint_state_pub.publish(msg)

        except RuntimeError as exc:
            self.get_logger().error(f"Feedback read failed: {exc}")

    def close_bus(self):
        # Disable torque on all connected motors
        if hasattr(self, "port") and self.port.is_open:
            for motor in self.motors:
                try:
                    reg = motor["model"].get_register("Torque Enable")

                    result, error = self.packet.write1ByteTxRx(
                        self.port,
                        motor["id"],
                        reg.address,
                        0
                    )
                    self.check_result(result, error)

                    self.get_logger().info(
                        f"Torque disabled: {motor['name']}"
                    )

                except Exception as exc:
                    self.get_logger().error(
                        f"Failed to disable torque on {motor['name']}: {exc}"
                    )
        if self.bulk_read is not None:
            self.bulk_read.clearParam()
            self.bulk_read = None
        if getattr(self, "bulk_write", None) is not None:
            self.bulk_write.clearParam()
            self.bulk_write = None
            
        if hasattr(self, "port") and self.port.is_open:
            self.port.closePort()

    def destroy_node(self):
        self.close_bus()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = DynamixelHardwareInterface()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
