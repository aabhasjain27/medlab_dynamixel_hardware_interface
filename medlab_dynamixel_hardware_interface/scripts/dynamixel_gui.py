#!/usr/bin/env python3
"""Generic Dynamixel GUI: torque, feedback, and manual calibration jogging (radians)."""

import math
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

from medlab_dynamixel_hardware_interface.srv import SetMotorTorqueEnable, SetMotorZero


class DynamixelGuiNode(Node):
    def __init__(self):
        super().__init__('dynamixel_gui_node', automatically_declare_parameters_from_overrides=True)

        def parameter(name, default):
            if not self.has_parameter(name):
                self.declare_parameter(name, default)
            return self.get_parameter(name).value

        self.motor_names = list(self.get_parameter('motor_names').value)

        self.feedback_topic = self.get_parameter('feedback_topic').value
        self.calib_topic = self.get_parameter('calib_topic').value
        self.torque_service = self.get_parameter('torque_service').value
        self.zero_service = self.get_parameter('zero_service').value
        self.calibration_service = self.get_parameter('calibration_service').value

        self.positions = {}
        self.position_lock = threading.Lock()
        self.feedback_sub = self.create_subscription(JointState, self.feedback_topic, self.feedbackCallback, 10)
        self.calib_pub = self.create_publisher(JointState, self.calib_topic, 10)
        self.torque_client = self.create_client(SetMotorTorqueEnable, self.torque_service)
        self.zero_client = self.create_client(SetMotorZero, self.zero_service)
        self.calibration_client = self.create_client(SetBool, self.calibration_service)
        self.get_logger().info('Dynamixel control GUI initialized')

    def feedbackCallback(self, msg):
        with self.position_lock:
            self.positions = dict(zip(msg.name, msg.position))

    def getPositions(self):
        with self.position_lock:
            return dict(self.positions)

    def setTorque(self, names, enable):
        request = SetMotorTorqueEnable.Request()
        request.names = list(names)
        request.enable = enable
        return self.torque_client.call_async(request)

    def setZero(self, name):
        request = SetMotorZero.Request()
        request.name = name
        return self.zero_client.call_async(request)

    def setCalibrationMode(self, enabled):
        request = SetBool.Request()
        request.data = enabled
        return self.calibration_client.call_async(request)

    def sendCalibrationPosition(self, name, position):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [name]
        msg.position = [float(position)]
        self.calib_pub.publish(msg)


class DynamixelGUI:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.root.title('Dynamixel Motor Control')
        self.root.geometry('1100x550')
        self.root.minsize(850, 350)
        self.position_labels = {}
        self.jog_vars = {}
        self.jog_scales = {}
        self.pending = []
        self.calibration_mode = False
        self.mode_request_pending = False
        self.status = tk.StringVar(value='Normal mode')
        self.createWidgets()
        self.updateGUI()

    def createWidgets(self):
        main = ttk.Frame(self.root, padding=15)
        main.pack(fill=tk.BOTH, expand=True)
        ttk.Label(main, text='Dynamixel Motor Control', font=('Helvetica', 14, 'bold')).pack(pady=(0, 12))

        self.mode_button = ttk.Button(main, text='Enter Calibration Mode', command=self.toggleCalibrationMode)
        self.mode_button.pack(fill=tk.X, pady=(0, 12))

        table = ttk.LabelFrame(main, text='Motor Controls', padding=10)
        table.pack(fill=tk.BOTH, expand=True)
        for col, heading in enumerate(('Motor', 'Feedback (rad)', 'Torque On', 'Torque Off')):
            ttk.Label(table, text=heading, font=('Helvetica', 10, 'bold')).grid(row=0, column=col, padx=6, pady=6)

        for row, name in enumerate(self.node.motor_names, start=1):
            ttk.Label(table, text=name).grid(row=row, column=0, sticky='w', padx=6, pady=6)
            label = ttk.Label(table, text='--', width=15, anchor='center')
            label.grid(row=row, column=1, padx=6)
            self.position_labels[name] = label
            ttk.Button(table, text='Enable', command=lambda n=name: self.setTorque([n], True)).grid(row=row, column=2, padx=3)
            ttk.Button(table, text='Disable', command=lambda n=name: self.setTorque([n], False)).grid(row=row, column=3, padx=3)

        if not self.node.motor_names:
            ttk.Label(table, text='No motor_names configured.').grid(row=1, column=0, columnspan=4, pady=15)

        self.calibration_frame = ttk.LabelFrame(main, text='Calibration Jog (motor radians)', padding=10)
        for row, name in enumerate(self.node.motor_names):
            ttk.Label(self.calibration_frame, text=name, width=24).grid(row=row, column=0, sticky='w', pady=5)
            var = tk.StringVar(value='')
            self.jog_vars[name] = var
            scale = ttk.Scale(self.calibration_frame, from_=-6.2832, to=6.2832, orient=tk.HORIZONTAL)
            scale.grid(row=row, column=1, padx=6, sticky='ew')
            scale.bind('<ButtonRelease-1>', lambda event, n=name: self.sliderJog(n))
            self.jog_scales[name] = scale
            ttk.Entry(self.calibration_frame, textvariable=var, width=14).grid(row=row, column=2, padx=6)
            ttk.Button(self.calibration_frame, text='Jog to Position', command=lambda n=name: self.sendJog(n)).grid(row=row, column=3, padx=6)
            ttk.Button(self.calibration_frame, text='Set Reference', command=lambda n=name: self.setZero(n)).grid(row=row, column=4, padx=6)
            ttk.Button(self.calibration_frame, text='Use Feedback', command=lambda n=name: self.useFeedback(n)).grid(row=row, column=5, padx=6)

        self.calibration_frame.columnconfigure(1, weight=1)
        ttk.Label(self.calibration_frame, text='Jog values are relative motor radians. Set Reference aligns physical position with latest normal target.').grid(
            row=len(self.node.motor_names), column=0, columnspan=6, sticky='w', pady=(8, 0)
        )

        actions = ttk.Frame(main)
        self.actions = actions
        actions.pack(fill=tk.X, pady=(12, 8))
        ttk.Button(actions, text='Enable All', command=lambda: self.setTorque(self.node.motor_names, True)).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=4)
        ttk.Button(actions, text='Disable All', command=lambda: self.setTorque(self.node.motor_names, False)).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=4)
        ttk.Label(main, textvariable=self.status, wraplength=1000).pack(fill=tk.X)

    def toggleCalibrationMode(self):
        if self.mode_request_pending:
            return
        if not self.node.calibration_client.service_is_ready():
            self.status.set('Calibration mode service unavailable')
            return
        desired = not self.calibration_mode
        self.mode_request_pending = True
        self.mode_button.configure(state='disabled')
        self.pending.append((self.node.setCalibrationMode(desired), 'mode', desired))
        self.status.set('Switching calibration mode...')

    def useFeedback(self, name):
        position = self.node.getPositions().get(name)
        if position is None:
            self.status.set(f'No feedback available for {name}')
            return
        self.jog_vars[name].set(f'{position:.4f}')
        self.jog_scales[name].configure(from_=position - 6.2832, to=position + 6.2832)
        self.jog_scales[name].set(position)

    def sliderJog(self, name):
        if not self.calibration_mode:
            return
        self.jog_vars[name].set(f'{self.jog_scales[name].get():.4f}')
        self.sendJog(name)

    def sendJog(self, name):
        if not self.calibration_mode:
            return
        try:
            position = float(self.jog_vars[name].get())
            if not math.isfinite(position):
                raise ValueError('non-finite position')
        except ValueError:
            self.status.set(f'{name}: enter a finite position in radians')
            return
        self.node.sendCalibrationPosition(name, position)
        self.status.set(f'Jog target sent: {name} = {position:.4f} rad')

    def setTorque(self, names, enable):
        if not names:
            self.status.set('No motors configured')
            return
        if not self.node.torque_client.service_is_ready():
            self.status.set('Torque service unavailable')
            return
        self.pending.append((self.node.setTorque(names, enable), 'request', None))
        self.status.set('Sending torque request...')

    def setZero(self, name):
        if not self.calibration_mode:
            return
        if not self.node.zero_client.service_is_ready():
            self.status.set('Set Reference service unavailable')
            return
        self.pending.append((self.node.setZero(name), 'request', None))
        self.status.set(f'Setting reference for {name}...')

    def updateGUI(self):
        positions = self.node.getPositions()
        for name, label in self.position_labels.items():
            if name in positions:
                label.config(text=f'{positions[name]:.3f}')

        for future, kind, desired in self.pending[:]:
            if not future.done():
                continue
            try:
                result = future.result()
                if result is None:
                    raise RuntimeError('Service returned no result')
                self.status.set(result.message)
                if kind == 'mode' and result.success:
                    self.calibration_mode = desired
                    if desired:
                        for name in self.node.motor_names:
                            if name in positions:
                                self.useFeedback(name)
                        self.calibration_frame.pack(fill=tk.X, pady=(0, 8), before=self.actions)
                    else:
                        self.calibration_frame.pack_forget()
                    self.mode_button.configure(text='Exit Calibration Mode' if desired else 'Enter Calibration Mode')
            except Exception as exc:
                self.status.set(f'Service failed: {exc}')
            if kind == 'mode':
                self.mode_request_pending = False
                self.mode_button.configure(state='normal')
            self.pending.remove((future, kind, desired))
        self.root.after(100, self.updateGUI)


def main(args=None):
    rclpy.init(args=args)
    node = DynamixelGuiNode()

    def spin():
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass

    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
    root = tk.Tk()
    root.protocol('WM_DELETE_WINDOW', root.destroy)
    DynamixelGUI(root, node)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        thread.join(timeout=2.0)
        node.destroy_node()


if __name__ == '__main__':
    main()
