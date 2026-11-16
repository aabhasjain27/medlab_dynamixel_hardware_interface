#!/usr/bin/env python3
"""Generic Dynamixel torque and zero-reference GUI (motor positions in radians)."""

import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import JointState

from medlab_dynamixel_hardware_interface.srv import SetMotorTorqueEnable, SetMotorZero


class DynamixelGuiNode(Node):
    def __init__(self):
        super().__init__('dynamixel_gui_node', automatically_declare_parameters_from_overrides=True)

        def parameter(name, default):
            if not self.has_parameter(name):
                self.declare_parameter(name, default)
            return self.get_parameter(name).value

        self.motor_names = list(parameter('motor_names', []))
        self.feedback_topic = parameter('feedback_topic', '/motor_feedback')
        self.torque_service = parameter('torque_service', '/dynamixels/set_motor_torque_enable')
        self.zero_service = parameter('zero_service', '/dynamixels/set_motor_zero')

        self.positions = {}
        self.position_lock = threading.Lock()
        self.feedback_sub = self.create_subscription(JointState, self.feedback_topic, self.feedbackCallback, 10)
        self.torque_client = self.create_client(SetMotorTorqueEnable, self.torque_service)
        self.zero_client = self.create_client(SetMotorZero, self.zero_service)
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


class DynamixelGUI:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.root.title('Dynamixel Motor Control')
        self.root.geometry('800x500')
        self.root.minsize(680, 300)
        self.position_labels = {}
        self.pending = []
        self.status = tk.StringVar(value='Ready')
        self.createWidgets()
        self.updateGUI()

    def createWidgets(self):
        main = ttk.Frame(self.root, padding=15)
        main.pack(fill=tk.BOTH, expand=True)
        ttk.Label(main, text='Dynamixel Motor Control', font=('Helvetica', 14, 'bold')).pack(pady=(0, 12))

        table = ttk.LabelFrame(main, text='Motor Controls', padding=10)
        table.pack(fill=tk.BOTH, expand=True)
        headings = ('Motor', 'Position (rad)', 'Torque On', 'Torque Off', 'Reference')
        for col, heading in enumerate(headings):
            ttk.Label(table, text=heading, font=('Helvetica', 10, 'bold')).grid(row=0, column=col, padx=6, pady=6)

        for i, name in enumerate(self.node.motor_names, start=1):
            ttk.Label(table, text=name).grid(row=i, column=0, sticky='w', padx=6, pady=6)
            position_label = ttk.Label(table, text='--', width=15, anchor='center')
            position_label.grid(row=i, column=1, padx=6)
            self.position_labels[name] = position_label
            ttk.Button(table, text='Enable', command=lambda n=name: self.setTorque([n], True)).grid(row=i, column=2, padx=3)
            ttk.Button(table, text='Disable', command=lambda n=name: self.setTorque([n], False)).grid(row=i, column=3, padx=3)
            ttk.Button(table, text='Set Zero', command=lambda n=name: self.setZero(n)).grid(row=i, column=4, padx=3)

        if not self.node.motor_names:
            ttk.Label(table, text='No motor_names configured.').grid(row=1, column=0, columnspan=5, pady=15)

        actions = ttk.Frame(main)
        actions.pack(fill=tk.X, pady=(12, 8))
        ttk.Button(actions, text='Enable All', command=lambda: self.setTorque(self.node.motor_names, True)).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=4)
        ttk.Button(actions, text='Disable All', command=lambda: self.setTorque(self.node.motor_names, False)).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=4)
        ttk.Label(main, textvariable=self.status, wraplength=750).pack(fill=tk.X)

    def setTorque(self, names, enable):
        if not names:
            self.status.set('No motors configured')
            return
        if not self.node.torque_client.service_is_ready():
            self.status.set('Torque service unavailable')
            return
        self.pending.append(self.node.setTorque(names, enable))
        self.status.set('Sending torque request...')

    def setZero(self, name):
        if not self.node.zero_client.service_is_ready():
            self.status.set('Set Zero service unavailable')
            return
        self.pending.append(self.node.setZero(name))
        self.status.set(f'Setting zero for {name}...')

    def updateGUI(self):
        positions = self.node.getPositions()
        for name, label in self.position_labels.items():
            if name in positions:
                label.config(text=f'{positions[name]:.3f}')

        for future in self.pending[:]:
            if future.done():
                try:
                    result = future.result()
                    self.status.set(result.message if result is not None else 'Service returned no result')
                except Exception as exc:
                    self.status.set(f'Service failed: {exc}')
                self.pending.remove(future)
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
