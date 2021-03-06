#! /usr/bin/python3
# -*- coding: utf-8 -*-

import sys

import rclpy
from rclpy.node import Node

from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtWidgets import QApplication, QDialog

from shipsim_msgs_module.msg import KTControl

from shipsim_kt_module.kt_controller_ui import Ui_Dialog


class KtControllerNode(Node):
    """ControllerNode."""

    def __init__(self):
        """init."""
        super().__init__("controller", namespace="ship1")

        self.declare_parameter("publish_address", "/ship1/control_input")
        publish_address = (
            self.get_parameter("publish_address").get_parameter_value().string_value
        )
        self.publisher = self.create_publisher(KTControl, publish_address, 10)


class ControllerNodeWorker(QThread):
    """ControllerNodeWorker."""

    signal = pyqtSignal()

    def __init__(self):
        """init."""
        super(ControllerNodeWorker, self).__init__()
        self.control_msg = KTControl()
        self.rate = 1.0

    def set_control_msg(self, msg):
        """set_control_msg."""
        self.control_msg = msg

    def run(self):
        """run."""
        rclpy.init()
        self.node = KtControllerNode()
        self.node.create_rate(self.rate)
        while rclpy.ok():
            rclpy.spin_once(self.node)

            self.node.publisher.publish(self.control_msg)

            self.node.get_logger().info(
                'Publishing: "%s", "%s"'
                % (self.control_msg.u, self.control_msg.rudder_angle_degree)
            )

            # self.control_msg = KTControl()


class ControllerUi(QDialog):
    """ControllerUI."""

    worker_thread = ControllerNodeWorker()

    def __init__(self, parent=None):
        """init."""
        super(ControllerUi, self).__init__(parent)
        self.ui = Ui_Dialog()
        self.ui.setupUi(self)
        self.sampling_freq = float(self.ui.samplingFreqEdit.text())

    def clicked_start(self):
        """clicked_forward button."""
        control_msg = self.worker_thread.control_msg
        control_msg.u = float(self.ui.UEdit.text())
        self.worker_thread.rate = float(self.ui.samplingFreqEdit.text())
        self.ui.samplingFreqEdit.setEnabled(False)
        control_msg.rudder_angle_degree = float(self.ui.rudderDial.value())
        self.worker_thread.start()

    def change_rudder_angle(self):
        """change_rudder_angle button."""
        rudder_angle_degree = self.ui.rudderDial.value()
        self.ui.rudderDialValueLabel.setText(str(rudder_angle_degree))
        control_msg = self.worker_thread.control_msg
        control_msg.rudder_angle_degree = float(rudder_angle_degree)

    def clicked_stop(self):
        """clicked_stop button."""
        self.worker_thread.control_msg = KTControl()


def main(args=None):
    """Run main."""
    app = QApplication(sys.argv)
    window = ControllerUi()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
