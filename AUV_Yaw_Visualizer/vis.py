import sys
import math
import rospy
# from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget
from PyQt5.QtGui import QPainter, QPen, QFont
from PyQt5.QtCore import Qt

class CompassWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Compass")
        self.setGeometry(100, 100, 400, 400)
        self.centralWidget = CompassWidget()
        self.setCentralWidget(self.centralWidget)

class CompassWidget(QWidget):
    def __init__(self):
        super().__init__()

        self.yaw = 0.0  # Initialize the yaw angle (in radians)

        # Initialize ROS node and subscribe to the orientation topic
        rospy.init_node('compass_gui')
        self.orientation_sub = rospy.Subscriber('/auv3/orientation', Vector3, self.orientationCallback)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Calculate the center of the widget
        center_x = int(self.width() / 2)
        center_y = int(self.height() / 2)

        # Calculate the needle length and position
        needle_length = int(min(self.width(), self.height()) / 3)
        x = center_x + int(needle_length * math.sin(self.yaw))
        y = center_y - int(needle_length * math.cos(self.yaw))

        # Draw compass circle
        painter.setPen(QPen(Qt.black, 2))
        painter.drawEllipse(center_x - needle_length, center_y - needle_length, 2 * needle_length, 2 * needle_length)

        # Draw compass needle
        painter.setPen(QPen(Qt.red, 3))
        painter.drawLine(center_x, center_y, x, y)

        # Draw cardinal directions
        font = QFont()
        font.setPointSize(12)
        painter.setFont(font)
        painter.setPen(QPen(Qt.black, 2))

        # North
        painter.drawText(center_x - 10, center_y - needle_length - 10, "N")
        # East
        painter.drawText(center_x + needle_length + 20, center_y + 5, "E")
        # South
        painter.drawText(center_x - 10, center_y + needle_length + 20, "S")
        # West
        painter.drawText(center_x - needle_length - 30, center_y + 5, "W")

    def orientationCallback(self, msg):
        # Update the yaw angle based on the received z value from ROS
        self.yaw = -msg.z
        print("i am here")
        self.update()

def main():
    app = QApplication(sys.argv)
    compassWindow = CompassWindow()
    compassWindow.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
