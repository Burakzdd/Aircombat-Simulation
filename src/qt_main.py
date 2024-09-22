import sys
import rospy
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import QThread, pyqtSignal
from controller.enemy_controller import EnemyController
from controller.dorlion_controller import DorlionController
from findDangerPlane.fuzzy_inference_system import FIS
import time
import path_planning.aStar as aStar
import numpy as np
from qui.air_combat_ui import Ui_MainWindow
import cv2
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QImage, QPixmap
from PyQt5 import QtCore
from PyQt5 import QtCore, QtWidgets
from PyQt5.QtCore import QTimer
import time

class AirCombatSimulation(QThread):
    log_signal = pyqtSignal(str)
    enemy_info_signal = pyqtSignal(int, str)

    def __init__(self, uav, enemys):
        super().__init__()
        self.uav = uav
        self.enemys = enemys
        self._is_running = True

    def run(self):
        self.uav.takeoff()
        for enemy in self.enemys:
            enemy.takeoff()
        self.log_signal.emit("--------------------------------\nDüşman uçakları seyir halinde\n--------------------------------")

        start_time = time.time()
        while (time.time() - start_time) < 10:
            for enemy in self.enemys:
                enemy.set_randomly_speed()
                self.uav.set_randomly_speed()

        if not self._is_running:
            return

        self.log_signal.emit("*************************\nEn Tehlikeli Uçak Bulunuyor\n*************************")    

        fis_inference = []
        for i, enemy in enumerate(self.enemys):
            enemy.stop_uav()
            features = enemy.get_features()
            enemy_fis = FIS(features[0], features[1], features[2], features[3])
            inf_ = enemy_fis.inference()
            fis_inference.append(inf_)
            enemy_info = f'-{features[0]}- \nDistance: {features[1]:.2f}\n Velocity: {features[2]:.2f}\n Position: {features[3]:.2f}\n Danger Level: {inf_:.2f}'

            self.enemy_info_signal.emit(i, enemy_info)
        self.log_signal.emit("*************************")
        most_dangerous_index = fis_inference.index(max(fis_inference))
        info_str_1 = "*************************"+f"\nMost Dangerous Plane: {most_dangerous_index+1} Danger Level: {max(fis_inference):.2f}\n"
        self.log_signal.emit(info_str_1)

        if not self._is_running:
            return

        danger_enemy = self.enemys[most_dangerous_index]
        
        for enemy in self.enemys:
            if enemy != danger_enemy:
                enemy.deleteEnemy()
        start_ = (round(self.uav.getPosition().x), round(self.uav.getPosition().y), round(self.uav.getPosition().z))
        end_ = (round(danger_enemy.getPosition().x), round(danger_enemy.getPosition().y), round(danger_enemy.getPosition().z))
        obstacles = np.array([(round(enemy.getPosition().x), round(enemy.getPosition().y), round(enemy.getPosition().z)) for i, enemy in enumerate(self.enemys) if i != most_dangerous_index])
        info_str_1 = info_str_1 + f"***********************\nStart Point: {start_} End Point: {end_}\n****************\n"
        self.log_signal.emit(info_str_1)
        
        path_points = aStar.astar(start_, end_, obstacles, obstacle_radius=0.5)
        print("Start:",start_)
        print("end:",end_)
        print("obstacles:",obstacles)
        print("path",path_points)
        info_str_1 = info_str_1 + f"Path Nodes:\n{path_points}\n"
        self.log_signal.emit(info_str_1)

        self.uav.setDangerUav(danger_enemy)
        self.uav.enemys = self.enemys
        self.uav.move_path(path_points,self.log_signal,info_str_1)

        while self._is_running:
            self.uav.detection()
            self.uav.tracking()
            
            danger_enemy.set_randomly_speed(over=10)
        danger_enemy.stop_uav()
        self.uav.stop_uav()
        
            
    def stop(self):
        self._is_running = False
        self.uav.stop_uav()
        for enemy in self.enemys:
            enemy.stop_uav()

class UAVApp(QMainWindow):
    def __init__(self):
        super().__init__()
        
        rospy.init_node('fuzzyInferenceSystem_node', anonymous=True)

        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.setWindowTitle('UAV Controller')

        self.uav = DorlionController()
        self.uav.image_signal.connect(self.update_image)
        self.ui.buttonStart.clicked.connect(self.start_uav)
        self.ui.buttonStop.clicked.connect(self.stop_uav)
        self.ui.buttonManual.clicked.connect(self.toggle_button)
        self.is_started = False

        self.enemys = []
        self.enemy_names = ["uav1", "uav2", "uav3", "uav4"]
        self.setup_enemies()

    def setup_enemies(self):
        for eName in self.enemy_names:
            enemy = EnemyController(eName)
            
            self.enemys.append(enemy)

    def start_uav(self):
        self.simulation = AirCombatSimulation(self.uav, self.enemys)
        self.simulation.log_signal.connect(self.log)
        self.simulation.enemy_info_signal.connect(self.update_enemy_info)
        self.simulation.start()
    
    def stop_uav(self):
        self.simulation.stop()
        self.simulation.wait()


    def log(self, message):
        self.ui.frame.setText(message)
        QApplication.processEvents()

    def update_enemy_info(self, index, info):
        if index == 0:
            self.ui.enemy1.setText(info)
            self.ui.enemy1.setAlignment(QtCore.Qt.AlignCenter)
        elif index == 1:
            self.ui.enemy2.setText(info)
            self.ui.enemy2.setAlignment(QtCore.Qt.AlignCenter)
        elif index == 2:
            self.ui.enemy3.setText(info)
            self.ui.enemy3.setAlignment(QtCore.Qt.AlignCenter)
        elif index == 3:
            self.ui.enemy4.setText(info)
            self.ui.enemy4.setAlignment(QtCore.Qt.AlignCenter)
        QApplication.processEvents()

    def update_image(self, cv_img):
        qt_img = self.convert_cv_qt(cv_img)
        self.ui.frame.setPixmap(qt_img)

    def convert_cv_qt(self, cv_img):
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        p = convert_to_Qt_format.scaled(640, 480, Qt.KeepAspectRatio)
        return QPixmap.fromImage(p)

    def toggle_button(self):
        if self.is_started:
            self.autoControl()
            self.ui.buttonManual.setText('Manuel')
        else:
            self.show_manual_controls()
            self.ui.buttonManual.setText('Auto')

        self.is_started = not self.is_started

    def show_manual_controls(self):
        self.ui.logo_right_top.hide()
        self.ui.logo_right_bottom.hide()
        self.ui.logo_left_bottom.setVisible(True)
        
        self.manualLayout = QtWidgets.QGridLayout()

        self.ui.buttonLeft = QtWidgets.QPushButton(self.ui.centralwidget)
        self.ui.buttonLeft.setObjectName("buttonLeft")
        self.ui.buttonLeft.setText("Left")
        # self.ui.buttonLeft.setCheckable(True)
        self.manualLayout.addWidget(self.ui.buttonLeft, 1, 0)

        self.ui.buttonRight = QtWidgets.QPushButton(self.ui.centralwidget)
        self.ui.buttonRight.setObjectName("buttonRight")
        self.ui.buttonRight.setText("Right")
        # self.ui.buttonRight.setCheckable(True)
        self.manualLayout.addWidget(self.ui.buttonRight, 1, 2)

        self.ui.buttonUp = QtWidgets.QPushButton(self.ui.centralwidget)
        self.ui.buttonUp.setObjectName("buttonUp")
        self.ui.buttonUp.setText("Up")
        # self.ui.buttonUp.setCheckable(True)
        self.manualLayout.addWidget(self.ui.buttonUp, 0, 1)

        self.ui.buttonDown = QtWidgets.QPushButton(self.ui.centralwidget)
        self.ui.buttonDown.setObjectName("buttonDown")
        self.ui.buttonDown.setText("Down")
        # self.ui.buttonDown.setCheckable(True)
        self.manualLayout.addWidget(self.ui.buttonDown, 2, 1)

        self.ui.buttonStopManual = QtWidgets.QPushButton(self.ui.centralwidget)
        self.ui.buttonStopManual.setObjectName("buttonStopManual")
        self.ui.buttonStopManual.setText("Hover")
        # self.ui.buttonStopManual.setCheckable(True)
        self.manualLayout.addWidget(self.ui.buttonStopManual, 1, 1)

        self.ui.gridLayout.addLayout(self.manualLayout, 1, 3)

        self.ui.buttonLeft.pressed.connect(lambda: self.start_timer(self.left_control))
        self.ui.buttonLeft.released.connect(self.stop_timer)
        self.ui.buttonRight.pressed.connect(lambda: self.start_timer(self.right_control))
        self.ui.buttonRight.released.connect(self.stop_timer)
        self.ui.buttonUp.pressed.connect(lambda: self.start_timer(self.up_control))
        self.ui.buttonUp.released.connect(self.stop_timer)
        self.ui.buttonDown.pressed.connect(lambda: self.start_timer(self.down_control))
        self.ui.buttonDown.released.connect(self.stop_timer)
        self.ui.buttonStopManual.clicked.connect(self.stop_manual_control)

        self.timer = QTimer()
        self.timer.timeout.connect(self.timer_action)

        self.current_action = None
        
    def autoControl(self):
        if self.manualLayout:
            while self.manualLayout.count():
                item = self.manualLayout.takeAt(0)
                widget = item.widget()
                if widget is not None:
                    widget.deleteLater()
            self.manualLayout = None
        self.ui.logo_left_bottom.hide()
        
        self.ui.logo_right_top.setVisible(True)

    def start_timer(self, action):
        self.current_action = action
        self.timer.start(100)

    def stop_timer(self):
        self.timer.stop()
        self.stop_manual_control()
        
    def timer_action(self):
        if self.current_action:
            self.current_action()


    def left_control(self):
        self.uav.getDangerUav().setVelocity(0,0,0,0,0,0.5)

    def right_control(self):
        self.uav.getDangerUav().setVelocity(0,0,0,0,0,-0.5)

    def up_control(self):
        self.uav.getDangerUav().setVelocity(0,0,0.5,0,0,0)

    def down_control(self):
        self.uav.getDangerUav().setVelocity(0,0,-0.5,0,0,0)

    def stop_manual_control(self):
        self.uav.getDangerUav().setVelocity(0,0,0,0,0,0)
        
if __name__ == '__main__':
    import os
    os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = '/home/burakzdd/.local/lib/python3.8/site-packages/cv2/qt/plugins'

    app = QApplication(sys.argv)
    ex = UAVApp()
    ex.show()
    sys.exit(app.exec_())
