from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtGui import QPixmap

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(863, 609)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayout = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout.setObjectName("gridLayout")

        self.leftLayout = QtWidgets.QVBoxLayout()
        self.leftLayout.setObjectName("leftLayout")
        self.logo_left_top = QtWidgets.QLabel(self.centralwidget)
        self.logo_left_top.setObjectName("logo_left_top")
        self.logo_left_top.setScaledContents(True)
        pixmap_left_top = QPixmap("/home/burakzdd/catkin_ws/src/air_combat_simulation/src/qui/esogu.png")
        self.logo_left_top.setPixmap(pixmap_left_top)
        self.logo_left_top.setFixedSize(100, 100)
        self.leftLayout.addWidget(self.logo_left_top, alignment=QtCore.Qt.AlignCenter)
        self.logo_left_bottom = QtWidgets.QLabel(self.centralwidget)
        self.logo_left_bottom.setObjectName("logo_left_bottom")
        self.logo_left_bottom.setScaledContents(True)
        pixmap_left_bottom = QPixmap("/home/burakzdd/catkin_ws/src/air_combat_simulation/src/qui/tusas-eng.png")

        self.logo_left_bottom.setPixmap(pixmap_left_bottom)
        self.logo_left_bottom.setFixedSize(100, 100)
        self.leftLayout.addWidget(self.logo_left_bottom, alignment=QtCore.Qt.AlignCenter)
        self.logo_left_bottom.hide()
        self.gridLayout.addLayout(self.leftLayout, 1, 0)

        self.frame = QtWidgets.QLabel(self.centralwidget)
        self.frame.setObjectName("frame")
        self.frame.setFixedSize(600, 400)
        self.gridLayout.addWidget(self.frame, 1, 1, 1, 2, alignment=QtCore.Qt.AlignCenter)

        self.rightLayout = QtWidgets.QVBoxLayout()
        self.rightLayout.setObjectName("rightLayout")
        self.logo_right_top = QtWidgets.QLabel(self.centralwidget)
        self.logo_right_top.setObjectName("logo_right_top")
        self.logo_right_top.setScaledContents(True)
        pixmap_right_top = QPixmap("/home/burakzdd/catkin_ws/src/air_combat_simulation/src/qui/tusas-eng.png")
        self.logo_right_top.setPixmap(pixmap_right_top)
        self.logo_right_top.setFixedSize(100, 100)
        self.rightLayout.addWidget(self.logo_right_top, alignment=QtCore.Qt.AlignCenter)
        self.logo_right_bottom = QtWidgets.QLabel(self.centralwidget)
        self.logo_right_bottom.setObjectName("logo_right_bottom")
        self.logo_right_bottom.setScaledContents(True)
        pixmap_right_bottom = QPixmap("")
        self.logo_right_bottom.setPixmap(pixmap_right_bottom)
        self.logo_right_bottom.setFixedSize(100, 100)
        self.rightLayout.addWidget(self.logo_right_bottom, alignment=QtCore.Qt.AlignCenter)
        self.gridLayout.addLayout(self.rightLayout, 1, 3)

        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.enemy1 = QtWidgets.QLabel(self.centralwidget)
        self.enemy1.setObjectName("enemy1")
        self.enemy1.setFixedHeight(150)
        self.horizontalLayout.addWidget(self.enemy1)
        self.enemy2 = QtWidgets.QLabel(self.centralwidget)
        self.enemy2.setObjectName("enemy2")
        self.enemy2.setFixedHeight(150)
        self.horizontalLayout.addWidget(self.enemy2)
        self.enemy3 = QtWidgets.QLabel(self.centralwidget)
        self.enemy3.setObjectName("enemy3")
        self.enemy3.setFixedHeight(150)
        self.horizontalLayout.addWidget(self.enemy3)
        self.enemy4 = QtWidgets.QLabel(self.centralwidget)
        self.enemy4.setObjectName("enemy4")
        self.enemy4.setFixedHeight(150)
        self.horizontalLayout.addWidget(self.enemy4)
        self.gridLayout.addLayout(self.horizontalLayout, 3, 0, 1, 4)

        self.buttonLayout = QtWidgets.QHBoxLayout()
        self.buttonLayout.setObjectName("buttonLayout")
        self.buttonStart = QtWidgets.QPushButton(self.centralwidget)
        self.buttonStart.setObjectName("buttonStart")
        self.buttonLayout.addWidget(self.buttonStart)
        self.buttonStop = QtWidgets.QPushButton(self.centralwidget)
        self.buttonStop.setObjectName("buttonStop")
        self.buttonLayout.addWidget(self.buttonStop)
        self.buttonManual = QtWidgets.QPushButton(self.centralwidget)
        self.buttonManual.setObjectName("buttonManual")
        self.buttonLayout.addWidget(self.buttonManual)
        self.gridLayout.addLayout(self.buttonLayout, 0, 1, 1, 2, alignment=QtCore.Qt.AlignCenter)

        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 863, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

        self.applyStyles()

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Air Combat Simulation"))
        self.frame.setText(_translate("MainWindow", "Simulation Frame"))
        self.frame.setAlignment(QtCore.Qt.AlignCenter)
        self.enemy1.setText(_translate("MainWindow", "ENEMY 1"))
        self.enemy2.setText(_translate("MainWindow", "ENEMY 2"))
        self.enemy3.setText(_translate("MainWindow", "ENEMY 3"))
        self.enemy4.setText(_translate("MainWindow", "ENEMY 4"))
        self.buttonStart.setText(_translate("MainWindow", "Start"))
        self.buttonStop.setText(_translate("MainWindow", "Stop"))
        self.buttonManual.setText(_translate("MainWindow", "Manual"))

    def applyStyles(self):
        self.centralwidget.setStyleSheet("background-color: #2E3440;")
        self.frame.setStyleSheet("background-color: #4C566A; color: white; font-size: 18px; border-radius: 10px; padding: 10px;")
        self.enemy1.setStyleSheet("color: white; font-size: 14px; border: 1px solid #4C566A; padding: 5px;")
        self.enemy2.setStyleSheet("color: white; font-size: 14px; border: 1px solid #4C566A; padding: 5px;")
        self.enemy3.setStyleSheet("color: white; font-size: 14px; border: 1px solid #4C566A; padding: 5px;")
        self.enemy4.setStyleSheet("color: white; font-size: 14px; border: 1px solid #4C566A; padding: 5px;")
        self.buttonStart.setStyleSheet("""
            QPushButton {
                background-color: #5E81AC;
                color: white;
                font-size: 14px;
                border-radius: 10px;
                padding: 10px;
            }
            QPushButton:hover {
                background-color: #81A1C1;
            }
        """)
        self.buttonStop.setStyleSheet("""
            QPushButton {
                background-color: #BF616A;
                color: white;
                font-size: 14px;
                border-radius: 10px;
                padding: 10px;
            }
            QPushButton:hover {
                background-color: #D08770;
            }
        """)
        self.buttonManual.setStyleSheet("""
            QPushButton {
                background-color: #A3BE8C;
                color: white;
                font-size: 14px;
                border-radius: 10px;
                padding: 10px;
            }
            QPushButton:hover {
                background-color: #8FBC8B;
            }
        """)