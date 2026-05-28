# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'qt_console.ui'
##
## Created by: Qt User Interface Compiler version 6.4.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QAction, QBrush, QColor, QConicalGradient,
    QCursor, QFont, QFontDatabase, QGradient,
    QIcon, QImage, QKeySequence, QLinearGradient,
    QPainter, QPalette, QPixmap, QRadialGradient,
    QTransform)
from PySide6.QtWidgets import (QApplication, QMainWindow, QMenu, QMenuBar,
    QSizePolicy, QStatusBar, QTabWidget, QTextEdit,
    QWidget)

class Ui_QtConsole(object):
    def setupUi(self, QtConsole):
        if not QtConsole.objectName():
            QtConsole.setObjectName(u"QtConsole")
        QtConsole.resize(799, 308)
        self.actionSettings = QAction(QtConsole)
        self.actionSettings.setObjectName(u"actionSettings")
        self.actionShow_Map = QAction(QtConsole)
        self.actionShow_Map.setObjectName(u"actionShow_Map")
        self.actionShow_HUD = QAction(QtConsole)
        self.actionShow_HUD.setObjectName(u"actionShow_HUD")
        self.centralwidget = QWidget(QtConsole)
        self.centralwidget.setObjectName(u"centralwidget")
        self.tabWidget = QTabWidget(self.centralwidget)
        self.tabWidget.setObjectName(u"tabWidget")
        self.tabWidget.setGeometry(QRect(0, 40, 791, 221))
        self.tab = QWidget()
        self.tab.setObjectName(u"tab")
        self.textEdit = QTextEdit(self.tab)
        self.textEdit.setObjectName(u"textEdit")
        self.textEdit.setGeometry(QRect(0, 0, 791, 191))
        self.textEdit.setReadOnly(True)
        self.tabWidget.addTab(self.tab, "")
        self.tab_2 = QWidget()
        self.tab_2.setObjectName(u"tab_2")
        self.tabWidget.addTab(self.tab_2, "")
        QtConsole.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(QtConsole)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 799, 22))
        self.menuFIle = QMenu(self.menubar)
        self.menuFIle.setObjectName(u"menuFIle")
        QtConsole.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(QtConsole)
        self.statusbar.setObjectName(u"statusbar")
        QtConsole.setStatusBar(self.statusbar)

        self.menubar.addAction(self.menuFIle.menuAction())
        self.menuFIle.addAction(self.actionSettings)
        self.menuFIle.addAction(self.actionShow_Map)
        self.menuFIle.addAction(self.actionShow_HUD)

        self.retranslateUi(QtConsole)

        self.tabWidget.setCurrentIndex(0)


        QMetaObject.connectSlotsByName(QtConsole)
    # setupUi

    def retranslateUi(self, QtConsole):
        QtConsole.setWindowTitle(QCoreApplication.translate("QtConsole", u"Qt Console", None))
        self.actionSettings.setText(QCoreApplication.translate("QtConsole", u"Settings", None))
        self.actionShow_Map.setText(QCoreApplication.translate("QtConsole", u"Show Map", None))
        self.actionShow_HUD.setText(QCoreApplication.translate("QtConsole", u"Show HUD", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab), QCoreApplication.translate("QtConsole", u"Messages", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_2), QCoreApplication.translate("QtConsole", u"Tab 2", None))
        self.menuFIle.setTitle(QCoreApplication.translate("QtConsole", u"MAVProxy", None))
    # retranslateUi

