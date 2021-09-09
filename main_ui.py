# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'olfc2.ui'
#
# Created by: PyQt5 UI code generator 5.15.4
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(610, 758)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(1)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(MainWindow.sizePolicy().hasHeightForWidth())
        MainWindow.setSizePolicy(sizePolicy)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout_5 = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.tabWidget = QtWidgets.QTabWidget(self.centralwidget)
        self.tabWidget.setObjectName("tabWidget")
        self.tab_setup = QtWidgets.QWidget()
        self.tab_setup.setObjectName("tab_setup")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.tab_setup)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.groupBox_2 = QtWidgets.QGroupBox(self.tab_setup)
        self.groupBox_2.setObjectName("groupBox_2")
        self.gridLayout_10 = QtWidgets.QGridLayout(self.groupBox_2)
        self.gridLayout_10.setObjectName("gridLayout_10")
        self.tabWidget_2 = QtWidgets.QTabWidget(self.groupBox_2)
        self.tabWidget_2.setObjectName("tabWidget_2")
        self.tab = QtWidgets.QWidget()
        self.tab.setObjectName("tab")
        self.gridLayout_11 = QtWidgets.QGridLayout(self.tab)
        self.gridLayout_11.setObjectName("gridLayout_11")
        self.label_inspect_hardware = QtWidgets.QLabel(self.tab)
        self.label_inspect_hardware.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_inspect_hardware.setObjectName("label_inspect_hardware")
        self.gridLayout_11.addWidget(self.label_inspect_hardware, 0, 0, 1, 1)
        self.treeview_inspect_hardware = QtWidgets.QTreeView(self.tab)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.treeview_inspect_hardware.sizePolicy().hasHeightForWidth())
        self.treeview_inspect_hardware.setSizePolicy(sizePolicy)
        self.treeview_inspect_hardware.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.treeview_inspect_hardware.setObjectName("treeview_inspect_hardware")
        self.gridLayout_11.addWidget(self.treeview_inspect_hardware, 1, 0, 1, 1)
        self.tabWidget_2.addTab(self.tab, "")
        self.tab_2 = QtWidgets.QWidget()
        self.tab_2.setObjectName("tab_2")
        self.gridLayout_12 = QtWidgets.QGridLayout(self.tab_2)
        self.gridLayout_12.setObjectName("gridLayout_12")
        self.label_inspect_experiment = QtWidgets.QLabel(self.tab_2)
        self.label_inspect_experiment.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_inspect_experiment.setObjectName("label_inspect_experiment")
        self.gridLayout_12.addWidget(self.label_inspect_experiment, 0, 0, 1, 1)
        self.treeview_inspect_experiment = QtWidgets.QTreeView(self.tab_2)
        self.treeview_inspect_experiment.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.treeview_inspect_experiment.setObjectName("treeview_inspect_experiment")
        self.gridLayout_12.addWidget(self.treeview_inspect_experiment, 1, 0, 1, 1)
        self.tabWidget_2.addTab(self.tab_2, "")
        self.gridLayout_10.addWidget(self.tabWidget_2, 0, 0, 1, 1)
        self.gridLayout_2.addWidget(self.groupBox_2, 1, 0, 1, 1)
        self.groupBox = QtWidgets.QGroupBox(self.tab_setup)
        self.groupBox.setObjectName("groupBox")
        self.gridLayout_9 = QtWidgets.QGridLayout(self.groupBox)
        self.gridLayout_9.setObjectName("gridLayout_9")
        self.label = QtWidgets.QLabel(self.groupBox)
        self.label.setObjectName("label")
        self.gridLayout_9.addWidget(self.label, 0, 0, 1, 1)
        spacerItem = QtWidgets.QSpacerItem(168, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout_9.addItem(spacerItem, 0, 1, 1, 1)
        self.combo_hardware_config = QtWidgets.QComboBox(self.groupBox)
        self.combo_hardware_config.setObjectName("combo_hardware_config")
        self.gridLayout_9.addWidget(self.combo_hardware_config, 0, 2, 1, 1)
        self.label_6 = QtWidgets.QLabel(self.groupBox)
        self.label_6.setObjectName("label_6")
        self.gridLayout_9.addWidget(self.label_6, 1, 0, 1, 1)
        spacerItem1 = QtWidgets.QSpacerItem(168, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout_9.addItem(spacerItem1, 1, 1, 1, 1)
        self.line_experiment_config = QtWidgets.QLineEdit(self.groupBox)
        self.line_experiment_config.setReadOnly(True)
        self.line_experiment_config.setObjectName("line_experiment_config")
        self.gridLayout_9.addWidget(self.line_experiment_config, 1, 2, 1, 1)
        self.button_experiment_config = QtWidgets.QPushButton(self.groupBox)
        self.button_experiment_config.setObjectName("button_experiment_config")
        self.gridLayout_9.addWidget(self.button_experiment_config, 1, 3, 1, 1)
        self.button_configure = QtWidgets.QPushButton(self.groupBox)
        self.button_configure.setObjectName("button_configure")
        self.gridLayout_9.addWidget(self.button_configure, 2, 3, 1, 1)
        self.gridLayout_2.addWidget(self.groupBox, 0, 0, 1, 1)
        self.tabWidget.addTab(self.tab_setup, "")
        self.tab_run = QtWidgets.QWidget()
        self.tab_run.setObjectName("tab_run")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.tab_run)
        self.verticalLayout.setObjectName("verticalLayout")
        self.groupBox_5 = QtWidgets.QGroupBox(self.tab_run)
        self.groupBox_5.setObjectName("groupBox_5")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.groupBox_5)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.listview_plan = QtWidgets.QListView(self.groupBox_5)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(1)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.listview_plan.sizePolicy().hasHeightForWidth())
        self.listview_plan.setSizePolicy(sizePolicy)
        self.listview_plan.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.listview_plan.setObjectName("listview_plan")
        self.horizontalLayout_2.addWidget(self.listview_plan)
        self.gridLayout_4 = QtWidgets.QGridLayout()
        self.gridLayout_4.setObjectName("gridLayout_4")
        self.combo_plan_select = QtWidgets.QComboBox(self.groupBox_5)
        self.combo_plan_select.setEnabled(False)
        self.combo_plan_select.setObjectName("combo_plan_select")
        self.gridLayout_4.addWidget(self.combo_plan_select, 0, 0, 1, 2)
        spacerItem2 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout_4.addItem(spacerItem2, 1, 0, 1, 1)
        self.button_plan_reload = QtWidgets.QPushButton(self.groupBox_5)
        self.button_plan_reload.setEnabled(False)
        self.button_plan_reload.setObjectName("button_plan_reload")
        self.gridLayout_4.addWidget(self.button_plan_reload, 1, 1, 1, 1)
        spacerItem3 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.gridLayout_4.addItem(spacerItem3, 2, 1, 1, 1)
        self.horizontalLayout_2.addLayout(self.gridLayout_4)
        self.verticalLayout.addWidget(self.groupBox_5)
        self.groupBox_4 = QtWidgets.QGroupBox(self.tab_run)
        self.groupBox_4.setObjectName("groupBox_4")
        self.gridLayout_3 = QtWidgets.QGridLayout(self.groupBox_4)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.label_4 = QtWidgets.QLabel(self.groupBox_4)
        self.label_4.setObjectName("label_4")
        self.gridLayout_3.addWidget(self.label_4, 0, 0, 1, 1)
        spacerItem4 = QtWidgets.QSpacerItem(98, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout_3.addItem(spacerItem4, 0, 1, 1, 1)
        self.button_flipper = QtWidgets.QPushButton(self.groupBox_4)
        self.button_flipper.setEnabled(False)
        self.button_flipper.setObjectName("button_flipper")
        self.gridLayout_3.addWidget(self.button_flipper, 0, 3, 1, 1)
        self.checkbox_flipper = QtWidgets.QCheckBox(self.groupBox_4)
        self.checkbox_flipper.setEnabled(False)
        self.checkbox_flipper.setObjectName("checkbox_flipper")
        self.gridLayout_3.addWidget(self.checkbox_flipper, 0, 4, 1, 1)
        self.label_5 = QtWidgets.QLabel(self.groupBox_4)
        self.label_5.setObjectName("label_5")
        self.gridLayout_3.addWidget(self.label_5, 1, 0, 1, 1)
        spacerItem5 = QtWidgets.QSpacerItem(165, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout_3.addItem(spacerItem5, 1, 1, 1, 1)
        self.combo_odortest = QtWidgets.QComboBox(self.groupBox_4)
        self.combo_odortest.setEnabled(False)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.combo_odortest.sizePolicy().hasHeightForWidth())
        self.combo_odortest.setSizePolicy(sizePolicy)
        self.combo_odortest.setObjectName("combo_odortest")
        self.combo_odortest.addItem("")
        self.gridLayout_3.addWidget(self.combo_odortest, 1, 2, 1, 1)
        self.button_odortest = QtWidgets.QPushButton(self.groupBox_4)
        self.button_odortest.setEnabled(False)
        self.button_odortest.setFlat(False)
        self.button_odortest.setObjectName("button_odortest")
        self.gridLayout_3.addWidget(self.button_odortest, 1, 3, 1, 1)
        self.checkbox_odortest = QtWidgets.QCheckBox(self.groupBox_4)
        self.checkbox_odortest.setEnabled(False)
        self.checkbox_odortest.setObjectName("checkbox_odortest")
        self.gridLayout_3.addWidget(self.checkbox_odortest, 1, 4, 1, 1)
        self.button_start = QtWidgets.QPushButton(self.groupBox_4)
        self.button_start.setEnabled(False)
        self.button_start.setObjectName("button_start")
        self.gridLayout_3.addWidget(self.button_start, 2, 4, 1, 1)
        self.button_reset_controller = QtWidgets.QPushButton(self.groupBox_4)
        self.button_reset_controller.setEnabled(False)
        self.button_reset_controller.setObjectName("button_reset_controller")
        self.gridLayout_3.addWidget(self.button_reset_controller, 2, 3, 1, 1)
        self.verticalLayout.addWidget(self.groupBox_4)
        self.groupBox_3 = QtWidgets.QGroupBox(self.tab_run)
        self.groupBox_3.setObjectName("groupBox_3")
        self.gridLayout = QtWidgets.QGridLayout(self.groupBox_3)
        self.gridLayout.setObjectName("gridLayout")
        self.label_3 = QtWidgets.QLabel(self.groupBox_3)
        self.label_3.setObjectName("label_3")
        self.gridLayout.addWidget(self.label_3, 0, 0, 1, 1)
        self.pbar_step = QtWidgets.QProgressBar(self.groupBox_3)
        self.pbar_step.setProperty("value", 0)
        self.pbar_step.setObjectName("pbar_step")
        self.gridLayout.addWidget(self.pbar_step, 0, 1, 1, 1)
        self.label_time_step = QtWidgets.QLabel(self.groupBox_3)
        self.label_time_step.setObjectName("label_time_step")
        self.gridLayout.addWidget(self.label_time_step, 0, 2, 1, 1)
        self.label_2 = QtWidgets.QLabel(self.groupBox_3)
        self.label_2.setObjectName("label_2")
        self.gridLayout.addWidget(self.label_2, 1, 0, 1, 1)
        self.pbar_total = QtWidgets.QProgressBar(self.groupBox_3)
        self.pbar_total.setProperty("value", 0)
        self.pbar_total.setObjectName("pbar_total")
        self.gridLayout.addWidget(self.pbar_total, 1, 1, 1, 1)
        self.label_time_total = QtWidgets.QLabel(self.groupBox_3)
        self.label_time_total.setObjectName("label_time_total")
        self.gridLayout.addWidget(self.label_time_total, 1, 2, 1, 1)
        self.verticalLayout.addWidget(self.groupBox_3)
        self.tabWidget.addTab(self.tab_run, "")
        self.tab_review = QtWidgets.QWidget()
        self.tab_review.setObjectName("tab_review")
        self.gridLayout_5 = QtWidgets.QGridLayout(self.tab_review)
        self.gridLayout_5.setObjectName("gridLayout_5")
        self.gridLayout_8 = QtWidgets.QGridLayout()
        self.gridLayout_8.setObjectName("gridLayout_8")
        self.groupBox_6 = QtWidgets.QGroupBox(self.tab_review)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.groupBox_6.sizePolicy().hasHeightForWidth())
        self.groupBox_6.setSizePolicy(sizePolicy)
        self.groupBox_6.setObjectName("groupBox_6")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.groupBox_6)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.button_load_review = QtWidgets.QPushButton(self.groupBox_6)
        self.button_load_review.setObjectName("button_load_review")
        self.verticalLayout_2.addWidget(self.button_load_review)
        self.button_save_review = QtWidgets.QPushButton(self.groupBox_6)
        self.button_save_review.setObjectName("button_save_review")
        self.verticalLayout_2.addWidget(self.button_save_review)
        self.gridLayout_8.addWidget(self.groupBox_6, 1, 1, 1, 1)
        self.listview_review_select = QtWidgets.QListView(self.tab_review)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(2)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.listview_review_select.sizePolicy().hasHeightForWidth())
        self.listview_review_select.setSizePolicy(sizePolicy)
        self.listview_review_select.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.listview_review_select.setObjectName("listview_review_select")
        self.gridLayout_8.addWidget(self.listview_review_select, 0, 1, 1, 1)
        self.toolBox = QtWidgets.QToolBox(self.tab_review)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(5)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.toolBox.sizePolicy().hasHeightForWidth())
        self.toolBox.setSizePolicy(sizePolicy)
        self.toolBox.setObjectName("toolBox")
        self.page = QtWidgets.QWidget()
        self.page.setGeometry(QtCore.QRect(0, 0, 377, 530))
        self.page.setObjectName("page")
        self.gridLayout_7 = QtWidgets.QGridLayout(self.page)
        self.gridLayout_7.setObjectName("gridLayout_7")
        self.treeview_exitstatus = QtWidgets.QTreeView(self.page)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(2)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.treeview_exitstatus.sizePolicy().hasHeightForWidth())
        self.treeview_exitstatus.setSizePolicy(sizePolicy)
        self.treeview_exitstatus.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.treeview_exitstatus.setObjectName("treeview_exitstatus")
        self.gridLayout_7.addWidget(self.treeview_exitstatus, 0, 0, 1, 1)
        self.toolBox.addItem(self.page, "")
        self.page_3 = QtWidgets.QWidget()
        self.page_3.setGeometry(QtCore.QRect(0, 0, 122, 139))
        self.page_3.setObjectName("page_3")
        self.verticalLayout_6 = QtWidgets.QVBoxLayout(self.page_3)
        self.verticalLayout_6.setObjectName("verticalLayout_6")
        self.textedit_extras = QtWidgets.QPlainTextEdit(self.page_3)
        self.textedit_extras.setObjectName("textedit_extras")
        self.verticalLayout_6.addWidget(self.textedit_extras)
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        spacerItem6 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_4.addItem(spacerItem6)
        self.combo_extras_format = QtWidgets.QComboBox(self.page_3)
        self.combo_extras_format.setObjectName("combo_extras_format")
        self.combo_extras_format.addItem("")
        self.combo_extras_format.addItem("")
        self.horizontalLayout_4.addWidget(self.combo_extras_format)
        self.verticalLayout_6.addLayout(self.horizontalLayout_4)
        self.toolBox.addItem(self.page_3, "")
        self.page_2 = QtWidgets.QWidget()
        self.page_2.setGeometry(QtCore.QRect(0, 0, 103, 103))
        self.page_2.setObjectName("page_2")
        self.gridLayout_6 = QtWidgets.QGridLayout(self.page_2)
        self.gridLayout_6.setObjectName("gridLayout_6")
        self.textedit_notes = QtWidgets.QPlainTextEdit(self.page_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(1)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.textedit_notes.sizePolicy().hasHeightForWidth())
        self.textedit_notes.setSizePolicy(sizePolicy)
        self.textedit_notes.setObjectName("textedit_notes")
        self.gridLayout_6.addWidget(self.textedit_notes, 0, 0, 1, 1)
        self.toolBox.addItem(self.page_2, "")
        self.gridLayout_8.addWidget(self.toolBox, 0, 0, 2, 1)
        self.gridLayout_5.addLayout(self.gridLayout_8, 0, 0, 1, 1)
        self.tabWidget.addTab(self.tab_review, "")
        self.verticalLayout_5.addWidget(self.tabWidget)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 610, 24))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.actionOpen = QtWidgets.QAction(MainWindow)
        self.actionOpen.setObjectName("actionOpen")

        self.retranslateUi(MainWindow)
        self.tabWidget.setCurrentIndex(0)
        self.tabWidget_2.setCurrentIndex(1)
        self.toolBox.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Olfactometer"))
        self.groupBox_2.setTitle(_translate("MainWindow", "Inspect"))
        self.label_inspect_hardware.setText(_translate("MainWindow", "(not configured)"))
        self.tabWidget_2.setTabText(self.tabWidget_2.indexOf(self.tab), _translate("MainWindow", "Hardware"))
        self.label_inspect_experiment.setText(_translate("MainWindow", "(not configured)"))
        self.tabWidget_2.setTabText(self.tabWidget_2.indexOf(self.tab_2), _translate("MainWindow", "Experiment"))
        self.groupBox.setTitle(_translate("MainWindow", "Choose"))
        self.label.setText(_translate("MainWindow", "Hardware"))
        self.label_6.setText(_translate("MainWindow", "Experiment"))
        self.button_experiment_config.setText(_translate("MainWindow", "Choose..."))
        self.button_configure.setText(_translate("MainWindow", "Configure"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_setup), _translate("MainWindow", "Setup"))
        self.groupBox_5.setTitle(_translate("MainWindow", "Experiment Plan"))
        self.button_plan_reload.setText(_translate("MainWindow", "Reload"))
        self.groupBox_4.setTitle(_translate("MainWindow", "Controls"))
        self.label_4.setText(_translate("MainWindow", "Flipper Mirror"))
        self.button_flipper.setText(_translate("MainWindow", "Toggle"))
        self.checkbox_flipper.setText(_translate("MainWindow", "Blocking"))
        self.label_5.setText(_translate("MainWindow", "Odor Test"))
        self.combo_odortest.setItemText(0, _translate("MainWindow", "Choose an odor..."))
        self.button_odortest.setText(_translate("MainWindow", "Deliver"))
        self.checkbox_odortest.setText(_translate("MainWindow", "Enable"))
        self.button_start.setText(_translate("MainWindow", "Start"))
        self.button_reset_controller.setText(_translate("MainWindow", "Reset"))
        self.groupBox_3.setTitle(_translate("MainWindow", "Timing"))
        self.label_3.setText(_translate("MainWindow", "Step"))
        self.label_time_step.setText(_translate("MainWindow", "--:--"))
        self.label_2.setText(_translate("MainWindow", "Total"))
        self.label_time_total.setText(_translate("MainWindow", "--:--"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_run), _translate("MainWindow", "Run"))
        self.groupBox_6.setTitle(_translate("MainWindow", "Import/Export"))
        self.button_load_review.setText(_translate("MainWindow", "Load..."))
        self.button_save_review.setText(_translate("MainWindow", "Save..."))
        self.toolBox.setItemText(self.toolBox.indexOf(self.page), _translate("MainWindow", "Status"))
        self.combo_extras_format.setItemText(0, _translate("MainWindow", "YAML"))
        self.combo_extras_format.setItemText(1, _translate("MainWindow", "JSON"))
        self.toolBox.setItemText(self.toolBox.indexOf(self.page_3), _translate("MainWindow", "Extras"))
        self.toolBox.setItemText(self.toolBox.indexOf(self.page_2), _translate("MainWindow", "Notes"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_review), _translate("MainWindow", "Review"))
        self.actionOpen.setText(_translate("MainWindow", "Open"))