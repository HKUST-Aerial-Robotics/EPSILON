#! /usr/bin/python2.7

# @file terminal_server.py
# @author HKUST Aerial Robotics Group
# @brief terminal server for testing
# @version 0.1
# @date 2019-02
# @copyright Copyright (c) 2019

# sys
import time
import sys
import rospy
import shutil
import random
import math
import numpy as np

# pyqtgraph
from PyQt4 import QtGui, QtCore
from PyQt4.QtGui import QFont
from PyQt4.QtGui import QSizePolicy
from PyQt4.QtGui import QApplication
from PyQt4.QtGui import QLabel, QPushButton, QSlider
from PyQt4.QtGui import QMainWindow
from PyQt4.QtGui import QGridLayout, QHBoxLayout, QVBoxLayout, QStatusBar
from PyQt4.QtGui import QGroupBox
from PyQt4.QtGui import QWidget
import pyqtgraph as pg
import pyqtgraph.opengl as gl

# msg
import message_filters
from geometry_msgs.msg import Twist
from hkust_msg_transformer.msg import PlainOutput
from hkust_msg_transformer.msg import PlainSscOutput

color_dict = {
    'red': '#ff0000',
    'green': '#00ff00',
    'blue': '#0000ff',
    'yellow': '#ffff00',
    'orange': '#FFA500',
    'magenta': '#ff00ff',
    'gray': '#808080',
    'cyan': '#00ffff',
    'tomato': '#FF6347',
    'coral': '#FF7F50',
    'indian red': '#CD5C5C',
    'light coral': '#F08080',
    'dark orange': '#FF8C00',
    'orange': '#FFA500',
    'gold': '#FFD700',
    'forest green': '#228B22',
    'lime': '#00FF00',
    'lime green': '#32CD32',
    'turquoise': '#40E0D0',
    'medium turquoise': '#48D1CC',
    'pale turquoise': '#AFEEEE',
    'aqua marine': '#7FFFD4',
    'dark violet': '#9400D3',
    'dark orchid': '#9932CC',
    'medium orchid': '#BA55D3',
    'purple': '#800080',
    'thistle': '#D8BFD8'
}


def get_rich_text(txt, size, weight, clr):
    color_txt = "<span style=\"font-size:{}pt; font-weight:{}; color:{};\" > {} </span>".format(
        size, weight, clr, txt)
    return color_txt

def get_vertex_list(x_lb, x_ub, y_lb, y_ub):
    x_list = []
    y_list = []

    x_list.append(x_lb)
    x_list.append(x_ub)
    x_list.append(x_ub)
    x_list.append(x_lb)
    x_list.append(x_lb)
    x_list.append(x_ub)
    x_list.append(x_ub)


    y_list.append(y_lb)
    y_list.append(y_lb)
    y_list.append(y_ub)
    y_list.append(y_ub)
    y_list.append(y_lb)
    y_list.append(y_lb)
    y_list.append(y_ub)

    return x_list, y_list

class GlTrajWithTxtWidget(gl.GLViewWidget):
    def __init__(self, parent=None):
        super(GlTrajWithTxtWidget, self).__init__(parent)
        self.qglColor(QtCore.Qt.white)
        self.info_set = []

    def set_info_set(self, info_set):
        self.info_set = info_set

    def paintGL(self, *args, **kwds):
        gl.GLViewWidget.paintGL(self, *args, **kwds)
        for info in self.info_set:
            x = info[0]
            y = info[1]
            z = info[2]
            txt = info[3]
            self.renderText(x, y, z, txt)


class EudmInfoWindow(QWidget):
    def __init__(self):
        super(EudmInfoWindow, self).__init__()

        # self.setStyleSheet("background-color: black;")
        self.setWindowTitle("Eudm Info")
        self.resize(800, 800)
        self.central_widget = QWidget(self)
        self.central_widget.setObjectName("central_widget")
        self.central_layout = QGridLayout()

        # Detail Box
        self.detail_box = QtGui.QTextBrowser()
        # self.detail_box.setGeometry(20, 20, 100, 100)
        self.detail_box.setObjectName("detail_box")
        detail_box_font = QFont()
        detail_box_font.setPointSize(10)
        self.detail_box.setFont(detail_box_font)
        self.detail_box.setVisible(False)

        # Traj Box
        self.plot_trajs = pg.PlotWidget(title='Trajs')
        self.plot_trajs.showGrid(x=True, y=True)
        self.plot_trajs.setXRange(-50, 200, padding=0)
        self.plot_trajs.setYRange(-15, 15, padding=0)
        self.plot_trajs.setAspectLocked()

        # Lon Vel Box
        self.plot_lonvel = pg.PlotWidget(title='Lon Vel')
        self.plot_lonvel.showGrid(x=True, y=True)
        self.plot_lonvel.setXRange(-10, 6, padding=0)
        self.plot_lonvel.setYRange(-1, 20, padding=0)
        self.curve_lonvel = self.plot_lonvel.getPlotItem().plot(pen=None, symbol = 'o', symbolSize = 5, symbolBrush=pg.mkBrush('r', width=1))
        self.curve_lonvel_xyqp = self.plot_lonvel.getPlotItem().plot(pen=pg.mkPen('g', width=2))
        self.curve_lonvel_plstate = self.plot_lonvel.getPlotItem().plot(pen=pg.mkPen('r', width=1), symbol = 'o', symbolSize = 2, symbolBrush=pg.mkBrush('r', width=1))

        self.legend_lonvel = self.plot_lonvel.addLegend()
        self.legend_lonvel.addItem(self.curve_lonvel, "Eudm")
        self.legend_lonvel.addItem(self.curve_lonvel_xyqp, "Ssc")
        self.legend_lonvel.addItem(self.curve_lonvel_plstate, "History")

        # Lon Acc Box
        self.plot_lonacc = pg.PlotWidget(title='Lon Acc')
        self.plot_lonacc.showGrid(x=True, y=True)
        self.plot_lonacc.setXRange(-10, 6, padding=0)
        self.plot_lonacc.setYRange(-5, 2.5, padding=0)
        self.curve_lonacc = self.plot_lonacc.getPlotItem().plot(pen=None, symbol = 'o', symbolSize = 5, symbolBrush=pg.mkBrush('r', width=1))
        self.curve_lonacc_xyqp = self.plot_lonacc.getPlotItem().plot(pen=pg.mkPen('g', width=2))
        self.curve_lonacc_plstate = self.plot_lonacc.getPlotItem().plot(pen=pg.mkPen('r', width=1), symbol = 'o', symbolSize = 2, symbolBrush=pg.mkBrush('r', width=1))

        self.legend_lonacc = self.plot_lonacc.addLegend()
        self.legend_lonacc.addItem(self.curve_lonacc, "Eudm")
        self.legend_lonacc.addItem(self.curve_lonacc_xyqp, "Ssc")
        self.legend_lonacc.addItem(self.curve_lonacc_plstate, "History")

        # Lat Acc Box
        self.plot_latacc = pg.PlotWidget(title='Lat Acc')
        self.plot_latacc.showGrid(x=True, y=True)
        self.plot_latacc.setXRange(-10, 6, padding=0)
        self.plot_latacc.setYRange(-2, 2, padding=0)
        self.curve_latacc = self.plot_latacc.getPlotItem().plot(pen=None, symbol = 'o', symbolSize = 5, symbolBrush=pg.mkBrush('r', width=1))
        self.curve_latacc_xyqp = self.plot_latacc.getPlotItem().plot(pen=pg.mkPen('g', width=2))
        self.curve_latacc_plstate = self.plot_latacc.getPlotItem().plot(pen=pg.mkPen('r', width=1), symbol = 'o', symbolSize = 2, symbolBrush=pg.mkBrush('r', width=1))

        self.legend_latacc = self.plot_latacc.addLegend()
        self.legend_latacc.addItem(self.curve_latacc, "Eudm")
        self.legend_latacc.addItem(self.curve_latacc_xyqp, "Ssc")
        self.legend_latacc.addItem(self.curve_latacc_plstate, "History")

        # Time Cost
        self.plot_timecost = pg.PlotWidget(title='Time Cost')
        self.plot_timecost.showGrid(x=True, y=True)
        self.plot_timecost.enableAutoScale()
        self.curve_timecost_eudm = self.plot_timecost.getPlotItem().plot(pen=pg.mkPen('r', width=1), symbol = 'o', symbolSize = 2, symbolPen=pg.mkPen('r', width=2))
        self.curve_timecost_ssc = self.plot_timecost.getPlotItem().plot(pen=pg.mkPen('g', width=1), symbol = 'o', symbolSize = 2, symbolPen=pg.mkPen('g', width=2))

        self.legend_timecost = self.plot_timecost.addLegend()
        self.legend_timecost.addItem(self.curve_timecost_eudm, "Eudm")
        self.legend_timecost.addItem(self.curve_timecost_ssc, "Ssc")

        # Frenet S-T
        self.plot_ffs = pg.PlotWidget(title='S-T')
        self.plot_ffs.showGrid(x=True, y=True)
        self.plot_ffs.setXRange(15, 100, padding=0)  # s
        self.plot_ffs.setYRange(-1, 6, padding=0)  # t
        self.curve_ffs_ref = self.plot_ffs.getPlotItem().plot(pen=None, symbol = 'o', symbolSize = 5, symbolBrush=pg.mkBrush('r', width=1))
        self.curve_ffs_qp = self.plot_ffs.getPlotItem().plot(pen=pg.mkPen('g', width=2))
        self.cube_items = []
        self.cube_ffs = self.plot_ffs.getPlotItem().plot(pen=pg.mkPen((255,255,0,100), width=1))

        # self.legend_ffs = self.plot_ffs.addLegend()
        # self.legend_ffs.addItem(self.curve_ffs_ref, "ref")
        # self.legend_ffs.addItem(self.curve_ffs_qp, "qp")

        # Frenet S'-T
        self.plot_ffsp = pg.PlotWidget(title='S_p-T')
        self.plot_ffsp.showGrid(x=True, y=True)
        self.plot_ffsp.setXRange(-1, 6, padding=0)  # s
        self.plot_ffsp.setYRange(-2, 25, padding=0)  # t
        self.curve_ffsp_ref = self.plot_ffsp.getPlotItem().plot(pen=None, symbol = 'o', symbolSize = 5, symbolBrush=pg.mkBrush('r', width=1))
        self.curve_ffsp_qp = self.plot_ffsp.getPlotItem().plot(pen=pg.mkPen('g', width=2))

        # self.legend_ffsp = self.plot_ffsp.addLegend()
        # self.legend_ffsp.addItem(self.curve_ffsp_ref, "ref")
        # self.legend_ffsp.addItem(self.curve_ffsp_qp, "qp")

        # Frenet S''-T
        self.plot_ffspp = pg.PlotWidget(title='S_pp-T')
        self.plot_ffspp.showGrid(x=True, y=True)
        self.plot_ffspp.setXRange(-1, 6, padding=0)  # s
        self.plot_ffspp.setYRange(-5, 2, padding=0)  # t
        self.curve_ffspp_ref = self.plot_ffspp.getPlotItem().plot(pen=None, symbol = 'o', symbolSize = 5, symbolBrush=pg.mkBrush('r', width=1))
        self.curve_ffspp_qp = self.plot_ffspp.getPlotItem().plot(pen=pg.mkPen('g', width=2))

        # self.legend_ffspp = self.plot_ffspp.addLegend()
        # self.legend_ffspp.addItem(self.curve_ffspp_ref, "ref")
        # self.legend_ffspp.addItem(self.curve_ffspp_qp, "qp")

        # Frenet D-T
        self.plot_ffd = pg.PlotWidget(title='D-T')
        self.plot_ffd.showGrid(x=True, y=True)
        self.plot_ffd.setXRange(-1, 6, padding=0)  # t
        self.plot_ffd.setYRange(-5, 5, padding=0)   # d
        self.curve_ffd_ref = self.plot_ffd.getPlotItem().plot(pen=None, symbol = 'o', symbolSize = 5, symbolBrush=pg.mkBrush('r', width=1))
        self.curve_ffd_qp = self.plot_ffd.getPlotItem().plot(pen=pg.mkPen('g', width=2))
        self.cube_ffd = self.plot_ffd.getPlotItem().plot(pen=pg.mkPen((255,255,0,100), width=1))

        # self.legend_ffd = self.plot_ffd.addLegend()
        # self.legend_ffd.addItem(self.curve_ffd_ref, "ref")
        # self.legend_ffd.addItem(self.curve_ffd_qp, "qp")

        # Frenet D'-T
        self.plot_ffdp = pg.PlotWidget(title='D_p-T')
        self.plot_ffdp.showGrid(x=True, y=True)
        self.plot_ffdp.setXRange(-1, 6, padding=0)  # t
        self.plot_ffdp.setYRange(-3, 3, padding=0)   # d
        self.curve_ffdp_ref = self.plot_ffdp.getPlotItem().plot(pen=None, symbol = 'o', symbolSize = 5, symbolBrush=pg.mkBrush('r', width=1))
        self.curve_ffdp_qp = self.plot_ffdp.getPlotItem().plot(pen=pg.mkPen('g', width=2))

        # self.legend_ffdp = self.plot_ffdp.addLegend()
        # self.legend_ffdp.addItem(self.curve_ffdp_ref, "ref")
        # self.legend_ffdp.addItem(self.curve_ffdp_qp, "qp")

        # Frenet D''-T
        self.plot_ffdpp = pg.PlotWidget(title='D_pp-T')
        self.plot_ffdpp.showGrid(x=True, y=True)
        self.plot_ffdpp.setXRange(-1, 6, padding=0)  # t
        self.plot_ffdpp.setYRange(-2, 2, padding=0)   # d
        self.curve_ffdpp_ref = self.plot_ffdpp.getPlotItem().plot(pen=None, symbol = 'o', symbolSize = 5, symbolBrush=pg.mkBrush('r', width=1))
        self.curve_ffdpp_qp = self.plot_ffdpp.getPlotItem().plot(pen=pg.mkPen('g', width=2))

        # self.legend_ffdpp = self.plot_ffdpp.addLegend()
        # self.legend_ffdpp.addItem(self.curve_ffdpp_ref, "ref")
        # self.legend_ffdpp.addItem(self.curve_ffdpp_qp, "qp")

        # Buttons
        self.bt_pause = QPushButton('Pause (P)', self)
        self.bt_pause.clicked.connect(self.pause)
        self.bt_pause.setShortcut(QtCore.Qt.Key_P)

        self.bt_prev = QPushButton('Prev (<-)', self)
        self.bt_prev.clicked.connect(self.draw_prev_frame)
        self.bt_prev.setEnabled(False)
        self.bt_prev.setShortcut(QtCore.Qt.Key_Left)

        self.bt_next = QPushButton('Next (->)', self)
        self.bt_next.clicked.connect(self.draw_next_frame)
        self.bt_next.setEnabled(False)
        self.bt_next.setShortcut(QtCore.Qt.Key_Right)

        self.bt_reset = QPushButton('Reset', self)
        self.bt_reset.clicked.connect(self.reset_frame)
        self.bt_reset.setEnabled(False)

        self.bt_clear = QPushButton('Clear', self)
        self.bt_clear.clicked.connect(self.clear)
        self.bt_clear.setEnabled(False)

        self.bt_hide = QPushButton('Hide Details', self)
        self.bt_hide.clicked.connect(self.hide_details)
        self.bt_hide.setEnabled(True)

        self.frame_info = QLabel('- / - , stamp: -', self)

        self.ctrl_slider = QSlider(QtCore.Qt.Horizontal)
        self.ctrl_slider.sliderMoved.connect(self.slider_draw_frame)
        self.ctrl_slider.setEnabled(False)

        self.ctrl_pannel = QGroupBox("")

        # self.central_layout.setColumnMinimumWidth(0, 400)
        # self.central_layout.setColumnMinimumWidth(1, 300)
        # self.central_layout.setRowMinimumHeight(2, 100)
        self.central_layout.setColumnStretch(0, 1)
        self.central_layout.setColumnStretch(1, 1)
        self.central_layout.setColumnStretch(2, 1)
        self.central_layout.setColumnStretch(3, 1)
        self.central_layout.setColumnStretch(4, 1)
        self.central_layout.setColumnStretch(5, 1)
        self.central_layout.setColumnStretch(6, 0)
        self.central_layout.setRowStretch(0, 4)
        self.central_layout.setRowStretch(1, 4)
        self.central_layout.setRowStretch(2, 4)
        self.central_layout.setRowStretch(3, 4)
        self.central_layout.setRowStretch(4, 4)
        self.central_layout.setRowStretch(5, 1)

        # Add all widgets
        self.central_layout.addWidget(self.detail_box, 0, 6, 5, 1)
        self.central_layout.addWidget(self.plot_trajs, 0, 0, 1, 6)
        self.central_layout.addWidget(self.plot_lonvel, 1, 0, 1, 3)
        self.central_layout.addWidget(self.plot_lonacc, 1, 3, 1, 3)
        self.central_layout.addWidget(self.plot_latacc, 2, 0, 1, 3)
        self.central_layout.addWidget(self.plot_timecost, 2, 3, 1, 3)
        self.central_layout.addWidget(self.plot_ffs, 3, 0, 1, 2)
        self.central_layout.addWidget(self.plot_ffsp, 3, 2, 1, 2)
        self.central_layout.addWidget(self.plot_ffspp, 3, 4, 1, 2)
        self.central_layout.addWidget(self.plot_ffd, 4, 0, 1, 2)
        self.central_layout.addWidget(self.plot_ffdp, 4, 2, 1, 2)
        self.central_layout.addWidget(self.plot_ffdpp, 4, 4, 1, 2)
        self.central_layout.addWidget(self.ctrl_pannel, 5, 0, 1, 7)

        # Control pannel
        cp_layout = QGridLayout()
        self.ctrl_pannel.setLayout(cp_layout)
        cp_layout.addWidget(self.frame_info, 0, 0, 1, 6)
        cp_layout.addWidget(self.bt_clear, 1, 0)
        cp_layout.addWidget(self.bt_prev, 1, 1)
        cp_layout.addWidget(self.bt_pause, 1, 2)
        cp_layout.addWidget(self.bt_next, 1, 3)
        cp_layout.addWidget(self.bt_reset, 1, 4)
        cp_layout.addWidget(self.bt_hide, 1, 5)
        cp_layout.addWidget(self.ctrl_slider, 2, 0, 1, 6)

        self.setLayout(self.central_layout)

        self.get_new = False
        self.is_pause = False
        self.draw_id = 0
        self.is_init = False

        self.eudm_invalid_cnt = 0
        self.ssc_invalid_cnt = 0

        self.kMaxSize = 500
        self.vec_stamp = []
        self.vec_bp_data = []
        self.vec_mp_data = []
        self.vec_plan_states = []
        self.vec_time_cost_eudm = []
        self.vec_time_cost_ssc = []
        self.buffer_size = 0

    def clear(self):
        self.is_init = False
        del self.vec_stamp[:]
        del self.vec_bp_data[:]
        del self.vec_mp_data[:]
        del self.vec_plan_states[:]
        del self.vec_time_cost_eudm[:]
        del self.vec_time_cost_ssc[:]
        self.draw_id = 0
        self.buffer_size = 0
        self.frame_info.setText('- / - , stamp: -')

    def pause(self):
        if (self.is_pause == True):
            self.is_pause = False
            # self.bt_pause.setText('Pause')
            self.bt_prev.setEnabled(False)
            self.bt_next.setEnabled(False)
            self.bt_reset.setEnabled(False)
            self.bt_clear.setEnabled(False)
            self.ctrl_slider.setEnabled(False)
        else:
            self.is_pause = True
            # self.bt_pause.setText('Resume')
            self.bt_prev.setEnabled(True)
            self.bt_next.setEnabled(True)
            self.bt_reset.setEnabled(True)
            self.bt_clear.setEnabled(True)
            self.ctrl_slider.setEnabled(True)

        self.draw_id = len(self.vec_bp_data) - 1
        self.ctrl_slider.setValue(self.buffer_size - 1)

    def slider_draw_frame(self):
        if (not self.is_init):
            return

        slider_id = self.ctrl_slider.value()
        self.draw_id = slider_id
        self.draw_frame(self.vec_bp_data[self.draw_id], self.vec_mp_data[self.draw_id], self.draw_id)

    def draw_prev_frame(self):
        if (not self.is_init):
            return

        frame_id = self.draw_id - 1
        self.draw_id = max(0, frame_id)
        self.draw_frame(self.vec_bp_data[self.draw_id], self.vec_mp_data[self.draw_id], self.draw_id)
        self.ctrl_slider.setValue(self.draw_id)

    def draw_next_frame(self):
        if (not self.is_init):
            return

        frame_id = self.draw_id + 1
        self.draw_id = min(self.buffer_size - 1, frame_id)
        self.draw_frame(self.vec_bp_data[self.draw_id], self.vec_mp_data[self.draw_id], self.draw_id)
        self.ctrl_slider.setValue(self.draw_id)

    def reset_frame(self):
        if (not self.is_init):
            return

        self.draw_id = self.buffer_size - 1
        self.draw_frame(self.vec_bp_data[self.draw_id], self.vec_mp_data[self.draw_id], self.draw_id)
        self.ctrl_slider.setValue(self.draw_id)

    def hide_details(self):
        if (self.detail_box.isVisible()):
            self.detail_box.setVisible(False)
        else:
            self.detail_box.setVisible(True)

    def set_sync_data(self, eudm_data, ssc_data):
        if (self.is_pause):
            return

        self.get_new = True
        self.bp_data = eudm_data
        self.mp_data = ssc_data
        self.stamp = eudm_data.header.stamp.to_time()

        self.vec_stamp.append(self.stamp)
        if (len(self.vec_stamp) > self.kMaxSize):
            del self.vec_stamp[0]

        self.vec_bp_data.append(eudm_data)
        if (len(self.vec_bp_data) > self.kMaxSize):
            del self.vec_bp_data[0]

        self.vec_mp_data.append(ssc_data)
        if (len(self.vec_mp_data) > self.kMaxSize):
            del self.vec_mp_data[0]

        plan_state = eudm_data.ego_state
        self.vec_plan_states.append(plan_state)
        if (len(self.vec_plan_states) > self.kMaxSize):
            del self.vec_plan_states[0]

        self.vec_time_cost_eudm.append(eudm_data.time_cost)
        if (len(self.vec_time_cost_eudm) > self.kMaxSize):
            del self.vec_time_cost_eudm[0]

        self.vec_time_cost_ssc.append(ssc_data.time_cost)
        if (len(self.vec_time_cost_ssc) > self.kMaxSize):
            del self.vec_time_cost_ssc[0]

        self.buffer_size = len(self.vec_bp_data)
        self.ctrl_slider.setMaximum(self.buffer_size - 1)

        self.is_init = True


    def update(self):
        if (not self.is_init):
            return
        if (self.is_pause):
            return
        if (not self.get_new):
            return
        self.get_new = False

        frame_data_eudm = self.bp_data
        frame_data_ssc = self.mp_data
        self.draw_frame(frame_data_eudm, frame_data_ssc, self.buffer_size - 1)


    def draw_frame(self, frame_data_eudm, frame_data_ssc, cur_id):
        self.detail_box.clear()

        if (not frame_data_eudm.valid):
            print("Eudm data invalid")
            return

        if (not frame_data_ssc.valid):
            print("Ssc data invalid")
            return

        winner_id = frame_data_eudm.winner_id
        eudm_start_stamp = frame_data_eudm.header.stamp.to_time()
        ssc_start_stamp = frame_data_ssc.header.stamp.to_time()
        ssc_replan_stamp = frame_data_ssc.qp_traj[0].header.stamp.to_time()
        # eudm_start_stamp = frame_data_eudm.elements[winner_id].ego_traj.traj[0].header.stamp.to_time()
        ego_state_stamp = frame_data_eudm.elements[winner_id].ego_traj.traj[0].header.stamp.to_time()
        original_winner_id = frame_data_eudm.original_winner_id
        # detail box
        self.detail_box.append('Eudm msg time stamp: {:.5f}'.format(eudm_start_stamp))
        self.detail_box.append('Is valid: ' + str(frame_data_eudm.valid))
        self.detail_box.append('Ego state time stamp: {:.5f}'.format(ego_state_stamp))
        self.detail_box.append('Final winner id: ' + str(winner_id))
        self.detail_box.append('original winner id: ' +
                               str(original_winner_id))
        self.detail_box.append(
            'Winner cost: ' + "{:.5f}".format(frame_data_eudm.winner_cost))
        self.detail_box.append('ongoing action: {:.2f} s'.format(
            frame_data_eudm.ongoing_action_duration))
        self.detail_box.append(' ')
        for ele in frame_data_eudm.elements:
            ele_txt = 'Id: ' + str(ele.id)
            ele_txt = ele_txt + ' ' + ele.lat_cmds + ' ' + ele.lon_cmds
            ele_txt = ele_txt + ' ' + "{:.5f}".format(ele.final_cost)
            ele_txt = ele_txt + ' ' + ele.sim_info

            if (ele.id == winner_id):
                ele_txt = get_rich_text(ele_txt, 12, 'bold', 'magenta')
                self.detail_box.append(ele_txt)
                continue

            if (ele.id == original_winner_id):
                ele_txt = get_rich_text(ele_txt, 10, 'bold', 'green')
                self.detail_box.append(ele_txt)
                continue

            if (not ele.valid):
                ele_txt = get_rich_text(ele_txt, 10, 'normal', 'gray')
                self.detail_box.append(ele_txt)
                continue

            if (ele.risky):
                ele_txt = get_rich_text(ele_txt, 10, 'normal', 'orange')
                self.detail_box.append(ele_txt)
                continue

            self.detail_box.append(ele_txt)

        # control pannel
        self.frame_info.setText('{} / {} , stamp: {:.3f}, dt: {:.3f} s, Eudm&Ssc t_diff: {:.5f} s'.format(
            cur_id, self.buffer_size - 1, eudm_start_stamp, eudm_start_stamp - self.stamp,
            eudm_start_stamp - ssc_start_stamp))

        # Plan result
        opt_ele = frame_data_eudm.elements[winner_id]
        vec_t = []
        vec_vel = []
        vec_lon_acc = []
        vec_lat_acc = []
        for state in opt_ele.ego_traj.traj:
            vec_t.append(state.header.stamp.to_time() - eudm_start_stamp)
            vec_vel.append(state.velocity)
            vec_lon_acc.append(state.acceleration)
            vec_lat_acc.append(state.curvature * state.velocity * state.velocity)
        self.curve_lonvel.setData(vec_t, vec_vel)
        self.curve_lonacc.setData(vec_t, vec_lon_acc)
        self.curve_latacc.setData(vec_t, vec_lat_acc)

        # QP x-y Traj
        vec_t_xyqp = []
        vec_vel_xyqp = []
        vec_lon_acc_xyqp = []
        vec_lat_acc_xyqp = []
        for state in frame_data_ssc.sampled_traj:
            vec_t_xyqp.append(state.header.stamp.to_time() - ssc_start_stamp)
            vec_vel_xyqp.append(state.velocity)
            vec_lon_acc_xyqp.append(state.acceleration)
            vec_lat_acc_xyqp.append(state.curvature * state.velocity * state.velocity)
        self.curve_lonvel_xyqp.setData(vec_t_xyqp, vec_vel_xyqp)
        self.curve_lonacc_xyqp.setData(vec_t_xyqp, vec_lon_acc_xyqp)
        self.curve_latacc_xyqp.setData(vec_t_xyqp, vec_lat_acc_xyqp)

        # Frenet Frame Ref States
        vec_t_ref = []
        vec_s_ref = []
        vec_sp_ref = []
        vec_spp_ref = []
        vec_d_ref = []
        vec_dp_ref = []
        vec_dpp_ref = []
        for state in frame_data_ssc.ref_ff_states:
            vec_t_ref.append(state.header.stamp.to_time() - ssc_start_stamp)
            vec_s_ref.append(state.vec_s[0])
            vec_sp_ref.append(state.vec_s[1])
            vec_spp_ref.append(state.vec_s[2])

            vec_d_ref.append(state.vec_dt[0])
            vec_dp_ref.append(state.vec_dt[1])
            vec_dpp_ref.append(state.vec_dt[2])
        self.curve_ffs_ref.setData(vec_s_ref, vec_t_ref)
        self.curve_ffsp_ref.setData(vec_t_ref, vec_sp_ref)
        self.curve_ffspp_ref.setData(vec_t_ref, vec_spp_ref)
        self.curve_ffd_ref.setData(vec_t_ref, vec_d_ref)
        self.curve_ffdp_ref.setData(vec_t_ref, vec_dp_ref)
        self.curve_ffdpp_ref.setData(vec_t_ref, vec_dpp_ref)

        # Frenet Frame QP Traj
        vec_t_qp = []
        vec_s_qp = []
        vec_sp_qp = []
        vec_spp_qp = []
        vec_d_qp = []
        vec_dp_qp = []
        vec_dpp_qp = []
        for state in frame_data_ssc.qp_traj:
            vec_t_qp.append(state.header.stamp.to_time() - ssc_start_stamp)
            vec_s_qp.append(state.vec_s[0])
            vec_sp_qp.append(state.vec_s[1])
            vec_spp_qp.append(state.vec_s[2])

            vec_d_qp.append(state.vec_dt[0])
            vec_dp_qp.append(state.vec_dt[1])
            vec_dpp_qp.append(state.vec_dt[2])
        self.curve_ffs_qp.setData(vec_s_qp, vec_t_qp)
        self.curve_ffsp_qp.setData(vec_t_qp, vec_sp_qp)
        self.curve_ffspp_qp.setData(vec_t_qp, vec_spp_qp)
        self.curve_ffd_qp.setData(vec_t_qp, vec_d_qp)
        self.curve_ffdp_qp.setData(vec_t_qp, vec_dp_qp)
        self.curve_ffdpp_qp.setData(vec_t_qp, vec_dpp_qp)

        # Corridor
        x_st_list = []
        y_st_list = []
        x_sd_list = []
        y_sd_list = []
        for cube in frame_data_ssc.corridor:
            # s-t
            x_st, y_st = get_vertex_list(cube.p_lb[0], cube.p_ub[0], cube.t_lb - ssc_start_stamp, cube.t_ub - ssc_start_stamp)
            x_st_list += x_st
            y_st_list += y_st

            # d-t
            x_sd, y_sd = get_vertex_list(cube.t_lb - ssc_start_stamp, cube.t_ub - ssc_start_stamp, cube.p_lb[1], cube.p_ub[1])
            x_sd_list += x_sd
            y_sd_list += y_sd

        self.cube_ffs.setData(x_st_list, y_st_list)
        self.cube_ffd.setData(x_sd_list, y_sd_list)

        # Plan States
        vec_t_plstate = []
        vec_vel_plstate = []
        vec_lon_acc_plstate = []
        vec_lat_acc_plstate = []
        for state in self.vec_plan_states:
            vec_t_plstate.append(state.header.stamp.to_time() - eudm_start_stamp)
            vec_vel_plstate.append(state.velocity)
            vec_lon_acc_plstate.append(state.acceleration)
            vec_lat_acc_plstate.append(state.curvature * state.velocity * state.velocity)
        self.curve_lonvel_plstate.setData(vec_t_plstate, vec_vel_plstate)
        self.curve_lonacc_plstate.setData(vec_t_plstate, vec_lon_acc_plstate)
        self.curve_latacc_plstate.setData(vec_t_plstate, vec_lat_acc_plstate)

        # Time cost
        self.curve_timecost_eudm.setData(self.vec_stamp, self.vec_time_cost_eudm)
        self.curve_timecost_ssc.setData(self.vec_stamp, self.vec_time_cost_ssc)

        # plot trajs
        self.plot_trajs.getPlotItem().clear()

        origin_x = opt_ele.ego_traj.traj[0].vec_position.x
        origin_y = opt_ele.ego_traj.traj[0].vec_position.y
        origin_angle = opt_ele.ego_traj.traj[0].angle
        cos_theta = math.cos(origin_angle)
        sin_theta = math.sin(origin_angle)

        ego_traj_x = []
        ego_traj_y = []
        for state in opt_ele.ego_traj.traj:
            dx = state.vec_position.x - origin_x
            dy = state.vec_position.y - origin_y
            ego_traj_x.append(cos_theta * dx + sin_theta * dy)
            ego_traj_y.append(-sin_theta * dx + cos_theta * dy)
        curve1 = self.plot_trajs.getPlotItem().plot(pen=pg.mkPen('r', width=2), symbol = 'o', symbolSize = 5, symbolBrush=pg.mkBrush('r', width=1), symbolPen=None)
        curve1.setData(ego_traj_x, ego_traj_y)

        for traj in opt_ele.surround_trajs:
            traj_x = []
            traj_y = []
            clr = list(color_dict.values())[traj.id % len(color_dict)]
            for state in traj.traj:
                dx = state.vec_position.x - origin_x
                dy = state.vec_position.y - origin_y
                traj_x.append(cos_theta * dx + sin_theta * dy)
                traj_y.append(-sin_theta * dx + cos_theta * dy)
            curve_tmp = self.plot_trajs.getPlotItem().plot(pen=pg.mkPen(clr, width=2), symbol = 'o', symbolSize = 5, symbolBrush=pg.mkBrush(clr, width=1), symbolPen=None)
            curve_tmp.setData(traj_x, traj_y)


class EudmPannel(QMainWindow):
    def __init__(self):
        super(EudmPannel, self).__init__()

        # self.setStyleSheet("background-color: black;")
        
        self.initUI()
        self.rcv_cnt = 0
        self.setWindowTitle("Eudm Pannel")
        self.resize(800, 800)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(100)

    def initUI(self):
        self.statusBar()
        self.eudm_info = EudmInfoWindow()
        self.setCentralWidget(self.eudm_info)

    def set_sync_data(self, eudm_msg, ssc_msg):
        self.eudm_info.set_sync_data(eudm_msg, ssc_msg)
        self.rcv_cnt = self.rcv_cnt + 1

    def update(self):
        self.eudm_info.update()
        self.statusBar().showMessage('Received: {} msgs'.format(self.rcv_cnt))


app = QtGui.QApplication([])
window = EudmPannel()
pg.setConfigOptions(antialias=True)


def sync_callback(eudm_msg, ssc_msg):
    window.set_sync_data(eudm_msg, ssc_msg)
    # dt = eudm_msg.header.stamp.to_time() - ssc_msg.header.stamp.to_time()

if __name__ == '__main__':
    rospy.init_node("eudm_info", anonymous=True)

    print("Sync Mode")
    eudm_sub_ = message_filters.Subscriber("/hkust/plain_output", PlainOutput)
    ssc_sub_ = message_filters.Subscriber("/hkust/plain_ssc_output", PlainSscOutput)
    ts = message_filters.TimeSynchronizer([eudm_sub_, ssc_sub_], 50)
    ts.registerCallback(sync_callback)

    window.show()
    app.exec_()
