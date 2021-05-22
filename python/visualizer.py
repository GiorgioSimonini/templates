'''----------------------------------------------------------------------------------------------------------------*
	Author: Giorgio Simonini
	Title: 	VISUALIZER CLASS
    Time: 	2021-05-22
	Description: 
		- template of a graphic window to plot stuffs, already set-up for multiple plots
	Functionalities: 
		- graphWindow:
			- is the main class, best if created in main code, in this case running in a thread
			- create the window that contain the plots, data is updated each SAMPLE_TIME ms
			- data needs to be updated externally, asincronously
			- two plot in the template: x-y and (t, [x,y,z])
		- graphPlotter:
			- contain thread that plot data in the graphWindow
    To do:
		- 
    Problems:
		- implemented in thread, documentation says that must be avoided :)

-----------------------------------------------------------------------------------------------------------------'''

import threading
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
import sys



#-------------------------#
#       GRAPH_WINDOW      #
#-------------------------#
class graphWindow(QtWidgets.QWidget):

	# ----- INIT CLASS ----- #
	def __init__(self, *args, **kwargs):

		# PARAMETERS
		self.SAMPLE_TIME = 50

		# initialize
		super(graphWindow, self).__init__(*args, **kwargs)
		
		# create a grid layout
		self.grid = QtWidgets.QGridLayout()
		self.setLayout(self.grid)

		# create widgets (each widget is a plot)
		self.xy_graphWidget = pg.PlotWidget()
		self.time_graphWidget = pg.PlotWidget()

		# add widgets to grid
		self.grid.addWidget(self.xy_graphWidget, 0, 0)
		self.grid.addWidget(self.time_graphWidget, 1, 0)
		
		# istantaneus value to plot (variables that must updated to plot)
		self.x = [0]
		self.y = [0]

		self.t = [0]
		self.x_t = [0]
		self.y_t = [0]
		self.z_t = [0]

		# backgrounds
		self.xy_graphWidget.setBackground('w')
		self.time_graphWidget.setBackground('w')

		# limits (auto if omitted)
		# self.xy_graphWidget.setXRange(-30, 30, padding=0)
		# self.xy_graphWidget.setYRange(-30, 30, padding=0)

		# colors
		pen_x = pg.mkPen(color=(255, 0, 0)) 	# red
		pen_y = pg.mkPen(color=(0, 0, 255)) 	# blue
		pen_z = pg.mkPen(color=(0, 0, 0)) 		# black

		# plots (xy is point cloud, time_graph is lined)
		self.xy_data =  self.xy_graphWidget.plot(self.x, self.y, pen=None, symbol='o', symbolSize=3, symbolBrush='r')
		self.xy_graphWidget.plot([0], [0], pen=None, symbol='o', symbolSize=15, symbolBrush='b')
		self.x_t_data =  self.time_graphWidget.plot(self.t, self.x_t, pen=pen_x)
		self.y_t_data =  self.time_graphWidget.plot(self.t, self.y_t, pen=pen_y)
		self.z_t_data =  self.time_graphWidget.plot(self.t, self.z_t, pen=pen_z)
	
		# timer for update
		self.timer = QtCore.QTimer()
		self.timer.setInterval(self.SAMPLE_TIME)
		self.timer.setTimerType(0)
		self.timer.timeout.connect(self.update_plot)
		self.timer.start()	


	# ----- UPDATE PLOT ----- #
	def update_plot(self):
		self.xy_data.setData(self.x, self.y)
		self.x_t_data.setData(self.t, self.x_t)
		self.y_t_data.setData(self.t, self.y_t)
		self.z_t_data.setData(self.t, self.z_t)



#--------------------#
#       PLOTTER      #
#--------------------#
class graphPlotter:

	# ----- INIT CLASS ----- #
	def __init__(self):

		# parameters
		self.TIME_LEN = 1000

		# graphWindow
		self.w = None		


	# ----- START THREAD ----- #
	def start(self):
		threading.Thread(target=self.QT_plot_thread).start()


	# ----- SHOW THREAD ----- #
	def QT_plot_thread(self):
		# QtGraph
		self.app = QtWidgets.QApplication(sys.argv)
		self.w = graphWindow()
		self.w.show()
		self.app.exec_() # MAGIC
		#sys.exit(app.exec_())

	
	# ----- UPDATE DATA ----- #
	def update_QT_data(self, x, y, x_t, y_t, z_t):

		if self.w != None:
			# update xy
			self.w.x = x
			self.w.y = y

			# update time plot
			self.w.t.append(self.w.t[-1]+1)
			self.w.x_t.append(x_t)
			self.w.y_t.append(y_t)
			self.w.z_t.append(z_t)
			# moving window
			if len(self.w.t) >= self.TIME_LEN:
				self.w.t.pop(0)
				self.w.x_t.pop(0)
				self.w.y_t.pop(0)
				self.w.z_t.pop(0)



# # ------------------ needed in your code: ------------------------- #
#
# # visualizing data
# x = []
# y = []
# x_t = y_t = z_t = []
# plotter = graphPlotter()
# plotter.start()
#
# while 1:
# 	# update your variables
#
#	# plot
# 	plotter.update_QT_data(x, y, x_t, y_t, z_t)
#
# # ----------------------------------------------------------------- #
