'''----------------------------------------------------------------------------------------------------------------*
	Author: Giorgio Simonini
	Title: 	UDP CLASS
    Time: 	2021-05-13
	Description: 
		- udp class, can be used to send and receive list of floats throught UDP
		- the idea is to have a simple structure that can be easily modified
	Functionalities: 
		- send list of floats
		- receive list of floats
    To do:
		- generalize for other data types
		- integrate get_data() into class
    Problems:

-----------------------------------------------------------------------------------------------------------------'''

import time
import socket
import threading
import struct


class udp_class:
	
	# ----- INIT CLASS ----- #
	def __init__(self, remote_ip, remote_port, local_ip, local_port):
		
		self.RECV_BYTES = 255

		# ips and ports
		self.__REMOTE_PORT = remote_port
		self.__REMOTE_IP = remote_ip
		self.__LOCAL_PORT = local_port
		self.__LOCAL_IP = local_ip
		
		# create a socket DGRAM: message like, no problem if lost
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		
		# assign ip and port to socket (should use ip in witch this program run)
		self.sock.bind((local_ip, local_port))

		# non-blocking behaviour
		self.sock.setblocking(0)
		
		# self.__packet_list = []		# if needed
		self.__last_packet = None
		# self.__to_send_list = []		# if needed
		self.__die_flag = False


	# ----- RECEIVE TASK ----- #
	def run(self):
		rx_buf = None
		while True:
			# Recieve ONE packet from udp
			try:
				# try to receive ONE packet
				rx_buf, addr = self.sock.recvfrom(self.RECV_BYTES)
				self.__last_packet = rx_buf
			except Exception as e:
				# self.__last_packet = None
				#print(e)
				pass

			if self.__die_flag == True:
				print("UDP Connection closed\n")
				self.sock.close()  
				return
			# time.sleep(RECV_PERIOD)


	# ----- GET PACKET ----- #
	# remove last packet when readed
	def getPacket(self):
		temp = self.__last_packet
		self.__last_packet = None
		return temp
	

	# ----- SEND ----- #
	# send list of float
	def send(self, message):
		tx_buf = struct.pack('%sf' % len(message), *message)
		self.sock.sendto(tx_buf, (self.__REMOTE_IP, self.__REMOTE_PORT))


	# ----- START RECEIVING THREAD ----- #
	def start(self):
		threading.Thread(target=self.run).start()
		time.sleep(0.5)


	# ----- END CONNECTION ----- #
	def close(self):
		self.__die_flag = True



# # ------------------ needed in your code: ------------------------- #
# # ----- GET DATA from PACKET ----- #

# DATA_LEN = 									# message lenght in bytes
# udp_connection = udp_class(...)				# create udp_class istance
# data = None									# unpacked data from network
# data_time	= None								# not implemented

# def get_data():
# 	msg = udp_connection.getPacket()					# try to get last packet
# 	global data
#	global data_time
# 	if msg != None and len(msg) == DATA_LEN:  			# if new packet arrived
# 		msg_iter = struct.iter_unpack('f', msg)			# get iterator containing data
# 		sens_data_tuple = list(msg_iter)				# convert to list of tuple (unpack returns tuple)
# 		data = [x[0] for x in sens_data_tuple]			# extract first element from each tuple
# 		data_time = time.time()							# better if time is taken when packet arrive
# # ----------------------------------------------------------------- #