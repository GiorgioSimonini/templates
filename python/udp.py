'''----------------------------------------------------------------------------------------------------------------*
	Author: Giorgio Simonini
	Title:
    Data:
	Description:
	Functionalities:
    To do:
    Problems:

-----------------------------------------------------------------------------------------------------------------'''

import socket
from time import sleep
import time


'''	------------
	| COSTANTS |
	------------'''

TS = 0.01					# sample time

UDP_IP = "192.168.x.x"		# host ip address
UDP_PORT_TX = 4210			# port to trasmit
# UDP_PORT_RX = 4209

TX_LEN = 256				# maximum tx lenght
RX_LEN = 256				# maximum rx lenght


'''	--------
	| INIT |
	--------'''

data_tx = bytearray(TX_LEN)		# data to trasmit
data_rx = bytearray(RX_LEN)		# data to receive

sock = socket.socket(	socket.AF_INET, 		# Internet
						socket.SOCK_DGRAM) 		# UDP

sock.setblocking(0)								# not stopping receiving behaviour


'''	--------
	| LOOP |
	--------'''

while True:
	
	# ----- RECEIVE ----- #
	try:
		data_rx, address = sock.recvfrom(RX_LEN)		# receive data from network if exist
	except:
		pass
	
	# ----- SEND ----- #
	'''
	data_tx[index] = byte
	or
	data_rx = "string_to_send"		# needed traslation from string to bytearray
	'''
	sock.sendto(data_tx, (UDP_IP, UDP_PORT_TX))			# send data to UDP_IP through UDP_PORT_TX

	sleep(TS)