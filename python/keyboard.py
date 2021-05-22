'''----------------------------------------------------------------------------------------------------------------*
	Author: 
		Giorgio Simonini		https://github.com/GiorgioSimonini
	Title:
    Data:
	Description:
	Functionalities:
    To do:
    Problems:

-----------------------------------------------------------------------------------------------------------------'''

import keyboard

while True:
	try:  # used try so that if user pressed other than the given key, error will not be shown
		if keyboard.is_pressed('w'):  # if key 'w' is pressed 
			print('avanti')
		if keyboard.is_pressed('s'):
			print('indietro')
		if keyboard.is_pressed('a'):
			print('sinistra')
		if keyboard.is_pressed('d'):
			print('destra')
		if keyboard.is_pressed('q'):
			break
	except:
		pass  # if user pressed a key other than the given key the loop will break