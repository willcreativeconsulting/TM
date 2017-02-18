from omxplayer import OMXPlayer
from time import sleep
import serial
import os
import subprocess

_1910 = '/home/pi/Desktop/1910.mp4'
_1920 = '/home/pi/Desktop/1920.mp4'
_1930 = '/home/pi/Desktop/1930.mp4'
_1940 = '/home/pi/Desktop/1940.mp4'
black = '/home/pi/Desktop/black.mp4'

#colocar tb o loop nos videos
#colocar o '-b'

ser = serial.Serial('/dev/ttyUSB0', 9600)

def read_serial():
	rx = ' '
	for i in range(0, 2400):
		sleep(0.1)
		rx = ser.read()
		print rx
		if rx == 'r': 
			#ready
			print "found r"
			return "READY"
		if rx == 'X':
			#shutdown
			print "going to halt"
			os.system('sudo halt')
			sleep(10)
	

	return "ERROR"
#


def play_video(year):
	try:
		ser.write('-' + year + '/')
		print('-' + year + '/')
		#player.load(black)
		time_to_wait = 0
		if read_serial() == "READY":
		#if 1:
			if year == "1910":
				myprocess = subprocess.Popen(['omxplayer','-b',_1910],stdin=subprocess.PIPE)
				#myprocess = subprocess.Popen(['omxplayer', _1910],stdin=subprocess.PIPE)
				sleep(5)
				myprocess.stdin.write('q')
			if year == "1920":
				myprocess = subprocess.Popen(['omxplayer','-b',_1920],stdin=subprocess.PIPE)
				#myprocess = subprocess.Popen(['omxplayer', _1920],stdin=subprocess.PIPE)
				sleep(5)
				myprocess.stdin.write('q')
			if year == "1930":
				myprocess = subprocess.Popen(['omxplayer','-b',_1930],stdin=subprocess.PIPE)
				#myprocess = subprocess.Popen(['omxplayer', _1930],stdin=subprocess.PIPE)
				sleep(5)
				myprocess.stdin.write('q')
			if year == "1940":
				myprocess = subprocess.Popen(['omxplayer','-b',_1940],stdin=subprocess.PIPE)
				#myprocess = subprocess.Popen(['omxplayer', _1940],stdin=subprocess.PIPE)
				sleep(5)
				myprocess.stdin.write('q')
	except:
		print "/n"
		sleep(1)
	return

		

while 1:
        play_video("1910")
        play_video("1920")
        play_video("1930")
        play_video("1940")
