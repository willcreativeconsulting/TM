from omxplayer import OMXPlayer
from time import sleep
import serial
import os
import subprocess
from usb.core import find as finddev

_1927 = '/home/pi/python/python-omxplayer-wrapper/videos_B2F/registo.mp4'
_1995 = '/home/pi/python/python-omxplayer-wrapper/videos_B2F/SBSR.mp4'
_2013 = '/home/pi/python/python-omxplayer-wrapper/videos_B2F/seleccao_1927.mp4'
_1980 = '/home/pi/python/python-omxplayer-wrapper/videos_B2F/anuncio_1980.mp4'
_1970 = '/home/pi/python/python-omxplayer-wrapper/videos_B2F/25_milhoes.mp4'
_1998 = '/home/pi/python/python-omxplayer-wrapper/videos_B2F/1998.mp4'
_1967 = '/home/pi/python/python-omxplayer-wrapper/videos_B2F/imprensa.mp4'
_1928 = '/home/pi/python/python-omxplayer-wrapper/videos_B2F/90_anos.mp4'

_tempo1 =  '/home/pi/python/python-omxplayer-wrapper/videos_B2F/tempo01.mp4'
_tempo2 =  '/home/pi/python/python-omxplayer-wrapper/videos_B2F/tempo02.mp4'
_tempo3 =  '/home/pi/python/python-omxplayer-wrapper/videos_B2F/tempo03.mp4'
_tempo4 =  '/home/pi/python/python-omxplayer-wrapper/videos_B2F/tempo04.mp4'


#ser = serial.Serial('/dev/ttyUSB1', 9600, timeout = 1)
USB_ID = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
#USB_ID = '/dev/bus/usb/001/007'
ser = serial.Serial(USB_ID, 9600, timeout = 1)

sleep(1)

def check_halt(time_s):
        try:
                for i in range(0, time_s):
                        rx = ser.read()
                        if rx.find("X") != -1:
                                #shutdown
                                print "going to halt"
                                os.system('sudo halt')
                                sleep(10)
        except:
                print "check_halt failed !!!"
                return

def read_serial():
	rx = ' ' 
	for i in range(0, 20):
		rx = ser.read()
		#print(rx + "read #" + i)
		sleep(0.1)		
		if rx == 'r': 
			#ready
			print "found r"
			return "READY"
		
		if rx == 'X':
			#shutdown
			print "going to halt"
			os.system('sudo halt')
			sleep(10)
	

	return "READY"


def play_video(year):
	try:
		ser.write('-' + year + '/')
		ser.flush()
		print('-' + year + '/')
		try:
                        if read_serial() == "READY":
                                myprocess_.stdin.write('q')
                                if year == "1927":
                                        myprocess = subprocess.Popen(['omxplayer','-b -z',_1927],stdin=subprocess.PIPE)
                                        check_halt(25)
                                        myprocess.stdin.write('q')
                                if year == "1995":
                                        myprocess = subprocess.Popen(['omxplayer','-b -z',_1995],stdin=subprocess.PIPE)
                                        check_halt(20)
                                        myprocess.stdin.write('q')
                                if year == "2013":
                                        myprocess = subprocess.Popen(['omxplayer','-b -z',_2013],stdin=subprocess.PIPE)
                                        check_halt(15)
                                        myprocess.stdin.write('q')
                                if year == "1980":
                                        myprocess = subprocess.Popen(['omxplayer','-b -z',_1980],stdin=subprocess.PIPE)
                                        check_halt(22)
                                        myprocess.stdin.write('q')
                                if year == "1970":
                                        myprocess = subprocess.Popen(['omxplayer','-b -z',_1970],stdin=subprocess.PIPE)
                                        check_halt(15)
                                        myprocess.stdin.write('q')
                                if year == "1998":
                                        myprocess = subprocess.Popen(['omxplayer','-b -z',_1998],stdin=subprocess.PIPE)
                                        check_halt(32)
                                        myprocess.stdin.write('q')
                                if year == "1967":
                                        myprocess = subprocess.Popen(['omxplayer','-b -z',_1967],stdin=subprocess.PIPE)
                                        check_halt(17)
                                        myprocess.stdin.write('q')
                                if year == "1928":
                                        myprocess = subprocess.Popen(['omxplayer','-b -z',_1928],stdin=subprocess.PIPE)
                                        check_halt(132)
                                        myprocess.stdin.write('q')
                except:
                        print "SERIAL PORT NOT READY TO READ!!!!"
                        return
	except:
		print "LOST SERIAL PORT!!!!"
		try:
                        myprocess.stdin.write('q')
                except:
                        print "Main video not running!!!!"		
		try:
                        myprocess_.stdin.write('q')
                except:
                        print "Transition video not running!!!!"
        try:
                print "Reseting Driver"
                dev = finddev(idVendor=0x10c4, idProduct=0xea60)
                dev.reset()
                ser.close()
        except:
                print "Failed to Reopen Driver"
                sleep(10)
	return


while 1:
        #ser.flushInput()
        myprocess_ = subprocess.Popen(['omxplayer','-b -z',_tempo1],stdin=subprocess.PIPE)
        sleep(5)
        ser = serial.Serial(USB_ID, 9600, timeout = 1)
        play_video("1927")
        #ser.flushInput()
        myprocess_ = subprocess.Popen(['omxplayer','-b -z',_tempo2],stdin=subprocess.PIPE)
        sleep(5)
        ser = serial.Serial(USB_ID, 9600, timeout = 1)
        play_video("1995")
        #ser.flushInput()
        myprocess_ = subprocess.Popen(['omxplayer','-b -z',_tempo3],stdin=subprocess.PIPE)
        sleep(9)
        ser = serial.Serial(USB_ID, 9600, timeout = 1)
        play_video("2013")
        #ser.flushInput()
        myprocess_ = subprocess.Popen(['omxplayer','-b -z',_tempo4],stdin=subprocess.PIPE)
        sleep(5)
        ser = serial.Serial(USB_ID, 9600, timeout = 1)
        play_video("1980")
        #ser.flushInput()
        myprocess_ = subprocess.Popen(['omxplayer','-b -z',_tempo1],stdin=subprocess.PIPE)
        sleep(5)
        ser = serial.Serial(USB_ID, 9600, timeout = 1)
        play_video("1970")
        #ser.flushInput()
        myprocess_ = subprocess.Popen(['omxplayer','-b -z',_tempo2],stdin=subprocess.PIPE)
        sleep(5)
        ser = serial.Serial(USB_ID, 9600, timeout = 1)
        play_video("1998")
        #ser.flushInput()
        myprocess_ = subprocess.Popen(['omxplayer','-b -z',_tempo3],stdin=subprocess.PIPE)
        sleep(5)
        ser = serial.Serial(USB_ID, 9600, timeout = 1)
        play_video("1967")
        #ser.flushInput()
        myprocess_ = subprocess.Popen(['omxplayer','-b -z',_tempo4],stdin=subprocess.PIPE)
        sleep(5)
        ser = serial.Serial(USB_ID, 9600, timeout = 1)
        play_video("1928")
