import socket
import sys
import time
import RPi.GPIO as GPIO
import wiringpi
import json
import camera
import threading
import random
from distanceSensor import *
import os
import Routines
import subprocess

CHUNK_SIZE = 4096


"""
12 -> left - front
13 -> left - back
4 -> right - front
27 -> right - back
18 -> serv
20 -> serv
21 -> serv
19 -> dist
26 -> dist
5  -> dist
6  -> dist
"""
class Agent(object):
    def __init__(self):
        GPIO.setmode(GPIO.BCM)

        out_pins = [5,6]
        for pin in out_pins:
            GPIO.setup(pin,GPIO.OUT)

        return

        self.direction = 'steady'
        out_pins = [12,13,4,27]
        self.speed = 0
        self.in_routine = False
        for pin in out_pins:
            GPIO.setup(pin,GPIO.OUT)
        self.pwms = [GPIO.PWM(pin,15) for pin in out_pins]
        for pwm in self.pwms:
            pwm.start(self.speed)
            pwm.ChangeFrequency(15)
        # distances
        self.distances = [0 for i in range(3)]
        # servos
        self.servo = 18
        wiringpi.wiringPiSetupGpio()
        wiringpi.pinMode(self.servo, wiringpi.GPIO.PWM_OUTPUT)
        wiringpi.pwmSetMode(wiringpi.GPIO.PWM_MODE_MS)
        wiringpi.pwmSetClock(192)
        self.pwm_range = 2000
        wiringpi.pwmSetRange(self.pwm_range)
        self.servo_pwm = int(self.pwm_range*0.055)
        wiringpi.pwmWrite(self.servo,self.servo_pwm) # 5.5% duty cycle

        self.in_routine = False
        self.routine = None
        self.sensor_data = {
            "left" :  0,
            "right" : 0,
            "speed" : self.speed,
            "inRoutine" : (self.in_routine == None),
        }

        self.last_right = 0
        self.last_left = 0
        self.servo_time_set = time.time()

        self.LED = 20
        GPIO.setup(  self.LED, GPIO.OUT )
        GPIO.output( self.LED, 1 )


        
    def set_direction(self,direction,hard=False):
        if self.direction == direction and not hard:
            return
        if direction == "left":
            dcycle = [0,1,1,0]
        if direction == "right":
            dcycle = [1,0,0,1]
        if direction == "back":
            dcycle = [0,1,0,1]
        if direction == "front":
            dcycle = [1,0,1,0]
        if direction == "steady":
            dcycle = [0,0,0,0]

        for pwm,i in zip(self.pwms,dcycle):
            if i == 0:
                pwm.ChangeDutyCycle(0)
            else:
                pwm.ChangeDutyCycle(self.speed)
        self.direction = direction


    def change_velocity(self,direction):
        if direction == "up":
            self.speed += 1
        else:
            self.speed -= 1

        if self.speed > 100:
            self.speed = 100
        if self.speed < 0:
            self.speed = 0
    def setDistance(self,i,d):
        self.distances[i] = d

    def setKeys(self,keys,sckt):
        if self.in_routine:
            if keys[u'buttons'][u'T']:
                self.kill_routine()
                self.in_routine = False
            else:
                return

        if keys[u'buttons'][u'ARROW_UP']:
            GPIO.output(6,0)
            GPIO.output(5,1)
        else if keys[u'buttons'][u'ARROW_DOWN']:
            GPIO.output(5,0)
            GPIO.output(6,1)

        return
        if keys[u'buttons'][u"O"]:
            if keys[u'arrows'][u'x'] == -1:
                self.start_routine("seesaw")
            if keys[u'arrows'][u'x'] == 1:
                self.start_routine("test")
            if keys[u'arrows'][u'y'] == 1:
                self.start_routine("straight_walls")
            if keys[u'arrows'][u'y'] == -1:
                self.start_routine("release_bolt")
            if keys[u'buttons'][u'L1']:
                self.start_routine("follow_wall_left")
            if keys[u'buttons'][u'R1']:
                self.start_routine("follow_wall_right")


            return
        if keys[u'buttons'][u'OPTION']:
            sckt.end()
            self.restart()
            sys.exit()

        if keys[u'buttons'][u'R1']:
            self.change_velocity('up')
        if keys[u'buttons'][u'L1']:
            self.change_velocity('down')
        if keys[u'arrows'][u'x'] == -1:
            self.set_direction('left')
        elif keys[u'arrows'][u'x'] == 1:
            self.set_direction('right')
        elif keys[u'arrows'][u'y'] == -1:
            self.set_direction('back')
        elif keys[u'arrows'][u'y'] == 1:
            self.set_direction('front')
        else:
            self.set_direction('steady')

        if keys[u'buttons'][u'START']:
            # system shutdown
            os.system("sudo shutdown -h now")

        self.setMovement(keys[u'joysticks'][u'left'][u'x'],keys[u'joysticks'][u'left'][u'y'])

        self.lockServo(keys[u'buttons'][u'X'])


        if keys[u'buttons'][u'R2']:
            self.setServo("up")
        elif keys[u'buttons'][u'L2']:
            self.setServo("down")

    def lockServo(self,lock):
        self.serv_lock = lock

    def kill_routine(self):
        if self.routine != None:
            self.in_routine = False
            self.routine.end()
            self.routine = None


    def start_routine(self,name):
        self.in_routine = True

        if name == "seesaw":
            self.routine = Routines.Seesaw(self)
        if name == "test":
            self.routine = Routines.Test(self)
        if name == "straight_walls":
            self.routine = Routines.StraightWalls(self)
        if name == "release_bolt":
            self.routine = Routines.ReleaseBolt(self)
        if name == "follow_wall_left":
            self.routine = Routines.FollowWallLeft(self)
        if name == "follow_wall_right":
            self.routine = Routines.FollowWallRight(self)
        self.routine.start()

    def restart(self):
        ps = subprocess.Popen("bash /home/pi/Desktop/robotZero/hard_updater.sh",shell=True)

    def setServo(self,direction):
        #if self.serv_lock == True:
        #    return
        #dcycle = val*8/2.0 + 2
        #dcycle = int((int(dcycle*3)/3.0)/100.0*self.pwm_range)

        #lastv = self.servo_pwm
        #if lastv != dcycle:
        #    wiringpi.pwmWrite(self.servo,dcycle)
        #    self.servo_pwm = dcycle
        if time.time() - self.servo_time_set < 0.1:
            return

        if direction == "up":
            self.servo_pwm += self.pwm_range*0.002
        if direction == "down":
            self.servo_pwm -= self.pwm_range*0.002

        if self.servo_pwm > self.pwm_range*0.12:
            self.servo_pwm = self.pwm_range*0.12
        if self.servo_pwm < self.pwm_range*0.02:
            self.servo_pwm = self.pwm_range*0.02
        wiringpi.pwmWrite(self.servo,int(self.servo_pwm))
        self.servo_time_set = time.time()

    def setLED(self,state):
        GPIO.output(self.LED,state)

    def connected(self,state):
        self.setLED(state)
        if not state:
            self.kill_routine()
            self.set_direction("steady")
            self.lockServo(True)

    def setMovement(self,x,y):
        y = int((y+1)*100 - 100)
        x = int((x+1)*100 - 100)

        left  = y - x
        right = y + x

        left =  ( 100 if left  >  100 else left)
        left =  (-100 if left  < -100 else left)
        right = (-100 if right < -100 else right)
        right = ( 100 if right >  100 else right)

        if left == self.last_left and right == self.last_right:
            return

        if left != self.last_left:
            self.last_left = left
            if left < 0:
                self.pwms[2].ChangeDutyCycle( 0 )
                self.pwms[3].ChangeDutyCycle( -left )
            else:
                self.pwms[3].ChangeDutyCycle( 0 )
                self.pwms[2].ChangeDutyCycle( left )

        if right != self.last_right:
            self.last_right = right
            if right < 0:
                self.pwms[0].ChangeDutyCycle( 0 )
                self.pwms[1].ChangeDutyCycle( -right )
            else:
                self.pwms[1].ChangeDutyCycle( 0 )
                self.pwms[0].ChangeDutyCycle( right )
                



class SocketListener(threading.Thread):
    def __init__(self,PORT,keys_manager):
        threading.Thread.__init__(self)
        #self.bcast = d_bcast
        self.km = keys_manager
        try: 
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except socket.error:
            print 'Failed to create socket'
            sys.exit()
        try:
            s.bind(("", PORT)) # as server
        except socket.error , msg:
            print 'Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
            sys.exit()
        print 'Socket bind complete'
        self.sckt = s
        #self.bcast.set_socket(s)


    def run(self):
        while True:
            msg, addr = self.sckt.recvfrom(CHUNK_SIZE)
            print msg
            #self.bcast.addIP(addr)
            try:
                json_keys = json.loads(msg)
                self.km.setKeys(json_keys,self)
            except:
                pass

    def end(self):
        self.sckt.close()
            
class VideoBroadcast(threading.Thread):
    def __init__(self,camera,lock,d_bcast):
        threading.Thread.__init__(self)
        self.cam = camera
        self.bcast = d_bcast
        self.lock = lock


    def run(self):
        while True:
            time.sleep(1.0/30.0)
            chunks = self.cam.get_image_slides() # video
            random.shuffle(chunks)
            self.lock.acquire()
            for chunk in chunks:
                self.bcast.sendData(chunk,'video')
            self.lock.release()

    def set_socket(self,skt):
        self.sckt = skt


class DataBroadcast(object):
    def __init__(self,limit=2):
        self.ips = {}
        self.limit = limit
        self.connected = True
        self.check = True


    def addIP(self,addr):
        ip = str(addr[0]) + ":" + str(addr[1])
        if not (ip in self.ips.keys()):
            if len(self.ips) == self.limit:
                self.ips.pop(ip,None)
            self.ips[ip] = addr

    def sendData(self,msg,msg_type):
        flag = ''
        if msg_type == 'video':
            flag = chr(0) + chr(0)
        if msg_type == 'sensor':
            flag = chr(0) + chr(1)

        if self.check:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.connect(("www.python.org", 80))
                s.close()
            except:
                self.connected = False

        for ip in self.ips.keys():
            try:
                self.sckt.sendto( flag + msg , self.ips[ip])
                self.connected = True
            except socket.error, (no,msg):
                if no == socket.errno.ENETUNREACH:
                    self.connected = False
    def set_socket(self,skt):
        self.sckt = skt


#error_file = open('error.log','w')
#sys.stderr = error_file
lock = threading.Lock()
key_m = Agent()
#key_m = KeyMTest()
#data_broadcast = DataBroadcast(limit=1)
#video_broad = VideoBroadcast(camera.VideoCamera(),lock,data_broadcast)
skt_manager = SocketListener(8000,key_m)

skt_manager.start()
#video_broad.start()


while True:
    time.sleep(1/30.0)
    '''sensor_data = {
        "left" :  distanceSensors.get(0),
        "right" : distanceSensors.get(1),
        "front" : distanceSensors.get(2),
        "speed" : key_m.speed,
        "inRoutine" : key_m.in_routine,
        "claw" : (key_m.servo_pwm/key_m.pwm_range*100-2)/10*100,
    }'''
    #key_m.sensor_data = sensor_data
    #lock.acquire()
    #data_broadcast.sendData(json.dumps(sensor_data),'sensor')
    #lock.release()

    key_m.connected(data_broadcast.connected)

    #print sensor_data


skt_manager.join()
#video_broad.join()