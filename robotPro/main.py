import socket
import sys
import time
import RPi.GPIO as GPIO
import wiringpi
import json
import threading
import random
import os
import Routines
import subprocess

CHUNK_SIZE = 4096


"""
5 -> forward
6 -> backward
17 -> left
4 -> right
18 -> hardware pwm / wrist

"""
class Agent(object):
    def __init__(self):
        GPIO.setmode(GPIO.BCM)

        # output pin definitions
        self.arm_up = 26
        self.arm_down = 13
        self.claw_open = 23
        self.claw_close = 24
        self.forward = 5
        self.backward = 6
        self.right = 4
        self.left = 17
        self.led = 18
        out_pins = [self.arm_up,self.arm_down,self.claw_open,self.claw_close,self.forward,self.backward,self.right,self.left,self.led]

        for pin in out_pins:
            GPIO.setup(pin,GPIO.OUT)

        self.forward_pwm  = GPIO.PWM(self.forward,15)
        self.backward_pwm = GPIO.PWM(self.backward,15)


        for pwm in [self.forward_pwm,self.backward_pwm]:
            pwm.start(0)
            pwm.ChangeFrequency(15)

        # hardware pwm
        '''
        self.hardware_pwm_pin = 18
        wiringpi.wiringPiSetupGpio()
        wiringpi.pinMode(self.hardware_pwm_pin, wiringpi.GPIO.PWM_OUTPUT)
        wiringpi.pwmSetMode(wiringpi.GPIO.PWM_MODE_MS)
        wiringpi.pwmSetClock(192)
        self.hardware_pwm_range = 2000
        wiringpi.pwmSetRange(self.hardware_pwm_range)
        self.hardware_pwm = int(self.hardware_pwm_range*0.055)
        wiringpi.pwmWrite(self.hardware_pwm_pin,self.hardware_pwm) # 5.5% duty cycle
        self.hardware_pwm_timeflag = time.time()
        '''

        # itermitent move
        self.itm_k = 100
        self.itm_checkpoint = time.time()
        self.itm_press = 0

        # connection cont
        self.led_cont = 100

        self.restarting = False

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


    ##########################################
    ################ NEW #####################
    ##########################################
    def setLed(self,status):
        on_off = (1 if status else 0)
        GPIO.output(self.led,on_off)

    def turn(self,direction):
        if direction == 'right':
            GPIO.output(self.left,0)
            GPIO.output(self.right,1)
        elif direction == 'left':
            GPIO.output(self.right,0)
            GPIO.output(self.left,1)
        else:
            GPIO.output(self.right,0)
            GPIO.output(self.left,0)

    # action = ('open'|'close')
    # open or close the claw
    def claw(self,action):
        if action == 'open':
            GPIO.output(self.claw_close,0)
            GPIO.output(self.claw_open,1)
        elif action == 'close':
            GPIO.output(self.claw_open,0)
            GPIO.output(self.claw_close,1)
        else:
            GPIO.output(self.claw_open,0)
            GPIO.output(self.claw_close,0)

    # action = ('up'|'dowm')
    # move up or down the claw
    def arm(self,action):
        if action == 'up':
            GPIO.output(self.arm_down,0)
            GPIO.output(self.arm_up,1)
        elif action == 'down':
            GPIO.output(self.arm_up,0)
            GPIO.output(self.arm_down,1)
        else:
            GPIO.output(self.arm_up,0)
            GPIO.output(self.arm_down,0)


    # forward or backward movement
    # k must be between -1 and 1
    def move(self,k):
        k = float(int(k*10)/10)*100
        if k >= 0:
            self.backward_pwm.ChangeDutyCycle(0)
            self.forward_pwm.ChangeDutyCycle(k)
        else:
            self.forward_pwm.ChangeDutyCycle(0)
            self.backward_pwm.ChangeDutyCycle(-k)

    # press should be -1,0,1 represening direction
    def intermitent_move(self,press):
        if (not abs(self.itm_press)) and abs(press):
            self.itm_checkpoint = time.time()
            self.move(press)
        if abs(self.itm_press) == 1 and press == 0:
            self.move(0)
        self.itm_press = press

    def set_itm_k(self,k):
        if k > 3000:
            self.itm_k = 3000
        elif k < 50:
            self.itm_k = 50
        else:
            self.itm_k = k

    def loop(self):
        # is in intermitent press
        if (abs(self.itm_press)):
            itm_time_delta = abs( self.itm_checkpoint - time.time() )*1000
            if ( itm_time_delta > self.itm_k ):
                self.move(0)
        # set led
        self.led_cont -= 1
        if self.led_cont < 0:
            self.setLed(False)
        else:
            self.setLed(True)

    '''
    def setHardwarePWM(self,direction):
        if time.time() - self.hardware_pwm_timeflag < 0.1:
            return

        if direction == "left":
            self.hardware_pwm += self.hardware_pwm_range*0.002
        if direction == "right":
            self.hardware_pwm -= self.hardware_pwm_range*0.002

        if self.hardware_pwm > self.hardware_pwm_range*0.12:
            self.hardware_pwm = self.hardware_pwm_range*0.12
        if self.hardware_pwm < self.hardware_pwm_range*0.02:
            self.hardware_pwm = self.hardware_pwm_range*0.02
        wiringpi.pwmWrite(self.hardware_pwm_pin,int(self.hardware_pwm))
        self.hardware_pwm_timeflag = time.time()
    '''

    def setKeys(self,keys,sckt):
        if self.restarting:
            return

        #### connection cont ####
        self.led_cont = 100

        #### forward/backward move ####
        # slow move
        if not keys[u'buttons'][u'ARROW_UP'] and not keys[u'buttons'][u'ARROW_DOWN']:
            self.move(keys[u'joysticks']['right']['y'])   
        # intermitent
        if keys[u'buttons'][u'ARROW_UP']:
            self.intermitent_move( 1)
        elif keys[u'buttons'][u'ARROW_DOWN']:
            self.intermitent_move(-1)
        else:
            self.intermitent_move( 0)

        if keys[u'buttons'][u'R1']:
            self.set_itm_k(self.itm_k + 50)
        elif keys[u'buttons'][u'L1']:
            self.set_itm_k(self.itm_k - 50)
        
        #### direction ####
        if keys[u'buttons'][u'ARROW_LEFT']:
            self.turn('left')
        elif keys[u'buttons'][u'ARROW_RIGHT']:
            self.turn('right')
        else:
            self.turn(None)

        #### arm ####
        # wrist
        '''
        if keys[u'buttons'][u'R3']:
            if keys[u'buttons'][u'R2']:
                self.setHardwarePWM('right')
            elif keys[u'buttons'][u'L2']:
                self.setHardwarePWM('left')
        '''
        # claw
        if keys[u'buttons'][u'L3']:
            if keys[u'buttons'][u'R2']:
                self.claw('close')
            elif keys[u'buttons'][u'L2']:
                self.claw('open')
            else:
                self.claw(None)
        # up/down arm
        else:
            if keys[u'buttons'][u'R2']:
                self.arm('down')
            elif keys[u'buttons'][u'L2']:
                self.arm('up')
            else:
                self.arm(None)


        
        if keys[u'buttons'][u'OPTION']:
            sckt.end()
            self.restart()

        if keys[u'buttons'][u'START']:
            self.shutdown()


        return

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
        self.restarting = True
        GPIO.cleanup()
        ps = subprocess.Popen("bash /home/pi/Desktop/robotOne/hard_updater.sh",shell=True)

    def shutdown(self):
        ps = subprocess.Popen('sudo halt',shell=True)




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
        self.server_addr = ('198.211.112.16',8001)
        threading.Thread.__init__(self)
        #self.bcast = d_bcast
        self.km = keys_manager

        # create socket
        try: 
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except socket.error:
            print 'Failed to create socket'
            sys.exit()

        # socket listen
        try:
            s.bind(("",0)) # as server
        except socket.error , msg:
            print 'Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
            sys.exit()
        print 'Socket bind complete'

        self.sckt = s
        self.driver_addr = None
        #self.bcast.set_socket(s)

    def request_driver_addr(self):
        sckt = self.sckt
        try:
            sckt.settimeout(3.0)
            sckt.sendto('ROBOT|PEER',self.server_addr)
            msg, addr = sckt.recvfrom(CHUNK_SIZE)
            if msg == 'null':
                self.driver_addr = None
                return
            host,port = msg.split(':')
            self.driver_addr = (host,int(port))
        except:
            self.driver_addr = None

    def ping_server(self):
        self.sckt.sendto('ROBOT|PING',self.server_addr)

    def run(self):
        last_server_time = 0
        server_ping_time = 5
        while True:
            # driver has not connected
            if self.driver_addr == None:
                self.request_driver_addr()
                last_server_time = time.time()
                time.sleep(3)
                continue

            if time.time() - last_server_time > server_ping_time:
                self.ping_server()
                last_server_time = time.time()


            # send drivers msg
            self.sckt.sendto(self.msg_out,self.driver_addr)

            try:
                msg, addr = self.sckt.recvfrom(CHUNK_SIZE)
            except:
                msg = None

            try:
                json_keys = json.loads(msg)
                self.km.setKeys(json_keys,self)
            except Exception as e:
                print 'invalid ingoing keys'

    def set_msg(self,msg):
        self.msg_out = msg

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
    key_m.loop()
    sensor_data = {
        "itm_k" :  key_m.itm_k,
    }
    skt_manager.set_msg(json.dumps(sensor_data))
    #key_m.sensor_data = sensor_data
    #lock.acquire()
    #data_broadcast.sendData(json.dumps(sensor_data),'sensor')
    #lock.release()

    #key_m.connected(data_broadcast.connected)



skt_manager.join()
#video_broad.join()