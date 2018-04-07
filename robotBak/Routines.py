import threading
import time

class Seesaw(threading.Thread):
    def __init__( self,agent ):
        threading.Thread.__init__(self)
        self.agent = agent
        self.terminate = False

    def run(self):
        agent = self.agent
        straight_time = 1.6
        time_start  = time.time()
        speed_aux   = agent.speed
        agent.speed = 100
        agent.set_direction("front")
        while True:
            if self.terminate:
                break
            if time.time() - time_start > straight_time:
                break
            time.sleep(0.01)
        agent.speed = speed_aux
        agent.set_direction("steady")
        self.end()

    def end(self):
        self.terminate = True


class Test(threading.Thread):
    def __init__( self,agent ):
        threading.Thread.__init__(self)
        self.agent = agent
        self.terminate = False

    def run(self):
        while True:
            time.sleep(0.5)
            print "in routine"
            if self.terminate:
                break

    def end(self):
        self.terminate = True
class ReleaseBolt(threading.Thread):
    def __init__( self,agent ):
        threading.Thread.__init__(self)
        self.agent = agent
        self.terminate = False

    def run(self):
        agent = self.agent
        straight_time = 0.12
        time_start  = time.time()
        speed_aux   = agent.speed
        agent.speed = 100
        agent.set_direction("back")
        while True:
            if self.terminate:
                break
            if time.time() - time_start > straight_time:
                break
            time.sleep(0.01)
        agent.speed = speed_aux
        agent.set_direction("steady")
        self.end()

    def end(self):
        self.terminate = True


class FollowWallLeft(threading.Thread):
    def __init__( self,agent ):
        threading.Thread.__init__(self)
        self.agent = agent
        self.terminate = False

    def run(self):
        agent = self.agent
        speed_aux   = agent.speed
        distance = agent.distances[0]
        while True:
            if self.terminate:
                break
            agent.setMovement((agent.distances[0]-distance)*0.02,0.15)
            time.sleep(0.01)
        agent.speed = speed_aux
        agent.set_direction("steady")
        self.end()

    def end(self):
        self.terminate = True

class FollowWallRight(threading.Thread):
    def __init__( self,agent ):
        threading.Thread.__init__(self)
        self.agent = agent
        self.terminate = False

    def run(self):
        agent = self.agent
        speed_aux   = agent.speed
        distance = agent.distances[1]
        while True:
            if self.terminate:
                break
            agent.setMovement((distance-agent.distances[1])*0.02,0.15)
            time.sleep(0.01)
        agent.speed = speed_aux
        agent.set_direction("steady")
        self.end()

    def end(self):
        self.terminate = True

class StraightWalls(threading.Thread):
    def __init__( self,agent ):
        threading.Thread.__init__(self)
        self.agent = agent
        self.terminate = False

    def run(self):
        agent = self.agent
        speed_aux   = agent.speed
        agent.setMovement(0,0.5)
        turn_state = "left"
        while True:
            if self.terminate:
                break
            agent.setMovement((agent.distances[0]-agent.distances[1])*0.02 - 0.01,1)
            time.sleep(0.01)
        agent.speed = speed_aux
        agent.set_direction("steady")
        self.end()

    def end(self):
        self.terminate = True
