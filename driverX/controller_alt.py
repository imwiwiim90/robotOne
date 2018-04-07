import pyglet
import time


class PS4Controller(object):
	def __init__(self):
		self.joystick = pyglet.input.get_joysticks()[0]

	def getKeys(self):
		j = self.joystick
		j.open()
		keys = {
			'joysticks' : {
				"left": {
					"x" : j.x,
					"y" : -j.y,
				},
				"right": {
					"x" : j.z,
					"y" : -j.rz,
				}
			},
			'arrows' : {
				'x' : j.hat_x,
				'y' : j.hat_y,
			},
			"back_buttons" : {
				"L" : j.rx+1,
                "R" : j.ry+1,
			},
			"buttons" : {
                "X" : j.buttons[1],
                "O" : j.buttons[2],
                "T" : j.buttons[3],
                "S" : j.buttons[0],
                "L1" : j.buttons[4],
                "R1" : j.buttons[5],
                "R2" : j.buttons[7],
                "R3" : j.buttons[11],
                "L2" : j.buttons[6],
                "PLAY" : j.buttons[12],
                "L3" : j.buttons[10],
                "OPTIONS": j.buttons[9],
                "SHARE": j.buttons[8],
                "PAD": j.buttons[13],
            },
		}

		j.close()
		return keys

class PS3Controller(object):
	def __init__(self):
		self.joystick = pyglet.input.get_joysticks()[0]

	def getKeys(self):
		j = self.joystick
		j.open()
		keys = {
			'joysticks' : {
				"left": {
					"x" : j.x,
					"y" : -j.y,
				},
				"right": {
					"x" : j.z,
					"y" : -j.rz,
				}
			},
			'arrows' : {
				'x' : j.hat_x,
				'y' : j.hat_y,
			},
			"back_buttons" : {
				"L" : j.rx+1,
                "R" : j.ry+1,
			},
			"buttons" : {
                "L3" : j.buttons[1],
                "R3" : j.buttons[2],
                "START" : j.buttons[3],
                "OPTION" : j.buttons[0],
                "ARROW_UP" : j.buttons[4],
                "ARROW_RIGHT" : j.buttons[5],
                "ARROW_LEFT" : j.buttons[7],
                "R1" : j.buttons[11],
                "ARROW_DOWN" : j.buttons[6],
                "T" : j.buttons[12],
                "L1" : j.buttons[10],
                "R2": j.buttons[9],
                "L2": j.buttons[8],
                "O": j.buttons[13],
            },
		}

		j.close()
		return keys
"""
p = PS4Controller()
while True:
	print p.getKeys()
"""