import turtle
import numpy as np


class Marker(object):
    def __init__(self):
        self.marker = turtle.Turtle()
        self.marker.speed(0)  # instant speed
        self.radius = 200
        self.length = 200

    def linear(self, start, stop):
        # draw start line
        self.marker.goto(-100, start)
        self.marker.forward(self.length)
        self.marker.penup()
        # draw stop line
        self.marker.goto(-100, stop)
        self.marker.pendown()
        self.marker.forward(self.length)
        self.marker.penup()
        # go to start position
        self.marker.goto(0, 0)
        self.marker.shape('arrow')
        self.marker.color('red')

    def set_pos(self, pos):
        self.marker.goto(0, pos)

    def angular(self, start, stop):
        # draw start line
        self.marker.left(start)
        self.marker.forward(self.radius)
        self.marker.penup()
        # draw end line
        self.marker.goto(0, 0)
        self.marker.setheading(0)
        self.marker.left(stop)
        self.marker.pendown()
        self.marker.forward(self.radius)
        self.marker.penup()
        # go to start position
        self.marker.goto(0, 0)
        self.marker.setheading(0)
        self.marker.shape('arrow')
        self.marker.color('red')

    def set_deg(self, rad):
        deg = np.rad2deg(rad)
        self.marker.setheading(deg)
        self.marker.goto(self.radius*np.cos(rad), self.radius*np.sin(rad))
