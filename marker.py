import turtle
import numpy as np


class Marker(object):
    def __init__(self):
        self.marker = turtle.Turtle()
        self.marker.speed(0)  # instant speed

    def linear(self, start, stop):
        # draw start line
        self.marker.goto(-100, start)
        self.marker.forward(200)
        self.marker.penup()
        # draw stop line
        self.marker.goto(-100, stop)
        self.marker.pendown()
        self.marker.forward(200)
        self.marker.penup()
        # go to start position
        self.marker.goto(0, 0)
        self.marker.shape('arrow')
        self.marker.color('red')

    def set_pos(self, pos):
        self.marker.goto(0, pos)

    def angular(self, start, stop):
        # draw start line
        radius = 200
        self.marker.left(start)
        self.marker.forward(radius)
        self.marker.penup()
        # draw end line
        self.marker.goto(0, 0)
        self.marker.setheading(0)
        self.marker.left(stop)
        self.marker.pendown()
        self.marker.forward(radius)
        self.marker.penup()
        # go to start position
        self.marker.goto(0, 0)
        self.marker.setheading(0)
        self.marker.shape('arrow')
        self.marker.color('red')

    def set_deg(self, rad):
        radius = 200
        deg = np.rad2deg(rad)
        self.marker.setheading(deg)
        self.marker.goto(radius*np.cos(rad), radius*np.sin(rad))

# rotate
# left/right degree, forward move
