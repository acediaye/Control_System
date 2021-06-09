import turtle


class Marker(object):
    def __init__(self, start, stop):
        self.marker = turtle.Turtle()
        self.marker.speed(0)  # instant speed
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

# rotate
# left/right degree, forward move
