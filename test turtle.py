import turtle

print(turtle.position())
turtle.sety(30)
print(turtle.position())
turtle.sety(-50)
print(turtle.ycor())

turtle.left(180)

p1 = turtle.Turtle()
p1.color('green')
p1.shape('turtle')
p1.penup()
p1.goto(-200, 100)

p2 = p1.clone()
p2.color('blue')
p2.penup()
p2.goto(-200, -100)

p1.goto(300, 100)
p1.pendown()
p1.circle(40)
p1.penup()
p1.goto(-200, 100)

p2.goto(300, -100)
p2.pendown()
p2.circle(40)
p2.penup()
p2.goto(-200, -100)

turtle.done()
