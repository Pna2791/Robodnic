# main.py
import turtle
import san
import time

start_x = start_y = -265
robot = turtle.Turtle()
fire = turtle.Turtle()
wheels = turtle.Turtle()
robot.color('blue')
robot.penup()
fire.color('red')
fire.penup()
wheels.color('green')
wheels.pensize(3)
wheels.speed('slowest')
robot.speed('slowest')
fire.speed('slowest')
text = turtle.Turtle()
text.pensize(5)
text.color("gray")
text.speed(0)
# robot.speed(0)
# fire.speed(0)

def note():
    text.penup()
    text.goto(-150, -350)
    text.pendown()
    text.color("red")
    text.write(str("Red dir: fire"),  align="center", font=("Arial", 8, "normal"))
    text.penup()
    text.color("blue")
    text.goto(- 150, -375)
    text.write(str("Blue dir: robot"), align="center", font=("Arial", 8, "normal"))
    text.hideturtle()

def reset(t):
    t.penup()
    t.setheading(180)
    t.goto(start_x, start_y)
    fire.setheading(180)

def rotate(t, angle):
    t.pendown()
    if angle > 0:
        t.right(angle)
    if angle < 0:
        t.left(-angle)
    t.penup()
    
        

def move(t, distance):
    t.pendown()
    if distance > 0:
        t.forward(distance / 10)
    if distance <= 0:
        t.backward(-distance / 10)
    pos = t.position()
    x, y = pos
    fire.goto(x, y)
    robot.goto(x, y)    
    t.penup()

def goto(t, hor, ver):
    t.pendown()
    t.goto(hor, ver)
    pos = t.position()
    x, y = pos
    fire.goto(x, y)
    robot.goto(x, y)  
    t.penup()

def steps_robot():
    # Bắt đầu vẽ đường đi của robot
    wheels.hideturtle()
    move(wheels, -2805)
    time.sleep(5)
    
    rotate(wheels, 45)
    move(wheels, 400)
    rotate(robot, 90)
    rotate(fire, 90)
    time.sleep(5)
    
    position = wheels.position()
    x, y = position
    goto(wheels, start_x, y)
    goto(wheels, start_x, start_y)
    fire.setheading(45)
    time.sleep(5)
    
    wheels.setheading(45)
    move(wheels, 300)
    rotate(robot, 90)
    rotate(fire, 90)
    time.sleep(5)
    
    rotate(wheels, -45)
    position = wheels.position()
    x, y = position
    goto(wheels, x, start_y + 730)
    goto(wheels, start_x, start_y + 730)
    fire.setheading(-45)
    time.sleep(5)
    
    goto(wheels, start_x, start_y)    

def main():
    # Gọi các hàm từ san.py để vẽ trục và hình học
    axes, window = san.setup_axes()
    san.draw_shapes(axes)
    san.draw_quadrants(axes)

    note()
    reset(robot)
    reset(fire)
    reset(wheels)
    steps_robot()
    
    
    # Chờ người dùng tắt cửa sổ
    window.mainloop()

if __name__ == "__main__":
    main()
