# ve_truc.py
import turtle

def setup_axes():
    # Tạo một cửa sổ turtle
    window = turtle.Screen()
    window.title("Robot Simulation")
    axes = turtle.Turtle()
    axes.speed(0)  # Tăng tốc độ vẽ

    # Vẽ trục x
    axes.penup()
    axes.goto(-300, -300)
    axes.pendown()
    axes.forward(400)  # Vẽ trục x với 8 đơn vị (mỗi đơn vị 100)

    for i in range(-3, 2):  # Đánh dấu các đơn vị trên trục x
        axes.penup()
        axes.goto(i * 100, -315)
        axes.pendown()
        axes.write(str(i + 3), align="center", font=("Arial", 8, "normal"))

    # Vẽ trục phụ x
    axes.penup()
    axes.goto(-300, 500)
    axes.pendown()
    axes.forward(400)

    # Vẽ trục y
    axes.penup()
    axes.goto(-300, -300)
    axes.pendown()
    axes.left(90)
    axes.forward(800)  # Vẽ trục y với 8 đơn vị (mỗi đơn vị 100)

    for i in range(-2, 6):  # Đánh dấu các đơn vị trên trục y
        axes.penup()
        axes.goto(-310, i * 100)
        axes.pendown()
        axes.write(str(i + 3), align="center", font=("Arial", 8, "normal"))

    # Vẽ trục phụ y
    axes.penup()
    axes.goto(100, -300)
    axes.pendown()
    axes.forward(800)
    axes.penup()

    return axes, window

def rectangle(t, hor, ver):
    t.pendown()
    for _ in range(2):
        t.forward(hor)
        t.right(90)
        t.forward(ver)
        t.right(90)
    t.penup()

def draw_shapes(axes):
    # Vẽ viền
    axes.right(90)
    axes.goto(-295, 495)
    axes.pendown()
    rectangle(axes, 390, 790)
    axes.penup()

    axes.pendown()
    rectangle(axes, 60, 60)
    axes.penup()
    axes.goto(-290, 490)
    axes.pendown()
    rectangle(axes, 50, 50)
    axes.penup()

    axes.goto(-295, -235)
    axes.pendown()
    rectangle(axes, 60, 60)
    axes.goto(-290, -240)
    rectangle(axes, 50, 50)
    axes.penup()

    axes.right(180)
    axes.goto(95, -295)
    axes.pendown()
    rectangle(axes, 52, 52)
    rectangle(axes, 49, 49)
    axes.penup()

    axes.goto(95, 443)
    rectangle(axes, 52, 52)
    axes.goto(95, 446)
    rectangle(axes, 49, 49)
    axes.penup()

def draw_quadrants(axes):
    Ox = Oy = -300

    axes.left(90)
    axes.goto(Ox + 90, Oy + 130)
    axes.pendown()
    axes.circle(40, 90)
    axes.goto(35, -300 + 120)
    axes.forward(60)
    axes.penup()

    axes.goto(Ox + 90, Oy + 130)
    axes.left(90)

    axes.pendown()
    axes.goto(Ox + 125, Oy + 375)
    axes.forward(50)
    axes.goto(Ox + 90, Oy + 800 - 130)
    axes.penup()

    axes.goto(Ox + 130, Oy + 710)
    axes.pendown()
    axes.left(90)
    axes.circle(40, 90)
    axes.penup()
    axes.goto(Ox + 130, Oy + 710)

    axes.right(90)
    axes.pendown()
    axes.goto(100, 500 - 127.86)
    axes.penup()
    axes.hideturtle()

def main():
    axes, window = setup_axes()
    draw_shapes(axes)
    draw_quadrants(axes)

    # Ẩn rùa sau khi vẽ xong các trục

    # Chờ người dùng tắt cửa sổ
    window.mainloop()

if __name__ == "__main__":
    main()
