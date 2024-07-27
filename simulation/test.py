import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random
import time

# Initialize the figure and axis
fig, ax = plt.subplots()
line, = ax.plot([], [], lw=2)
ax.set_ylim(-100, 100)
ax.set_xlim(0, 200)  # Keep a fixed width for the x-axis for better visualization
xdata, ydata = [], []

# Function to initialize the animation
def init():
    line.set_data([], [])
    return line,

# Function to update the data for the animation
def update(frame):
    xdata.append(frame)
    ydata.append(random.randint(-100, 100))  # Append random data between -100 and 100

    # Only keep the latest 50 points for visualization
    if len(xdata) > 50:
        xdata.pop(0)
        ydata.pop(0)
    
    
    line.set_data(, ydata)
    return line,

# Create the animation
ani = animation.FuncAnimation(fig, update, frames=range(100), init_func=init, blit=True, interval=40)

# Display the animation
plt.show()
