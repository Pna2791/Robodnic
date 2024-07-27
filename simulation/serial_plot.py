import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random
from collections import deque

count = 0
window_size = 200
# Initialize the figure and axis
fig, ax = plt.subplots(figsize=(20,5))
x_data = list(range(window_size))
y_data_1 = deque(maxlen=200)
y_data_2 = deque(maxlen=200)
ln1, = plt.plot([], [], 'r-', label='pos_1')
ln2, = plt.plot([], [], 'b-', label='pos_2')

# Setting up the plot limits and labels
ax.set_xlim(0, window_size)  # Show 200 samples
ax.set_ylim(-100, 200)  # Adjust this based on the expected range of pos_1 and pos_2
ax.set_xlabel('Sample')
ax.set_ylabel('Value')
ax.legend()

def init():
    ln1.set_data([], [])
    ln2.set_data([], [])
    return ln1, ln2

def update(frame):
    global count
    count += 1
    # Simulate receiving new data every 0.05 seconds (20 samples per second)
    new_data = {'pos_1': random.randint(-100, 100), 'pos_2': random.randint(-100, 100)+100}
    

    y_data_1.append(new_data['pos_1'])
    y_data_2.append(new_data['pos_2'])
    len_ = len(y_data_1)
    # print(x_data, y_data_1, y_data_2)
    
    # Update plot data
    ln1.set_data(x_data[:len_], y_data_1)
    ln2.set_data(x_data[:len_], y_data_2)
    
    return ln1, ln2

# Create an animation
ani = animation.FuncAnimation(fig, update, init_func=init, blit=True, interval=50)
plt.tight_layout()
# Show the plot
plt.show()
