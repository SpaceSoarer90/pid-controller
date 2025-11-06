import matplotlib.pyplot as plt
import time
import random

plt.ion()  # interactive mode on

fig, ax = plt.subplots()
xdata, ydata = [], []
(line,) = ax.plot([], [], 'b-')

ax.set_xlabel('Time (s)')
ax.set_ylabel('Value')
ax.set_title('Live Updating Plot')

start_time = time.time()

while True:
    # Simulate new data every second
    t = time.time() - start_time
    value = random.uniform(0, 10)  # dummy variable

    xdata.append(t)
    ydata.append(value)

    # Keep only recent 20 points (optional)
    if len(xdata) > 20:
        xdata = xdata[-20:]
        ydata = ydata[-20:]

    # Update the line
    line.set_xdata(xdata)
    line.set_ydata(ydata)
    ax.relim()
    ax.autoscale_view()

    plt.draw()
    plt.pause(1.0)  # update every 1 second

