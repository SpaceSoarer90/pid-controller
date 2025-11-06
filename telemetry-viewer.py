from enum import IntEnum
import matplotlib
import matplotlib.pyplot as plt
import serial
import time

# this is just an enum class
class DataColumn(IntEnum):
    TIMESTAMP = 0

    SETPOINT = 1
    CURRPOINT = 2
    ERROR = 3
    OUTPUT = 4

    RAW_OUTPUT = 5
    P_TERM = 6
    I_TERM = 7
    D_TERM = 8
    KP = 9
    KI = 10
    KD = 11
    DT = 12

    PWM_OUTPUT = 13
    SAMPLING_INTERVAL = 14
    CALCULATION_TIME = 15
    SATURATED = 16

MIN_Y_AXIS = -300
MAX_Y_AXIS = 300

plt.ion()

fig, ax = plt.subplots()
data = {
    'timestamp': [],
    'setpoint': [],
    'currpoint': [],
    'error': [],
    'output': []
}
# create two 2d plots with their datas empty initially
# make the color of the first blue and the second green
plots = ax.plot([], [], 'b-', [], [], 'g-', [], [], 'r-', [], [], 'y-')

ax.set_xlabel('Time (s)')
ax.set_ylabel('RPM')
ax.set_title('Setpoint, Currpoint, Error, Output')
ax.set_ylim(MIN_Y_AXIS, MAX_Y_AXIS)
# ax.set_autoscale_on(False)
matplotlib.use("Qt5agg")

device = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)

time.sleep(2)

print("sending to device!")
device.write(b'r')
time.sleep(0.1) #yes, i know it sucks... bear with me pls
device.write(b'200!')

update_plot_time = 1.0 #default value

plots[0].set_zorder(len(plots))

while True:
    if device.in_waiting > 0:
        raw_data_line = device.readline().decode('ascii').strip()

        if raw_data_line[0] != '#':
            print(raw_data_line)
            continue

        data_frame = raw_data_line.lstrip('# ').split(',')
        data_frame = [float(data) for data in data_frame]

        # print(data_frame[DataColumn.SETPOINT],
        #       data_frame[DataColumn.CURRPOINT],
        #       data_frame[DataColumn.ERROR],
        #       data_frame[DataColumn.OUTPUT])
        print(data_frame[-1]) # print the delta pulses

        data['timestamp'].append(data_frame[DataColumn.TIMESTAMP])
        data['setpoint'].append(data_frame[DataColumn.SETPOINT])
        data['currpoint'].append(data_frame[DataColumn.CURRPOINT])
        data['error'].append(data_frame[DataColumn.ERROR])
        data['output'].append(data_frame[DataColumn.OUTPUT])
        update_plot_time = data_frame[DataColumn.SAMPLING_INTERVAL]

        # i need to change this
        for plot in plots:
            plot.set_xdata(data['timestamp'])

        plots[0].set_ydata(data['setpoint'])
        plots[1].set_ydata(data['currpoint'])
        plots[2].set_ydata(data['error'])
        plots[3].set_ydata(data['output'])


        ax.relim()
        ax.autoscale_view()

        # no need for plt.pause since this only runs when there's new data

    fig.canvas.draw_idle()
    fig.canvas.start_event_loop(update_plot_time / 2.0)

    pass
