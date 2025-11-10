import csv
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

plt.ion()

DATAPLOT = 0
METERTEXT = 1

SYSTEMPLOT = 0
TERMPLOT = 1

METER = 0
TEXT = 1

UNSCALED = 0
SCALED = 1

fig = plt.figure(layout='constrained')
subfig = fig.subfigures(1, 2, width_ratios=[5,1])
dataplot_fig = subfig[DATAPLOT].subfigures(2, 1, height_ratios=[1,1])
metertext_fig = subfig[METERTEXT].subfigures(2, 1, height_ratios=[2,1])

########################################################
dataplot_fig[SYSTEMPLOT].suptitle('System Plot')
dataplot_fig[TERMPLOT].suptitle('PID Terms Plot')

sys_ax = dataplot_fig[SYSTEMPLOT].subplots(1, 1)
sys_ax.set_title('Setpoint, Currpoint, Error, Output')
# sys_ax.legend()
sys_plots = sys_ax.plot([], [], 'b-', [], [], 'g-', [], [], 'r-', [], [], 'y-')

term_ax = dataplot_fig[TERMPLOT].subplots(1, 2)
term_ax[UNSCALED].set_title('Unscaled')
term_ax[SCALED].set_title('Scaled')
# term_ax[UNSCALED].legend()
# term_ax[SCALED].legend()
unscaled_term_plots = term_ax[UNSCALED].plot([], [], 'b-', [], [], 'r-', [], [], 'g-')
scaled_term_plots = term_ax[SCALED].plot([], [], 'b-', [], [], 'r-', [], [], 'g-')
########################################################

########################################################
OUTPUT = 0
PWM_OUTPUT = 1

MIN_OUTPUT = 0.0
MAX_OUTPUT = 300.0

MIN_PWM_OUTPUT = 0
MAX_PWM_OUTPUT = 255

meter_data = {
    'output': 20.0,
    'pwm_output': 255
}
meter_data_tuple = tuple(meter_data.values())

metertext_fig[METER].suptitle('Output Meters')
metertext_fig[TEXT].suptitle('Textual Telemetry')

outmeter = metertext_fig[METER].subplots(1, 2)
outmeter[OUTPUT].bar('OUTPUT', meter_data['output'])
outmeter[OUTPUT].set_ylim(MIN_OUTPUT, MAX_OUTPUT)

outmeter[PWM_OUTPUT].bar('PWM OUTPUT', meter_data['pwm_output'])
outmeter[PWM_OUTPUT].set_ylim(MIN_PWM_OUTPUT, MAX_PWM_OUTPUT)

########################################################

sys_data = []
with open('pid_dummy_data.csv', newline='') as csvfile:
    data = csv.reader(csvfile, delimiter=',', quotechar='|')
    for line in data:
        line = [float(val) for val in line]
        sys_data.append(line)

TIMESTAMP = 0
SETPOINT = 1
CURRPOINT = 2
ERROR = 3
OUTPUT = 4

timestamp = []
data = {
    'setpoint': [],
    'currpoint': [],
    'error': [],
    'output': [],
}

timestep = 0
while timestep < len(sys_data):
    timestamp.append(sys_data[timestep][TIMESTAMP])
    data['setpoint'].append(sys_data[timestep][SETPOINT])
    data['currpoint'].append(sys_data[timestep][CURRPOINT])
    data['error'].append(sys_data[timestep][ERROR])
    data['output'].append(sys_data[timestep][OUTPUT])

    for plot in sys_plots:
        plot.set_xdata(timestamp)

    sys_plots[0].set_ydata(data['setpoint'])
    sys_plots[1].set_ydata(data['currpoint'])
    sys_plots[2].set_ydata(data['error'])
    sys_plots[3].set_ydata(data['output'])

    sys_ax.relim()
    sys_ax.autoscale_view()

    delta_timestamp = sys_data[timestep][TIMESTAMP] - sys_data[timestep - 1][TIMESTAMP] if timestep > 0 else 0.1
    delta_timestamp = round(delta_timestamp, ndigits=1)
    print(round(delta_timestamp, ndigits=1))

    fig.canvas.draw_idle()
    fig.canvas.start_event_loop(delta_timestamp)

    timestep += 1

plt.show()
plt.pause(10.0)
