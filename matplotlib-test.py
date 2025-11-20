import matplotlib.pyplot as plt
import serial
import time
from enum import IntEnum

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
subfig = fig.subfigures(1, 2, width_ratios=[5, 1])
dataplot_fig = subfig[DATAPLOT].subfigures(2, 1, height_ratios=[1, 1])
metertext_fig = subfig[METERTEXT].subfigures(2, 1, height_ratios=[2, 1])

########################################################
dataplot_fig[SYSTEMPLOT].suptitle('System Plot')
dataplot_fig[TERMPLOT].suptitle('PID Terms Plot')

sys_ax = dataplot_fig[SYSTEMPLOT].subplots(1, 1)
term_ax = dataplot_fig[TERMPLOT].subplots(1, 2)

sys_plots = sys_ax.plot([], [], 'b-', [], [], 'g-', [], [], 'r-', [], [], 'y-')
unscaled_term_plots = term_ax[UNSCALED].plot([], [], 'b-', [], [], 'r-', [], [], 'g-')
scaled_term_plots = term_ax[SCALED].plot([], [], 'b-', [], [], 'r-', [], [], 'g-')

sys_ax.set_title('Setpoint, Currpoint, Error, Output')
term_ax[UNSCALED].set_title('Unscaled')
term_ax[SCALED].set_title('Scaled')

MIN_OUTPUT, MAX_OUTPUT = 0, 400
sys_ax.set_xlabel('Time (s)')
sys_ax.set_ylabel('Speed (RPM)')
sys_ax.set_ylim(MIN_OUTPUT - 50, MAX_OUTPUT + 50)

sys_ax.legend(['Setpoint', 'Currpoint', 'Error', 'Output'])
term_ax[UNSCALED].legend(['Proportional', 'Integral', 'Derivative'])
term_ax[SCALED].legend(['Proportional', 'Integral', 'Derivative'])
########################################################

########################################################
OUTPUT = 0
PWM_OUTPUT = 1

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
bar1 = outmeter[OUTPUT].bar('OUTPUT', meter_data['output'])[0]
outmeter[OUTPUT].set_ylim(MIN_OUTPUT, MAX_OUTPUT)

bar2 = outmeter[PWM_OUTPUT].bar('PWM OUTPUT', meter_data['pwm_output'])[0]
outmeter[PWM_OUTPUT].set_ylim(MIN_PWM_OUTPUT, MAX_PWM_OUTPUT)

outmeters = [bar1, bar2]

########################################################

device = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2.0)

print('Sending RPM Command to device!')
device.write(b'r')
time.sleep(0.1)
device.write(b'200!')

########################################################
class DataColumn(IntEnum):
    TIMESTAMP = 0

    SETPOINT = 1
    CURRPOINT = 2
    ERROR = 3
    OUTPUT = 4

    RAW_OUTPUT = 5
    PROPORTIONAL = 6
    INTEGRAL = 7
    DERIVATIVE = 8
    P_TERM = 9
    I_TERM = 10
    D_TERM = 11
    KP = 12
    KI = 13
    KD = 14
    DT = 15

    PWM_OUTPUT = 16
    SAMPLING_INTERVAL = 17
    CALCULATION_TIME = 18
    SATURATED = 19

########################################################

data = {
    'timestamp': [],
    'setpoint': [],
    'currpoint': [],
    'error': [],
    'output': [],

    'proportional': [],
    'integral': [],
    'derivative': [],
    'p_term': [],
    'i_term': [],
    'd_term': [],
    'kp': 0,
    'ki': 0,
    'kd': 0,

    'pwm_output': []
}

update_plot_time = 1.0

while True:
    if device.in_waiting > 0:
        raw_data_line = device.readline().decode('ascii').strip()

        if raw_data_line[0] != '#':
            print(f"not data: {raw_data_line}")
            continue
        
        data_frame = raw_data_line.lstrip('# ').split(',')
        data_frame = [float(data) for data in data_frame]

        data['timestamp'].append(data_frame[DataColumn.TIMESTAMP])
        data['setpoint'].append(data_frame[DataColumn.SETPOINT])
        data['currpoint'].append(data_frame[DataColumn.CURRPOINT])
        data['error'].append(data_frame[DataColumn.ERROR])
        data['output'].append(data_frame[DataColumn.OUTPUT])

        data['proportional'].append(data_frame[DataColumn.PROPORTIONAL])
        data['integral'].append(data_frame[DataColumn.INTEGRAL])
        data['derivative'].append(data_frame[DataColumn.DERIVATIVE])
        data['p_term'].append(data_frame[DataColumn.P_TERM])
        data['i_term'].append(data_frame[DataColumn.I_TERM])
        data['d_term'].append(data_frame[DataColumn.D_TERM])

        data['kp'] = data_frame[DataColumn.KP]
        data['ki'] = data_frame[DataColumn.KI]
        data['kd'] = data_frame[DataColumn.KD]
        data['pwm_output'].append(data_frame[DataColumn.PWM_OUTPUT])

        plot_list_list = [sys_plots[:], unscaled_term_plots[:], scaled_term_plots[:]]

        for plot_list in plot_list_list:
            for plot in plot_list:
                plot.set_xdata(data['timestamp'])

        sys_plots[0].set_ydata(data['setpoint'])
        sys_plots[1].set_ydata(data['currpoint'])
        sys_plots[2].set_ydata(data['error'])
        sys_plots[3].set_ydata(data['output'])

        unscaled_term_plots[0].set_ydata(data['proportional'])
        unscaled_term_plots[1].set_ydata(data['integral'])
        unscaled_term_plots[2].set_ydata(data['derivative'])

        scaled_term_plots[0].set_ydata(data['p_term'])
        scaled_term_plots[1].set_ydata(data['i_term'])
        scaled_term_plots[2].set_ydata(data['d_term'])

        outmeters[OUTPUT].set_height(data['output'][-1])
        outmeters[PWM_OUTPUT].set_height(data['pwm_output'][-1])

        update_plot_time = data_frame[DataColumn.SAMPLING_INTERVAL]

        sys_ax.relim()
        sys_ax.autoscale_view()

        term_ax[UNSCALED].relim()
        term_ax[UNSCALED].autoscale_view()
        term_ax[SCALED].relim()
        term_ax[SCALED].autoscale_view()

    fig.canvas.draw_idle()
    fig.canvas.start_event_loop(update_plot_time / 2.0)
