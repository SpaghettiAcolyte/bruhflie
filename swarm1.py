import multiprocessing
import threading
import time
from datetime import datetime
from pathlib import Path
from time import sleep

import cflib
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory, Swarm
from cflib.crazyflie.syncLogger import SyncLogger

from matplotlib import animation
from matplotlib import pyplot as plt
import matplotlib._color_data as mcd
from random import shuffle
import csv
from random import choice

# Change uris and sequences according to your setup
URI1 = 'radio://0/80/2M/E7E7E7E7E5'
#URI2 = 'radio://0/80/2M/E7E7E7E7E1'
#URI3 = 'radio://0/80/2M/E7E7E7E7E5'
#URI4 = 'radio://0/80/2M/E7E7E7E7E1'

sequence1 = [
    (1.1, 1.1, 0.0, 3.0),
    (1.7, 1.1, 0.5, 3.0),
    (2.0, 1.1, 0.0, 3.0),
]

seq_args = {
    URI1: [sequence1],
    #URI2: [sequence1],
    #URI3: [sequence3],
}

uris = [
    URI1,
    #URI2,
]


def init_log(path='/home/stranger/Content/Python/Crazyflie/Logs/'):
    date = (str(datetime.now()).split('.')[0]).split(' ')[0]
    t = str(datetime.now()).split('.')[0].split(' ')[1]
    Path(path + date).mkdir(parents=True, exist_ok=True)
    filename = path + date + '/position_logo_' + date + '_' + t + '.txt'
    log_file = open(filename, 'w')
    return log_file


log_file = init_log()


def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            print("{} {} {}".
                  format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break


def wait_for_param_download(scf):
    while not scf.cf.param.is_updated:
        time.sleep(1.0)
    print('Parameters downloaded for', scf.cf.link_uri)


def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)


def take_off(cf, position):
    take_off_time = 1.0
    sleep_time = 0.1
    steps = int(take_off_time / sleep_time)
    vz = position[2] / take_off_time

    # print(vz)

    for i in range(steps):
        cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(sleep_time)


def land(cf, position):
    landing_time = 1.0
    sleep_time = 0.1
    steps = int(landing_time / sleep_time)
    vz = -position[2] / landing_time

    # print(vz)

    for _ in range(steps):
        cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(sleep_time)

    cf.commander.send_stop_setpoint()
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)


def run_sequence(scf, sequence):
    #time.sleep(30.0)
    try:
        cf = scf.cf
        take_off(cf, sequence[0])
        for position in sequence:
            print('Setting position {}'.format(position))
            end_time = time.time() + position[3]
            while time.time() < end_time:
                cf.commander.send_position_setpoint(position[0],
                                                    position[1],
                                                    position[2], 0)
                time.sleep(0.1)
        land(cf, sequence[-1])
    except Exception as e:
        print(e)


def start_thread(target, *args):
    thread = threading.Thread(
        target=target,
        args=args
    )
    thread.daemon = True
    thread.start()
    return thread


def get_colors():
    colors = [name for name in mcd.XKCD_COLORS if name != 'white' or name != 'ivory']
    shuffle(colors)
    '''colors = ['8B0000', 'A52A2A', 'B22222', 'DC143C', 'FF0000',
              'FF7F50', 'CD5C5C', 'FF4500', 'FF8C00', 'FFD700',
              'B8860B', 'BDB76B', '808000', '9ACD32', '556B2F',
              '7CFC00', 'ADFF2F', '228B22', '00FF00', '8FBC8F',
              '00FF7F', '66CDAA', '3CB371', '20B2AA', '2F4F4F',
              '008B8B', '00CED1', '4682B4', '1E90FF', '191970',
              '000080', '0000FF', '8A2BE2', '4B0082', '483D8B',
              '8B008B', '9932CC', 'FF1493', '8B4513', 'D2691E',
              'CD853F', 'F4A460', '708090', '696969', '000000']'''
    for c in colors:
        yield c
    while True:
        yield 'black'


def real_time_plotting(data, ndrones):
    fig = plt.figure() # окно со всеми графиками
    axes = list() # подграфики
    lines_2d = [[] for _ in range(ndrones)]
    lines_3d = [[] for _ in range(ndrones)]

    # 2D подграфики и линии #
    axes.append(fig.add_subplot(2, 3, 1))
    axes.append(fig.add_subplot(2, 3, 2))
    axes.append(fig.add_subplot(2, 3, 3))
    colors = get_colors()
    drone_colors = [next(colors) for _ in range(ndrones)]
    for i in range(3):
        color = next(colors)
        if i > 0:
            axes[i].set_yticklabels([])
        axes[i].xaxis.label.set_color(color)
        axes[i].yaxis.label.set_color(color)
        axes[i].set(xlim=[-0.1, 3], ylim=[-0.1, 3])
        axes[i].grid()
        for j in range(ndrones):
            line = axes[i].plot([], [], lw=1, color=drone_colors[j])[0]
            lines_2d[j].append(line)

    # 3D подграфик и линии #
    axes.append(fig.add_subplot(2, 3, 5, projection='3d'))
    axes[-1].xaxis.label.set_color(color)
    axes[-1].yaxis.label.set_color(color)
    axes[-1].zaxis.label.set_color(color)
    axes[-1].set(xlim=[-0.1, 3], ylim=[-0.1, 3], zlim=[-0.1, 3])
    for j in range(ndrones):
        line = axes[-1].plot([], [], [], lw=1, color=drone_colors[j])[0]
        lines_3d[j].append(line)

    # "кастомизация" подграфиков
    axes[0].set(title='Top view', xlabel='X-Axis', ylabel='Y-Axis')
    axes[1].set(title='Side view', xlabel='X-Axis', ylabel='Z-Axis')
    axes[2].set(title='Front view', xlabel='Y-Axis', ylabel='Z-Axis')
    axes[3].set(title=None, xlabel='X', ylabel='Y', zlabel='Z')

    # задание размерности (тестировалось на экране с разрешением 1366x768)
    fig.subplots_adjust(
        top=0.95,
        bottom=0.035,
        left=0.1,
        right=0.9,
        hspace=0.135,
        wspace=0.120
    )

    # инициализация линий, которые затем будут обновляться
    def init():
        for drone in lines_2d:
            for line in drone:
                line.set_data([], [])
        for drone in lines_3d:
            for line in drone:
                line.set_data_3d([], [], [])
        return (*[l for drone in lines_2d for l in drone],
                *[l for drone in lines_3d for l in drone])

    xdata = dict() # [[] for _ in range(ndrones)]
    ydata = dict() # [[] for _ in range(ndrones)]
    zdata = dict() # [[] for _ in range(ndrones)]

    # вызывается при каждом callback-е позиции
    def animate(framedata):
        if xdata.get(framedata[0], None) is None:
            xdata[framedata[0]] = list()
        if ydata.get(framedata[0], None) is None:
            ydata[framedata[0]] = list()
        if zdata.get(framedata[0], None) is None:
            zdata[framedata[0]] = list()

        xdata[framedata[0]].append(framedata[1])
        ydata[framedata[0]].append(framedata[2])
        zdata[framedata[0]].append(framedata[3])

        a = 1
        for k, drone in enumerate(sorted(xdata.keys())):
            for line in lines_2d[k]:
                if a == 1:
                    line.set_data(xdata[drone], ydata[drone])
                elif a == 2:
                    line.set_data(xdata[drone], zdata[drone])
                elif a == 3:
                    a = 1
                    line.set_data(ydata[drone], zdata[drone])
                a += 1
            for line in lines_3d[k]:
                line.set_data_3d(xdata[drone], ydata[drone], zdata[drone])
        return (*[l for drone in lines_2d for l in drone],
                *[l for drone in lines_3d for l in drone])

    anim = animation.FuncAnimation(fig, animate, init_func=init,
                                   frames=data, interval=0, blit=True)
    plt.show()


position_data = multiprocessing.Queue() # list()


def yield_position(data):
    last_id_value = dict()
    while True:
        try:
            pos = data.get_nowait()
        except multiprocessing.queues.Empty:
            continue

        if last_id_value.get(pos[0], None) == pos[1:]:
            continue

        yield pos
        last_id_value[pos[0]] = pos[1:]


def position_callback(timestamp, data, logconf):
    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']

    # position_data.append([x, y, z])
    # print(logconf.cf.link_uri)
    position_data.put_nowait([logconf.cf.link_uri, round(x, 3), round(y, 3), round(z, 3)])
    print('{}: ({}, {}, {})'.format(logconf.cf.link_uri[-2:], x, y, z))
    log_file.write('{},{},{},{},{}\n'.format(logconf.cf.link_uri, datetime.utcnow().timestamp(), x, y, z))


def start_position_printing(scf):
    log_conf = LogConfig(name='Position', period_in_ms=100)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback)
    log_conf.start()


def battery_callback(timestamp, data, logconf):
    bat_level = data['pm.batteryLevel']
    state = data['pm.state']
    vbat = data['pm.vbat']

    print('{}: [\n\tlevel: {}\n\tstate: {}\n\tvolts: {}\n]'.format(logconf.cf.link_uri[-2:], bat_level, state, vbat))
    # log_file.write('{},{},{},{},{}\n'.format(logconf.cf.link_uri,
    #                                         datetime.utcnow().timestamp(), bat_level, state, vbat))


def start_battery_printing(scf):
    log_conf = LogConfig(name='Battery', period_in_ms=1000)
    log_conf.add_variable('pm.batteryLevel', 'uint8_t')
    log_conf.add_variable('pm.state', 'int8_t')
    log_conf.add_variable('pm.vbat', 'float')

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(battery_callback)
    log_conf.start()


def fly_params_callback(timestamp, data, logconf):
    accelz = data['controller.accelz']
    pitch = data['stabilizer.pitch']
    roll = data['stabilizer.roll']
    thrust = data['stabilizer.thrust']
    yaw = data['stabilizer.yaw']


    print('{}: [\n\taccelz: {}\n\tpitch: {}\n\troll:{}\n\tthrust: {}\n\tyaw: {}\n]'.format(
          logconf.cf.link_uri[-2:],accelz, pitch, roll, thrust, yaw))
    # log_file.write('{},{},{},{},{}\n'.format(logconf.cf.link_uri,
    #                                         datetime.utcnow().timestamp(), bat_level, state, vbat))


def start_fly_params_printing(scf):
    log_conf = LogConfig(name='Fly_params', period_in_ms=100)
    log_conf.add_variable('controller.accelz', 'float')
    log_conf.add_variable('stabilizer.pitch', 'float')
    log_conf.add_variable('stabilizer.roll', 'float')
    log_conf.add_variable('stabilizer.thrust', 'float')
    log_conf.add_variable('stabilizer.yaw', 'float')

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(fly_params_callback)
    log_conf.start()


def get_coord_test():
    import random as r
    while True:
        # position_data.append([r.random() * 3, r.random() * 3, r.random() * 3])
        # position_data.put_nowait(['00', r.random() * 3, r.random() * 3, r.random() * 3])
        position_data.put_nowait([choice(['00', '01']), r.random() * 3, r.random() * 3, r.random() * 3])
        sleep(0.2)


def read_csv_coords(file):
    with open(file) as csv_file:
        csv_reader = csv.DictReader(csv_file, delimiter=',')
        coords = list()
        for row in csv_reader:
            yield [(float(row['x']), float(row['y']), float(row['z']), float(row['time']))]


if __name__ == '__main__':

    ndrones = len(uris)
    #c = read_csv_coords('coordinates.csv')
    #real_time_plotting(c, 1)
    # logging.basicConfig(level=logging.DEBUG)
    '''cflib.crtp.init_drivers(enable_debug_driver=False)
    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        #print('Waiting for parameters to be downloaded...')
        swarm.parallel(reset_estimator)
        swarm.parallel(wait_for_param_download)
        swarm.parallel(start_position_printing)
        swarm.parallel(start_battery_printing)
        swarm.parallel(start_fly_params_printing)
        d = yield_position(position_data)
        # real_time_plotting(d)
        p = multiprocessing.Process(target=real_time_plotting, args=(d, ndrones))
        p.start()
        #swarm.parallel(run_sequence, args_dict=seq_args)
        p.join()'''
    start_thread(get_coord_test)
    sleep(1)
    d = yield_position(position_data)
    real_time_plotting(d, 2)
    log_file.close()
