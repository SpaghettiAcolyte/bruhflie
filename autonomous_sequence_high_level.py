# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2018 Bitcraze AB
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.
"""
Simple example that connects to one crazyflie (check the address at the top
and update it to your crazyflie address) and uses the high level commander
to send setpoints and trajectory to fly a figure 8.

This example is intended to work with any positioning system (including LPS).
It aims at documenting how to set the Crazyflie in position control mode
and how to send setpoints using the high level commander.
"""
import multiprocessing
import time
from datetime import datetime
from pathlib import Path
from random import shuffle

import cflib.crtp
import matplotlib._color_data as mcd
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.mem import Poly4D
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.syncLogger import SyncLogger
from matplotlib import animation
from matplotlib import pyplot as plt

# The trajectory to fly
# See https://github.com/whoenig/uav_trajectories for a tool to generate
# trajectories

# Duration,x^0,x^1,x^2,x^3,x^4,x^5,x^6,x^7,y^0,y^1,y^2,y^3,y^4,y^5,y^6,y^7,z^0,z^1,z^2,z^3,z^4,z^5,z^6,z^7,yaw^0,yaw^1,yaw^2,yaw^3,yaw^4,yaw^5,yaw^6,yaw^7
figure8 = [
    [1.050000, 0.000000, -0.000000, 0.000000, -0.000000, 0.830443, -0.276140, -0.384219, 0.180493, -0.000000, 0.000000, -0.000000, 0.000000, -1.356107, 0.688430, 0.587426, -0.329106, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.710000, 0.396058, 0.918033, 0.128965, -0.773546, 0.339704, 0.034310, -0.026417, -0.030049, -0.445604, -0.684403, 0.888433, 1.493630, -1.361618, -0.139316, 0.158875, 0.095799, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.620000, 0.922409, 0.405715, -0.582968, -0.092188, -0.114670, 0.101046, 0.075834, -0.037926, -0.291165, 0.967514, 0.421451, -1.086348, 0.545211, 0.030109, -0.050046, -0.068177, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.700000, 0.923174, -0.431533, -0.682975, 0.177173, 0.319468, -0.043852, -0.111269, 0.023166, 0.289869, 0.724722, -0.512011, -0.209623, -0.218710, 0.108797, 0.128756, -0.055461, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.560000, 0.405364, -0.834716, 0.158939, 0.288175, -0.373738, -0.054995, 0.036090, 0.078627, 0.450742, -0.385534, -0.954089, 0.128288, 0.442620, 0.055630, -0.060142, -0.076163, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.560000, 0.001062, -0.646270, -0.012560, -0.324065, 0.125327, 0.119738, 0.034567, -0.063130, 0.001593, -1.031457, 0.015159, 0.820816, -0.152665, -0.130729, -0.045679, 0.080444, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.700000, -0.402804, -0.820508, -0.132914, 0.236278, 0.235164, -0.053551, -0.088687, 0.031253, -0.449354, -0.411507, 0.902946, 0.185335, -0.239125, -0.041696, 0.016857, 0.016709, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.620000, -0.921641, -0.464596, 0.661875, 0.286582, -0.228921, -0.051987, 0.004669, 0.038463, -0.292459, 0.777682, 0.565788, -0.432472, -0.060568, -0.082048, -0.009439, 0.041158, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.710000, -0.923935, 0.447832, 0.627381, -0.259808, -0.042325, -0.032258, 0.001420, 0.005294, 0.288570, 0.873350, -0.515586, -0.730207, -0.026023, 0.288755, 0.215678, -0.148061, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [1.053185, -0.398611, 0.850510, -0.144007, -0.485368, -0.079781, 0.176330, 0.234482, -0.153567, 0.447039, -0.532729, -0.855023, 0.878509, 0.775168, -0.391051, -0.713519, 0.391628, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
]


def init_log(path='/home/stranger/Content/Python/Crazyflie/Logs/'):
    date = (str(datetime.now()).split('.')[0]).split(' ')[0]
    t = str(datetime.now()).split('.')[0].split(' ')[1]
    Path(path + date).mkdir(parents=True, exist_ok=True)
    filename = path + date + '/position_logo_' + date + '_' + t + '.txt'
    log_file = open(filename, 'w')
    return log_file


log_file = init_log()


class Uploader:
    def __init__(self):
        self._is_done = False

    def upload(self, trajectory_mem):
        print('Uploading data')
        trajectory_mem.write_data(self._upload_done)

        while not self._is_done:
            time.sleep(0.2)

    def _upload_done(self, mem, addr):
        print('Data uploaded')
        self._is_done = True


def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.0005

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


def reset_estimator(cf):
    cf = cf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)


def activate_high_level_commander(cf):
    cf = cf.cf
    cf.param.set_value('commander.enHighLevel', '1')


def activate_mellinger_controller(cf):
    cf = cf.cf
    cf.param.set_value('stabilizer.controller', '2')


def upload_trajectory(cf, trajectory_id, trajectory):
    cf = cf.cf
    trajectory_mem = cf.mem.get_mems(MemoryElement.TYPE_TRAJ)[0]

    total_duration = 0
    for row in trajectory:
        duration = row[0]
        x = Poly4D.Poly(row[1:9])
        y = Poly4D.Poly(row[9:17])
        z = Poly4D.Poly(row[17:25])
        yaw = Poly4D.Poly(row[25:33])
        trajectory_mem.poly4Ds.append(Poly4D(duration, x, y, z, yaw))
        total_duration += duration

    Uploader().upload(trajectory_mem)
    cf.high_level_commander.define_trajectory(trajectory_id, 0,
                                              len(trajectory_mem.poly4Ds))
    return total_duration


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
        if framedata == 'stop':
            return (*[l for drone in lines_2d for l in drone],
                    *[l for drone in lines_3d for l in drone])

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


def run_sequence(scf):
    # 8 figure
    cf = scf.cf
    duration = upload_trajectory(cf, 1, figure8)
    commander = cf.high_level_commander
    commander.takeoff(1.0, 2.0)
    time.sleep(3.0)
    relative = True
    commander.start_trajectory(1, 1.0, relative)
    time.sleep(duration)
    commander.land(0.0, 2.0)
    time.sleep(2)
    commander.stop()

    # forward/backward
    '''cf = scf.cf
    flight_time = 2
    commander = cf.high_level_commander

    commander.takeoff(0.5, 2.0)
    time.sleep(3)

    for _ in range(5):
        commander.go_to(1, 0, 0, 0, flight_time, relative=True)
        time.sleep(flight_time + 1)

    for _ in range(5):
        commander.go_to(-1, 0, 0, 0, flight_time, relative=True)
        time.sleep(flight_time + 1)

    commander.land(0.0, 2.0)
    time.sleep(2)
    commander.stop()'''


# URI to the Crazyflie to connect to
uris = {
    'radio://0/80/2M/E7E7E7E7E9',
    # 'radio://0/80/2M/E7E7E7E7E3',
    # 'radio://0/80/2M/E7E7E7E7E5',
    # 'radio://0/80/2M/E7E7E7E7EB',
}


if __name__ == '__main__':
    ndrones = len(uris)

    cflib.crtp.init_drivers(enable_debug_driver=False)
    '''with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        trajectory_id = 1

        activate_high_level_commander(cf)
        #activate_mellinger_controller(cf)
        duration = upload_trajectory(cf, trajectory_id, figure8)
        print('The sequence is {:.1f} seconds long'.format(duration))
        reset_estimator(cf)
        run_sequence(cf, trajectory_id, duration) '''

    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        swarm.parallel_safe(activate_high_level_commander)
        # swarm.parallel_safe(reset_estimator)
        swarm.parallel_safe(start_position_printing)
        d = yield_position(position_data)
        p = multiprocessing.Process(target=real_time_plotting, args=(d, ndrones))
        p.start()
        # swarm.parallel_safe(run_sequence)
        p.join()

    log_file.close()
