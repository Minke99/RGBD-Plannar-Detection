import socket
import time
import numpy as np
import threading
from tqdm import tqdm
import struct


class UdpRigidBodies(object):
    def __init__(self, udp_ip="0.0.0.0", udp_port=22222):
        self.udp_flag = 0
        self._udpStop = False
        self._udp_data = None
        self._udpThread = None
        self._udpThread_on = False
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self.num_bodies = 0

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
        self._sock.bind((self.udp_ip, self.udp_port))

        self.sample_rate = -1  # flag
        self.sample_rate = self.get_sample_rate()
        self.sample_time = 1 / self.sample_rate
        print('UDP receiver initialized')

        self.udp_send_flag = False

    def get_sample_rate(self):
        if self.sample_rate == -1:
            print('Computing sample rate...')
            time_list = []
            for _ in tqdm(range(1000), desc="Processing...", leave=True, position=0):
                time_list.append(time.time())
                _, _ = self._sock.recvfrom(100)  # buffer size is 8192 bytes
            d_time = np.diff(time_list)
            sample_time = np.mean(d_time)

            print('Sample rate: ', '%.2f' % (1/sample_time), 'Hz')
            return 1/sample_time
        else:
            return self.sample_rate

    def start_thread(self):
        if not self._udpThread_on:
            self._udpThread = threading.Thread(target=self._udp_worker, args=(), )
            self._udp_data = b'1'
            self._udpThread.start()
            self._udpThread_on = True
            time.sleep(1)
            print('Upd thread start')
            self.num_bodies = len(self._udp_data)/14
            if self.num_bodies % 1 == 0:
                self.num_bodies = int(self.num_bodies)
                print('Number of rigid bodies: ' + str(self.num_bodies))
            else:
                print('error: incorrect data')
        else:
            print('New upd thread is not started')

    def _udp_worker(self, ):
        if not self._udpThread_on:
            # print('Receive data without data processing')
            # main loop

            while not self._udpStop:
                self.udp_flag = self.udp_flag + 1
                # self._synced_event.clear()
                udp_data_temp, _ = self._sock.recvfrom(100)  # buffer size is 8192 bytes
                # self._udp_data_ready.clear()
                self._udp_data = udp_data_temp

                if self.udp_send_flag:
                    self._sock.sendto("a".encode(), ("192.168.0.172", 2390))
                    self.udp_send_flag = False
                    print(11)

                # self._udp_data_ready.set()
                # self._synced_event.set()
                # if self._sync_on:
                #     self._synced_event_2.wait()
                #     self._synced_event_2.clear()
                # print(self.udp_flag)
                # print(1)
            # print('upd thread stopped')

    def stop_thread(self, ):
        # self._sync_on = False
        self._udpStop = True
        time.sleep(self.sample_time)
        print('upd thread stopped')

    def get_data(self, ):
        # get current data
        # self._sync_on = False
        # self._udp_data_ready.wait()
        return self._udp_data

    def get_data_sync(self, ):
        self.udp_flag = self.udp_flag + 1
        udp_data_temp, _ = self._sock.recvfrom(100)  # buffer size is 8192 bytes
        self._udp_data = udp_data_temp
        return self._udp_data


class DataProcessor(object):
    def __init__(self, num_bodies, sample_rate):
        self.num_bodies = num_bodies
        self.sample_rate = sample_rate
        body_name = [i for i in range(1, self.num_bodies + 1)]
        self.keys = ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
        self.data_list = {name: {key: 0.0 for key in self.keys} for name in body_name}

        self.save_list_name = []
        self.save_list_data = []

        for body in body_name:
            for key in self.keys:
                self.save_list_name.append('b' + str(body) + '_' + key)
                self.save_list_data.append(0.0)

    def process_data(self, udp_data):
        for i in range(1, self.num_bodies + 1):
            x, y, z, qx, qy, qz, qw = struct.unpack("hhhhhhh", udp_data[(i*14 - 14):i*14])
            self.data_list[i]['x'] = x * 0.0005
            self.data_list[i]['y'] = y * 0.0005
            self.data_list[i]['z'] = z * 0.0005
            self.data_list[i]['qx'] = qx * 0.001
            self.data_list[i]['qy'] = qy * 0.001
            self.data_list[i]['qz'] = qz * 0.001
            self.data_list[i]['qw'] = qw * 0.001

        i = 0
        for body in self.data_list:
            for key in self.keys:
                self.save_list_data[i] = self.data_list[body][key]
                i = i + 1
        return self.data_list, self.save_list_data


class UdpRigidBodies2(object):
    def __init__(self, udp_ip="0.0.0.0", udp_port=22222):
        self.len_data = 100
        self.udp_flag = 0
        self._udpStop = False
        self._udp_data = None
        self._udp_data_time = time.time()
        self._udpThread = None
        self._udpThread_on = False
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self.num_bodies = 0

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
        self._sock.bind((self.udp_ip, self.udp_port))

        self.sample_rate = -1  # flag
        self.sample_rate = self.get_sample_rate()
        self.sample_time = 1 / self.sample_rate
        print('UDP receiver initialized')

    def get_sample_rate(self):
        if self.sample_rate == -1:
            print('Computing sample rate...')
            time_list = []
            for _ in tqdm(range(1000), desc="Processing...", leave=True, position=0):
                time_list.append(time.time())
                udp_data_temp, _ = self._sock.recvfrom(100)  # buffer size is 8192 bytes
                self._udp_data_time = time.time()
                self.len_data = len(udp_data_temp)
            d_time = np.diff(time_list)
            sample_time = np.mean(d_time)

            print('Sample rate: ', '%.2f' % (1/sample_time), 'Hz')
            print('UDP data size: ', '%.2f' % (self.len_data))

            return 1/sample_time
        else:
            return self.sample_rate

    def start_thread(self):
        if not self._udpThread_on:
            self._udpThread = threading.Thread(target=self._udp_worker, args=(), )
            self._udp_data = b'1'
            self._udpThread.start()
            self._udpThread_on = True
            time.sleep(1)
            print('Upd thread start')
            self.num_bodies = len(self._udp_data)/14
            if self.num_bodies % 1 == 0:
                self.num_bodies = int(self.num_bodies)
                print('Number of rigid bodies: ' + str(self.num_bodies))
            else:
                print('error: incorrect data')
        else:
            print('New upd thread is not started')

    def _udp_worker(self, ):
        if not self._udpThread_on:
            while not self._udpStop:
                self.udp_flag = self.udp_flag + 1
                udp_data_temp, _ = self._sock.recvfrom(self.len_data)  # buffer size is 8192 bytes
                self._udp_data_time = time.time()
                self._udp_data = udp_data_temp

                # if self.udp_send_flag:
                #     self._sock.sendto("a".encode(), ("192.168.0.172", 2390))
                #     self.udp_send_flag = False
                #     print(11)

                # self._udp_data_ready.set()
                # self._synced_event.set()
                # if self._sync_on:
                #     self._synced_event_2.wait()
                #     self._synced_event_2.clear()
                # print(self.udp_flag)
                # print(1)
            # print('upd thread stopped')

    def stop_thread(self, ):
        # self._sync_on = False
        self._udpStop = True
        time.sleep(self.sample_time)
        print('upd thread stopped')

    def get_data(self, ):
        # get current data
        # self._sync_on = False
        # self._udp_data_ready.wait()
        return self._udp_data, self._udp_data_time

    def get_data_sync(self, ):
        self.udp_flag = self.udp_flag + 1
        udp_data_temp, _ = self._sock.recvfrom(self.len_data)  # buffer size is 8192 bytes
        self._udp_data = udp_data_temp
        return self._udp_data
