# ============= Logging ================
class LoggingExample:
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link uri and disconnects after 5s.
    """

    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """

        self._cf = Crazyflie(rw_cache='./cache')

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        print('Connecting to %s' % link_uri)

        # Try to connect to the Crazyflie
        self._cf.open_link(link_uri)

        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)

        # The definition of the logconfig can be made before connecting
        # Inertia Info Logging
        self._lg_stab = LogConfig(name='InerialInfo', period_in_ms=10)
        self._lg_stab.add_variable('stateEstimate.roll', 'float')
        self._lg_stab.add_variable('stateEstimate.pitch', 'float')
        # self._lg_stab.add_variable('stateEstimate.yaw', 'float')
        self._lg_stab.add_variable('stateEstimateZ.rateRoll', 'int16_t')
        self._lg_stab.add_variable('stateEstimateZ.ratePitch', 'int16_t')
        self._lg_stab.add_variable('acc.y', 'float')
        self._lg_stab.add_variable('acc.z', 'float')
        self._lg_stab.add_variable('acc.x', 'float')
   
        try:
            self._cf.log.add_config(self._lg_stab)
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)

            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
    
            # Start the logging
            self._lg_stab.start()
      
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')

        # Start a timer to disconnect in 10s
        # t = Timer(5, self._cf.close_link)
        # t.start()

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback froma the log API when data arrives"""
        global roll_CF,pitch_CF,yaw_CF,wx_CF,wy_CF,accx_CF,accy_CF,accz_CF
        roll_CF = data['stateEstimate.roll']
        pitch_CF = data['stateEstimate.pitch']
        yaw_CF = 0
        wx_CF = float(data['stateEstimateZ.rateRoll'])/1000
        wy_CF = float(data['stateEstimateZ.ratePitch'])/1000
        accy_CF = float(data['acc.y'])
        accz_CF = float(data['acc.z'])
        accx_CF = float(data['acc.x'])

        # print('[%d][%s]: %s' % (timestamp, logconf.name, data))



    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the speficied address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.is_connected = False
