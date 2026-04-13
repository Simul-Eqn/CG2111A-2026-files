from pyrplidar_serial import PyRPlidarSerial
from pyrplidar_protocol import *
import pyrplidar_protocol



class PyRPlidar:

    def __init__(self):
        self.lidar_serial = None
        self.measurements = None
        self._last_port = None
        self._last_baudrate = None
        self._last_timeout = None
    
    def __del__(self):
        self.disconnect()

    

    def connect(self, port="/dev/ttyUSB0", baudrate=115200, timeout=3):
        self._last_port = port
        self._last_baudrate = baudrate
        self._last_timeout = timeout
        self.lidar_serial = PyRPlidarSerial()
        self.lidar_serial.open(port, baudrate, timeout)
        # print("PyRPlidar Info : device is connected")


    def _reconnect_and_restart_scan(self):
        if self._last_port is None or self._last_baudrate is None or self._last_timeout is None:
            raise PyRPlidarConnectionError("PyRPlidar Error : reconnect parameters are unavailable")

        try:
            self.send_command(RPLIDAR_CMD_STOP)
        except Exception:
            pass

        self.disconnect()
        self.connect(port=self._last_port, baudrate=self._last_baudrate, timeout=self._last_timeout)
        self.send_command(RPLIDAR_CMD_SCAN)
        return self.receive_discriptor()


    def disconnect(self):
        if self.lidar_serial is not None:
            self.lidar_serial.close()
            self.lidar_serial = None
            # print("PyRPlidar Info : device is disconnected")



    def send_command(self, cmd, payload=None):
        if self.lidar_serial == None:
            raise PyRPlidarConnectionError("PyRPlidar Error : device is not connected")

        self.lidar_serial.send_data(PyRPlidarCommand(cmd, payload).raw_bytes)

    def receive_discriptor(self):
        if self.lidar_serial == None:
            raise PyRPlidarConnectionError("PyRPlidar Error : device is not connected")
        
        discriptor = PyRPlidarResponse(self.lidar_serial.receive_data(RPLIDAR_DESCRIPTOR_LEN))
        
        if discriptor.sync_byte1 != RPLIDAR_SYNC_BYTE1[0] or discriptor.sync_byte2 != RPLIDAR_SYNC_BYTE2[0]:
            raise PyRPlidarProtocolError("PyRPlidar Error : sync bytes are mismatched", hex(discriptor.sync_byte1), hex(discriptor.sync_byte2))
        return discriptor

    def receive_data(self, discriptor):
        if self.lidar_serial == None:
            raise PyRPlidarConnectionError("PyRPlidar Error : received data length is mismatched")
        
        data = self.lidar_serial.receive_data(discriptor.data_length)
        if len(data) != discriptor.data_length:
            raise PyRPlidarProtocolError()
        return data


    def stop(self):
        self.send_command(RPLIDAR_CMD_STOP)

    def reset(self):
        self.send_command(RPLIDAR_CMD_RESET)

    def set_motor_pwm(self, pwm):
        self.lidar_serial.set_dtr(False)
        self.send_command(RPLIDAR_CMD_SET_MOTOR_PWM, struct.pack("<H", pwm))
    
    

    def get_info(self):
        self.send_command(RPLIDAR_CMD_GET_INFO)
        discriptor = self.receive_discriptor()
        data = self.receive_data(discriptor)
        return PyRPlidarDeviceInfo(data)

    def get_health(self):
        self.send_command(RPLIDAR_CMD_GET_HEALTH)
        discriptor = self.receive_discriptor()
        data = self.receive_data(discriptor)
        return PyRPlidarHealth(data)

    def get_samplerate(self):
        self.send_command(RPLIDAR_CMD_GET_SAMPLERATE)
        discriptor = self.receive_discriptor()
        data = self.receive_data(discriptor)
        return PyRPlidarSamplerate(data)

    def get_lidar_conf(self, payload):
        self.send_command(RPLIDAR_CMD_GET_LIDAR_CONF, payload)
        discriptor = self.receive_discriptor()
        data = self.receive_data(discriptor)
        return data

    def get_scan_mode_count(self):
        data = self.get_lidar_conf(struct.pack("<I", RPLIDAR_CONF_SCAN_MODE_COUNT))
        count = struct.unpack("<H", data[4:6])[0]
        return count

    def get_scan_mode_typical(self):
        data = self.get_lidar_conf(struct.pack("<I", RPLIDAR_CONF_SCAN_MODE_TYPICAL))
        typical_mode = struct.unpack("<H", data[4:6])[0]
        return typical_mode

    def get_scan_modes(self):
        
        scan_modes = []
        scan_mode_count = self.get_scan_mode_count()
        
        for mode in range(scan_mode_count):
            scan_mode = PyRPlidarScanMode(
                            self.get_lidar_conf(struct.pack("<IH", RPLIDAR_CONF_SCAN_MODE_NAME, mode)),
                            self.get_lidar_conf(struct.pack("<IH", RPLIDAR_CONF_SCAN_MODE_MAX_DISTANCE, mode)),
                            self.get_lidar_conf(struct.pack("<IH", RPLIDAR_CONF_SCAN_MODE_US_PER_SAMPLE, mode)),
                            self.get_lidar_conf(struct.pack("<IH", RPLIDAR_CONF_SCAN_MODE_ANS_TYPE, mode)))
            # print(scan_mode)
            scan_modes.append(scan_mode)
        
        return scan_modes

    

    def start_scan(self):
        print('STARTING SCAN, UNMISTAKEABLE PRINT')
        self.send_command(RPLIDAR_CMD_SCAN)
        discriptor = self.receive_discriptor()
    
        def scan_generator():
            nonlocal discriptor
            invalid_frames = 0
            hard_resyncs = 0

            def shift_resync(raw, max_shift=5):
                if self.lidar_serial is None:
                    return raw, False

                shifted = raw
                for _ in range(max_shift):
                    if PyRPlidarMeasurement.is_valid_raw_bytes(shifted):
                        return shifted, True
                    tail = self.lidar_serial.receive_data(1)
                    if len(tail) != 1:
                        break
                    shifted = shifted[1:] + tail
                return shifted, PyRPlidarMeasurement.is_valid_raw_bytes(shifted)

            while True:
                data = self.receive_data(discriptor)
                if not PyRPlidarMeasurement.is_valid_raw_bytes(data):
                    data, ok = shift_resync(data)
                    if ok:
                        invalid_frames += 1
                        if invalid_frames == 1 or (invalid_frames % 25) == 0:
                            print(f"PyRPlidar warning: shifted-byte resync hit count={invalid_frames}")
                    else:
                        invalid_frames += 1
                        if invalid_frames == 1 or (invalid_frames % 25) == 0:
                            print(f"PyRPlidar warning: dropped invalid normal frame count={invalid_frames}")

                        if invalid_frames >= 100:
                            hard_resyncs += 1
                            print(f"PyRPlidar warning: forcing SCAN resync attempt={hard_resyncs}")
                            self.send_command(RPLIDAR_CMD_STOP)
                            self.send_command(RPLIDAR_CMD_SCAN)
                            discriptor = self.receive_discriptor()
                            invalid_frames = 0

                            if hard_resyncs >= 3:
                                print("PyRPlidar warning: escalating to full reconnect+scan restart")
                                try:
                                    discriptor = self._reconnect_and_restart_scan()
                                    print("PyRPlidar warning: full reconnect recovery succeeded")
                                    hard_resyncs = 0
                                except Exception as exc:
                                    print(f"PyRPlidar warning: full reconnect recovery failed: {exc}")
                        continue

                elif invalid_frames > 0:
                    print(f"PyRPlidar warning: recovered normal stream after invalid={invalid_frames}")
                    invalid_frames = 0

                yield PyRPlidarMeasurement(data)
    
        return scan_generator

    
    
    def start_scan_express(self, mode):
        self.send_command(RPLIDAR_CMD_EXPRESS_SCAN, struct.pack("<BI", mode, 0x00000000))
        discriptor = self.receive_discriptor()

        if discriptor.data_type == 0x82:
            capsule_type = PyRPlidarScanCapsule
        elif discriptor.data_type == 0x84:
            capsule_type = PyRPlidarScanUltraCapsule
        elif discriptor.data_type == 0x85:
            capsule_type = PyRPlidarScanDenseCapsule
        else:
            raise PyRPlidarProtocolError("RPlidar Error : scan data type is not supported")
        
        def scan_generator():
            dropped_capsules = 0

            def read_valid_capsule():
                nonlocal dropped_capsules
                while True:
                    data = self.receive_data(discriptor)
                    capsule = capsule_type(data)
                    if getattr(capsule, "valid_header", True):
                        if dropped_capsules:
                            print(f"PyRPlidar warning: recovered capsule sync after dropped={dropped_capsules}")
                            dropped_capsules = 0
                        return capsule
                    dropped_capsules += 1
                    if dropped_capsules == 1 or (dropped_capsules % 50) == 0:
                        print(f"PyRPlidar warning: dropped invalid capsule header/checksum count={dropped_capsules}")

            capsule_prev = read_valid_capsule()
            
            while True:
                capsule_current = read_valid_capsule()
                
                nodes = capsule_type._parse_capsule(capsule_prev, capsule_current)
                for index, node in enumerate(nodes):
                     yield PyRPlidarMeasurement(raw_bytes=None, measurement_hq=node)
    
                capsule_prev = capsule_current

        return scan_generator

    
    def force_scan(self):
        self.send_command(RPLIDAR_CMD_FORCE_SCAN)
        discriptor = self.receive_discriptor()
        
        def scan_generator():
            while True:
                data = self.receive_data(discriptor)
                yield PyRPlidarMeasurement(data)
        
        return scan_generator
    

