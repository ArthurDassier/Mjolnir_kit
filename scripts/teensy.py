"""
Haoru Xue | hxue@ucsd.edu | Triton-AI | http://triton-ai.eng.ucsd.edu/ | 2020

Works with a Teensy microcontroller (MCU) and RC transmitter.

In manual mode, RC controller signal (speed, throttle, steering) is sent to the Teensy before polled and stored by
the on-board computer.

In autonomous mode, speed and steering instruction is sent by the on-board computer to the Teensy, who will then
apply the appropriate throttle and steering.

Communication is done on serial.

Watchdog is in place to shutdown the system after losing connection with the Teensy for too long (100ms)

APIs:

Transmission from the on-board computer starts with either "command" or "poll", and are always terminated by '\n'
The on-board computer can poll the speed, throttle, and steering, or command the car with speed, steering or shutdown.
e.g. "command speed 314\n" "command shutdown\n" "poll steering\n" "poll\n" (which means poll everything)

Transmission from the teensy is only needed after a polling request, containing the polled attribute and value,
terminated by '\n'. If everything is polled, send separate messages for each attribute.
e.g. "steering -0.45\n" "throttle 0.758\n"
"""

import serial
import time
import re

from TritonRacerSim.components.controller import DriveMode
from TritonRacerSim.utils.types import ModelType

class TeensyMC_Test():
    def __init__(self, cfg):
        # Params for tweaking
        super().__init__(inputs=['mux/steering', 'mux/throttle', 'ai/speed', 'usr/mode'], outputs=[], threaded=True)
        teensy_cfg = cfg['teensy']
        calibrate_cfg = cfg['calibration']
        self.timeout = teensy_cfg['watchdog_trigger_time']  # ms before watchdog kicks in
        self.pollInterval = teensy_cfg['poll_interval']  # ms between each polling

        self.running = True
        self.ser = serial.Serial(port=teensy_cfg['port'], baudrate=teensy_cfg['baudrate'])
        self.mode = DriveMode.HUMAN

        self.watchdog_subthread = Watchdog(threshold=self.timeout, callback=self.watchdog_alert_subthread)
        self.watchdog_mainthread = Watchdog(threshold=self.timeout, callback=self.watchdog_alert_mainthread)

        self.throttle = 0.0
        self.steering = 0.0
        self.speed = 0.0

        self.left_pulse = calibrate_cfg['max_left_pwm']
        self.right_pulse = calibrate_cfg['max_right_pwm']
        self.neutral_steering_pulse = calibrate_cfg['neutral_steering_pwm']
        self.max_pulse = calibrate_cfg['max_forward_pwm']
        self.min_pulse = calibrate_cfg['max_reverse_pwm']
        self.zero_pulse = calibrate_cfg['zero_throttle_pwm']


    # def onStart(self):        # self.send(sbc_message + '\n')

    #     # Process all available information before leavingy=3000)
    #     self.watchdog_mainthread.start_watchdog(delay=3000)

    def thread_step(self):
        while self.running:
            self.__poll()
            time.sleep(self.pollInterval / 1000.0)

    def __poll(self):
        """Get input values from Teensy in manual mode"""
        # Go over all messages (one per line) in the serial buffer
        # Store the values on board (speed,throttle,steering)
        # Check next message
        # If none left, poll function is done
        while self.ser.in_waiting:
            mcu_message = self.ser.readline().decode().lower()  # The message coming in
            number_in_message = re.findall(r'\d+\.*\d*', mcu_message)  # Find number in message

            self.watchdog_subthread.reset_countdown()  # Reset watchdog as soon as data is received
            if 'speed' in mcu_message:
                self.speed = number_in_message[0]
            elif 'throttle' in mcu_message:
                self.throttle = number_in_message[0]
            elif 'steering' in mcu_message:
                self.steering = number_in_message[0]


    def __command(self, throttle=None, speed=None, steering=0, shutdown=False):
        """Send Instructions to Teensy"""
        msg = ''
        if not shutdown:
            if throttle is not None:
                msg = f'commandThrottle_{throttle}\n'
            else:
                msg = f'commandSpeed_{speed}\n'

            msg += f'commandSteering_{steering}\n'

        else:
            msg = 'commandShutdown\n'
        # print (msg)
        self.send(msg)

    # Check to see what should be returned depending on usr/mode (send throttle, speed, etc)
    # Also check for model types and make sure a speed model is being used (send speed)
    # else just send throttle
    def step(self, *args): 
        self.watchdog_mainthread.reset_countdown()
        self.steering, self.throttle, self.speed, carMode = args

        # What is this for?
        #if (throttle < 0 and self.throttle >= 0):
        #    self.__start_reverse()

        # Check if drivemode is set full AI mode 
        if carMode == DriveMode.AI:
            # and is using a speed based model
            if cfg['ai_model']['model_type'] == ModelType.CNN_2D_SPD_CTL:
                self.__command(speed=self.speed, steering=self.steering)
        
            # Else we are not using a speed based model (throttle base)
            else: 
                self.__command(throttle=self.throttle, steering=self.steering)
        
        # Human Mode or Partial AI (Steering)
        else:
            self.__command(throttle=self.throttle, steering=self.steering)

    def onShutdown(self):
        self.running = False
        self.__command(shutdown=True)
        self.watchdog_mainthread.shutdown()
        self.watchdog_subthread.shutdown()
        self.ser.close()

    def watchdog_alert_mainthread(self):
        """Callback function when mainthread watchdog is alarmed
            Called when donkey main thread is stuck"""

        self.onShutdown()
        raise RuntimeError(f'[Watchdog Alert] Tritonracer mainthread failed to update new instructions within {self.timeout}ms.')

    def watchdog_alert_subthread(self):
        '''Callback function when subthread watchdog is alarmed
            Serial disconnect, thread locked, etc.'''
        
        self.onShutdown()  
        raise RuntimeError(f'[Watchdog Alert] Teensy did not respond to polling within {self.timeout}ms.')

    def getName(self):
        return 'Teensy, Throttle Control Only Version'

    def __calibrate(self):
        msg = f'''calibrateMaxForward_{self.max_pulse}
        calibrateMaxReverse_{self.min_pulse}
        calibrateNeutralThrottle_{self.zero_pulse}
        calibrateMaxLeft_{self.left_pulse}
        calibrateMaxRight_{self.right_pulse}
        calibrateNeutralSteering_{self.neutral_steering_pulse}\n
        '''
        self.ser.write(bytes(msg, 'utf-8'))

    def __start_reverse(self):
        msg = f'''commandThrottle_-0.5
        commandThrottle_0.0\n
        '''
        self.send(msg)

    def send(self, msg):
        # print(msg)
        self.ser.write(bytes(msg, 'utf-8'))


# class Watchdog:
#     """Trigger the callback function if timer reaches threshold (ms), unless reset"""

#     def __init__(self, threshold, callback):
#         self.threshold = threshold / 1000.0
#         self.callback = callback
#         self.reset = True
#         self.running = True

#     def start_watchdog(self, delay=500):
#         """Start the watchdog countdown after the delay (ms)"""
#         delay_second = delay / 1000.0
#         print(f'Watchdog will be engaged in {delay_second} seconds.')
#         from threading import Thread
#         t = Thread(target=self.__watching, args=(delay,), daemon=False)
#         t.start()

#     def __watching(self, delay):
#         delay_second = delay / 1000.0
#         time.sleep(delay_second)

#         while self.running:
#             if (self.reset):
#                 self.reset = False
#             else:
#                 self.callback()
#             time.sleep(self.threshold)

#     def reset_countdown(self):
#         """Reset the watchdog countdown"""
#         self.reset = True

#     def shutdown(self):
#         """End the watchdog"""
#         self.running = False
