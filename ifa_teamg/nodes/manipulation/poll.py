from __future__ import print_function
from stretch_body.dynamixel_XL430 import *
from stretch_body.dynamixel_hello_XL430 import DynamixelHelloXL430
import time
import threading
import struct
import array as arr
import logging
from dynamixel_sdk.robotis_def import *
import dynamixel_sdk.port_handler as prh
import dynamixel_sdk.packet_handler as pch
import threading
import serial
from future.builtins import input
import sys
from stretch_body.dynamixel_XL430 import *
import argparse
import stretch_body.device
d = stretch_body.device.Device(name='dummy_device') # to initialize logging config

# The code can be found in the following directory:
# /opt/ros/melodic/lib/python2.7/dist-packages/dynamixel_sdk/
import dynamixel_sdk.group_bulk_read as gbr
import dynamixel_sdk.group_sync_read as gsr

class VFAPollinator(DynamixelHelloXL430):
    """
    API to the VFA pollinator
    The pollinator motion is non-linear w.r.t to motor motion due to its design
    As such, the position of the pollinator is represented at as unit-less value, 'pct'
    The Pct ranges from approximately -100 (fully closed) to approximately +50 (fully open)
    A Pct of zero is the fingertips just touching
    """
    def __init__(self, chain=None):
        DynamixelHelloXL430.__init__(self, 'vfa_pollinator')
        self.status['pos_pct']= 0.0
        pt_lock=None
        self.pt_lock = threading.RLock() if pt_lock is None else pt_lock
        
        
        self.chain = chain
        self.status={'timestamp_pc':0,'comm_errors':0,'pos':0,'vel':0,'effort':0,'temp':0,'shutdown':0, 'hardware_error':0,
                     'input_voltage_error':0,'overheating_error':0,'motor_encoder_error':0,'electrical_shock_error':0,'overload_error':0,
                     'stalled':0,'stall_overload':0,'pos_ticks':0,'vel_ticks':0,'effort_ticks':0}
        self.thread_rate_hz = 15.0
        self.trajectory = RevoluteTrajectory()
        self._waypoint_ts = None
        self._waypoint_vel = self.params['motion']['trajectory_max']['vel_r']
        self._waypoint_accel = self.params['motion']['trajectory_max']['accel_r']

        #Share bus resource amongst many XL430s



        if self.params['flip_encoder_polarity']:
            wr_max = self.ticks_to_world_rad(self.params['range_t'][0])
            wr_min = self.ticks_to_world_rad(self.params['range_t'][1])
        else:
            wr_max = self.ticks_to_world_rad(self.params['range_t'][1])
            wr_min = self.ticks_to_world_rad(self.params['range_t'][0])
        self.soft_motion_limits = {'collision': [None, None], 'user': [None, None],'hard': [wr_min,wr_max],'current': [wr_min,wr_max]}

        self.is_homing=False
        self.status_mux_id = 0
        self.was_runstopped = False
        self.comm_errors = DynamixelCommErrorStats(name,logger=self.logger)
         # ###########  Device Methods #############

    def get_soft_motion_limits(self):
        """
            Return the currently applied soft motion limits: [min, max]

            The soft motion limit restricts joint motion to be <= its physical limits.

            There are three types of limits:
            Hard: The physical limits
            Collision: Limits set by RobotCollision to avoid collisions
            User: Limits set by the user software

            The joint is limited to the most restrictive range of the Hard / Collision/ User values.
            Specifying a value of None for a limit indicates that no constraint exists for that limit type.
            This allows a User limits and Collision limits to coexist.
            For example, a user can temporarily restrict the range of motion beyond the current collision limits.
            Then, by commanding User limits of None, the joint limits will revert back to the collision settings.
        """
        return self.soft_motion_limits['current']

    def set_soft_motion_limit_min(self,x,limit_type='user' ):
        """
        x: value to set a joints limit to
        limit_type: 'user' or 'collision'
        """
        self.soft_motion_limits[limit_type][0]=x
        self.soft_motion_limits['current'][0]=max(filter(lambda x: x is not None, [self.soft_motion_limits['collision'][0],self.soft_motion_limits['hard'][0],self.soft_motion_limits['user'][0]]))


    def set_soft_motion_limit_max(self,x,limit_type='user' ):
        """
        x: value to set a joints limit to
        limit_type: 'user' or 'collision'
        """
        self.soft_motion_limits[limit_type][1]=x
        self.soft_motion_limits['current'][1]=min(filter(lambda x: x is not None, [self.soft_motion_limits['collision'][1],self.soft_motion_limits['hard'][1],self.soft_motion_limits['user'][1]]))

    # ###################################################

    def do_ping(self, verbose=False):
        return self.motor.do_ping(verbose)

    def startup(self, threaded=False):
        Device.startup(self, threaded=threaded)
        try:
            if self.motor.do_ping(verbose=False):
                self.hw_valid = True
                self.motor.disable_torque()
                if self.params['use_multiturn']:
                    self.motor.enable_multiturn()
                else:
                    self.motor.enable_pos()
                    if self.params['range_t'][0]<0 or self.params['range_t'][1]>4095:
                        self.logger.warning('Warning: Invalid position range for %s'%self.name)
                self.motor.set_pwm_limit(self.params['pwm_limit'])
                self.motor.set_temperature_limit(self.params['temperature_limit'])
                self.motor.set_min_voltage_limit(self.params['min_voltage_limit'])
                self.motor.set_max_voltage_limit(self.params['max_voltage_limit'])
                self.motor.set_P_gain(self.params['pid'][0])
                self.motor.set_I_gain(self.params['pid'][1])
                self.motor.set_D_gain(self.params['pid'][2])
                self.motor.set_return_delay_time(self.params['return_delay_time'])
                self.motor.set_profile_velocity(self.rad_per_sec_to_ticks(self.params['motion']['default']['vel']))
                self.motor.set_profile_acceleration(self.rad_per_sec_sec_to_ticks(self.params['motion']['default']['accel']))
                self.v_des=self.params['motion']['default']['vel']
                self.a_des=self.params['motion']['default']['accel']
                self.is_calibrated=self.motor.is_calibrated()
                self.enable_torque()
                self.pull_status()
                return True
            else:
                self.logger.warning('DynamixelHelloXL430 Ping failed... %s' % self.name)
                print('DynamixelHelloXL430 Ping failed...', self.name)
                return False
        except DynamixelCommError:
            self.logger.warning('DynamixelHelloXL430 Ping failed... %s' % self.name)
            print('DynamixelHelloXL430 Ping failed...', self.name)
            return False

    def _thread_loop(self):
        self.pull_status()
        self.update_trajectory()

    def stop(self):
        Device.stop(self)
        self._waypoint_ts, self._waypoint_vel, self._waypoint_accel = None, None, None
        if self.hw_valid:
            if self.params['disable_torque_on_stop']:
                self.disable_torque()
            self.hw_valid = False

    def pull_status(self,data=None):
        if not self.hw_valid:
            return

        pos_valid = True
        vel_valid = True
        eff_valid = True
        temp_valid = True
        err_valid = True

        #First pull new data from servo
        #Or bring in data from a synchronized read
        if data is None:
            try:
                x = self.motor.get_pos()
                if not self.motor.last_comm_success and self.params['retry_on_comm_failure']:
                    x = self.motor.get_pos()
                pos_valid = self.motor.last_comm_success

                v = self.motor.get_vel()
                if not self.motor.last_comm_success and self.params['retry_on_comm_failure']:
                    v = self.motor.get_vel()
                vel_valid = self.motor.last_comm_success

                if self.status_mux_id == 0:
                    eff = self.motor.get_load()
                    if not self.motor.last_comm_success and self.params['retry_on_comm_failure']:
                        eff = self.motor.get_load()
                    eff_valid = self.motor.last_comm_success
                else:
                    eff = self.status['effort_ticks']

                if self.status_mux_id == 1:
                    temp = self.motor.get_temp()
                    if not self.motor.last_comm_success and self.params['retry_on_comm_failure']:
                        temp = self.motor.get_temp()
                    temp_valid = self.motor.last_comm_success
                else:
                    temp = self.status['temp']

                if self.status_mux_id == 2:
                    err = self.motor.get_hardware_error()
                    if not self.motor.last_comm_success and self.params['retry_on_comm_failure']:
                        err = self.motor.get_hardware_error()
                    err_valid = self.motor.last_comm_success
                else:
                    err = self.status['hardware_error']

                self.status_mux_id = (self.status_mux_id + 1) % 3


                if not pos_valid or not vel_valid or not eff_valid or not temp_valid or not err_valid:
                    raise DynamixelCommError
                ts = time.time()
            except(termios.error, DynamixelCommError):
                #self.logger.warning('Dynamixel communication error on %s: '%self.name)
                self.motor.port_handler.ser.reset_output_buffer()
                self.motor.port_handler.ser.reset_input_buffer()
                self.comm_errors.add_error(rx=True,gsr=False)
                return
        else:
            x = data['x']
            v = data['v']
            eff = data['eff']
            temp = data['temp']
            ts = data['ts']
            err = data['err']

        #Now update status dictionary
        if pos_valid:
            self.status['pos_ticks'] = x
            self.status['pos'] = self.ticks_to_world_rad(float(x))
        if vel_valid:
            self.status['vel_ticks'] = v
            self.status['vel'] = self.ticks_to_world_rad_per_sec(float(v))
        if eff_valid:
            self.status['effort_ticks'] = eff
            self.status['effort'] = self.ticks_to_pct_load(float(eff))
        if temp_valid:
            self.status['temp'] = float(temp)
        if err_valid:
            self.status['hardware_error'] = err

        self.status['timestamp_pc'] = ts

        self.status['hardware_error'] = err
        self.status['input_voltage_error'] = self.status['hardware_error'] & 1 != 0
        self.status['overheating_error'] = self.status['hardware_error'] & 4 != 0
        self.status['motor_encoder_error'] = self.status['hardware_error'] & 8 != 0
        self.status['electrical_shock_error'] = self.status['hardware_error'] & 16 != 0
        self.status['overload_error'] = self.status['hardware_error'] & 32 != 0

        #Finally flag if stalled at high effort for too long
        self.status['stalled']=abs(self.status['vel'])<self.params['stall_min_vel']
        over_eff=abs(self.status['effort']) > self.params['stall_max_effort']

        if self.status['stalled']:
            if not over_eff:
                self.ts_over_eff_start = None
            if over_eff and self.ts_over_eff_start is None: #Mark the start of being stalled and over-effort
                self.ts_over_eff_start = time.time()
            if self.ts_over_eff_start is not None and time.time()-self.ts_over_eff_start>self.params['stall_max_time']:
                self.status['stall_overload'] = True
            else:
                self.status['stall_overload'] = False
        else:
            self.ts_over_eff_start=None
            self.status['stall_overload'] = False

    def mark_zero(self):
        if not self.hw_valid:
            return
        x=self.motor.get_pos()
        print('Marking current position of (ticks)',x,' as 0 (rad)')
        self.params['zero_t']=x
        self.write_device_params(self.name, self.params)

    def wait_until_at_setpoint(self,timeout=15.0):
        """Polls for moving status to wait until at commanded position goal

        Returns
        -------
        bool
            True if success, False if timeout
        """
        ts = time.time()
        while time.time() - ts < timeout:
            if self.motor.get_moving_status() & 1 == 1:
                return True
            time.sleep(0.1)
        return False

    def pretty_print(self):
        if not self.hw_valid:
            print('----- HelloXL430 ------ ')
            print('Servo not on bus')
            return
        print('----- HelloXL430 ------ ')
        print('Name',self.name)
        print('Position (rad)', self.status['pos'])
        print('Position (deg)', rad_to_deg(self.status['pos']))
        print('Position (ticks)', self.status['pos_ticks'])
        print('Velocity (rad/s)', self.status['vel'])
        print('Velocity (ticks/s)', self.status['vel_ticks'])
        print('Effort (%)', self.status['effort'])
        print('Effort (ticks)', self.status['effort_ticks'])
        print('Temp', self.status['temp'])
        print('Comm Errors', self.motor.comm_errors)
        print('Hardware Error', self.status['hardware_error'])
        print('Hardware Error: Input Voltage Error: ',self.status['input_voltage_error'])
        print('Hardware Error: Overheating Error: ', self.status['overheating_error'])
        print('Hardware Error: Motor Encoder Error: ',self.status['motor_encoder_error'])
        print('Hardware Error: Electrical Shock Error: ', self.status['electrical_shock_error'])
        print('Hardware Error: Overload Error: ', self.status['overload_error'])
        print('Timestamp PC', self.status['timestamp_pc'])
        print('Range (ticks)',self.params['range_t'])
        print('Range (rad) [',self.ticks_to_world_rad(self.params['range_t'][0]), ' , ',self.ticks_to_world_rad(self.params['range_t'][1]),']')
        print('Stalled',self.status['stalled'])
        print('Stall Overload',self.status['stall_overload'])
        print('Is Calibrated',self.is_calibrated)
        #self.motor.pretty_print()

    def step_sentry(self, robot):
        if self.hw_valid and self.robot_params['robot_sentry']['dynamixel_stop_on_runstop'] and self.params['enable_runstop']:
            is_runstopped = robot.pimu.status['runstop_event']
            if is_runstopped is not self.was_runstopped:
                if is_runstopped:
                    self.stop_trajectory()
                    self.disable_torque()
                else:
                    self.enable_torque()
            self.was_runstopped = is_runstopped

    # #####################################

    def reboot(self):
        if not self.hw_valid:
            return
        self.motor.do_reboot()

    def enable_torque(self):
        if not self.hw_valid:
            return
        self.motor.enable_torque()

    def disable_torque(self):
        if not self.hw_valid:
            return
        self.motor.disable_torque()

    def move_to(self,x_des, v_des=None, a_des=None):
        if not self.hw_valid:
            return
        if self.params['req_calibration'] and not self.is_calibrated:
            self.logger.warning('Dynamixel not calibrated: %s' % self.name)
            print('Dynamixel not calibrated:', self.name)
            return
        try:
            self.set_motion_params(v_des,a_des)
            x_des = min(max(self.get_soft_motion_limits()[0], x_des), self.get_soft_motion_limits()[1])
            t_des = self.world_rad_to_ticks(x_des)
            t_des = max(self.params['range_t'][0], min(self.params['range_t'][1], t_des))
            self.motor.go_to_pos(t_des)
        except (termios.error, DynamixelCommError):
            #self.logger.warning('Dynamixel communication error on: %s' % self.name)
            self.comm_errors.add_error(rx=False, gsr=False)


    def set_motion_params(self,v_des=None,a_des=None):
        try:
            if not self.hw_valid:
                return

            v_des = v_des if v_des is not None else self.params['motion']['default']['vel']
            v_des = min(self.params['motion']['max']['vel'], v_des)
            if v_des != self.v_des:
                self.motor.set_profile_velocity(self.rad_per_sec_to_ticks(v_des))
                self.v_des = v_des

            a_des = a_des if a_des is not None else self.params['motion']['default']['accel']
            a_des = min(self.params['motion']['max']['accel'], a_des)
            if a_des != self.a_des:
                self.motor.set_profile_acceleration(self.rad_per_sec_sec_to_ticks(a_des))
                self.a_des = a_des
        except (termios.error, DynamixelCommError):
            #self.logger.warning('Dynamixel communication error on: %s' % self.name)
            self.comm_errors.add_error(rx=False, gsr=False)


    def move_by(self,x_des, v_des=None, a_des=None):
        if not self.hw_valid:
            return
        try:
            if abs(x_des) > 0.00002: #Avoid drift
                x=self.motor.get_pos()
                if not self.motor.last_comm_success and self.params['retry_on_comm_failure']:
                    x = self.motor.get_pos()

                if self.motor.last_comm_success:
                    cx=self.ticks_to_world_rad(x)
                    self.move_to(cx + x_des, v_des, a_des)
                else:
                    self.logger.debug('Move_By comm failure on %s' % self.name)
        except (termios.error, DynamixelCommError):
            #self.logger.warning('Dynamixel communication error on: %s' % self.name)
            self.comm_errors.add_error(rx=False, gsr=False)

    def quick_stop(self):
        if not self.hw_valid:
            return
        try:
            self.motor.disable_torque()
            self.motor.enable_torque()
        except (termios.error, DynamixelCommError):
            self.comm_errors.add_error(rx=False, gsr=False)

    def enable_pos(self):
        if not self.hw_valid:
            return
        try:
            self.motor.disable_torque()
            if self.params['use_multiturn']:
                self.motor.enable_multiturn()
            else:
                self.motor.enable_pos()
            self.motor.set_profile_velocity(self.rad_per_sec_to_ticks(self.v_des))
            self.motor.set_profile_acceleration(self.rad_per_sec_sec_to_ticks(self.a_des))
            self.motor.enable_torque()
        except (termios.error, DynamixelCommError):
            self.comm_errors.add_error(rx=False, gsr=False)

    def enable_pwm(self):
        if not self.hw_valid:
            return
        try:
            self.motor.disable_torque()
            self.motor.enable_pwm()
            self.motor.enable_torque()
        except (termios.error, DynamixelCommError):
            self.comm_errors.add_error(rx=False, gsr=False)


    def set_pwm(self,x):
        if not self.hw_valid:
            return
        try:
            self.motor.set_pwm(x)
        except (termios.error, DynamixelCommError):
            self.comm_errors.add_error(rx=False, gsr=False)

    # ######### Waypoint Trajectory Interface ##############################

    def follow_trajectory(self, v_r=None, a_r=None, req_calibration=True, move_to_start_point=True):
        """Starts executing a waypoint trajectory

        `self.trajectory` must be populated with a valid trajectory before calling
        this method.

        Parameters
        ----------
        v_r : float
            velocity limit for trajectory in radians per second
        a_r : float
            acceleration limit for trajectory in radians per second squared
        req_calibration : bool
            whether to allow motion prior to homing
        move_to_start_point : bool
            whether to move to the trajectory's start to avoid a jump, this
            time to move doesn't count against the trajectory's timeline
        """
        # check if joint valid, homed, and previous trajectory not executing
        if not self.hw_valid:
            self.logger.warning('Dynamixel connection to hardware not valid')
            return False
        if req_calibration:
            if not self.is_calibrated:
                self.logger.warning('Dynamixel not homed')
                return False
        if self._waypoint_ts is not None:
            self.logger.error('Dynamixel waypoint trajectory already active')
            return False

        # check if trajectory valid
        vel_limit = v_r if v_r is not None else self.params['motion']['trajectory_max']['vel_r']
        acc_limit = a_r if a_r is not None else self.params['motion']['trajectory_max']['accel_r']
        valid, reason = self.trajectory.is_valid(vel_limit, acc_limit)
        if not valid:
            self.logger.warning('Dynamixel trajectory not valid: {0}'.format(reason))
            return False
        if valid and reason == "must have atleast two waypoints":
            # skip this device
            return True

        # set defaults
        self._waypoint_vel = min(abs(v_r), self.params['motion']['trajectory_max']['vel_r']) \
            if v_r is not None else self.params['motion']['trajectory_max']['vel_r']
        self._waypoint_accel = min(abs(a_r), self.params['motion']['trajectory_max']['accel_r']) \
            if a_r is not None else self.params['motion']['trajectory_max']['accel_r']

        # move to start point
        if move_to_start_point:
            self.move_to(self.trajectory[0].position)
            if not self.wait_until_at_setpoint():
                self.logger.warning('Dynamixel unable to reach starting point')
                return False

        # start trajectory
        self._waypoint_ts = time.time()
        p0, _, _ = self.trajectory.evaluate_at(time.time() - self._waypoint_ts)
        self.move_to(p0, self._waypoint_vel, self._waypoint_accel)
        return True

    def update_trajectory(self):
        """Updates hardware with the next position goal of `self.trajectory`

        This method must be called frequently to enable complete trajectory execution
        and preemption of future segments. If used with `stretch_body.robot.Robot` or
        with `self.startup(threaded=True)`, a background thread is launched for this.
        Otherwise, the user must handle calling this method.
        """
        # check if joint valid, previous trajectory not executing, and not runstopped
        if not self.hw_valid or self._waypoint_ts is None:
            return
        if self.was_runstopped:
            return

        if (time.time() - self._waypoint_ts) < self.trajectory[-1].time:
            p1, _, _ = self.trajectory.evaluate_at(time.time() - self._waypoint_ts)
            self.move_to(p1, self._waypoint_vel, self._waypoint_accel)
        else:
            self.move_to(self.trajectory[-1].position, self._waypoint_vel, self._waypoint_accel)
            self._waypoint_ts, self._waypoint_vel, self._waypoint_accel = None, None, None

    def stop_trajectory(self):
        """Stop waypoint trajectory immediately and resets hardware
        """
        self._waypoint_ts, self._waypoint_vel, self._waypoint_accel = None, None, None

    # ###########  Device Methods #############

    def set_soft_motion_limit_max(self,x,limit_type='user' ):
        """
        x: value to set a joints limit to
        limit_type: 'user' or 'collision'
        """
        self.soft_motion_limits[limit_type][1]=x
        self.soft_motion_limits['current'][1]=min(filter(lambda x: x is not None, [self.soft_motion_limits['collision'][1],self.soft_motion_limits['hard'][1],self.soft_motion_limits['user'][1]]))

    # ###################################################

    def do_ping(self, verbose=False):
        return self.motor.do_ping(verbose)

    
    def pretty_print(self):
        if not self.hw_valid:
            print('----- HelloXL430 ------ ')
            print('Servo not on bus')
            return
        print('----- HelloXL430 ------ ')
        print('Name',self.name)
        print('Position (rad)', self.status['pos'])
        print('Position (deg)', rad_to_deg(self.status['pos']))
        print('Position (ticks)', self.status['pos_ticks'])
        print('Velocity (rad/s)', self.status['vel'])
        print('Velocity (ticks/s)', self.status['vel_ticks'])
        print('Effort (%)', self.status['effort'])
        print('Effort (ticks)', self.status['effort_ticks'])
        print('Temp', self.status['temp'])
        print('Comm Errors', self.motor.comm_errors)
        print('Hardware Error', self.status['hardware_error'])
        print('Hardware Error: Input Voltage Error: ',self.status['input_voltage_error'])
        print('Hardware Error: Overheating Error: ', self.status['overheating_error'])
        print('Hardware Error: Motor Encoder Error: ',self.status['motor_encoder_error'])
        print('Hardware Error: Electrical Shock Error: ', self.status['electrical_shock_error'])
        print('Hardware Error: Overload Error: ', self.status['overload_error'])
        print('Timestamp PC', self.status['timestamp_pc'])
        print('Range (ticks)',self.params['range_t'])
        print('Range (rad) [',self.ticks_to_world_rad(self.params['range_t'][0]), ' , ',self.ticks_to_world_rad(self.params['range_t'][1]),']')
        print('Stalled',self.status['stalled'])
        print('Stall Overload',self.status['stall_overload'])
        print('Is Calibrated',self.is_calibrated)
        #self.motor.pretty_print()

    def step_sentry(self, robot):
        if self.hw_valid and self.robot_params['robot_sentry']['dynamixel_stop_on_runstop'] and self.params['enable_runstop']:
            is_runstopped = robot.pimu.status['runstop_event']
            if is_runstopped is not self.was_runstopped:
                if is_runstopped:
                    self.stop_trajectory()
                    self.disable_torque()
                else:
                    self.enable_torque()
            self.was_runstopped = is_runstopped

    # #####################################

    


    def set_motion_params(self,v_des=None,a_des=None):
        try:
            if not self.hw_valid:
                return

            v_des = v_des if v_des is not None else self.params['motion']['default']['vel']
            v_des = min(self.params['motion']['max']['vel'], v_des)
            if v_des != self.v_des:
                self.motor.set_profile_velocity(self.rad_per_sec_to_ticks(v_des))
                self.v_des = v_des

            a_des = a_des if a_des is not None else self.params['motion']['default']['accel']
            a_des = min(self.params['motion']['max']['accel'], a_des)
            if a_des != self.a_des:
                self.motor.set_profile_acceleration(self.rad_per_sec_sec_to_ticks(a_des))
                self.a_des = a_des
        except (termios.error, DynamixelCommError):
            #self.logger.warning('Dynamixel communication error on: %s' % self.name)
            self.comm_errors.add_error(rx=False, gsr=False)


    def move_by(self,x_des, v_des=None, a_des=None):
        if not self.hw_valid:
            return
        try:
            if abs(x_des) > 0.00002: #Avoid drift
                x=self.motor.get_pos()
                if not self.motor.last_comm_success and self.params['retry_on_comm_failure']:
                    x = self.motor.get_pos()

                if self.motor.last_comm_success:
                    cx=self.ticks_to_world_rad(x)
                    self.move_to(cx + x_des, v_des, a_des)
                else:
                    self.logger.debug('Move_By comm failure on %s' % self.name)
        except (termios.error, DynamixelCommError):
            #self.logger.warning('Dynamixel communication error on: %s' % self.name)
            self.comm_errors.add_error(rx=False, gsr=False)

   

    def enable_pos(self):
        if not self.hw_valid:
            return
        try:
            self.motor.disable_torque()
            if self.params['use_multiturn']:
                self.motor.enable_multiturn()
            else:
                self.motor.enable_pos()
            self.motor.set_profile_velocity(self.rad_per_sec_to_ticks(self.v_des))
            self.motor.set_profile_acceleration(self.rad_per_sec_sec_to_ticks(self.a_des))
            self.motor.enable_torque()
        except (termios.error, DynamixelCommError):
            self.comm_errors.add_error(rx=False, gsr=False)

    def enable_pwm(self):
        if not self.hw_valid:
            return
        try:
            self.motor.disable_torque()
            self.motor.enable_pwm()
            self.motor.enable_torque()
        except (termios.error, DynamixelCommError):
            self.comm_errors.add_error(rx=False, gsr=False)


    def set_pwm(self,x):
        if not self.hw_valid:
            return
        try:
            self.motor.set_pwm(x)
        except (termios.error, DynamixelCommError):
            self.comm_errors.add_error(rx=False, gsr=False)



    def startup(self):
        return DynamixelHelloXL430.startup(self)

    def home(self,move_to_zero=True):
        DynamixelHelloXL430.home(self,single_stop=False,move_to_zero=move_to_zero,delay_at_stop=3.0)

    def pretty_print(self):
        print('--- Stretchpollinator ----')
        print("Position (%)",self.status['pos_pct'])
        DynamixelHelloXL430.pretty_print(self)
# ##########################################

    def ticks_to_world_rad_per_sec(self,t):
        rps_servo=self.ticks_to_rad_per_sec(t)
        return self.polarity*rps_servo/self.params['gr']

    def ticks_to_world_rad(self,t):
        t=t-self.params['zero_t']
        rad_servo = self.ticks_to_rad(t)
        return (self.polarity*rad_servo/self.params['gr'])

    def world_rad_to_ticks(self,r):
        rad_servo = r*self.params['gr']*self.polarity
        t= self.rad_to_ticks(rad_servo)
        return t+self.params['zero_t']

    def world_rad_to_ticks_per_sec(self,r):
        rad_per_sec_servo = r*self.params['gr']*self.polarity
        t= self.rad_per_sec_to_ticks(rad_per_sec_servo)
        return t


    def ticks_to_rad(self,t):
        return deg_to_rad((360.0 * t / 4096.0))

    def rad_to_ticks(self,r):
        return int( 4096.0 * rad_to_deg(r) / 360.0)

    def ticks_to_rad_per_sec(self,t):
        rpm= t*0.229
        return deg_to_rad(rpm*360/60.0)

    def rad_per_sec_to_ticks(self,r):
        rpm=rad_to_deg(r)*60/360.0
        return int(rpm/0.229)

    def ticks_to_rad_per_sec_sec(self,t):
        rpmm=t*214.577
        return deg_to_rad(rpmm*360/60.0/60.0)

    def rad_per_sec_sec_to_ticks(self,r):
        rpmm=rad_to_deg(r)*60*60/360.0
        return int(rpmm/214.577)

    def ticks_to_pct_load(self,t):
        #-100 to 100.0
        return t/10.24

    def movetostartpos(self):
        self.enable_pos()
        #x_r = self.ticks_to_world_rad(self,pct)
        x_r = self.ticks_to_world_rad(8200)	
        DynamixelHelloXL430.move_to(self,x_des=x_r)
        print ('moved to start position')
     
    def movetopos2(self):
        self.enable_pos()
        #x_r = self.ticks_to_world_rad(self,pct)
        x_r = self.ticks_to_world_rad(6000)	
        DynamixelHelloXL430.move_to(self,x_des=x_r)
        print ('moved to position 2')

    def movetopos3(self):
        self.enable_pos()
        #x_r = self.ticks_to_world_rad(self,pct)
        x_r = self.ticks_to_world_rad(9200)	
        DynamixelHelloXL430.move_to(self,x_des=x_r)
        print ('moved to position 3')


    def movetozero(self):
        self.enable_pos()
            #x_r = self.ticks_to_world_rad(self,pct)
        x_r = self.ticks_to_world_rad(0)	
        DynamixelHelloXL430.move_to(self,x_des=x_r)
        print ('moved to zero')


    def open(self):
        self.enable_pos()
        x_r = self.ticks_to_world_rad(1000)
        DynamixelHelloXL430.move_to(self,x_des=x_r)

   
    def close(self):
        self.enable_pwm()
        DynamixelHelloXL430.set_pwm(self, -800)
        ts=time.time()
        time.sleep(1.0)
        timeout=False
        while self.motor.is_moving() and not timeout:
            timeout=time.time()-ts>15.0
            time.sleep(0.5)
        DynamixelHelloXL430.set_pwm(self,0)
        if timeout:
            print('Timed out')