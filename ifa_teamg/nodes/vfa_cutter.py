from __future__ import print_function
from stretch_body.dynamixel_hello_XL430 import DynamixelHelloXL430
import time

class VFACutter(DynamixelHelloXL430):
    """
    API to the VFA Cutter
    The cutter motion is non-linear w.r.t to motor motion due to its design
    As such, the position of the cutter is represented at as unit-less value, 'pct'
    The Pct ranges from approximately -100 (fully closed) to approximately +50 (fully open)
    A Pct of zero is the fingertips just touching
    """
    def __init__(self, chain=None):
        DynamixelHelloXL430.__init__(self, 'vfa_cutter', chain)
        self.status['pos_pct']= 0.0
        self.pct_max_open=self.world_rad_to_pct(self.ticks_to_world_rad(self.params['range_t'][1])) #May be a bit greater than 50 given non-linear calibration
        self.poses = {'zero': 0,
                      'open': self.pct_max_open,
                      'close': -100}

    def startup(self):
        return DynamixelHelloXL430.startup(self)

    def home(self,move_to_zero=False):
        DynamixelHelloXL430.home(self,single_stop=False,move_to_zero=move_to_zero,delay_at_stop=3.0)

    def pretty_print(self):
        print('--- Stretchcutter ----')
        print("Position (%)",self.status['pos_pct'])
        DynamixelHelloXL430.pretty_print(self)

    def pose(self,p,v_r=None, a_r=None):
        """
        p: Dictionary key to named pose (eg 'close')
        """
        self.move_to(self.poses[p],v_r,a_r)

    def move_to(self,pct, v_r=None, a_r=None):
        """
        pct: commanded absolute position (Pct).
        v_r: velocity for trapezoidal motion profile (rad/s).
        a_r: acceleration for trapezoidal motion profile (rad/s^2)
        """
        x_r=self.pct_to_world_rad(pct)
        DynamixelHelloXL430.move_to(self,x_des=x_r, v_des=v_r, a_des=a_r)


    def move_by(self,delta_pct,v_r=None,a_r=None):
        """
        delta_pct: commanded incremental motion (pct).
        v_r: velocity for trapezoidal motion profile (rad/s).
        a_r: acceleration for trapezoidal motion profile (rad/s^2)
        """
        self.pull_status() #Ensure up to date
        self.move_to(self.status['pos_pct']+delta_pct,v_r,a_r)

    ############### Utilities ###############

    def pull_status(self,data=None):
        DynamixelHelloXL430.pull_status(self,data)
        self.status['pos_pct']=self.world_rad_to_pct(self.status['pos'])

    def pct_to_world_rad(self,pct):
        pct_to_tick = -1 * ((self.params['zero_t'] - self.params['range_t'][0]) / 100.0)
        t=((-1*pct)-100)*pct_to_tick
        r = DynamixelHelloXL430.ticks_to_world_rad(self, t)
        return r

    def world_rad_to_pct(self,r):
        pct_to_tick = -1 * ((self.params['zero_t'] - self.params['range_t'][0]) / 100.0)
        t = DynamixelHelloXL430.world_rad_to_ticks(self,r)
        pct = -1*((t / pct_to_tick)+100)
        return pct

    def step_sentry(self, robot):
        """This sentry attempts to prevent the cutter servo from overheating during a prolonged grasp
        When the servo is stalled and exerting an effort above a threshold it will command a 'back off'
        position (slightly opening the grasp). This reduces the PID steady state error and lowers the
        commanded current. The cutter's spring design allows it to retain its grasp despite the backoff.
        """
        DynamixelHelloXL430.step_sentry(self, robot)
        if self.hw_valid and self.robot_params['robot_sentry']['stretch_cutter_overload'] and not self.is_homing:
            if self.status['stall_overload']:
                if self.status['effort'] < 0: #Only backoff in open direction
                    self.logger.debug('Backoff at stall overload')
                    self.move_by(self.params['stall_backoff'])

    def open(self):
        self.enable_pos()
        x_r = self.ticks_to_world_rad(1000)
        DynamixelHelloXL430.move_to(self,x_des=x_r)

    def open_slight(self):
        self.enable_pos()
        x_r = self.ticks_to_world_rad(300)
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
	
