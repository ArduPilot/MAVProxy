"""
Python port of the ArduPilot basic PID controller.

AC_PID_Basic.h
Generic PID algorithm, with EEPROM-backed storage of constants.
"""

import math
import sys


# fftype.h: L63
def is_zero(value):
    return math.fabs(value) < sys.float_info.epsilon


# AP_Math.h: L75
def is_negative(value):
    return value <= (-1.0 * sys.float_info.epsilon)


# AP_Math.h: L64
def is_positive(value):
    return value >= sys.float_info.epsilon


# AP_Math.cpp: L266
def constrain_value_line(amt, low, high):
    # the check for NaN as a float prevents propagation of floating point
    # errors through any function that uses constrain_value(). The normal
    # float semantics already handle -Inf and +Inf
    if amt is float("nan"):
        raise Exception("constraining nan")
        return (low + high) / 2.0

    if amt < low:
        return low

    if amt > high:
        return high

    return amt


# AP_Math.h: L173
def constrain_float(amt, low, high):
    return constrain_value_line(float(amt), float(low), float(high))


# AP_Math.cpp: L400
# calculate a low pass filter alpha value
def calc_lowpass_alpha_dt(dt, cutoff_freq):
    if is_negative(dt) or is_negative(cutoff_freq):
        raise Exception("invalid_arg_or_result")
        return 1.0
    if is_zero(cutoff_freq):
        return 1.0
    if is_zero(dt):
        return 0.0
    rc = 1.0 / (2.0 * math.pi * cutoff_freq)
    return dt / (dt + rc)


class AP_PIDInfo:
    """
    Data used in PID controllers

    This structure provides information on the internal member data of
    a PID.  It provides an abstract way to pass PID information around,
    useful for logging and sending mavlink messages.
    """

    def __init__(self):
        self.target = float(0.0)
        self.actual = float(0.0)
        self.error = float(0.0)
        self.out = float(0.0)
        self.P = float(0.0)
        self.I = float(0.0)
        self.D = float(0.0)
        self.FF = float(0.0)
        self.DFF = float(0.0)
        self.Dmod = float(0.0)
        self.slew_rate = float(0.0)
        self.limit = bool(False)
        self.PD_limit = bool(False)
        self.reset = bool(False)
        self.I_term_set = bool(False)


class AC_PID_Basic:
    """
    AC_PID_Basic

    Copter PID control class
    """

    def __init__(
        self,
        initial_p,
        initial_i,
        initial_d,
        initial_ff,
        initial_imax,
        initial_filt_E_hz,
        initial_filt_D_hz,
    ):
        """
        Constructor for PID
        """
        # parameters
        self._kp = initial_p
        self._ki = initial_i
        self._kd = initial_d
        self._kff = initial_ff
        self._kimax = initial_imax
        self._filt_E_hz = initial_filt_E_hz  # PID error filter frequency in Hz
        self._filt_D_hz = initial_filt_D_hz  # PID derivative filter frequency in Hz

        # internal variables
        self._target = 0.0  # target value to enable filtering
        self._error = 0.0  # error value to enable filtering
        self._derivative = 0.0  # last derivative for low-pass filter
        self._integrator = 0.0  # integrator value

        # true when input filter should be reset during next call to set_input
        self._reset_filter = True

        self._pid_info = AP_PIDInfo()

    # set target and measured inputs to PID controller and calculate outputs
    # target and error are filtered
    # the derivative is then calculated and filtered
    # the integral is then updated based on the setting of the limit flag
    def update_all(self, target, measurement, dt, limit=False):
        return self.update_all_1(
            target,
            measurement,
            dt,
            (limit and is_negative(self._integrator)),
            (limit and is_positive(self._integrator)),
        )

    def update_all_1(self, target, measurement, dt, limit_neg, limit_pos):
        """
        update_all - set target and measured inputs to PID controller and calculate outputs
        target and error are filtered
        the derivative is then calculated and filtered
        the integral is then updated based on the setting of the limit flag
        """
        # don't process inf or NaN
        if (
            not math.isfinite(target)
            or math.isnan(target)
            or not math.isfinite(measurement)
            or math.isnan(measurement)
        ):
            raise Exception("invalid_arg_or_result")
            return 0.0

        self._target = target

        # reset input filter to value received
        if self._reset_filter:
            self._reset_filter = False
            self._error = self._target - measurement
            self._derivative = 0.0
        else:
            error_last = self._error
            self._error += self.get_filt_E_alpha(dt) * (
                (self._target - measurement) - self._error
            )

            # calculate and filter derivative
            if is_positive(dt):
                derivative = (self._error - error_last) / dt
                self._derivative += self.get_filt_D_alpha(dt) * (
                    derivative - self._derivative
                )

        # update I term
        self.update_i(dt, limit_neg, limit_pos)

        P_out = self._error * self._kp
        D_out = self._derivative * self._kd

        self._pid_info.target = self._target
        self._pid_info.actual = measurement
        self._pid_info.error = self._error
        self._pid_info.P = self._error * self._kp
        self._pid_info.I = self._integrator
        self._pid_info.D = self._derivative * self._kd
        self._pid_info.FF = self._target * self._kff
        self._pid_info.out = P_out + self._integrator + D_out + self._target * self._kff

        return self._pid_info.out

    def update_i(self, dt, limit_neg, limit_pos):
        """
        update the integral
        if the limit flags are set the integral is only allowed to shrink
        """
        if not is_zero(self._ki):
            # Ensure that integrator can only be reduced if the output is saturated
            if not (
                (limit_neg and is_negative(self._error))
                or (limit_pos and is_positive(self._error))
            ):
                self._integrator += (self._error * self._ki) * dt
                self._integrator = constrain_float(
                    self._integrator, -self._kimax, self._kimax
                )

        else:
            self._integrator = 0.0

    # get results from pid controller
    def get_p(self):
        return self._error * _kp

    def get_i(self):
        return self._integrator

    def get_d(self):
        return self._derivative * _kd

    def get_ff(self):
        return self._target * _kff

    def get_error(self):
        return self._error

    # reset the integrator
    def reset_I(self):
        self._integrator = 0.0

    # input and D term filter will be reset to the next value provided to set_input()
    def reset_filter(self):
        self._reset_filter = True

    # get accessors
    @property
    def kP(self):
        return self._kp

    @property
    def kI(self):
        return self._ki

    @property
    def kD(self):
        return self._kd

    @property
    def ff(self):
        return self._kff

    @property
    def filt_E_hz(self):
        return self._filt_E_hz

    @property
    def filt_D_hz(self):
        return self._filt_D_hz

    @property
    def imax(self):
        return self._kimax

    def get_filt_E_alpha(self, dt):
        return calc_lowpass_alpha_dt(dt, self._filt_E_hz)

    def get_filt_D_alpha(self, dt):
        return calc_lowpass_alpha_dt(dt, self._filt_D_hz)

    # set accessors
    @kP.setter
    def kP(self, value):
        self._kp = value

    @kI.setter
    def kI(self, value):
        self._ki = value

    @kD.setter
    def kD(self, value):
        self._kd = value

    @ff.setter
    def ff(self, value):
        self._kff = value

    @imax.setter
    def imax(self, value):
        self._kimax = math.fabs(value)

    @filt_E_hz.setter
    def filt_E_hz(self, hz):
        self._filt_E_hz = math.fabs(hz)

    @filt_D_hz.setter
    def filt_D_hz(self, hz):
        self._filt_D_hz = math.fabs(hz)

    # integrator setting functions
    def set_integrator_2(self, target, measurement, i):
        self.set_integrator_1(target - measurement, i)

    def set_integrator_1(self, error, i):
        self.set_integrator(i - error * self._kp)

    def set_integrator(self, i):
        self._integrator = constrain_float(i, -self._kimax, self._kimax)

    @property
    def pid_info(self):
        return self._pid_info
