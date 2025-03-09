'''Joystick control classes'''


def scale(val,
          inlow=-1, inhigh=1,
          outlow=1000, outhigh=2000):
    '''Scale an in value in the range (inlow, inhigh) to the
    range (outlow, outhigh).'''
    return (
        ((float(val) - inlow) / (inhigh - inlow)) *
        (outhigh - outlow) + outlow
    )


class Control (object):
    '''Base class for all controls'''
    def __init__(self, joystick,
                 inlow=-1, inhigh=1,
                 outlow=1000, outhigh=2000,
                 values=[]):
        self.joystick = joystick
        self.inlow = inlow
        self.inhigh = inhigh
        self.outlow = outlow
        self.outhigh = outhigh
        self.values = values


class Button (Control):
    '''A Button acts like a momentary switch.  When pressed, the
    corresponding channel value is set to `outhigh`; when released,
    the value is set to `outlow`.'''

    def __init__(self, joystick, id, **kwargs):
        super(Button, self).__init__(joystick, **kwargs)
        self.id = id

    @property
    def value(self):
        state = self.joystick.get_button(self.id)
        if state:
            return self.outhigh
        else:
            return self.outlow


class ToggleButton (Control):
    '''A ToggleButton acts like a toggle or rotary switch.
    Initially, the corresponding channel value is set to the first value in `values`;
    when button `id` is pressed consecutively, the values are iterated on each press.
    Iteration is looped by starting from the beginning when last value is reached.'''

    def __init__(self, joystick, id, **kwargs):
        super(ToggleButton, self).__init__(joystick, **kwargs)
        self.id = id
        # index of value to set next
        self._current_value_index = 0
        # current value
        self._current_value = self.values[self._current_value_index]
        # last button state
        self._last_state = False

    @property
    def value(self):
        state = self.joystick.get_button(self.id)
        # button is pressed and last call it was not?
        if state and state != self._last_state:
            # choose new value for next press
            if self._current_value_index >= len(self.values)-1:
                # start over with first value
                self._current_value_index = 0
            else:
                # choose next value
                self._current_value_index += 1

            # get next value from "values" list
            self._current_value = self.values[self._current_value_index]

        # save new button state
        self._last_state = state

        return self._current_value


class MultiButton (Control):
    '''A MultiButton maps multiple buttons to the same channel like a
    multiple-position switch.  When a button is pressed, the channel
    is set to the corresponding value.  When a button is released, no
    changes is made to the channel.'''

    def __init__(self, joystick, buttons, **kwargs):
        super(MultiButton, self).__init__(joystick, **kwargs)
        self.buttons = buttons
        self._value = buttons[0]['value']

    @property
    def value(self):
        for button in self.buttons:
            state = self.joystick.get_button(button['id'])
            if state:
                self._value = button['value']
                break

        return self._value


class Axis (Control):
    '''An Axis maps a joystick axis to a channel.  Set `invert` to
    `True` in order to reverse the direction of the input.'''

    def __init__(self, joystick, id, invert=False, **kwargs):
        super(Axis, self).__init__(joystick, **kwargs)
        self.id = id
        self.invert = invert

    @property
    def value(self):
        val = self.joystick.get_axis(self.id)
        if self.invert:
            val = -val

        return scale(val, inlow=self.inlow, inhigh=self.inhigh, outlow=self.outlow, outhigh=self.outhigh)


class Hat (Control):
    '''A Hat maps one axis of a hat as if it were a toggle switch.
    When the axis goes negative, the corresponding channel value is
    set to `outputlow`.  When the axis goes positive, the value is set
    to `outputhigh`.  No change is made when the axis returns to 0.'''

    def __init__(self, joystick, id, axis, **kwargs):
        super(Hat, self).__init__(joystick, **kwargs)
        self.id = id
        self.axis = axis
        self._value = self.outlow

    @property
    def value(self):
        x, y = self.joystick.get_hat(self.id)

        value = x if self.axis == 'x' else y

        if value != 0:
            self._value = scale(value,
                                outlow=self.outlow, outhigh=self.outhigh)

        return self._value


class Joystick (object):
    '''A Joystick manages a collection of Controls.'''

    def __init__(self, joystick, controls):
        self.joystick = joystick
        self.controls = controls

        self.chan_max = max(control['channel']
                            for control in controls['controls'])
        self.channels = [None] * self.chan_max

        self.joystick.init()

        for control in controls['controls']:
            if control['type'] == 'button':
                kwargs = {k: control[k]
                          for k in control.keys()
                          if k in ['outlow', 'outhigh']}

                handler = Button(self.joystick, control['id'], **kwargs)

            elif control['type'] == 'toggle':
                kwargs = {k: control[k]
                          for k in control.keys()
                          if k in ['values']}

                handler = ToggleButton(self.joystick, control['id'], **kwargs)

            elif control['type'] == 'axis':
                kwargs = {k: control[k]
                          for k in control.keys()
                          if k in ['inlow', 'inhigh',
                                   'outlow', 'outhigh', 'invert']}

                handler = Axis(self.joystick, control['id'], **kwargs)

            elif control['type'] == 'multibutton':
                handler = MultiButton(self.joystick,
                                   buttons=control['buttons'])
            elif control['type'] == 'hat':
                kwargs = {k: control[k]
                          for k in control.keys()
                          if k in ['outlow', 'outhigh']}

                handler = Hat(self.joystick, control['id'], control['axis'])

            self.channels[control['channel']-1] = handler

    def read(self):
        '''Returns an array of channel values.  Return 0 for channels
        not specified in the control definition.'''

        return [int(handler.value) if handler is not None else 0
                for handler in self.channels]
