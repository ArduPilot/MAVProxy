#!/usr/bin/env python

'''This is a script that will help you identify controls on your joystick
in order to create an appropriate joystick definition for the joystick
module.  You can run it like this:

    python -m MAVProxy.modules.mavproxy_joystick.findjoy
'''

from __future__ import print_function

import argparse
import collections
import pygame
import sys
import time
import time

State = collections.namedtuple('State',
                               ['axes', 'buttons', 'hats', 'balls'])


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('joystick', nargs='?', type=int)

    return p.parse_args()


def list_joysticks():
    '''Print a list of available joysticks'''

    print('Available joysticks:')
    print()
    for jid in range(pygame.joystick.get_count()):
        j = pygame.joystick.Joystick(jid)
        print('({}) {}'.format(jid, j.get_name()))


def select_joystick():
    '''Allow user to select a joystick from a menu'''

    list_joysticks()

    while True:
        print('Select a joystick (L to list, Q to quit)'),
        choice = sys.stdin.readline().strip()

        if choice.lower() == 'l':
            list_joysticks()
        elif choice.lower() == 'q':
            return
        elif choice.isdigit():
            jid = int(choice)
            if jid not in range(pygame.joystick.get_count()):
                print('Invalid joystick.')
                continue
            break
        else:
            print('What?')

    return jid


def get_joystick_state(joy):
    axes = [joy.get_axis(x) for x in range(joy.get_numaxes())]
    buttons = [joy.get_button(x) for x in range(joy.get_numbuttons())]
    hats = [joy.get_hat(x) for x in range(joy.get_numhats())]
    balls = [joy.get_ball(x) for x in range(joy.get_numballs())]

    state = State(axes, buttons, hats, balls)

    return state


def main():
    print('\n'.join([
        'This script will help you identify axis, button, hat, and ',
        'ball ids on your joystick in order to create new joystick ',
        'definitions for the mavproxy_joystick module.',
        ''
    ]))

    pygame.init()
    pygame.joystick.init()

    args = parse_args()

    if args.joystick is None:
        args.joystick = select_joystick()
        if args.joystick is None:
            print('No joystick selected.')
            return

    joy = pygame.joystick.Joystick(args.joystick)
    joy.init()

    print('Joystick: {}'.format(joy.get_name()))
    print('  Axes: {}'.format(joy.get_numaxes()))
    print('  Buttons: {}'.format(joy.get_numbuttons()))
    print('  Hats: {}'.format(joy.get_numhats()))
    print('  Balls: {}'.format(joy.get_numballs()))
    print()

    pygame.event.clear()
    while True:
        print('Move a control on your joystick')
        print()
        e = pygame.event.wait()
        if e.type == pygame.JOYAXISMOTION:
            print('Axis', e.axis)
        elif e.type in [pygame.JOYBUTTONDOWN, pygame.JOYBUTTONUP]:
            print('Button', e.button)
        elif e.type == pygame.JOYHATMOTION:
            print('Hat', e.hat)
        elif e.type == pygame.JOYBALLMOTION:
            print('Ball', e.ball)

        # consume events
        t0 = time.time()
        while True:
            t1 = time.time()
            if not pygame.event.get() and (t1 - t0 > 1):
                break

if __name__ == '__main__':
    main()
