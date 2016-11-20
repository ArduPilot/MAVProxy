MAVProxy has a flexible joystick driver that lets you map your
joystick to RC channels using configuration files written in [YAML][]
syntax.

[yaml]: https://en.wikipedia.org/wiki/YAML

## File format

A joystick definition be located either inside the `MAVProxy` python
module, or in your `.mavproxy` directory (normally `$HOME/.mavproxy`)
in the `joysticks` directory.

A minimal joystick definition might look like this:

    description: This is a sample joystick definition.
    match:
      - ACME Joystick Company*
    controls:

      # map axis id 2 to rc channel 1
      - channel: 1
        type: axis
        id: 2

      # map axis id 3 to rc channel 2
      - channel: 2
        type: axis
        id: 3

      # map axis id 1 to rc channel 3
      - channel: 3
        type: axis
        id: 1
        invert: true

      # map axis id 0 to rc channel 4
      - channel: 4
        type: axis
        id: 0

The configuration file is made up of the following sections.

### description

This section is a simple string describing the module.  It will be
displayed in the output of `joystick status` when the given joystick
definition is active.

    description: This is a description.

You can take advantage of YAML block constructs for longer
descriptions, like this:

    description: >
      This is a much longer description.  It may contain many lines
      and even multiple paragraphs, although if you're trying to put
      documentation into the description maybe you should find a
      better place for it.

### match

This is a list of [fnmatch][] patterns that will be matched against
device names.  The first definition found with a match will be
selected as the active joystick definition.

For example, given a `match` section like this:

    match:
      - ACME Joystick Co*
      - Widget Ltd*

The definition would be selected for a device identified as a "ACME
Joystick Company Funpad 3000" or for a device identified as a "Widget
Ltd Joy-o-tron".

[fnmatch]: https://docs.python.org/2/library/fnmatch.html


### controls

This is the section that maps joystick inputs to rc channels.  It
consists of a list of control definitions of the form:

    - channel: <rc channel number>
      type: (axis|button|multibutton|hat)
      id: <axis id|button id|hat id>
      ...zero or more type-specific parameters...

For example:

    - channel: 1
      type: axis
      id: 1

See "Control types" for more information.

## Control types

All the control types will accept the following additional parameters:

- `inlow`, `inhigh` -- these define the input range of the control.
  Default to `-1` and `1`, respectively.
- `outlow`, `outhigh` -- these define the output range of the rc
  channel.  Default to `1000` and `2000`, respectively.

### axis

An `axis` control uses the value of `inlow`, `inhigh`, `outlow`, and
`outhigh` to scale input values read from the controller to rc channel
overrides provided to mavproxy.

Example:

    - channel: 1
      type: axis
      id: 1

### button

A `button` acts like a momentary switch.  When pressed, the
corresponding channel value is set to `outhigh`; when released,
the value is set to `outlow`.

Example:

    - channel: 1
      type: button
      id: 1

### multibutton

A `multibutton` maps multiple buttons to the same channel like a
multiple-position switch.  When a button is pressed, the channel
is set to the corresponding value.  When a button is released, no
changes is made to the channel.

Example:

    - channel: 5
      type: multibutton
      buttons:
        - id: 0
          value: 1200
        - id: 1
          value: 1300
        - id: 2
          value: 1400
        - id: 3
          value: 1500

### hat

A `hat` maps one axis of a hat as if it were a toggle switch.
When the axis goes negative, the corresponding channel value is
set to `outputlow`.  When the axis goes positive, the value is set
to `outputhigh`.  No change is made when the axis returns to 0.

Example:

    - channel: 7
      type: hat
      id: 0
      axis: x
    - channel: 8
      type: hat
      id: 0
      axis: y

## Creating new joystick definitions

The `joystick` module includes a simple script to help you identify
controls on your joystick.  Run it like this:

    python -m MAVProxy.modules.mavproxy_joystick.findjoy

And then follow the prompts.
