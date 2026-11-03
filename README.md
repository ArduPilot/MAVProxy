![GitHub Actions](https://github.com/ardupilot/MAVProxy/actions/workflows/windows_build.yml/badge.svg)

MAVProxy

This is a MAVLink ground station written in python. 

Please see https://ardupilot.org/mavproxy/index.html for more information

What this fork adds: outputctl
------------------------------

This fork of [ardupilot/MAVProxy](https://github.com/ardupilot/MAVProxy)
adds the `outputctl` module. It lets a ground station (GCS) add and remove
MAVProxy's UDP outputs at runtime, over UDP, without restarting MAVProxy.
A GCS can ask a running MAVProxy to start sending it the vehicle's
telemetry, and to stop again when it is done.

- **No restart:** outputs are added and removed while MAVProxy keeps
  forwarding telemetry.
- **Leaves the existing links alone:** the master link, `--out` outputs and
  outputs added on the console with `output add` are never changed.
- **Uses MAVProxy's own outputs:** an added output behaves exactly like
  `output add udpout:<ip>:<port>`, and it shows up in `output list`.
- **Restricted by default:** the module listens on `127.0.0.1:14600` and
  only accepts requests from `127.0.0.1`. Remote control has to be turned on
  explicitly, with a bind address and a list of allowed GCS IPs. Requests
  from any other sender are rejected.

### Quick start

Load the module when starting MAVProxy:

```
mavproxy.py --master=udp:127.0.0.1:14550 --load-module=outputctl
```

Then send requests with the example client:

```
python docs/outputctl_client.py ADD 192.168.2.117 14550
python docs/outputctl_client.py LIST
python docs/outputctl_client.py REMOVE 192.168.2.117 14550
python docs/outputctl_client.py CLEAR
```

Each request is a single line of text in a UDP datagram, so any program can
send them:

| request              | reply on success |
|----------------------|------------------|
| `ADD <ip> <port>`    | `OK Added <ip>:<port>` |
| `REMOVE <ip> <port>` | `OK Removed <ip>:<port>` |
| `LIST`               | `OK List <n>`, then one line per added output |
| `CLEAR`              | `OK Cleared <n> outputs` |

`REMOVE` and `CLEAR` only remove outputs that were added with `ADD`. Failed
requests are answered with `ERR <reason>`.

To control MAVProxy from a GCS on another machine, bind the control socket
to MAVProxy's address on that network and allow only the GCS's IP. In this
example, MAVProxy's address is 192.168.2.10 and the GCS is 192.168.2.117:

```
mavproxy.py --master=udp:127.0.0.1:14550 --load-module=outputctl --cmd="outputctl set bind 192.168.2.10; outputctl set allow 192.168.2.117"
```

On the MAVProxy console, `outputctl status` shows the socket and its
settings.

### More information

- [docs/outputctl.md](docs/outputctl.md): the full protocol, error replies,
  settings, security notes and Windows notes
- [docs/outputctl_client.py](docs/outputctl_client.py): example client
  (`--host` and `--port` select the MAVProxy to control)
- [tests/test_mavproxy_outputctl.py](tests/test_mavproxy_outputctl.py):
  the tests, run with `python -m pytest -q tests/test_mavproxy_outputctl.py`
  or `python -m unittest tests.test_mavproxy_outputctl`

The module code is in
[MAVProxy/modules/mavproxy_outputctl.py](MAVProxy/modules/mavproxy_outputctl.py).

This ground station was developed as part of the CanberraUAV OBC team
entry

License
-------

MAVProxy is released under the GNU General Public License v3 or later


Maintainers
-----------

The best way to discuss MAVProxy with the maintainers is to join the
mavproxy channel on ArduPilot discord at https://ardupilot.org/discord

Lead Developers: Andrew Tridgell and Peter Barker

Windows Maintainer: Stephen Dade

MacOS Maintainer: Rhys Mainwaring
