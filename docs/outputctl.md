# outputctl: add and remove UDP outputs at runtime from a GCS

The `outputctl` module lets a GCS ask a running MAVProxy, over UDP, to start
or stop forwarding MAVLink to a destination. It is the network equivalent of
typing `output add` / `output remove` on the MAVProxy console:

- MAVProxy keeps running. The master link, the `--out` outputs and any
  outputs added with `output add` are never changed by a request.
- An added output is an ordinary MAVProxy output, made by the same code as
  `output add udpout:<ip>:<port>`. It gets the same messages as the `--out`
  outputs. What its GCS sends back is forwarded to the vehicle under the same
  `mavfwd` settings. It shows up in `output list`.
- Requests are handled on MAVProxy's main loop, which also forwards the
  telemetry. A change to the outputs therefore never happens in the middle of
  forwarding. The control socket is non-blocking, and at most 32 requests are
  handled per loop iteration, so a flood of requests cannot stall telemetry.

The module is opt-in. Nothing listens until it is loaded.

## Starting MAVProxy

Add `--load-module=outputctl` to the usual command. The control socket then
listens on `127.0.0.1:14600` and accepts requests from `127.0.0.1` only.

```
mavproxy.py --master=udp:127.0.0.1:14550 --load-module=outputctl
```

SITL in WSL sending to the Windows host at 172.24.96.1:14552, with a
`--out` output kept alongside:

```
mavproxy.py --master=udp:172.24.96.1:14552 --out=udp:127.0.0.1:14551 --load-module=outputctl
```

A GCS on another machine (192.168.2.117) controlling a MAVProxy whose
address on that network is 192.168.2.10:

```
mavproxy.py --master=udp:127.0.0.1:14550 --load-module=outputctl --cmd="outputctl set bind 192.168.2.10; outputctl set allow 192.168.2.117"
```

The socket is opened once startup has finished, so settings given with
`--cmd` or in a startup script apply before it is first opened. Settings
changed later take effect straight away: changing `bind` or `port` reopens
the socket.

### Settings

| setting       | default     | meaning |
|---------------|-------------|---------|
| `bind`        | `127.0.0.1` | IPv4 address to listen on. `0.0.0.0` listens on every interface and prints a warning. |
| `port`        | `14600`     | UDP port to listen on. `0` lets the OS choose; `outputctl status` shows the port it chose. |
| `allow`       | `127.0.0.1` | Senders whose requests are accepted: IPv4 addresses and networks, comma separated, eg `192.168.2.117,10.0.0.0/24`. If any entry is invalid, every request is rejected. |
| `max_outputs` | `16`        | The most outputs which can be added (1-256). |

### Console commands

```
outputctl status              # socket state, settings and the added outputs
outputctl list                # the added outputs
outputctl clear               # remove the added outputs
outputctl set <name> <value>  # change a setting
```

If the socket cannot be opened, for instance because the port is in use, the
reason is printed once. MAVProxy carries on, and retries every 5 seconds.

## Protocol

- Transport: UDP over IPv4. Each request is one datagram, and its reply is
  one datagram sent to the address and port the request came from. Send each
  request from a fresh socket, or tell the replies apart by their text, since
  a late reply to an earlier request can otherwise be taken for the next one.
- Requests: ASCII text of at most 256 bytes. The command word is
  case-insensitive. Words are separated by spaces or tabs, and a trailing
  CR/LF is ignored. Control characters are refused.
- Replies: ASCII text. The first line starts with `OK ` or `ERR `. Lines are
  separated by `\n`, with no newline at the end.

### Requests

| request                  | success reply |
|--------------------------|---------------|
| `ADD <ip> <port>`        | `OK Added <ip>:<port>` |
| `REMOVE <ip> <port>`     | `OK Removed <ip>:<port>` |
| `LIST`                   | `OK List <n>`, then `n` lines of `<ip>:<port> active` |
| `CLEAR`                  | `OK Cleared <n> outputs` |

- `<ip>` is a unicast IPv4 address in plain dotted-quad form (`192.168.2.117`).
  Hostnames, IPv6, `0.0.0.0`, multicast, `255.255.255.255` and leading zeros
  (`192.168.002.117`) are refused.
- `<port>` is a decimal number from 1 to 65535.
- `LIST` lists only the outputs added with `ADD`, oldest first. `active`
  means the output is registered and being sent to. UDP cannot tell whether
  anything is receiving.
- `REMOVE` and `CLEAR` only ever remove outputs added with `ADD`.

Example exchange:

```
-> ADD 192.168.2.117 14550
<- OK Added 192.168.2.117:14550
-> ADD 10.10.10.5 14551
<- OK Added 10.10.10.5:14551
-> LIST
<- OK List 2
   192.168.2.117:14550 active
   10.10.10.5:14551 active
-> REMOVE 192.168.2.117 14550
<- OK Removed 192.168.2.117:14550
-> CLEAR
<- OK Cleared 1 outputs
```

### Errors

| reply | cause |
|-------|-------|
| `ERR Unauthorized sender` | the sender's address is not in `allow` |
| `ERR Empty request` | the datagram was empty or only whitespace |
| `ERR Request too long (max 256 bytes)` | |
| `ERR Request is not ASCII text` | |
| `ERR Request is not printable text` | it held control characters |
| `ERR Unknown command: <word>` | |
| `ERR Usage: ADD <ip> <port>` | wrong number of words; also for `REMOVE`, `LIST` and `CLEAR` |
| `ERR Invalid IP address: <ip>` | not a valid IPv4 address; ends with ` (not a unicast address)` for `0.0.0.0`, multicast and broadcast |
| `ERR Port must be 1-65535` | |
| `ERR Destination already exists: <ip>:<port>` | already added with `ADD` |
| `ERR Destination already exists: <ip>:<port> (configured output)` | already a `--out` or `output add` output |
| `ERR Destination is a MAVProxy socket: <ip>:<port>` | MAVProxy receives on that address itself, so sending there would loop |
| `ERR Too many outputs (max <n>)` | `max_outputs` reached |
| `ERR Failed to open output <ip>:<port>: <reason>` | the OS refused the socket (see the Windows notes) |
| `ERR Not found: <ip>:<port>` | `REMOVE` of something not added (or already removed) |
| `ERR Not a dynamic output: <ip>:<port>` | `REMOVE` of a `--out` or `output add` output |
| `ERR Internal error: <reason>` | a bug; the traceback is printed on the MAVProxy console |

A GCS should match on the text before the first `:`. A reply can be lost, so
retry after a timeout, and treat `ERR Destination already exists: <ip>:<port>`
after a retried `ADD`, and `ERR Not found: <ip>:<port>` after a retried
`REMOVE`, as success. `outputctl_client.py` in this directory does all of this.

Every accepted request except a successful `LIST` is printed on the MAVProxy
console. Rejected senders are summarised at most every 5 seconds.

## Lifetime

Added outputs are not saved anywhere. They are removed, and their sockets
closed, when the module is unloaded or reloaded and when MAVProxy exits. If an
added output is removed on the console with `output remove`, outputctl notices
and stops listing it.

## Security

- The socket listens on `127.0.0.1` unless `bind` says otherwise. Listening on
  every interface takes an explicit `outputctl set bind 0.0.0.0`.
- Every datagram's source address is checked against `allow` before anything
  else is done with it. An invalid `allow` rejects everything.
- On Windows the socket is opened with `SO_EXCLUSIVEADDRUSE`, so no other
  program can bind the same port and read the requests.
- The source address of a UDP datagram can be forged by anyone who can reach
  the socket, so `allow` is a filter, not authentication. On a network you do
  not trust, keep `bind` on an interface only trusted hosts can reach, or
  firewall the port.

## Windows notes

pymavlink 2.4.50 binds each UDP output socket on Windows to
`0.0.0.0:<destination port>`. This applies to `--out` and `output add`
outputs as much as to added ones, and it means that on Windows:

- Two outputs with the same destination port cannot exist together, even to
  different IPs. The second `ADD` is answered with
  `ERR Failed to open output <ip>:<port>: [WinError 10048] ...`. Give each
  destination its own port.
- A port that another program on the same PC listens on cannot be used for an
  output.
- A GCS on the same PC receives an output's messages, but anything it sends
  back to MAVProxy reaches itself rather than MAVProxy. A GCS on another host,
  WSL included, is not affected.

None of this applies on Linux, or with pymavlink versions which do not bind
UDP outputs.

## Testing

Automated tests (in-process tests, plus runs of a real `mavproxy.py` between a
fake vehicle and real UDP receivers):

```
python -m pytest -q tests/test_mavproxy_outputctl.py
```

By hand, with SITL:

1. Start SITL so that it sends to this PC, eg to 172.24.96.1:14552 from WSL.
2. Start MAVProxy as usual with `--load-module=outputctl`, eg
   `mavproxy.py --master=udp:172.24.96.1:14552 --out=udp:127.0.0.1:14551 --load-module=outputctl`.
   Check that the `--out` GCS on 14551 shows the vehicle, and that the
   console prints `outputctl: listening on 127.0.0.1:14600`.
3. Start a second GCS listening on another port, eg QGroundControl on UDP
   14560 on another machine, or `mavproxy.py --master=udpin:0.0.0.0:14560`.
4. `python docs/outputctl_client.py ADD <that machine's IP> 14560`. It
   answers `OK Added ...`, and the second GCS shows the vehicle straight away.
5. `python docs/outputctl_client.py LIST` shows it as active. Try a
   duplicate `ADD`, a bad address (`ADD 999.1.1.1 14560`) and a bad port
   (`ADD 127.0.0.1 0`): each is answered with an `ERR`, and both GCSs carry on.
6. `python docs/outputctl_client.py REMOVE <IP> 14560`. The second GCS loses
   the vehicle, and the `--out` GCS does not.
7. Add two outputs again and `python docs/outputctl_client.py CLEAR`. Both
   stop, and `--out` carries on.
8. `exit` on the MAVProxy console (or Ctrl-C). It prints
   `outputctl: closed control socket` and `outputctl: removed N dynamic outputs`.

Wireshark with the filter `udp.port == 14600` shows the requests and replies.
`mavlink` as the filter shows the forwarded telemetry if the MAVLink dissector
is installed.
