#!/usr/bin/env python
'''enable/disable and configure a mavproxy router

TO USE:
    router help
        explains the router module

    router create
        create the router (not started)

    router delete
        delete the router and stop it

    router dir
        show the directory with JSON config files

    router dir reset
        set the directory with JSON config files to Current Working Directory

    router dir /path/to/dir
        set the directory with JSON config files to /path/to/dir.
        By default, this directory is the Current Working Directory

    router load my_router
        load the router named my_router.json. To load it, you must create a dict in the file,
        and place it in the selected directory (By default, Current Working Directory)

    router save my_router
        save the router in the file my_router.json,
        saved in the selected directory (By default, Current Working Directory)

    router show
        show the whole configuration

    router show ADDRESS
        show the whole sub-configuration related to ADDRESS

    router show ADDRESS SYSID
        show the compid-level sub-configuration related to SYSID for ADDRESS

    router show ADDRESS SYSID COMPID
        show the filter related to (SYSID, COMPID) and ADDRESS

    router check ADDRESS SYSID MSG_TYPE
        check if MSG_TYPE is accepted from SYSID (all compids) to ADDRESS

    router check ADDRESS SYSID COMPID MSG_TYPE
        check if MSG_TYPE is accepted from (SYSID, COMPID) to ADDRESS

    router clear
        clear the whole router

    router clear ADDRESS
        clear the config related to ADDRESS

    router clear ADDRESS SYSID
        clear msg_types accepted from SYSID at ADDRESS

    router clear ADDRESS SYSID COMPID
        clear msg_types accepted from (SYSID, COMPID) at ADDRESS

    router add ADDRESS
        add an address to the router

    router add ADDRESS SYSID
        ensure SYSID entry exists under ADDRESS

    router add ADDRESS SYSID COMPID MSG_TYPE1 MSG_TYPE2 ...
        add MSG_TYPEs accepted from (SYSID, COMPID) to ADDRESS

    router add ADDRESS SYSID MSG_TYPE1 MSG_TYPE2 ...
        add MSG_TYPEs accepted from SYSID (all compids) to ADDRESS

    router remove ADDRESS
        remove an address from the router

    router remove ADDRESS SYSID
        remove a SYSID entry from the config related to ADDRESS

    router remove ADDRESS SYSID COMPID
        remove a COMPID entry from SYSID at ADDRESS

    router remove ADDRESS SYSID MSG_TYPE1 MSG_TYPE2 ...
        remove MSG_TYPEs from the filter of SYSID (all compids) at ADDRESS

    router remove ADDRESS SYSID COMPID MSG_TYPE1 MSG_TYPE2 ...
        remove MSG_TYPEs from the filter of (SYSID, COMPID) at ADDRESS

    router set ADDRESS SYSID MSG_TYPE1 MSG_TYPE2 ...
        set the filter for SYSID (all compids) to exactly those MSG_TYPEs

    router set ADDRESS SYSID COMPID MSG_TYPE1 MSG_TYPE2 ...
        set the filter for (SYSID, COMPID) to exactly those MSG_TYPEs

    router start
        start the router

    router stop
        stop the router

NOTES:
    - in address, sysid or compid field, you can put "other", meaning that every address/sysid/compid
      that isn't mentioned is linked to this one.
    - in your JSON file, compid is not mandatory
    - in msg_types field, you can put:
        a str: meaning that only this msg_type from (sysid, compid) can be forwarded to this address;
        a list of str: meaning that only those msg_types from (sysid, compid) can be forwarded;
        None: meaning that no message from (sysid, compid) can be forwarded to this address;
        "all": meaning that every message from (sysid, compid) can be forwarded to this address;
        "all/MSG_TYPE1,MSG_TYPE2": meaning that every message except MSG_TYPE1 and MSG_TYPE2 can be forwarded.
'''

import os
import json

from MAVProxy.modules.lib import mp_module


help_str = """
Router configuration overview
-----------------------------

This router controls which MAVLink message types are forwarded to each network address.
Configuration is hierarchical: address → source system (sysid) → source component (compid) → message filter.

Key concepts:
- Address: network endpoint (without protocol prefix). If not listed, the special "other" address may be used as a fallback.
- SysID and CompID: MAVLink identifiers of the sender. Each can be listed explicitly or replaced by "other" to match everything else.
- Filter rules: decide which message types from that (sysid, compid) are forwarded to the given address.
- None or null      → no messages
- "all"             → all messages
- "all/MSG1,MSG2"   → all except MSG1, MSG2
- "MSG1" or ["MSG1","MSG2"] → only these messages

Configuration format (JSON):
{address: {sysid: {compid: filter}}}

Example:
{
    "192.168.2.1:14550": {
        "1": {
            "191": "HEARTBEAT",
            "other": ["ATTITUDE","GLOBAL_POSITION_INT"]
        },
        "other": "all/STATUSTEXT"
    },
    "other": {
        "other": null
    }
}

Explanation (messages sent *to 192.168.2.1:14550*):
- From sysid 1, compid 191 → only HEARTBEAT is forwarded.
- From sysid 1, any other compid → ATTITUDE and GLOBAL_POSITION_INT are forwarded.
- From all other sysids → all messages except STATUSTEXT are forwarded.
- From any sysid/compid going to any other address → nothing is forwarded.

Important notes:
- If an address/sysid/compid is not listed and there is no "other" entry at that level, then messages from it will NOT be forwarded at all.
- In the example above, no message is forwarded anywhere else: only the cases listed explicitly (or via "other") are valid.
- In short: anything not matched by an explicit rule or an "other" fallback is dropped.
- If the router is disabled, MAVProxy behaves normally and forwards all messages by default (no filtering).

Configs are plain JSON files. Use `router save` to export and `router load` to restore them.
"""


class FilterOut:
    '''class useful to filter msg types to forward'''
    def __init__(self):
        self.allow_all = False  # If True, all values are allowed except exceptions; if False, only exceptions values are allowed
        self.exceptions = set()  # set of exceptions from the rule above

    def add(self, value: str|list|None):
        '''adds values to the allowed list. Accepts a list or a single value.'''
        if isinstance(value, list):
            for val in value:
                self.add(val)
            return

        if value is None or value == "None" or value == "null":
            return

        if value == "all":  # We add everything
            self.allow_all = True
            self.exceptions.clear()

        elif value.startswith("all/"):  # We add everything except some
            not_added_values = set()
            for val in value[4:].split(","):  # Every msg_type that aren't added (i.e. exceptions)
                not_added_values.add(val.upper())
            if self.allow_all:  # Exceptions = msg_types not forwarded -> remove any exception not in not_added_values
                for val in self.exceptions:
                    if val not in not_added_values:
                        self.exceptions.remove(val)
            else:  # Exceptions = msg_types forwarded -> set allow_all to True and exceptions to some of not_added_values
                self.allow_all = True
                exceptions = set()
                for val in not_added_values:
                    if val not in self.exceptions: # The value isn't already forwarded and we don't want it added -> not forwarded
                        exceptions.add(val)
                self.exceptions.clear()
                for val in exceptions:
                    if val not in self.exceptions:
                        self.exceptions.add(val)

        else:  # We add only one thing
            if self.allow_all:  # Exceptions = msg_types not forwarded
                if value in self.exceptions:
                    self.exceptions.remove(value.upper())
            else:  # Exceptions = msg_types forwarded
                if value not in self.exceptions:
                    self.exceptions.add(value.upper())

    def remove(self, value: str|list|None):
        '''removes values from the allowed list. Accepts a list or a single value.'''
        if isinstance(value, list):
            for val in value:
                self.remove(val)
            return

        if value is None or value == "None" or value == "null":
            return

        if value == "all":  # We remove everything
            self.allow_all = False
            self.exceptions.clear()

        elif value.startswith("all/"):  # We remove everything except some
            not_removed_values = set()
            for val in value[4:].split(","):  # Every msg_types that are added (i.e. exceptions)
                not_removed_values.add(val.upper())
            if not self.allow_all:  # Exceptions = msg_types forwarded -> remove any exception not in not_removed_values
                for val in self.exceptions:
                    if val not in not_removed_values:
                        self.exceptions.remove(val)
            else:  # Exceptions = msg_types not forwarded -> set allow_all to False and exceptions to not_removed_values
                self.allow_all = False
                exceptions = set()
                for val in not_removed_values:
                    if val not in self.exceptions:  # The value is already forwarded and we don't want it removed -> forwarded
                        exceptions.add(val)
                self.exceptions.clear()
                for val in exceptions:
                    if val not in self.exceptions:
                        self.exceptions.add(val)

        else:  # We remove only one thing
            if not self.allow_all:  # Exceptions = msg_types forwarded
                if value in self.exceptions:
                    self.exceptions.remove(value.upper())
            else:  # Exceptions = msg_types not forwarded
                if value not in self.exceptions:
                    self.exceptions.add(value.upper())

    def set(self, value: str|list|None):
        '''explicitly sets the allowed values.'''
        if value is None or value == "None" or value == "null":
            self.allow_all = False
            self.exceptions.clear()
        elif isinstance(value, str):
            if value == "all":
                self.allow_all = True
                self.exceptions.clear()
            elif value.startswith("all/"):
                self.allow_all = True
                self.exceptions.clear()
                for val in value[4:].split(","):
                    self.exceptions.add(val.upper())
            else:
                self.allow_all = False
                self.exceptions.clear()
                self.exceptions.add(value.upper())
        elif isinstance(value, list):
            self.allow_all = False
            for val in value:
                self.exceptions.add(val.upper())

    def check(self, msg_type: str) -> bool:
        '''check if a msg_type can be forwarded'''
        if self.allow_all:
            return msg_type not in self.exceptions
        return msg_type in self.exceptions

    def save(self) -> None|str|list[str]:
        '''returns an object for saving the router_config.'''
        if self.allow_all:
            return f"all/{','.join(self.exceptions)}" if self.exceptions else "all"
        if not self.exceptions:
            return None
        if len(self.exceptions) == 1:
            return next(iter(self.exceptions))
        return list(self.exceptions)

    def __repr__(self) -> str:
        '''string representation used for printing.'''
        if self.allow_all:
            return f"all/{','.join(self.exceptions)}" if self.exceptions else "all"
        return ", ".join(self.exceptions) if self.exceptions else "null"


def _parse_id(key: str|int) -> str|int:
    '''if key looks like an integer (or is int), return int; otherwise return string as-is.'''
    if isinstance(key, str) and key.isdigit():
        return int(key)
    return key

def _is_id(key: str|int):
    return isinstance(key, int) or (isinstance(key, str) and key.isdigit()) or key == "other"

def _parse_address(address: str):
    if address.startswith(("udp:", "tcp:")):
        return address[4:]
    if address.startswith(("udpin:", "tcpin:")):
        return address[6:]
    if address.startswith(("udpout:", "tcpout:")):
        return address[7:]
    return address


class RouterModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(RouterModule, self).__init__(mpstate, "router", "router control (address / sysid / compid / filtering)", public=True)
        self.add_command('router', self.cmd_router, "router control",
                         ["<help|create|delete|dir|load|save|show|check|clear|add|remove|set|start|stop>"])
        self.router_config = None  # internal structure: { address: { sysid (int or 'other'): { compid (int or 'other'): FilterOut } } }
        self.router_enabled = False
        self.router_config_dir = os.getcwd()

    def cmd_router(self, args):
        '''handle router subcommands'''
        if len(args) < 1:
            self.cmd_router_show([])
        elif args[0] == "create":
            if len(args) != 1:
                print("Usage: router create")
                return
            self.cmd_router_create()
        elif args[0] == "delete":
            if len(args) != 1:
                print("Usage: router delete")
                return
            self.cmd_router_delete()
        elif args[0] == "dir":
            if len(args) > 2 or (len(args) >= 2 and args[1] == "help"):
                print("Usage 1: router dir")
                print("Usage 2: router dir reset")
                print("Usage 3: router dir /path/to/dir")
                return
            self.cmd_router_dir(args[1:])
        elif args[0] == "load":
            if len(args) != 2 or (len(args) == 2 and args[1] == "help"):
                print("Usage: router load FILE_NAME")
                return
            self.cmd_router_load(args[1:])
        elif args[0] == "save":
            if len(args) != 2 or (len(args) == 2 and args[1] == "help"):
                print("Usage: router save FILE_NAME")
                return
            self.cmd_router_save(args[1:])
        elif args[0] == "show":
            if len(args) > 4 or (len(args) >= 2 and args[1] == "help"):
                print("Usage 1: router show")
                print("Usage 2: router show ADDRESS")
                print("Usage 3: router show ADDRESS SRCSYSTEM")
                print("Usage 4: router show ADDRESS SRCSYSTEM COMPID")
                return
            self.cmd_router_show(args[1:])
        elif args[0] == "check":
            if len(args) not in [4, 5] or (len(args) == 4 and _is_id(args[-1])) or (len(args) >= 2 and args[1] == "help"):
                print("Usage 1: router check ADDRESS SRCSYSTEM MSG_TYPE")
                print("Usage 2: router check ADDRESS SRCSYSTEM COMPID MSG_TYPE")
                return
            self.cmd_router_check(args[1:])
        elif args[0] == "clear":
            if len(args) > 4 or (len(args) >= 2 and args[1] == "help"):
                print("Usage 1: router clear")
                print("Usage 2: router clear ADDRESS")
                print("Usage 3: router clear ADDRESS SRCSYSTEM")
                print("Usage 4: router clear ADDRESS SRCSYSTEM COMPID")
                return
            self.cmd_router_clear(args[1:])
        elif args[0] == "add":
            if len(args) == 1 or (len(args) >= 2 and args[1] == "help"):
                print("Usage 1: router add ADDRESS")
                print("Usage 2: router add ADDRESS SRCSYSTEM")
                print("Usage 3: router add ADDRESS SRCSYSTEM COMPID")
                print("Usage 4: router add ADDRESS SRCSYSTEM MSG_TYPE1 MSG_TYPE2 ...")
                print("Usage 5: router add ADDRESS SRCSYSTEM COMPID MSG_TYPE1 MSG_TYPE2 ...")
                return
            self.cmd_router_add(args[1:])
        elif args[0] == "remove":
            if len(args) == 1 or (len(args) >= 2 and args[1] == "help"):
                print("Usage 1: router remove ADDRESS")
                print("Usage 2: router remove ADDRESS SRCSYSTEM")
                print("Usage 3: router remove ADDRESS SRCSYSTEM COMPID")
                print("Usage 4: router remove ADDRESS SRCSYSTEM MSG_TYPE1 MSG_TYPE2 ...")
                print("Usage 5: router remove ADDRESS SRCSYSTEM COMPID MSG_TYPE1 MSG_TYPE2 ...")
                return
            self.cmd_router_remove(args[1:])
        elif args[0] == "set":
            if len(args) < 4 or (len(args) == 4 and _is_id(args[-1])) or (len(args) >= 2 and args[1] == "help"):
                print("Usage 1: router set ADDRESS SRCSYSTEM MSG_TYPE1 ...")
                print("Usage 2: router set ADDRESS SRCSYSTEM COMPID MSG_TYPE1 ...")
                return
            self.cmd_router_set(args[1:])
        elif args[0] == "start":
            if len(args) != 1:
                print("Usage: router start")
                return
            self.cmd_router_start()
        elif args[0] == "stop":
            if len(args) != 1:
                print("Usage: router stop")
                return
            self.cmd_router_stop()
        elif args[0] == "help":
            print(help_str)
            print("")
            print("usage: router <help|create|delete|load|save|show|check|clear|add|remove|set|start|stop>")
        else:
            print("usage: router <help|create|delete|load|save|show|check|clear|add|remove|set|start|stop>")

    def cmd_router_create(self):
        '''initialize the router'''
        if self.router_config is None:
            self.router_config = {}
            print("Router created")
        else:
            print("Router already created")

    def cmd_router_delete(self):
        '''delete the router'''
        if self.router_enabled:
            self.router_enabled = False
            print("Router disabled")
        if self.router_config is not None:
            self.router_config.clear()
            self.router_config = None
        print("Router deleted")

    def cmd_router_dir(self, args):
        '''set a directory with JSON files'''
        if not args:
            print(f"Selected directory: {self.router_config_dir}")
            return
        if args[0] == "reset":
            directory = os.getcwd()
        else:
            directory_raw = " ".join(args)
            directory = directory_raw.replace("'", "").replace('"', "")
        self.router_config_dir = directory
        print(f"Selected directory changed to {directory}")

    def cmd_router_load(self, args):
        '''load a router_config from a JSON file'''
        self.load(args[0])

    def cmd_router_save(self, args):
        '''save a router_config in a JSON file'''
        if self.router_config is None:
            print("No router_config to save")
            return
        # impose .json extension
        if not args[0].endswith(".json"):
            filename: str = f"{args[0]}.json"
        else:
            filename: str = args[0]

        router_to_save = {}
        for address, subconfig in self.router_config.items():
            router_to_save[address] = {}
            for sysid, compdict in subconfig.items():
                syskey = str(sysid)  # JSON keys must be strings
                router_to_save[address][syskey] = {}
                for compid, filterout in compdict.items():
                    compkey = str(compid)
                    router_to_save[address][syskey][compkey] = filterout.save()

        try:
            filepath = os.path.join(self.router_config_dir, filename)
            with open(filepath, "w", encoding="utf-8") as f:
                json.dump(router_to_save, f, ensure_ascii=False, indent=4)
            print(f"Router saved to {filename}")
        except Exception as e:
            print(f"Error while saving {filename}: {e}")

    def cmd_router_show(self, args):
        '''show the router_config'''
        if self.router_config is None:
            print("No router config created")
            return

        # no args: show entire structure
        if len(args) == 0:
            print("{")
            for addr, subdict in self.router_config.items():
                print(f'    {addr}: ' + '{')
                for sysid, compdict in subdict.items():
                    print(f'        {sysid}: ' + '{')
                    for compid, fo in compdict.items():
                        print(f'            {compid}: {fo}')
                    print("        }")
                print("    }")
            print("}")
            return

        address = _parse_address(args[0])
        if address not in self.router_config:
            print(f"address {address} not in router config")
            return

        if len(args) == 1:
            # show address-level
            print("{")
            for sysid, compdict in self.router_config[address].items():
                print(f'    {sysid}: ' + '{')
                for compid, fo in compdict.items():
                    print(f'        {compid}: {fo}')
                print("    }")
            print("}")
            return

        sysid = _parse_id(args[1])
        if sysid not in self.router_config[address]:
            print(f"sysid {sysid} not related to address {address}")
            return

        if len(args) == 2:
            # show sysid-level (all compids)
            print("{")
            for compid, fo in self.router_config[address][sysid].items():
                print(f'    {repr(compid)}: {fo}')
            print("}")
            return

        compid = _parse_id(args[2])
        if compid not in self.router_config[address][sysid]:
            print(f"compid {compid} not related to (address, sysid) ({address}, {sysid})")
            return

        print(self.router_config[address][sysid][compid])

    def cmd_router_check(self, args):
        '''show the router_config'''
        if self.router_config is None:
            print("False")
            return

        # address resolution (exact or 'other')
        address = _parse_address(args[0])

        if address not in self.router_config:
            if "other" in self.router_config:
                address = "other"
            else:
                print("False")
                return

        # sysid resolution (prefer numeric sysid, else 'other')
        sysid = _parse_id(args[1])
        if not _is_id(sysid):
            print(f"Invalid sysid: {sysid}")
            return
        if sysid not in self.router_config[address]:
            if "other" in self.router_config[address]:
                sysid = "other"
            else:
                print("False")
                return

        arg2 = _parse_id(args[2])
        if not _is_id(arg2):
            # arg2 is a MSG_TYPE, not a compid
            msg_type = arg2
            if not self.router_config[address][sysid]: # no compid in subconfig
                print("False")
                return
            # We check if every compid accepts or not
            n = 0
            for fo in self.router_config[address][sysid].values():
                if fo.check(msg_type):
                    n += 1
            if n == 0:
                print("False")
                return
            if n == len(set(self.router_config[address][sysid])):
                print("True")
                return
            print("Depends on compid")
            return

        # arg2 is compid
        compid = arg2
        if compid not in self.router_config[address][sysid]:
            if "other" in self.router_config[address][sysid]:
                compid = "other"
            else:
                print("False")
                return

        print(self.router_config[address][sysid][compid].check(args[3]))

    def cmd_router_clear(self, args):
        '''clear a part of the router_config'''
        if self.router_config is None:
            print("No router config available")
            return
        if len(args) == 0:
            self.router_config.clear()
            print("Whole router cleared")
            return

        address = _parse_address(args[0])
        if address not in self.router_config:
            print(f"Invalid address: {address}")
            return

        if len(args) == 1:
            # clear all under address
            self.router_config[address].clear()
            print(f"Subconfig linked to {address} cleared")
            return

        # args[1] is sysid
        sysid = _parse_id(args[1])
        if sysid not in self.router_config[address]:
            print(f"Invalid sysid {sysid} related to address {address}")
            return

        if len(args) == 2:
            # clear sysid
            self.router_config[address][sysid].clear()
            print(f"Subconfig linked to (address, sysid) ({address}, {sysid}) cleared")
            return

        # args[2] is compid
        compid = _parse_id(args[2])
        if compid not in self.router_config[address][sysid]:
            print(f"Invalid compid {compid} related to (address, sysid) ({address}, {sysid})")
            return

        self.router_config[address][sysid][compid].set(None)
        print(f"Subconfig linked to (address, sysid, compid) ({address}, {sysid}, {compid}) cleared")

    def cmd_router_add(self, args):
        '''add a part of the router_config'''
        if self.router_config is None:
            self.router_config = {}
            print("Router created")

        if len(args) == 0:
            return

        address = _parse_address(args[0])

        # if only address supplied -> done
        if len(args) == 1:
            if address not in self.router_config:
                self.router_config[address] = {}
                print(f"address {address} created")
            return

        # parse sysid
        sysid = _parse_id(args[1])
        if not _is_id(sysid):
            print(f"Invalid sysid: {sysid}")
            return

        # We create it afterward, in case sysid is invalid
        if address not in self.router_config:
            self.router_config[address] = {}
            print(f"address {address} created")

        if sysid not in self.router_config[address]:
            # create an empty compid dict; if user provided a simple value later it will be set
            self.router_config[address][sysid] = {}
            print(f"sysid {sysid} related to address {address} created")

        if len(args) == 2:
            return

        # decide if third arg is compid or the first msg_type
        # if third looks like an integer => it's compid
        arg2 = _parse_id(args[2])
        if _is_id(arg2):
            compid = arg2
            # ensure compid exists
            if compid not in self.router_config[address][sysid]:
                self.router_config[address][sysid][compid] = FilterOut()
                print(f"compid {compid} related to (address, sysid) ({address}, {sysid}) created")
            # remaining args are msg types to add
            if len(args) > 3:
                self.router_config[address][sysid][compid].add(args[3:])
                msg_types = args[3:] if len(args[3:]) > 1 else args[3]
                print(f"msg_type(s) {msg_types} related to (address, sysid, compid) ({address}, {sysid}, {compid}) added")
        else:
            # third is actually a msg_type -> operate on compid 'other'
            if not self.router_config[address][sysid]: # No compid mentionned
                self.router_config[address][sysid]["other"] = FilterOut()
                print(f"compid other related to (address, sysid) ({address}, {sysid}) created")
            if len(args) > 2:
                for fo in self.router_config[address][sysid].values():
                    fo.add(args[2:])
                msg_types = args[2:] if len(args[2:]) > 1 else args[2]
                print(f"msg_type(s) {msg_types} related to (address, sysid) ({address}, {sysid}) (all compids) added")

    def cmd_router_remove(self, args):
        '''remove a part of the router_config'''
        if self.router_config is None:
            print("No router config available")
            return

        if len(args) == 0:
            self.router_config.clear()
            print("Whole router removed")
            return

        address = _parse_address(args[0])
        if address not in self.router_config:
            print(f"Invalid address: {address}")
            return

        if len(args) == 1:
            # remove address entirely
            self.router_config.pop(address, None)
            print(f"address {address} removed")
            return

        # args[1] is sysid
        sysid = _parse_id(args[1])
        if not _is_id(sysid):
            print(f"Invalid sysid: {sysid}")
            return

        if sysid not in self.router_config[address]:
            print(f"Invalid sysid {sysid} related to address {address}")
            return

        if len(args) == 2:
            # remove the entire sysid entry
            self.router_config[address].pop(sysid, None)
            print(f"sysid {sysid} related to address {address} removed")
            return

        # args[2] might be compid or a msg type (but for remove we expect compid if present)
        arg2 = _parse_id(args[2])
        if _is_id(arg2):
            compid = arg2
            if compid not in self.router_config[address][sysid]:
                print(f"Invalid compid {compid} related to (address, sysid) ({address}, {sysid})")
                return
            if len(args) == 3:
                # remove the compid entry entirely
                self.router_config[address][sysid].pop(compid, None)
                print(f"compid {compid} related to (address, sysid) ({address}, {sysid}) removed")
            else:
                # remove listed msg types from this compid
                self.router_config[address][sysid][compid].remove(args[3:])
                msg_types = args[3:] if len(args[3:]) > 1 else args[3]
                print(f"msg_type(s) {msg_types} related to (address, sysid, compid) ({address}, {sysid}, {compid}) removed")
        else:
            # third isn't an int: treat it as msg types on 'other' compid
            if not self.router_config[address][sysid]: # no compid
                print(f"subconfig related to (address, sysid) ({address}, {sysid}) incomplete")
                return

            for fo in self.router_config[address][sysid].values():
                fo.remove(args[2:])
            msg_types = args[2:] if len(args[2:]) > 1 else args[2]
            print(f"msg_type(s) {msg_types} related to (address, sysid) ({address}, {sysid}) (all compids) removed")

    def cmd_router_set(self, args):
        '''set a part of the router_config'''
        if self.router_config is None:
            self.router_config = {}

        address = _parse_address(args[0])

        sysid = _parse_id(args[1])
        if not _is_id(sysid):
            print(f"Invalid sysid: {sysid}")

        if address not in self.router_config: # We create it afterward, in case of invalid sysid
            self.router_config[address] = {}

        if sysid not in self.router_config[address]:
            self.router_config[address][sysid] = {}

        arg2 = _parse_id(args[2])
        if _is_id(arg2): # arg2 is compid
            compid = arg2
            # ensure compid exists
            if compid not in self.router_config[address][sysid]:
                self.router_config[address][sysid][compid] = FilterOut()
            # set the filter to the provided remainder
            self.router_config[address][sysid][compid].set(args[3:])
            print(f"Filter related to (address, sysid, compid) ({address}, {sysid}, {compid}) set to {args[3:] if len(args[3:]) > 0 else None}")
        else:
            # third is not integer => it's a msg_type for compid 'other'
            self.router_config[address][sysid].clear()
            self.router_config[address][sysid]["other"] = FilterOut()
            self.router_config[address][sysid]["other"].set(args[2:])
            print(f"Filter related to (address, sysid) ({address}, {sysid}) set to {args[2:] if len(args[2:]) > 0 else None}")

    def cmd_router_start(self):
        '''start the router'''
        if self.router_config is None:
            print("Impossible to start router: router_config is None")
        else:
            if not self.router_enabled:
                self.router_enabled = True
                print("Router started")
            else:
                print("Router already started")

    def cmd_router_stop(self):
        '''stop the router'''
        if self.router_enabled:
            self.router_enabled = False
            print("Router stopped")
        else:
            print("Router already stopped")

    def load(self, filename):
        '''load a router_config from a JSON file.'''
        if not filename.endswith(".json"):
            filename = f"{filename}.json"

        filepath = os.path.join(self.router_config_dir, filename)
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"File {filename} not found in {self.router_config_dir}")

        try:
            with open(filepath, "r", encoding="utf-8") as f:
                router_config_raw = json.load(f) # The config created by the user
        except Exception as e:
            raise ImportError(f"Error while loading JSON {filename}: {e}")

        try:
            router_config = {}
            if not isinstance(router_config_raw, dict):
                raise ValueError("Data loaded isn't a dict")

            for address, subconfig in router_config_raw.items():
                if not isinstance(address, str):
                    raise ValueError(f'{address} is not a string')
                # remove udp:/tcp: prefixes
                address = _parse_address(address)
                router_config[address] = {}

                if not isinstance(subconfig, dict):
                    raise ValueError(f'subconfig {subconfig} related to address {address} is not a dict')

                for sysid, sys_val in subconfig.items():
                    sysid = _parse_id(sysid)
                    if not isinstance(sysid, int) and sysid != "other":
                        raise ValueError(f"sysid {sysid} related to address {address} is not an integer or 'other'")
                    router_config[address][sysid] = {}

                    if isinstance(sys_val, dict):
                        # in this case, we work with compid too
                        for compid, filter in sys_val.items():
                            compid = _parse_id(compid)
                            if not isinstance(compid, int) and compid != "other":
                                raise ValueError(f"compid {compid} related to (address, sysid) ({address}, {sysid}) is not an integer or 'other'")
                            router_config[address][sysid][compid] = FilterOut()
                            if isinstance(filter, str) or filter is None:
                                router_config[address][sysid][compid].set(filter)
                            elif isinstance(filter, list):
                                for msgtype in filter:
                                    if not isinstance(msgtype, str):
                                        raise ValueError(f"MSG_TYPE {msgtype} related to (address, sysid, srcComponent) ({address}, {sysid}, {compid}) is not a string")
                                router_config[address][sysid][compid].set(filter)
                            else: # Not an str or None or a list of str
                                raise ValueError(f"filter {filter} related to (address, sysid, srcComponent) ({address}, {sysid}, {compid}) is not a string, None or a list of strings")
                    else:
                        # In this case, we only work with sysid
                        compid = "other"
                        filter = sys_val
                        router_config[address][sysid][compid] = FilterOut()
                        if isinstance(filter, str) or filter is None:
                            router_config[address][sysid][compid].set(filter)
                        elif isinstance(filter, list):
                            for msgtype in filter:
                                if not isinstance(msgtype, str):
                                    raise ValueError(f"MSG_TYPE {msgtype} related to (address, sysid) ({address}, {sysid}) is not a string")
                            router_config[address][sysid][compid].set(filter)
                        else: # Not an str or None or a list of str
                            raise ValueError(f"filter {filter} related to (address, sysid) ({address}, {sysid}) is not a string, None or a list of strings")

            self.router_config = router_config
            print(f"Router successfully loaded from {filename}")
            return True

        except Exception as e:
            print(f"Error while generating configuration: {e}")
            return False

    def start(self):
        '''start the router'''
        if self.router_config is not None:
            if self.router_enabled:
                print("Router already started")
            else:
                self.router_enabled = True
                print("Router started")
        else:
            print("No router config available")

    def check(self, msg, address: str):
        '''check if a msg can be forwarded to an address using (sysid, srcComponent) resolution'''
        if self.router_config is None:
            return False

        # address resolution (exact or 'other')
        if address not in self.router_config:
            if "other" in self.router_config:
                address = "other"
            else:
                return False

        # sysid resolution (prefer numeric sysid, else 'other')
        sysid = "other"
        if hasattr(msg, 'get_srcSystem'):
            sysid = msg.get_srcSystem()
        if sysid not in self.router_config[address]:
            if "other" in self.router_config[address]:
                sysid = "other"
            else:
                return False

        # compid resolution (prefer numeric compid, else 'other')
        compid = "other"
        if hasattr(msg, 'get_srcComponent'):
            compid = msg.get_srcComponent()
        if compid not in self.router_config[address][sysid]:
            if "other" in self.router_config[address][sysid]:
                compid = "other"
            else:
                return False
        return self.router_config[address][sysid][compid].check(msg.get_type())


def init(mpstate):
    '''initialise module'''
    return RouterModule(mpstate)
