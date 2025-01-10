import datetime
import os
import re
import shutil
import sys
import time
from time import monotonic

from avnav_api import AVNApi

sys.path.insert(0, os.path.dirname(__file__))

import ship_sim

SOURCE = "simulator"
TIME_FACTOR = "time_factor"
SEND_INTERVAL = "send_interval"
AIS_INTERVAL = "ais_interval"
NOISE_FACTOR = "noise_factor"
NMEA_FILTER = "nmea_filter"
RESTART = "restart_time"
AP_MODE = "AP_mode"
REPLAY_FILE = "replay_file"

NMEA_PRIORITY = "nmea_priority"
CONFIG = [
    {
        "name": TIME_FACTOR,
        "description": "sim speed-up time factor",
        "type": "FLOAT",
        "default": 1,
    },
    {
        "name": NOISE_FACTOR,
        "description": "artifical noise factor",
        "type": "FLOAT",
        "default": 1,
    },
    {
        "name": SEND_INTERVAL,
        "description": "NMEA send interval (s)",
        "type": "FLOAT",
        "default": 1,
    },
    {
        "name": AIS_INTERVAL,
        "description": "AIS data send interval (s),0=AIS disabled",
        "type": "FLOAT",
        "default": 10,
    },
    {
        "name": NMEA_FILTER,
        "description": "NMEA filter, sentences to send",
        "type": "STRING",
        "default": "$RMC,$VHW,$DBT,$MWV,$XDR",
    },
    {
        "name": NMEA_PRIORITY,
        "description": "NMEA priority",
        "type": "NUMBER",
        "default": 50,
    },
    {
        "name": RESTART,
        "description": "time (s) after which the simulation is restarted (0=no restart)",
        "type": "NUMBER",
        "default": 0,
    },
    {
        "name": AP_MODE,
        "description": "autopilot mode: 0=off 1=COG->BRG 2=HDT->BRG 3=steer to opt. VMC",
        "type": "NUMBER",
        "default": 1,
    },
    {
        "name": REPLAY_FILE,
        "description": "file with NMEA data to replay",
        "type": "STRING",
        "default": "",
    },
]


class Plugin(object):

    @classmethod
    def pluginInfo(cls):
        return {
            "description": "simple NMEA based simulator",
            "config": CONFIG,
            "data": [],
        }

    def get_file(self, filename):
        fn = os.path.join(self.api.getDataDir(), "tracks", filename)
        if os.path.isfile(fn):
            return fn

        fn = os.path.join(self.api.getDataDir(), "user", "viewer", filename)
        if not os.path.isfile(fn):
            source = os.path.join(os.path.dirname(__file__), filename)
            if os.path.isfile(source):
                shutil.copyfile(source, fn)

        return fn

    def __init__(self, api: AVNApi):
        self.api = api
        self.api.registerEditableParameters(CONFIG, self.changeParam)
        self.api.registerRestart(self.stop)
        self.seq = 0
        self.simconf_file = self.get_file("ship_sim.json")
        self.saveAllConfig()

    def stop(self):
        pass

    def getConfigValue(self, name):
        defaults = self.pluginInfo()["config"]
        for cf in defaults:
            if cf["name"] == name:
                return self.api.getConfigValue(name, cf.get("default"))
        return self.api.getConfigValue(name)

    def saveAllConfig(self):
        d = {}
        defaults = self.pluginInfo()["config"]
        for cf in defaults:
            v = self.getConfigValue(cf.get("name"))
            d.update({cf.get("name"): v})
        self.api.saveConfigValues(d)
        return

    def changeConfig(self, newValues):
        self.api.saveConfigValues(newValues)

    def changeParam(self, param):
        self.api.saveConfigValues(param)
        self.read_config()

    def read_config(self):
        config = {}
        for c in CONFIG:
            name = c["name"]
            TYPES = {"FLOAT": float, "NUMBER": int, "BOOLEAN": lambda s: s == "True"}
            value = self.getConfigValue(name)
            value = TYPES.get(c.get("type"), str)(value)
            config[name] = value
        self.config = config
        self.config_changed = True

    def get_rmb(self):
        self.seq, data = self.api.fetchFromQueue(
            self.seq, 100, filter="$RMB", waitTime=0.1
        )
        return data[-1].strip() if data else None

    def run(self):
        self.api.log("started")
        self.read_config()
        self.api.setStatus("STARTED", "running")

        simconf = ship_sim.read(self.simconf_file)

        def send_nmea(sentences):
            nmea = set()
            for l in sentences:
                # print(l)
                nmea.add(l[:6])
                self.api.addNMEA(
                    l,
                    source=SOURCE,
                    omitDecode=False,
                    sourcePriority=self.config[NMEA_PRIORITY],
                )

            self.api.setStatus(
                "NMEA", f"sending {sorted(nmea)} {self.config[REPLAY_FILE]}"
            )

        def serve(data, recv):
            rmb = self.get_rmb()
            if rmb:
                recv(rmb)
            send_nmea(data.splitlines())

        def replay_nmea():
            nmea_filter = self.config[NMEA_FILTER].strip().replace("$", "").split(",")
            nmea_filter = list(filter(lambda e:e,nmea_filter))
            replay = self.get_file(self.config[REPLAY_FILE])
            lines = []
            with open(replay) as f:
                t0 = None
                for l in f.readlines():
                    if self.api.shouldStopMainThread() or self.config_changed:
                        break
                    l = l.strip()
                    if re.match("[$!][A-Z]{5},", l):
                        if re.match(r"\$[A-Z]{2}RMC,", l):
                            # print(l)
                            send_nmea(lines)
                            lines = []
                            m = re.match(r"\$[A-Z]{5},(\d\d)(\d\d)(\d\d)\.?(\d\d)", l)
                            if m:
                                hms = list(map(int, m.groups()))
                                # print(m.groups(), hms)
                                t1 = datetime.time(
                                    hms[0],
                                    hms[1],
                                    hms[2],
                                    hms[3] * 1000 if len(hms) > 3 else 0,
                                )
                                dt = (
                                    datetime.datetime.combine(datetime.date.today(), t1)
                                    - datetime.datetime.combine(
                                        datetime.date.today(), t0
                                    )
                                    if t0
                                    else datetime.timedelta()
                                )
                                # print(t1, t0, dt, dt.total_seconds())
                                dt = min(max(0, dt.total_seconds()), 10)
                                t0 = t1
                            else:
                                dt = self.config[SEND_INTERVAL]
                            time.sleep(dt / self.config[TIME_FACTOR])
                        if l[3:6] in nmea_filter or not nmea_filter:
                            # print(l)
                            lines.append(l)

        while not self.api.shouldStopMainThread():
            try:
                self.config_changed = False
                if self.config[REPLAY_FILE]:
                    replay_nmea()
                else:
                    t0, r = monotonic(), self.config[RESTART] / self.config[TIME_FACTOR]
                    kwargs = {
                        k: self.config[k]
                        for k in (
                            TIME_FACTOR,
                            SEND_INTERVAL,
                            AIS_INTERVAL,
                            NOISE_FACTOR,
                            NMEA_FILTER,
                            AP_MODE,
                        )
                    }
                    kwargs["server"] = serve
                    kwargs["print"] = False
                    kwargs["stop"] = (
                        lambda: self.api.shouldStopMainThread()
                        or self.config_changed
                        or (r and monotonic() - t0 > r)
                    )
                    ship_sim.loop(simconf, **kwargs)
            except Exception as x:
                print(x)
                time.sleep(1)
                self.api.setStatus("ERROR", f"{x}")
