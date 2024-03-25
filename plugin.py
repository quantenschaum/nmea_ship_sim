import os
import shutil
import sys

from avnav_api import AVNApi

sys.path.insert(0, os.path.dirname(__file__))

import ship_sim

SOURCE = "simulator"
TIME_FACTOR = "time_factor"
SEND_INTERVAL = "send_interval"
AIS_INTERVAL = "ais_interval"
NOISE_FACTOR = "noise_factor"
NMEA_FILTER = "nmea_filter"

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
    "description": "AIS data send interval (s)",
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
]


class Plugin(object):

  @classmethod
  def pluginInfo(cls):
    return {
      'description': 'simple NMEA based simulator',
      "config": CONFIG,
      'data': []
    }

  def get_file(self, filename):
    fn = os.path.join(self.api.getDataDir(), "user", "viewer", filename)

    if not os.path.isfile(fn):
      source = os.path.join(os.path.dirname(__file__), filename)
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
    self.seq, data = self.api.fetchFromQueue(self.seq, 100, filter="$RMB", waitTime=0.1)
    return data[-1].strip() if data else None

  def run(self):
    self.api.log("started")
    self.read_config()
    self.api.setStatus('STARTED', 'running')

    simconf = ship_sim.read(self.simconf_file)

    def serve(data, recv):
      rmb = self.get_rmb()
      if rmb:
        recv(rmb)
      for l in data.splitlines():
        self.api.addNMEA(
          l,
          source=SOURCE,
          omitDecode=False,
          sourcePriority=self.config[NMEA_PRIORITY],
        )
      self.api.setStatus('NMEA', 'sending data')

    while not self.api.shouldStopMainThread():
      try:
        self.config_changed = False
        kwargs = {k: self.config[k] for k in (TIME_FACTOR, SEND_INTERVAL, AIS_INTERVAL, NOISE_FACTOR, NMEA_FILTER)}
        kwargs["server"] = serve
        kwargs["print"] = False
        kwargs["stop"] = lambda: self.api.shouldStopMainThread() or self.config_changed
        ship_sim.loop(simconf, **kwargs)
      except Exception as x:
        self.api.setStatus("ERROR", f"{x}")
