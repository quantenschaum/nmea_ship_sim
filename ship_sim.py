#!/usr/bin/env python3
import builtins
import json
import re
import select
import socket
import sys
from datetime import datetime, timedelta
from math import sin, cos, radians, degrees, atan2, sqrt, copysign, isfinite, asin, nan, pi
from os.path import isfile
from random import gauss
from time import monotonic
from time import sleep

# pip install numpy scipy pyais
import numpy
import pyais
import scipy

TIME_FACTOR = 1  # speedup time
SEND_INTERVAL = 1  # interval for sending NMEA data
AIS_INTERVAL = 10  # interval (s) for emitting AIS sentences
NOISE_FACTOR = 1  # scale measurement noise
AUTO_PILOT = 2  # enable autopilot, set to 2 to steer to optimal VMC
POS_JSON = "ship_pos.json"  # store position and heading in this file
# NMEA sentences with are sent to clients
NMEA_FILTER = "RMC,VHW,DBT,MWV,XDR"

start = monotonic()

print = builtins.print


def main():
  config = read("ship_sim.json")
  kwargs = {}

  tcp_port = config.get("tcp_port", 6000)
  kwargs["server"] = Server("", tcp_port).serve

  loop(config, **kwargs)


def loop(config, **kwargs):
  global TIME_FACTOR, SEND_INTERVAL, AIS_INTERVAL, NOISE_FACTOR, NMEA_FILTER, print
  TIME_FACTOR = kwargs.get("time_factor", TIME_FACTOR)
  SEND_INTERVAL = kwargs.get("send_interval", SEND_INTERVAL)
  AIS_INTERVAL = kwargs.get("ais_interval", AIS_INTERVAL)
  NOISE_FACTOR = kwargs.get("noise_factor", NOISE_FACTOR)
  NMEA_FILTER = kwargs.get("nmea_filter", NMEA_FILTER).replace("$", "").split(",")
  stop = kwargs.get("stop", lambda: False)
  server = kwargs["server"]

  do_print = kwargs.get("print", True)
  if not do_print:
    print = lambda *a, **kw: None

  polar = Polar(config["polar"])
  heel = Polar(config["heel"])
  atons = config.get("atons", [])
  ships = [Ship(**d, _polar=polar, _pheel=heel) for d in config["ships"]]

  own = ships[0]

  def nmea():
    sentences = own.nmea()
    t = monotonic()
    for s in ships[1:]:
      if s.ais_class == "A":
        if t - s.ais_time_dynamic > AIS_INTERVAL:
          sentences += "\n".join(ais_1(s)) + "\n"
          s.ais_time_dynamic = t
        if t - s.ais_time_static > 5 * AIS_INTERVAL:
          sentences += "\n".join(ais_5(s)) + "\n"
          s.ais_time_static = t
      else:
        if t - s.ais_time_dynamic > AIS_INTERVAL:
          sentences += "\n".join(ais_19(s)) + "\n"
          s.ais_time_dynamic = t

    for a in atons:
      if t - a.get("ais_time", 0) > 3 * AIS_INTERVAL:
        sentences += "\n".join(ais_21(a)) + "\n"
        a["ais_time"] = t

    return sentences

  t0 = monotonic()
  while not stop():
    t1 = monotonic()
    for s in ships:
      s.current_set = own.current_set
      s.current_drift = own.current_drift
      s.wind_dir_ground = own.wind_dir_ground
      s.wind_speed_ground = own.wind_speed_ground
      s.update()
    if t1 - t0 > SEND_INTERVAL:
      print(own)
      if isfile(POS_JSON):
        write(POS_JSON, [own.position, own.heading_true])
      server(nmea(), own.autopilot)
      t0 = t1
    sleep(1 / TIME_FACTOR)


class Ship:
  props = {
    "position": [0, 0],
    "heading_true": 0,  # degrees
    "speed_thr_water": 0,  # knots
    "sailing": False,  # if to calculate boat speed from wind via polar
    "rate_of_turn": 0,  # deg/sec
    # "mag_deviation": 0,
    "mag_variation": 0,
    "rudder_angle": 0,
    "engine_rpm": 0,
    "depth": 10,
    "wind_speed_ground": 0,
    "wind_dir_ground": 0,
    "current_set": 0,
    "current_drift": 0,
    "leeway_factor": 8,
    "mmsi": 0,
    "imo": 0,
    "name": "",
    "callsign": "",
    "destination": "",
    "ais_class": "A",
    "ais_navstat": 0,
  }

  def __init__(self, **kwargs):
    self.load(kwargs)
    self.t0=monotonic()
    self.GWD=self.wind_dir_ground
    self.GWS=self.wind_speed_ground
    self.SET=self.current_set
    self.DFT=self.current_drift
    self.time = datetime.utcnow()
    self.sign = 0  # used by autopilot
    self.ais_time_dynamic = 0  # used for tracking AIS interval
    self.ais_time_static = 0  # for static data
    self.update(0)

  def load(self, props):
    self.__dict__.update(Ship.props)
    self.__dict__.update(props)
    if self.rate_of_turn:
      self.rudder_angle = nan

  def save(self):
    return {k: self.__dict__[k] for k in Ship.props.keys() if k in self.__dict__}

  def update(self, delta_t=1):
    "update state, simple forward integration of motion to new position"
    self.time = self.time + timedelta(seconds=delta_t)
    if TIME_FACTOR == 1:
      self.time = datetime.utcnow()

    if isfinite(self.rudder_angle):
      # rudder angle causes rate of turn, but proportional to speed
      rot = self.rudder_angle * max(0.2, min(2, self.speed_thr_water / 2)) / 4
      b = 0.2  # delayed change (exponential smoothing)
      self.rate_of_turn += b * (rot - self.rate_of_turn)

    self.heading_true = to360(self.heading_true + self.rate_of_turn * delta_t)
    self.heading_mag = to360(self.heading_true - self.mag_variation)
    # simple deviation model to simulate compass heading
    self.mag_deviation = 3 * sin(radians(self.heading_mag + 52))
    self.heading_cmp = to360(self.heading_mag - self.mag_deviation)


    def shift(t,coeff={}):
      return sum(a*sin(2*pi*f*t) for f,a in coeff.items())

    t = TIME_FACTOR*(monotonic()-self.t0)
    self.wind_dir_ground = to360(self.GWD+shift(t,{1/600:10,1/333:6,1/123:3}))
    self.wind_speed_ground = self.GWS+shift(t,{1/66:1,1/222:1.5,1/674:2})

    self.wind_angle_ground = to180(self.wind_dir_ground - self.heading_true)

    self.current_drift = self.DFT*shift(t,{1/(12.5*3600):1}) # 12.5h period
    self.current_set = to360(self.SET + (0 if self.current_drift>0 else 180))
    self.current_drift = abs(self.current_drift)

    self.wind_dir_true, self.wind_speed_true = add_polar(
      (self.current_set, self.current_drift),
      (self.wind_dir_ground, self.wind_speed_ground),
    )
    self.wind_angle_true = to180(self.wind_dir_true - self.heading_true)

    if self.sailing:
      # boat speed from true wind from polar data
      stw = self._polar.value(self.wind_angle_true, self.wind_speed_true)
      b = 0.05  # delayed change
      self.speed_thr_water += b * (stw - self.speed_thr_water)

      self.heel_angle = copysign(self._pheel.value(
        self.wind_angle_true, self.wind_speed_true
      ),-self.wind_angle_true)
      self.leeway = (
        clamp(
          self.leeway_factor * self.heel_angle / self.speed_thr_water ** 2,
          (-30, +30),
        )
        if self.speed_thr_water
        else 0
      )
    else:
      self.heel_angle, self.leeway = 0, 0

    self.course_thr_water = to360(self.heading_true + self.leeway)

    self.course_over_ground, self.speed_over_ground = add_polar(
      (self.current_set, self.current_drift),
      (self.course_thr_water, self.speed_thr_water),
    )

    distance = self.speed_over_ground * delta_t / 3600
    self.position = project(self.position, self.course_over_ground, distance)

    self.wind_dir_app, self.wind_speed_app = add_polar(
      (self.course_over_ground, self.speed_over_ground),
      (self.wind_dir_ground, self.wind_speed_ground),
    )
    self.wind_angle_app = to180(self.wind_dir_app - self.heading_true)

    # self.wind_dir_ground=to360(noisy(self.wind_dir_ground,absolute=0.5))
    # self.wind_speed_ground=max(5,min(20,noisy(self.wind_speed_ground,absolute=0.5)))

  def __str__(self):
    return "\n".join(
      [
        f"TME {self.time}",
        f"POS {self.position}",
        f"MAG {self.mag_deviation}D {self.mag_variation}V",
        f"HDG {self.heading_true}T {self.heading_mag}M {self.heading_cmp}C  ROT {self.rate_of_turn}",
        f"LEE {self.leeway}  HEL {self.heel_angle}",
        f"CRS {self.course_thr_water}  STW {self.speed_thr_water}",
        f"SET {self.current_set}  DRF {self.current_drift}",
        f"COG {self.course_over_ground}  SOG {self.speed_over_ground}",
        f"GWD {self.wind_dir_ground}  GWA {self.wind_angle_ground}  GWS {self.wind_speed_ground}",
        f"TWD {self.wind_dir_true}  TWA {self.wind_angle_true}  TWS {self.wind_speed_true}",
        f"AWD {self.wind_dir_app}  AWA {self.wind_angle_app}  AWS {self.wind_speed_app}",
        f"DPT {self.depth}  RUD {self.rudder_angle}  RPM {self.engine_rpm}",
      ]
    )

  def nmea(self):
    # https://gpsd.gitlab.io/gpsd/NMEA.html
    t = self.time
    hhmmss = f"{t.hour:02d}{t.minute:02d}{t.second:02d}"
    hhmmssss = hhmmss + f"{t.microsecond * 1e-6:.2f}"[1:]
    ddmmyy = f"{t.day:02d}{t.month:02d}{t.year % 100:02d}"
    sats = 12
    fix_quality = 8  # 0=NO 1=GPS 2=DGPS 8=SIM
    hdop = noisy(1, relative=10)
    lat = noisy(self.position[0], absolute=hdop / 1852 / 60)
    latdeg, latmin = divmod(abs(lat) * 60, 60)
    latsgn = "S" if lat < 0 else "N"
    lon = noisy(self.position[1], absolute=hdop / 1852 / 60 * cos(radians(lat)))
    londeg, lonmin = divmod(abs(lon) * 60, 60)
    lonsgn = "W" if lon < 0 else "E"
    sog = noisy(self.speed_over_ground)
    cog = to360(noisy(self.course_over_ground, absolute=1))
    stw = noisy(self.speed_thr_water)
    altitude = noisy(3)
    rot = noisy(self.rate_of_turn * 60)
    depth = noisy(self.depth)  # below transducer

    magdev = self.mag_deviation
    magdevdir = "W" if magdev < 0 else "E"
    magvar = self.mag_variation
    magvardir = "W" if magvar < 0 else "E"

    heading = to360(noisy(self.heading_true, absolute=1))
    heading_mag = to360(noisy(self.heading_mag, absolute=1))
    heading_cmp = to360(noisy(self.heading_cmp, absolute=1))

    wind_speed_true = noisy(self.wind_speed_true)
    wind_dir_true = to360(noisy(to360(self.wind_dir_true), absolute=1))
    wind_angle_true = to180(noisy(to360(self.wind_angle_true), absolute=1))
    wind_speed_app = noisy(self.wind_speed_app)
    wind_angle_app = to180(noisy(to360(self.wind_angle_app), absolute=1))

    sentences = [
      f"GPRMC,{hhmmssss},A,{latdeg:02.0f}{latmin:06.3f},{latsgn},{londeg:03.0f}{lonmin:06.3f},{lonsgn},{sog:05.1f},{cog:05.1f},{ddmmyy},{abs(magvar):.1f},{magvardir}",
      f"GPGGA,{hhmmssss},{latdeg:02.0f}{latmin:05.2f},{latsgn},{londeg:03.0f}{lonmin:05.2f},{lonsgn},{fix_quality},{sats:02d},{hdop:03.1f},{altitude:.1f},M,{0:.1f},M,,",
      f"GPZDA,{hhmmssss},{t.day:02d},{t.month:02d},{t.year:04d},{1:02d},{0:02d}",
      f"GPGLL,{latdeg:02.0f}{latmin:07.04f},{latsgn},{londeg:03.0f}{lonmin:07.04f},{lonsgn},{hhmmssss},A",
      f"GPVTG,{cog:.1f},T,,,{sog:.1f},N,,",
      f"HCHDG,{heading_cmp:.1f},{abs(magdev):.1f},{magdevdir},{abs(magvar):.1f},{magvardir}",
      f"HCHDM,{heading_mag:.1f},M",
      f"HCHDT,{heading:.1f},T",
      f"VWVHW,{heading:.1f},T,{heading_mag:.1f},M,{stw:.1f},N,,",
      f"SDDBT,,,{depth:.1f},M,,",  # below transducer
      f"SDDBS,,,{depth:.1f},M,,",  # below surface
      # f"WIMWD,{wind_dir_true:.1f},T,,,{wind_speed_true:.1f},N,,",
      # f"WIMWV,{wind_angle_true:.1f},T,{wind_speed_true:.1f},N,A",
      f"WIMWV,{wind_angle_app:.1f},R,{wind_speed_app:.1f},N,A",
      f"HCROT,{rot:.1f},A",
      f"RIRSA,{self.rudder_angle:.1f},A,,",
      f"ERRPM,E,1,{noisy(self.engine_rpm):.1f},,A",
      f"CCVDR,{self.current_set:.1f},T,,,{self.current_drift:.1f},N",
      f"HCXDR,A,{self.heel_angle:.0f},D,ROLL"
    ]

    sentences = [
      f"${s}*{nmea_crc(s):02X}" for s in sentences if s[2:5] in NMEA_FILTER
    ]

    return "".join(f"{s}\n" for s in sentences)

  def autopilot(self, data):
    # receive waypoint to steer to
    nmea = decode_nmea(data)
    if not nmea:
      return
    brg, xte = nmea
    cts = brg  # course to steer
    brg_twd = to180(brg - self.wind_dir_true)  # BRG from TWD
    msg = ""
    if self.sailing:
      max_xte = 1
      big_xte = abs(xte) > max_xte
      if AUTO_PILOT == 2 and 15 < abs(brg_twd) < 170 and not big_xte:
        cts = self._polar.vmc_angle(self.wind_dir_true, self.wind_speed_true, brg)
        self.sign = 0
        msg = "OPTVMC"
      else:
        min_twa = self._polar.angle(self.wind_speed_true, True)
        max_twa = self._polar.angle(self.wind_speed_true, False)
        if abs(brg_twd) < min_twa:  # too high upwind
          if not self.sign or big_xte:
            self.sign = copysign(1, xte if big_xte else brg_twd)
          cts = to360(self.wind_dir_true + self.sign * min_twa)
          msg = "upwind layline"
        elif abs(brg_twd) > max_twa:  # too low downwind
          if not self.sign or big_xte:
            self.sign = copysign(1, -xte if big_xte else brg_twd)
          cts = to360(self.wind_dir_true + self.sign * max_twa)
          msg = "downwind layline"
        else:
          self.sign = 0
    crs = self.heading_true  # if s.sign or hdt_cog > 60 else s.cog
    cer = to180(cts - crs)  # course error
    if TIME_FACTOR > 2:
      self.rudder_angle = 0
      self.heading_true = to360(self.heading_true + cer)
    else:
      self.rudder_angle = round(copysign(min(10, 0.2 * abs(cer)), cer))
      print("CTS", cts, "XTE", xte, "CER", cer, "RUD", self.rudder_angle, msg)


class Polar:
  def __init__(self, data):
    self.data = data
    self.spline = None

  def has_angle(self, upwind):
    return ("beat_angle" if upwind else "run_angle") in self.data

  def angle(self, tws, upwind):
    angle = self.data["beat_angle" if upwind else "run_angle"]
    return numpy.interp(tws, self.data["TWS"], angle)

  def value(self, twa, tws):
    if not self.spline:
      val = "STW" if "STW" in self.data else "heel"
      self.spline = scipy.interpolate.RectBivariateSpline(
        self.data["TWA"], self.data["TWS"], self.data[val]
      )
    return max(0.0, float(self.spline(abs(twa), tws)))

  def vmc_angle(self, twd, tws, brg, s=1):
    brg_twd = to180(brg - twd)  # BRG from wind

    def vmc(twa):
      # negative sign for minimizer
      return -self.value(twa, tws) * cos(radians(s * twa - abs(brg_twd)))

    res = scipy.optimize.minimize_scalar(vmc, bounds=(0, 180))

    if res.success:
      return to360(twd + s * copysign(res.x, brg_twd))


class Server:
  def __init__(self, addr, port):
    # self.send = send
    # self.receive = receive
    if socket.has_dualstack_ipv6():
      self.server = socket.create_server(
        (addr, port), family=socket.AF_INET6, dualstack_ipv6=True
      )
    else:
      self.server = socket.create_server((addr, port))

    self.conns = []

  def serve(self, data_to_send, received):
    try:
      rx, tx, er = select.select([self.server], [], [self.server], 0)
      # print("server", rx, tx, er)
      for so in rx:
        conn, addr = so.accept()
        print("accepted", conn, file=sys.stderr)
        conn.setblocking(False)
        self.conns.append(conn)

      if not self.conns:
        return

      rx, tx, er = select.select(self.conns, self.conns, self.conns, 0)
      # print("connections", rx, tx, er)

      if tx:
        # data = self.send()
        print(data_to_send, file=sys.stderr)
        for co in tx:
          try:
            # print("TX", co)
            co.send(data_to_send.encode())
            # print(data, file=sys.stderr)
          except Exception as x:
            print(x, co, file=sys.stderr)
            self.conns.remove(co)

      for co in rx:
        try:
          # print("RX", co)
          data = co.recv(4096).decode()
          if data:
            print(data, file=sys.stderr)
            # self.receive(data)
            received(data)
        except Exception as x:
          print(x, co, file=sys.stderr)
          sleep(3)
          self.conns.remove(co)

      for co in er:
        print("ERROR", co, file=sys.stderr)
        sleep(3)
        self.conns.remove(co)

    except Exception as x:
      print(x, file=sys.stderr)
      sleep(3)


def to360(a):
  "limit a to [0,360)"
  return a % 360


def to180(a):
  "limit a to [-180,+180)"
  return to360(a + 180) - 180


def toCart(p):
  # to cartesian with phi going clock-wise from north
  return p[1] * sin(radians(p[0])), p[1] * cos(radians(p[0]))


def toPol(c):
  # to polar with phi going clock-wise from north
  return to360(90 - degrees(atan2(c[1], c[0]))), sqrt(c[0] ** 2 + c[1] ** 2)


def add_polar(a, b):
  "sum of polar vectors (phi,r)"
  a, b = toCart(a), toCart(b)
  s = a[0] + b[0], a[1] + b[1]
  return toPol(s)


def project(point, bearing, distance):
  "great circle projection: point -(bearing,distance)-> point"
  lat1, lon1 = (radians(a) for a in point)
  bearing = radians(bearing)
  distance *= 1852  # to meters
  distance /= 6371008.8  # angular distance

  lat2 = asin(sin(lat1) * cos(distance) + cos(lat1) * sin(distance) * cos(bearing))
  y = sin(bearing) * sin(distance) * cos(lat1)
  x = cos(distance) - sin(lat1) * sin(lat2)
  lon2 = lon1 + atan2(y, x)

  return [to180(degrees(a)) for a in (lat2, lon2)]


def nmea_crc(senctence):
  crc = 0
  for c in senctence:
    crc = crc ^ ord(c)
  return crc & 0xFF


def write(filename, data):
  with open(filename, "w") as f:
    # f.write(data)
    json.dump(data, f, indent=2)


def read(filename):
  with open(filename) as f:
    return json.load(f)


def noisy(value, absolute=0, relative=1):
  "add gaussian random to simulate measurement noise"
  return (
    gauss(
      value,
      NOISE_FACTOR * (absolute if absolute else abs(value * relative / 100)),
    )
    if NOISE_FACTOR
    else value
  )


def decode_nmea(data):
  for l in data.splitlines():
    try:
      # print(l)
      # $GPRMB,A,1.70,L,8,9,5509.36,N,01335.04,E,7.8,261.9,5.7,V,A*51
      # $GPAPB,A,A,1.70,L,N,V,,274.5,T,9,261.9,T,261.9,T*4F
      m = re.match(
        r"\$..(APB|RMB)" + ",([^,]*)" * 13 + r"(,([^,]*))?\*..",
        l,
      )
      if m:
        # print(m, m.groups())
        o = 0 if m.group(1) == "RMB" else 1
        brg = float(m.group(12))
        xte = float(m.group(3 + o)) * (-1 if m.group(4 + o) == "L" else 1)
        return brg, xte

    except Exception as x:
      print(x, file=sys.stderr)


def clamp(x, limits):
  "limit x to limits [a,b]"
  a, b = limits
  assert a < b, limits
  return max(a, min(b, x))


def deg(d, m=0, s=0):
  return d + m / 60 + s / 3600


def ais_encode(**kwargs):
  return pyais.encode_dict(kwargs, talker_id="AIVDM")


def ais_1(s):
  return ais_encode(
    msg_type=1,
    mmsi=s.mmsi,
    second=s.time.second if TIME_FACTOR == 1 else 60,
    lat=s.position[0],
    lon=s.position[1],
    course=s.course_over_ground,
    speed=s.speed_over_ground,
    status=1 if s.speed_over_ground < 0.2 else 8 if s.sailing else s.ais_navstat,
    heading=s.heading_true,
    turn=s.rate_of_turn * 60,
  )


def ais_5(s):
  return ais_encode(
    msg_type=5,
    mmsi=s.mmsi,
    imo=s.imo,
    callsign=s.callsign,
    shipname=s.name,
    ship_type=36 if s.sailing else 70,
    draught=5,
    to_bow=100,
    to_stern=100,
    to_port=20,
    to_starboard=20,
    destination=s.destination,
  )


def ais_19(s):
  return ais_encode(
    msg_type=19,
    mmsi=s.mmsi,
    second=s.time.second if TIME_FACTOR == 1 else 60,
    shipname=s.name,
    lat=s.position[0],
    lon=s.position[1],
    course=s.course_over_ground,
    speed=s.speed_over_ground,
    heading=s.heading_true,
    to_bow=6,
    to_stern=6,
    to_port=2,
    to_starboard=2,
  )


def ais_21(data):
  data["msg_type"] = 21
  return ais_encode(**data)
  return ais_encode(
    msg_type=21,
    mmsi=123456789,
    name="",
    lat=0,
    lon=0,
    off_position=False,
    virtual_aid=False,
  )


if __name__ == "__main__":
  main()
