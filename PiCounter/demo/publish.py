import json
import paho.mqtt.client as mqtt
import datetime

# This is the Publisher
now = datetime.datetime.now()

client = mqtt.Client()
def on_connect(client, userdata, flags, rc):
    client.subscribe("topic2")


def on_message(client, userdata, msg):
    print("Message received-> " + msg.topic + " " + str(msg.payload))

client.on_connect = on_connect
client.on_message = on_message

client.connect("18.237.68.217")
##client.loop_forever()
##client.publish("topic1", json.dumps(m1.__dict__));
##client.disconnect();

def format(string):
    if string[0] == "0":
      return string[1:]
    return string

class Message:
  def __init__(self, device, deviceid, longitude, latitude, location, datetime, time, day, month,
               year, weekday, enter, exit):
    self.device = device
    self.deviceid = deviceid
    self.longitude = longitude
    self.latitude = latitude
    self.location = location
    self.datetime = datetime
    self.time = time
    self.day = day
    self.month = month
    self.year = year
    self.weekday = weekday
    self.enter = enter
    self.exit = exit

  def dictStr(self):
    d = {}
    d["device"] = self.device
    d["deviceid"] = self.deviceid
    d["longitude"] = self.longitude
    d["latitude"] = self.latitude
    d["location"] = self.location
    d["datetime"] = self.datetime
    d["time"] = self.time
    d["day"] = self.day
    d["month"] = self.month
    d["year"] = self.year
    d["weekday"] = self.weekday
    d["enter"] = self.enter
    d["exit"] = self.exit
    return json.dumps(d)

datetime = now.strftime("%Y/%m/%d %H")
time = format(now.strftime("%H"))
day = format(now.strftime("%d"))
month = format(now.strftime("%m"))
year = now.strftime("%Y")
weekday = now.strftime("%w")

m1 = Message("rpi4", 16, 455, 566, "School, Gym", datetime, time, day, month, year, weekday, 9, 0)
# m1 = Message("rpi4", 16, 400, 400, "entrance", "2021/05/01 7", 7, 1, 5, 2021, 4, 11, 15)

##m = json.dumps(m1.__dict__)

##client.publish("topic1", json.dumps(m1.__dict__));
client.publish("topic1", m1.dictStr());
##client.subscribe("topic2");
##client.disconnect();

##t = json.loads(m)
