import json
import paho.mqtt.client as mqtt

# This is the Publisher

client = mqtt.Client()
def on_connect(client, userdata, flags, rc):
    client.subscribe("topic2")


def on_message(client, userdata, msg):
    print("Message received-> " + msg.topic + " " + str(msg.payload))

client.on_connect = on_connect
client.on_message = on_message

client.connect("52.43.181.166")
##client.loop_forever()
##client.publish("topic1", json.dumps(m1.__dict__));
##client.disconnect();

class Message:
  def __init__(self, device, deviceid, longitude, latitude, location, time, enter, exit, peopleinbuilding):
    self.device = device
    self.deviceid = deviceid
    self.longitude = longitude
    self.latitude = latitude
    self.location = location
    self.time = time
    self.enter = enter
    self.exit = exit
    self.peopleinbuilding = peopleinbuilding

  def dictStr(self):
    d = {}
    d["device"] = self.device
    d["deviceid"] = self.deviceid
    d["longitude"] = self.longitude
    d["latitude"] = self.latitude
    d["location"] = self.location
    d["time"] = self.time
    d["enter"] = self.enter
    d["exit"] = self.exit
    d["peopleinbuilding"] = self.peopleinbuilding
    return json.dumps(d)

m1 = Message("rpi4", 36, 455, 566, "School, Gym", "12:23", 23, 32, 24)

##m = json.dumps(m1.__dict__)

##client.publish("topic1", json.dumps(m1.__dict__));
client.publish("topic1", m1.dictStr());
##client.subscribe("topic2");
##client.disconnect();

##t = json.loads(m)
