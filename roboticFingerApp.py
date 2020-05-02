#Owen Gibson, Brian Rupp, Ryan Miller
#Dr. Gregory Lee
#EECS 398/399
#3D Printed Biomimetic Robotic Finger

import time
import json

from kivy.app import App
from kivy.properties import NumericProperty, AliasProperty
from kivy.uix.label import Label
from kivy.clock import Clock
from paho.mqtt.client import Client
from finger_common import *

MQTT_CLIENT_ID = "finger_ui"

class RoboticFingerApp(App):

    _phalanx = NumericProperty()
    _metacarpal = NumericProperty()
    _updatingUI = False
    _motorID = 0

    def _get_phalanx(self):
        return self._phalanx

    def _set_phalanx(self, value):
        self._phalanx = value

    def _get_metacarpal(self):
        return self._metacarpal

    def _set_metacarpal(self, value):
        self._metacarpal = value

    phalanx = AliasProperty(_get_phalanx, _set_phalanx, bind=['_phalanx'])
    metacarpal = AliasProperty(_get_metacarpal, _set_metacarpal, bind=['_metacarpal'])

    def on_start(self):
        self._publish_clock = None
        self.mqtt = Client(client_id=MQTT_CLIENT_ID)
        self.mqtt.on_connect = self.on_connect
        self.mqtt.connect(MQTT_BROKER_HOST, port=MQTT_BROKER_PORT,
                          keepalive=MQTT_BROKER_KEEP_ALIVE_SECS)
        self.mqtt.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        print ("Connection Worked")
        self.mqtt.publish(client_state_topic(MQTT_CLIENT_ID), "1",
                          qos=2, retain=True)

    def _update_goal_position(self):
        self._updatingUI = True
        msg = {}
        if self._motorID == 1:
            msg = {'goalPosition': self._metacarpal,
                   'motorID': self._motorID,
                   'client': MQTT_CLIENT_ID}
        elif self._motorID == 2:
            msg = {'goalPosition': self._phalanx,
                   'motorID': self._motorID,
                   'client': MQTT_CLIENT_ID}
        self.mqtt.publish(TOPIC_SET_FINGER_CONFIG, json.dumps(msg), qos=1)
        self._publish_clock = None
        self._updatingUI = False
        self._motorID = 0

    def on_metacarpal(self, instance, value):
        if self._updatingUI:
            return
        self._motorID = 1
        if self._publish_clock is None:
            self._publish_clock = Clock.schedule_once(
                lambda dt: self._update_goal_position(), 0.01)

    def on_phalanx(self, instance, value):
        if self._updatingUI:
            return
        self._motorID = 2
        if self._publish_clock is None:
            self._publish_clock = Clock.schedule_once(
                lambda dt: self._update_goal_position(), 0.01)

    # def printMetacarpalSliderValue(self, instance, value):
    #     print("Metacarpal Value: {:.0f}".format(value))
    #
    # def printPhalanxSliderValue(self, instance, value):
    #     print("Phalanx Value: {:.0f}".format(value))

    def build(self):
        #titleLabel = Label(text="Biomimetic 3D Printed Robotic Finger", pos=(0,0))
        return


if __name__ == '__main__':
    RoboticFingerApp().run()
