import utils
from TouchSensorWireless import MultiProtocolReceiver
from pianogenie import startController
utils.programSensor(1)
utils.programSensor(2)
myReceiver = MultiProtocolReceiver()
myReceiver.runCustomMethod(startController)