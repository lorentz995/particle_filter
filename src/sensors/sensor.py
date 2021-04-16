from abc import ABCMeta, abstractmethod


class Sensor(metaclass=ABCMeta):

	@abstractmethod
	def __init__(self):
		pass

	@abstractmethod
	def change_settings(self):
		pass

	@abstractmethod
	def publish_observation(self, data):
		pass
