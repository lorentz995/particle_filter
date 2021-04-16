from abc import ABCMeta, abstractmethod


class ActionManager(metaclass=ABCMeta):

	@abstractmethod
	def __init__(self):
		pass

	@abstractmethod
	def change_settings(self):
		pass

	@abstractmethod
	def get_actions(self, data):
		pass
