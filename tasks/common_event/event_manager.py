# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  
"""
event manager
"""


class SimpleEvent:
    def __init__(self, func, params=None):
        self.func = func
        self.params = params or {}

    def trigger(self, env):
        return self.func(env, **self.params)

class SimpleEventManager:
    def __init__(self):
        self._events = {}

    def register(self, name, event):
        self._events[name] = event

    def trigger(self, name, env):
        event = self._events.get(name)
        if event:
            return event.trigger(env)
        else:
            print(f"Event {name} not registered")