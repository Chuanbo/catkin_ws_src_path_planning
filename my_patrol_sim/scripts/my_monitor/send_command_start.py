#!/usr/bin/env python
# -*- coding: utf-8 -*-

import lcm
from exlcm import command_t

lc = lcm.LCM()

msg = command_t()

msg.is_start = True
msg.is_stop = False

lc.publish("ROBOTCOMMAND", msg.encode())
