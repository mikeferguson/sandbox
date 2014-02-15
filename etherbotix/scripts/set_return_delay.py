#!/usr/bin/env python

import sys
from etherbotix import *

e = EtherBotiX()

id = int(sys.argv[1])
val = int(sys.argv[2])

e.send(0x83, [255, 255, id, 4, 3, 5, val, 255-((id+4+3+5+val)%256) ])

