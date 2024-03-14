#!/usr/bin/env python3
import toplevel


if __name__ == '__main__':
    toplevel = toplevel.TopLevelLoop(goal = (5.0, 1.0, ), debug = 1)
    toplevel.run()
