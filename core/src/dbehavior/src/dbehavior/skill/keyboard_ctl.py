from __future__ import print_function

from dbehavior.core import Skill
from dbehavior.btree.core import Status
from dbehavior.util import ActionGenerator, VecPos

import termios
import tty
import sys
import select

class KeyboardBindings(object):
    """
    Bindings for ``KeyboardController``.
    """

    def __init__(self, bb):
        self.bb = bb
        self.action = ActionGenerator()
        self.move = {
            'w': (3, 0, 0),
            'a': (0, 1, 0),
            's': (-1, 0, 0),
            'd': (0, -1, 0),
            'x': (0, 0, 1),
            'z': (2, 0, 1),
            'c': (2, 0, -1),
        }
        self.speed = {
            ',': (1.1, 1),
            '.': (.9, 1),
            '<': (1, 1.1),
            '>': (1, .9),
        }

        self.kick = {
            'q': self.action.kick_ball(self.bb.vision_info.robot_pos),
            # 'e': self.action.kick_right(),
            # 'n': self.action.side_kick_left(),
            # 'm': self.action.side_kick_right(),
            # 'v': self.action.fast_kick_left(),
            # 'b': self.action.fast_kick_right()
        }

        self.goalie = {
            'i': self.action.goalie_left(),
            # 'o': self.action.goalie_mid(),
            'p': self.action.goalie_right(),
        }

        self.standup = {
            'f': 'stand_up_back',
            'g': 'stand_up_front'
        }

        self.head = {
            'h': (0, 5),
            'l': (0, -5),
            'j': (5, 0),
            'k': (-5, 0),
        }

        self.help = 'u'

        self.stand = '1'
        self.step = '2'
        self.exit = ('\03', '\04', '\1a')


class KeyboardController(Skill):
    """
    Action controlled by keyboard.
    """
    def __init__(self, bb):
        super(KeyboardController, self).__init__(bb)

        self.usage = \
            """
Reading from the keyboard and Publishing to Motion!
---------------------------
Moving around:
        w
   a    s    d

x : turn around
z : turn left; c : turn right
1 : stand
2 : step

q : left kick;      e : right kick
n : left side kick; m : right side kick
v : left fast kick; b : right fast kick

i : goalie left; o : goalie mid; p : goalie right
f : set up from back down
g : set up from front down

h/j/k/l: head turn leftward/downward/upward/rightward

r/anything else : crouch

,/. : increase/decrease only linear speed by 10%
</> : increase/decrease only angular speed by 10%

u : print usage

CTRL-C to quit
            """
        print(self.usage)
        # key bindings
        self.bindings = KeyboardBindings(bb)
        # initial params
        self.speed = 1.5
        self.turn = 5
        self.pitch = 0
        self.yaw = 0

    @staticmethod
    def get_key():
        """
        Get pressed key in keyboard.
        :return: pressed key
        :rtype: str
        """
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def execute(self):
        key = self.get_key()
        # print(key)

        # initialize head
        self.look_at(self.pitch, self.yaw)

        if key in self.bindings.move.keys():
            x = self.bindings.move[key][0] * self.speed
            y = self.bindings.move[key][1] * self.speed
            th = self.bindings.move[key][2] * self.turn
            if abs(x) > 1 or abs(y) > 1 or abs(th) > 1:
                dis = VecPos(x, y, th)
                self.goto_field(dis)
                print('walk ({}, {}, {})'.format(x,y,th))
        elif key in self.bindings.kick.keys():
            self.do(self.bindings.kick[key])
        elif key in self.bindings.speed.keys():
            self.speed = self.speed * self.bindings.speed[key][0]
            self.turn = self.turn * self.bindings.speed[key][1]
            print("currently:\tspeed {}\tturn {} ".format(self.speed, self.turn))
        elif key == self.bindings.help:
            print(self.usage)
        elif key in self.bindings.goalie.keys():
            self.do(self.bindings.goalie[key])
        elif key in self.bindings.standup:
            pass
        elif key in self.bindings.head:
            self.pitch += self.bindings.head[key][0]
            self.yaw += self.bindings.head[key][1]

            # limit value
            self.pitch = min(self.bb.param.max_pitch, self.pitch)
            self.pitch = max(self.bb.param.min_pitch, self.pitch)
            self.yaw = min(self.bb.param.max_yaw, self.yaw)
            self.yaw = max(-self.bb.param.max_yaw, self.yaw)

            self.look_at(self.pitch, self.yaw, 10, 10)
            print('look at ({},{})'.format(self.pitch, self.yaw))
        elif key == self.bindings.stand:
            self.crouch()
        elif key == self.bindings.step:
            self.step()
        elif key in self.bindings.exit:
            sys.exit(0)
        else:
            self.crouch()

        return Status.RUNNING
