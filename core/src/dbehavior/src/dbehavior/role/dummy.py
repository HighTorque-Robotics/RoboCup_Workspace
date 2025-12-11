from dbehavior.btree.branch import Parallel, Sequence
from dbehavior.btree.leaf import Wait
from dbehavior.btree.decorator import RepeatSeconds
from dbehavior.core import Role
from dbehavior.skill import (KeyboardController, JoystickController,
                             GetImageCapture, GetImageRotate,
                             FindBall, ScanField)


class Dummy(Role):
    """
    Dummy role, controlled by external signal source.
    """


class Doll(Dummy):
    """
    Doll role controlled by joystick.
    """

    def __init__(self, bb):
        super(Doll, self).__init__(bb)
        self.root = JoystickController(bb)
        self.add_child(self.root)


class SmartDoll(Dummy):
    """
    Doll role controlled by joystick, whose head is finding ball.
    """

    def __init__(self, bb):
        super(SmartDoll, self).__init__(bb)
        self.root = Parallel(
            FindBall(bb),
            JoystickController(bb, head_ctl=False),
        )
        self.add_child(self.root)


class DemoMain(Dummy):
    """
    DemoMain role, controlled by keyboard.
    """

    def __init__(self, bb):
        super(DemoMain, self).__init__(bb)
        self.root = KeyboardController(bb)
        self.add_child(self.root)


class GetImage(Dummy):
    """
    GetImage role to get image by rotating head.
    """

    def __init__(self, bb):
        super(GetImage, self).__init__(bb)
        self.root = Sequence(
            # Wait(5),
            Sequence(
                RepeatSeconds(GetImageRotate(bb), 0.5),
                RepeatSeconds(GetImageCapture(bb), 0.5)
            )
            # GetImageRotate(bb),
            # GetImageCapture(bb)
        )
        self.add_child(self.root)


class Foo(Dummy):
    """
    Foo role for testing.
    """

    def __init__(self, bb, root):
        super(Foo, self).__init__(bb)
        self.root = root
        self.add_child(self.root)
