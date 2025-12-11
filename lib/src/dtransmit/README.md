Why this package?
===

To transimit ros messages and other messages over udp, 

because: 

    1. ROS doesn't support sending messages over different masters
    2. We need to communicate between different robots, and we need to simulating & monitoring.

Why udp?
===

Because TCP protocol is not allowed to used according to RoboCup humanoid rules. So ZMQ is not 

feasible is this scenario.


So we use Boost::asio.


Main usage
===

1. Sending localization ros msg and other useful info back to monitor, a Qt gui.
2. Sharing localization ros msg between robots.
3. Listening to GameController.


How?
===

Manually serialize and deserialize ros msg, and sendRos by udp socket.
