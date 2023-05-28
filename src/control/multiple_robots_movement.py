from py_publisher_spiral import TurtleBot
from multiprocessing import Process
import rclpy
from rclpy.executors import Executor
from rclpy.node import Node
import subprocess


class Control:
    def __init__(self, robot_names: list):
        self.robot_names = robot_names
        self.robots = []
        self.setup()

    def setup(self):
        for name in self.robot_names:
            self.robots.append(TurtleBot(name))


    def action(self, actions: list):
        assert len(self.robots) == len(actions)
        proc = []
        for i in range(len(self.robots)):
            cmd = 'python3 src/control/py_publisher_spiral.py ' + str(self.robot_names[i]) + ' ' + str(actions[i])
            s = subprocess.call(cmd, shell=True)
        # for i in range(len(self.robots)):
        #     print(self.robots[i].publisher)
        #     p = Process(target=self.robots[i].move, args=(actions[i],))
        #     p.start()
        #     proc.append(p)
        # for p in proc:
        #     p.join()

        
if __name__ == '__main__':
    rclpy.init(args=None)
    names = ['turtlebot_0', 'turtlebot_1']
    c = Control(names)

    c.action([1, 1])
    rclpy.shutdown()