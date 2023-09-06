"""
Environment for rrt_2D
@author: huiming zhou
"""


class Env:
    def __init__(self):
        # self.x_range = (0, 50)
        # self.y_range = (0, 30)
        self.x_range = (-1000, 3500)
        self.y_range = (-1000, 0)
        self.obs_boundary = self.obs_boundary()
        self.obs_circle = self.obs_circle()
        self.obs_rectangle = self.obs_rectangle()

    @staticmethod
    def obs_boundary():
        obs_boundary = [
        ]
        return obs_boundary

    @staticmethod
    def obs_rectangle():
        obs_rectangle = [
        ]
        return obs_rectangle

    @staticmethod
    def obs_circle():
        obs_cir = [
            [0, 0, 500]
        ]

        return obs_cir
