from SFGE import *
from math import *

class RaycastStar(Component):

    def init(self):
        self.initial_distance = 1000
        self.max_distance = self.initial_distance

        self.nbRaycast = 100

        self.directions = [Vector2f()] * self.nbRaycast

        self.dir = Vector2f(1, 0)

        ratio = 360 / self.nbRaycast

        angle = 0;

        self.time = 0;

        for i in range(self.nbRaycast):
            self.directions[i] = RaycastStar.rotation(self, ratio)

    def update(self, dt):
        self.time += dt;

        mousePosition = input_manager.mouse.local_position(graphic_manager.get_window())
        position = Vector2f(mousePosition.x, mousePosition.y)

        for i in range(self.nbRaycast):
            physic_manager.raycast(self.directions[i], position, self.max_distance)

    def rotation(self, angle):
        angle = radians(angle)

        direction = self.dir;

        direction.x = (cos(angle) * direction.x) - (sin(angle) * direction.y)
        direction.y = (sin(angle) * direction.x) + (cos(angle) * direction.y)

        self.dir = Vector2f(direction.x, direction.y)

        return direction
