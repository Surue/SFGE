from SFGE import *
from math import *

class RaycastStar(Component):
    def init(self):
        self.max_distance = 1000

        self.nbRaycast = 20;

        self.directions = [Vector2f()] * self.nbRaycast

        ratio = 360 / self.nbRaycast

        angle = 0;

        for i in range(self.nbRaycast):
            self.directions[i] = RaycastStar.rotation(self, i * ratio)



    def update(self, dt):
        mousePosition = input_manager.mouse.local_position(graphic_manager.get_window())
        position = Vector2f(mousePosition.x, mousePosition.y)

        for i in range(self.nbRaycast):
            physic_manager.raycast(self.directions[i], position, self.max_distance)


    def rotation(self, angle):
        print(angle)
        angle = radians(angle)

        direction = Vector2f(1, 0);

        direction.x = (cos(angle) * direction.x) - (sin(angle) * direction.y)
        direction.y = (sin(angle) * direction.x) + (cos(angle) * direction.y)

        return direction
