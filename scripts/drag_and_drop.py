from SFGE import *

class DragAndDrop(Component):

    def init(self):
        pass

    def update(self, dt):
        self.positionMouse = input_manager.mouse.local_position(self)
        print("[ " + self.positionMouse.x + ", " + self.positionMouse.y + "]")
        print("Salut")
        pass