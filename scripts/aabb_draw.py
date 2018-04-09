from SFGE import *

class ColliderDraw(Component):
    def init(self):
        colliders = self.game_object.get_components(Component.Collider)
        for c in colliders:
            c.debug_draw(engine)
    
    def update(self, dt):
        pass