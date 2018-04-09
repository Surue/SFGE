from SFGE import *

class AabbDraw(Component):
    def init(self):
        colliders = self.game_object.get_components(Component.Collider)
        for c in colliders:
            c.debug_draw_aabb(engine)
    
    def update(self, dt):
        pass