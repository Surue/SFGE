from SFGE import *
import random


class BoxRandomMovement(Component):
    def init(self):
        d = p2Vec2(random.uniform(0, 0.5) - 0.25, random.uniform(0, 0.5) - 0.25)
        self.body = self.game_object.get_component(Component.Body)
        self.body.velocity = d

    def update(self, dt):
        d = self.body.velocity
        position = self.game_object.transform.get_position()
        if position.x < 0 or position.x > 1280:
            d.x = d.x * -1
            self.body.velocity = d
        if position.y < 0 or position.y > 720:
            d.y = d.y * -1
            self.body.velocity = d

    def on_collision_enter(self, collider):
        pass

    def on_collision_exit(self, collider):
        pass

    def on_trigger_enter(self, collider):
        pass

    def on_trigger_exit(self, collider):
        pass