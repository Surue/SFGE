from SFGE import *
import random


class BoxRandomMovement(Component):
    def init(self):
        self.body = self.game_object.get_component(Component.Body)
        self.originalVelocity = p2Vec2(random.uniform(0, 0.5) - 0.25, random.uniform(0, 0.5) - 0.25)
        self.body.velocity = self.originalVelocity

    def update(self, dt):
        position = self.game_object.transform.get_position()
        if position.x < 0 or position.x > 1280:
            self.originalVelocity.x = self.originalVelocity.x * -1
            self.body.velocity = self.originalVelocity
        if position.y < 0 or position.y > 720:
            self.originalVelocity.y = self.originalVelocity.y * -1
            self.body.velocity = self.originalVelocity

    def on_collision_enter(self, collider):
        pass

    def on_collision_exit(self, collider):
        pass

    def on_trigger_enter(self, collider):
        pass

    def on_trigger_exit(self, collider):
        pass