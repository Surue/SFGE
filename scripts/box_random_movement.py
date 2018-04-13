from SFGE import *
import random


class BoxRandomMovement(Component):
    def init(self):
        d = p2Vec2(random.uniform(0, 0.5) - 0.25, random.uniform(0, 0.5) - 0.25)
        self.body = self.game_object.get_component(Component.Body)
        self.body.velocity = d;
    def update(self, dt):
        if (self.game_object.transform.get_position().x < 0):
            d = p2Vec2(self.body.velocity.x * -1, self.body.velocity.y)
            self.body.velocity = d;
        if (self.game_object.transform.get_position().x > 1280):
            d = p2Vec2(self.body.velocity.x * -1, self.body.velocity.y)
            self.body.velocity = d;
        if (self.game_object.transform.get_position().y < 0):
            d = p2Vec2(self.body.velocity.x, self.body.velocity.y * -1)
            self.body.velocity = d;
        if (self.game_object.transform.get_position().y > 720):
            d = p2Vec2(self.body.velocity.x, self.body.velocity.y * -1)
            self.body.velocity = d;
    def on_collision_enter(self, collider):
        pass
    def on_collision_exit(self, collider):
        pass
    def on_trigger_enter(self, collider):
        pass
    def on_trigger_exit(self, collider):
        pass