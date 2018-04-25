from SFGE import *

class TransformRotation(Component):

    def init(self):
        pass
    def update(self, dt):
        if input_manager.keyboard.is_key_held(KeyboardManager.Key.Q):
            self.game_object.transform.set_euler_angle(self.game_object.transform.get_euler_angle() - 1)
        if input_manager.keyboard.is_key_held(KeyboardManager.Key.E):
            self.game_object.transform.set_euler_angle(self.game_object.transform.get_euler_angle() + 1)

    def on_collision_enter(self, collider):
        pass

    def on_collision_exit(self, collider):
        pass

    def on_trigger_enter(self, collider):
        pass

    def on_trigger_exit(self, collider):
        pass

