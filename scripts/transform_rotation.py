from SFGE import *

class TransformRotation(Component):

    def init(self):
        pass
    def update(self, dt):
        if input_manager.keyboard.is_key_held(KeyboardManager.Key.Left):
            self.game_object.transform.set_euler_angle(self.game_object.transform.get_euler_angle() - 1)
        if input_manager.keyboard.is_key_held(KeyboardManager.Key.Right):
            self.game_object.transform.set_euler_angle(self.game_object.transform.get_euler_angle() + 1)