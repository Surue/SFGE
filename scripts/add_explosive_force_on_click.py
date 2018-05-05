from SFGE import *

class AddExplosiveForceOnClick(Component):

    def init(self):
        self.body = self.game_object.get_component(Component.Body)
    def update(self, dt):
        if(input_manager.mouse.is_button_down(MouseManager.Button.Left)):
            mousePosition = input_manager.mouse.local_position(graphic_manager.get_window())
            position = self.game_object.transform.get_position()

            if (mousePosition.x < position.x + 75 and mousePosition.x > position.x - 75):
                if (mousePosition.y < position.y + 75 and mousePosition.y > position.y - 75):
                    self.body.add_explosive_force(100, 400, position)


    def on_collision_enter(self, collider):
        pass

    def on_collision_exit(self, collider):
        pass

    def on_trigger_enter(self, collider):
        pass

    def on_trigger_exit(self, collider):
        pass