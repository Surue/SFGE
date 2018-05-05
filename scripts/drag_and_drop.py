from SFGE import *

class DragAndDrop(Component):

    def init(self):
        self.body = self.game_object.get_component(Component.Body)
        self.original_gravity_scale = self.body.gravity_scale

        self.is_drag = bool(0)
    def update(self, dt):
        if(self.is_drag):
            mousePosition = input_manager.mouse.local_position(graphic_manager.get_window())
            self.game_object.transform.set_position(Vector2f(mousePosition.x, mousePosition.y))
            self.body.set_position(Vector2f(mousePosition.x, mousePosition.y))
            self.body.gravity_scale = 0

        if(input_manager.mouse.is_button_down(MouseManager.Button.Left)):
            mousePosition = input_manager.mouse.local_position(graphic_manager.get_window())
            position = self.game_object.transform.get_position()

            if (mousePosition.x < position.x + 75 and mousePosition.x > position.x - 75):
                if (mousePosition.y < position.y + 75 and mousePosition.y > position.y - 75):
                    self.is_drag = bool(1)

        if(input_manager.mouse.is_button_up(MouseManager.Button.Left)):
            self.is_drag = bool(0)
            self.body.gravity_scale = self.original_gravity_scale

    def on_collision_enter(self, collider):
        pass

    def on_collision_exit(self, collider):
        pass

    def on_trigger_enter(self, collider):
        pass

    def on_trigger_exit(self, collider):
        pass