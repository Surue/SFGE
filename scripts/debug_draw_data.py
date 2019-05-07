from SFGE import *

class DebugDrawData(Component):

    def init(self):
        engine.set_debug_draw_data()

    def update(self, dt):
        if input_manager.keyboard.is_key_down(KeyboardManager.Key.Num1):
            engine.set_debug_draw_data_flag_switch(0x0001)
        if input_manager.keyboard.is_key_down(KeyboardManager.Key.Num2):
            engine.set_debug_draw_data_flag_switch(0x0002)
        if input_manager.keyboard.is_key_down(KeyboardManager.Key.Num3):
            engine.set_debug_draw_data_flag_switch(0x0004)
        if input_manager.keyboard.is_key_down(KeyboardManager.Key.Num4):
            engine.set_debug_draw_data_flag_switch(0x0008)
        if input_manager.keyboard.is_key_down(KeyboardManager.Key.Num5):
            engine.set_debug_draw_data_flag_switch(0x0010)
        if input_manager.keyboard.is_key_down(KeyboardManager.Key.Num6):
            engine.set_debug_draw_data_flag_switch(0x0020)
        if input_manager.keyboard.is_key_down(KeyboardManager.Key.Num7):
            engine.set_debug_draw_data_flag_switch(0x0040)