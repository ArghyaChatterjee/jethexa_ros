# This file is one part of JetHexa
# button_helper.py realizes button scanning and state transformation 

import time
import logging
from enum import Enum


class ButtonState(Enum):
    NORMAL = 1
    PRESSED = 2  # press button
    LONG_PRESSED = 3  # long press button


class ButtonHelper:
    def __init__(self,
                 read_button,
                 button_name="",
                 combo_threshold: float = 0.25,  # Combo maximum interval time
                 long_press_threshold: float = 1.5,  # long press time
                 pressed_value: int = 0,
                 pressed_cb=None,
                 released_cb=None,
                 clicked_cb=None,
                 combo_cb=None,
                 long_pressed_cb=None,
                 logger: logging.Logger = None):
        self.read_button = read_button
        self.button_name = button_name
        self.button_io_state = pressed_value
        self.combo_threshold = combo_threshold
        self.long_press_threshold = long_press_threshold
        self.pressed_value = pressed_value
        self.pressed_cb = pressed_cb if pressed_cb is not None else self.__default_pressed_cb
        self.released_cb = released_cb if released_cb is not None else self.__default_released_cb
        self.clicked_cb = clicked_cb if clicked_cb is not None else self.__default_clicked_cb
        self.combo_cb = combo_cb if combo_cb is not None else self.__default_combo_cb
        self.long_pressed_cb = long_pressed_cb if long_pressed_cb is not None else self.__default_long_pressed_cb
        self.logger = logger

        self.button_click_count = 0
        self.button_press_timestamp = 0
        self.button_state = ButtonState.NORMAL

    def update_button(self):
        button_io_state = self.read_button()
        if button_io_state != self.button_io_state:  # Anti-shake. Only when the IO statu read twice in a row is the same, it will be recognized
            self.button_io_state = button_io_state
            return

        if self.button_state == ButtonState.NORMAL:
            if self.button_io_state == self.pressed_value:  # when IO is 0, press the button
                self.button_state = ButtonState.PRESSED
                t_now = time.time()
                if (t_now - self.button_press_timestamp) < self.combo_threshold:
                    self.button_click_count += 1
                else:
                    self.button_click_count = 1  # Restart the count when the time since the last press exceeds the combo threshold
                self.button_press_timestamp = t_now  # take down the button pressed time
                if callable(self.pressed_cb):
                    self.pressed_cb()
            else:
                if self.button_click_count != 0:  # The button is released and there is a click count, that is, a click or combo occurred before, and the combo ends here
                    if (time.time() - self.button_press_timestamp) > self.combo_threshold:
                        if callable(self.combo_cb):
                            self.combo_cb(self.button_click_count)
                        self.button_click_count = 0
                        self.button_press_timestamp = 0

        elif self.button_state == ButtonState.PRESSED:
            if self.button_io_state != self.pressed_value:  # the button is released
                self.button_state = ButtonState.NORMAL
                if callable(self.released_cb):
                    self.released_cb(ButtonState.PRESSED)
                if callable(self.clicked_cb):
                    self.clicked_cb(self.button_click_count)
            else:
                if (time.time() - self.button_press_timestamp) > self.long_press_threshold:
                    self.button_state = ButtonState.LONG_PRESSED
                    if callable(self.long_pressed_cb):
                        self.long_pressed_cb()

        elif self.button_state == ButtonState.LONG_PRESSED:
            if self.button_io_state != self.pressed_value:
                self.button_state = ButtonState.NORMAL
                if callable(self.released_cb):
                    self.released_cb(ButtonState.LONG_PRESSED)
                self.button_press_timestamp = 0

        else:
            pass

    def __default_pressed_cb(self):
        if self.logger is not None:
            self.logger.debug("NORMAL -> PRESSED")

    def __default_long_pressed_cb(self):
        if self.logger is not None:
            self.logger.debug("PRESSED -> LONG_PRESSED")

    def __default_released_cb(self, from_):
        if self.logger is not None:
            if from_ == ButtonState.PRESSED:
                self.logger.debug("PRESSED -> NORMAL")
            elif from_ == ButtonState.LONG_PRESSED:
                self.logger.debug("LONG_PRESSED -> NORMAL")

    def __default_clicked_cb(self, count):
        if self.logger is not None:
            self.logger.debug("CLICK %d" % count)

    def __default_combo_cb(self, count):
        if self.logger is not None:
            self.logger.debug("COMBO x %d" % count)
