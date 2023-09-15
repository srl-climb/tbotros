from __future__ import annotations

import tkinter as tk
from typing import Callable
from geometry_msgs.msg import Pose
from tbotlib import TransformMatrix
from .tkinter_tooltip import TkToolTip

class TkTimer():
    # A class representing a Tkinter-based timer

    def __init__(self, parent: tk.Toplevel = None, callback: Callable = None, timeout_ms: int = 500) -> None:
        self._parent = parent
        self._callback = callback
        self._timeout_ms = timeout_ms

    def run(self):
        self._callback()
        self._parent.after(self._timeout_ms, self.run)
        # Run the timer's callback function and schedule the next run after a specified timeout


class TkEntry(tk.Entry):

    def __init__(self, master: tk.Toplevel = None, **args):

        super().__init__(master, **args)

    def get_data(self):

        return self.get()


class TkUIntEntry(TkEntry):

    def __init__(self, **args):

        super().__init__(validate='focusout', validatecommand=self.validate_command, invalidcommand=self.invalid_command, **args)

        self.configure(justify='center')
        self.insert(0, str(0))

    def validate_command(self):
        
        value = self.get()
        try:
            if int(value) < 0:
                return False
            return True
        except ValueError:
            return False
    
    def invalid_command(self):

        self.delete(0, tk.END)
        self.insert(0, str(0))

    def get_data(self) -> int:

        return int(self.get())


class TkFloatEntry(TkEntry):

    def __init__(self, max_val: float = float('inf'), min_val: float = -float('inf'), default_val: float = 0, **args):

        assert min_val <= default_val <= max_val, "Maximum, minimum or default value mismatch"
        
        super().__init__(validate='focusout', validatecommand=self.validate_command, invalidcommand=self.invalid_command, **args)

        self.max_val = max_val
        self.min_val = min_val
        self.default_val = default_val
        self.configure(justify='center')
        self.insert(0, str(self.default_val))

    def validate_command(self):
        
        value = self.get()
        if self.is_float(value) and self.min_val <= float(value) <= self.max_val:
            return True
        else:
            return False
    
    def invalid_command(self):
        
        value = self.get()
        if self.is_float(value):
            value = max(min(self.max_val, float(value)), self.min_val)
        else:
            value = self.default_val
        self.delete(0, tk.END)
        self.insert(0, str(value))

    def get_data(self) -> float:

        return float(self.get())
    
    def is_float(self, value: str):
        
        try:
            float(value)
            return True
        except ValueError:
            return False

class TkLabel(tk.Label):

    def __init__(self, **kwargs):

        super().__init__(**kwargs)

    def update_data(self, value):
        
        self.config(text = value)

    
class TkStringLabel(TkLabel):

    def __init__(self, enable_tooltip: bool = False, **kwargs):

        super().__init__(relief="sunken", **kwargs)

        self.config(width=15, height=1)

        self._enable_tooltip = enable_tooltip

        if self._enable_tooltip:
            self._tooltip = TkToolTip(self, '')
        
    def update_data(self, value):
        
        self.config(text = str(value))

        if self._enable_tooltip:
            self._tooltip.text = str(value)


class TkFloatLabel(TkLabel):

    def __init__(self, digits: int = 3, **kwargs):

        super().__init__(relief="sunken", **kwargs)

        self._digits = digits
        self.config(width=15, height=1)

    def update_data(self, value: float):
        
        self.config(text = str(round(value, self._digits)))


class TkArrayLabel(TkLabel):

    def __init__(self, digits: int = 3, length: int = 3, **kwargs):

        super().__init__(relief="sunken", **kwargs)

        self._digits = digits
        self._length = length
        self.config(width=20, height=1)

        self.update_data([0] * self._length)

    def update_data(self, value: list):
        
        text = ''
        for i in range(self._length):
            text = text + str(round(value[i], self._digits)) + '; '
        text = text.rstrip('; ')

        self.config(text = text)

class TkErrorCodeLabel(TkLabel):

    def __init__(self, **kwargs):

        super().__init__(relief="sunken", **kwargs)

        self.config(width=15, height=1)

        self.true_color = "#C1FFC1"
        self.false_color = "#FFC0CB"

    def update_data(self, value: int):
        
        self.config(text = str(value))

        if value > 0:
            self.config(bg=self.false_color)
        else:
            self.config(bg=self.true_color)


class TkBoolLabel(TkLabel):

    def __init__(self, invert_colors: bool = False, **kwargs):

        super().__init__(relief="sunken", **kwargs)

        self.config(width=15, height=1)

        if invert_colors:
            self.true_color = "#FFC0CB"
            self.false_color = "#C1FFC1"
        else:
            self.true_color = "#C1FFC1"
            self.false_color = "#FFC0CB"
        self.default_color = "gray"

        self.update_data("NONE")

    def update_data(self, value: bool):
        
        if value == 1:
            self.config(bg=self.true_color)
            self.config(text = 'TRUE')
        elif value == 0:
            self.config(bg=self.false_color)
            self.config(text = 'FALSE')
        else:
            self.config(bg=self.default_color)
            self.config(text = 'NONE')


class TkActionStatusLabel(TkLabel):
    
    def __init__(self, **kwargs):

        super().__init__(relief="sunken", **kwargs)

        self.config(width=15, height=1)

        self.update_data(0)

    def update_data(self, value: int):
        
        if value == 6:
            self.config(text = 'ABORTED', bg = "#FFC0CB")
        elif value == 5:
            self.config(text = 'CANCELED', bg = "#FFC0CB")
        elif value == 4:
            self.config(text = 'SUCCEEDED', bg = "#C1FFC1")
        elif value == 3:
            self.config(text = 'CANCELING', bg = "#FFECC1")
        elif value == 2:
            self.config(text = 'EXECUTING', bg = "#FFECC1")
        elif value == 1:
            self.config(text = 'ACCEPTED', bg = "#FFECC1")
        else:
            self.config(text = 'UNKNOWN', bg = "gray")


class TkLabelFrame(tk.LabelFrame):

    def __init__(self, **kwargs):

        super().__init__(**kwargs)

    def update_data(self):

        pass


class TkPoseLabelFrame(TkLabelFrame):

    def __init__(self, text = 'Pose', **args):

        super().__init__(text = text, **args)

        label = TkLabel(master = self, text = 'Position:')
        label.grid(row = 0, column = 0)
        self.pos_label = TkArrayLabel(master = self, length = 3, digits = 3)
        self.pos_label.grid(row = 0, column = 1)

        label = TkLabel(master = self, text = 'Orientation:')
        label.grid(row = 1, column = 0)
        self.rot_label = TkArrayLabel(master = self, length = 4, digits = 3)
        self.rot_label.grid(row = 1, column = 1)

    def update_data(self, value: Pose):

        self.pos_label.update_data([value.position.x, 
                                    value.position.y, 
                                    value.position.z])
        self.rot_label.update_data([value.orientation.w, 
                                    value.orientation.x, 
                                    value.orientation.y, 
                                    value.orientation.z])
        
class TkPoseEntryFrame(TkLabelFrame):

    def __init__(self, text = 'Pose', **args):

        super().__init__(text = text, **args)

        label_frame = TkLabelFrame(master = self, text = 'Position')
        label_frame.grid(row = 0, column = 0)
        label = TkLabel(master = label_frame, text = 'X:')
        label.grid(row = 0, column = 0)
        self.x_entry = TkFloatEntry(master = label_frame)
        self.x_entry.grid(row = 0, column = 1)
        label = TkLabel(master = label_frame, text = 'Y:')
        label.grid(row = 1, column = 0)
        self.y_entry = TkFloatEntry(master = label_frame)
        self.y_entry.grid(row = 1, column = 1)
        label = TkLabel(master = label_frame, text = 'Z:')
        label.grid(row = 2, column = 0)
        self.z_entry = TkFloatEntry(master = label_frame)
        self.z_entry.grid(row = 2, column = 1)

        label_frame = TkLabelFrame(master = self, text = 'Orientation')
        label_frame.grid(row = 1, column = 0)
        label = TkLabel(master = label_frame, text = 'X:')
        label.grid(row = 0, column = 0)
        self.rx_entry = TkFloatEntry(master = label_frame)
        self.rx_entry.grid(row = 0, column = 1)
        label = TkLabel(master = label_frame, text = 'Y:')
        label.grid(row = 1, column = 0)
        self.ry_entry = TkFloatEntry(master = label_frame)
        self.ry_entry.grid(row = 1, column = 1)
        label = TkLabel(master = label_frame, text = 'Z:')
        label.grid(row = 2, column = 0)
        self.rz_entry = TkFloatEntry(master = label_frame)
        self.rz_entry.grid(row = 2, column = 1)
    
    def get_data(self):

        T = TransformMatrix([self.x_entry.get_data(),
                             self.y_entry.get_data(),
                             self.z_entry.get_data(),
                             self.rx_entry.get_data(),
                             self.ry_entry.get_data(),
                             self.rz_entry.get_data()])
        
        pose = Pose()
        pose.position.x = T.r[0]
        pose.position.y = T.r[1]
        pose.position.z = T.r[2]
        pose.orientation.w = T.q[0]
        pose.orientation.x = T.q[1]
        pose.orientation.y = T.q[2]
        pose.orientation.z = T.q[3]

        return pose

class TkButton(tk.Button):

    def __init__(self, **kwargs):

        super().__init__(**kwargs)


class TkCancelButton(tk.Button):

    def __init__(self, text: str = 'CANCEL', **kwargs):

        super().__init__(background="#FFC0CB", text=text, **kwargs)

class TkScrollbar(tk.Scrollbar):

    def __init__(self, **args):

        super().__init__(**args)

class TkCanvas(tk.Canvas):

    def __init__(self, **args):

        super().__init__(**args)

class TkFrame(tk.Frame):

    def __init__(self, **args):

        super().__init__(**args)

class TkScrollFrame(TkFrame):

    def __init__(self, master: tk.Toplevel = None, **args):

        frame_y_scrollbar = TkFrame(master = master)
        frame_y_scrollbar.pack(fill = tk.BOTH, expand = 1)
        frame_x_scrollbar = TkFrame(master = frame_y_scrollbar)
        frame_x_scrollbar.pack(fill = tk.X, side = tk.BOTTOM)

        canvas = TkCanvas(master = frame_y_scrollbar)
        canvas.pack(side = tk.LEFT, fill = tk.BOTH, expand = 1)

        # add scrollbars to canvas
        x_scrollbar = TkScrollbar(master = frame_x_scrollbar, orient = tk.HORIZONTAL, command = canvas.xview)
        x_scrollbar.pack(side = tk.BOTTOM, fill = tk.X)
        y_scrollbar = TkScrollbar(master = frame_y_scrollbar, orient = tk.VERTICAL, command = canvas.yview)
        y_scrollbar.pack(side = tk.RIGHT, fill = tk.Y)

        # configure canvas
        canvas.configure(xscrollcommand = x_scrollbar.set)
        canvas.configure(yscrollcommand = y_scrollbar.set)
        canvas.bind("<Configure>", lambda e: canvas.config(scrollregion = canvas.bbox(tk.ALL))) 

        # create scrollar frame INSIDE the canvas
        super().__init__(master = canvas, **args)

        # Add frame a window in Canvas
        canvas.create_window((0,0), window = self, anchor="nw")

class TkOptionMenu(tk.OptionMenu):

    def __init__(self, master, values: list, **kwargs):

        self.variable = tk.StringVar()
        self.values = values

        super().__init__(master, self.variable, *self.values, **kwargs)

        self.config(width=10, height=1)
        self.variable.set(self.values[0]) # put value element as default

    def get_index(self):

        return self.values.index(self.variable.get())

    def get_data(self):

        return self.variable.get()




