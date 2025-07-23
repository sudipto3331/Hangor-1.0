import tkinter as tk
import pygame
import threading
import time

# Tkinter GUI setup first (to avoid SDL conflict on macOS)
root = tk.Tk()
root.title("Joystick Calibration & Monitor")

# Placeholder lists
axis_sliders = []
button_labels = []

# Frame for joystick name
joystick_name_var = tk.StringVar()
joystick_name_var.set("Joystick: Not Connected")
tk.Label(root, textvariable=joystick_name_var, font=("Arial", 12, "bold")).pack(pady=10)

# Frames for axes and buttons
frame_axes = tk.LabelFrame(root, text="Axes", padx=10, pady=10)
frame_axes.pack(padx=10, pady=5, fill="x")

frame_buttons = tk.LabelFrame(root, text="Buttons", padx=10, pady=10)
frame_buttons.pack(padx=10, pady=5, fill="x")

# Background joystick monitor thread
def joystick_loop():
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        joystick_name_var.set("No joystick found")
        return

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    joystick_name_var.set(f"Joystick: {joystick.get_name()}")

    axis_count = joystick.get_numaxes()
    button_count = joystick.get_numbuttons()

    # Create sliders for axes
    for i in range(axis_count):
        label = tk.Label(frame_axes, text=f"Axis {i}")
        label.grid(row=i, column=0)
        slider = tk.Scale(frame_axes, from_=-1000, to=1000, length=400,
                          orient=tk.HORIZONTAL, resolution=1)
        slider.set(0)
        slider.grid(row=i, column=1)
        axis_sliders.append(slider)

    # Create labels for buttons
    for i in range(button_count):
        label = tk.Label(frame_buttons, text=f"Button {i}")
        label.grid(row=i, column=0)
        state = tk.Label(frame_buttons, text="RELEASED", bg="lightgray", width=12)
        state.grid(row=i, column=1)
        button_labels.append(state)

    # Loop to read and update
    while True:
        pygame.event.pump()

        for i in range(axis_count):
            val = int(joystick.get_axis(i) * 1000)
            axis_sliders[i].set(val)

        for i in range(button_count):
            pressed = joystick.get_button(i)
            if pressed:
                button_labels[i].config(text="PRESSED", bg="green")
            else:
                button_labels[i].config(text="RELEASED", bg="lightgray")

        time.sleep(0.05)

# Launch the joystick monitor after the Tk window is ready
threading.Thread(target=joystick_loop, daemon=True).start()

root.mainloop()
