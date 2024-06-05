import pygame

def print_controller_inputs():
    # Initialize the joystick module
    pygame.init()
    pygame.joystick.init()

    # Check for connected joysticks
    joystick_count = pygame.joystick.get_count()

    if joystick_count == 0:
        print("No controller found.")
        return

    # Initialize the first joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print("Name:", joystick.get_name())
    print("Number of Axes:", joystick.get_numaxes())
    print("Number of Buttons:", joystick.get_numbuttons())
    print("Number of Hats:", joystick.get_numhats())

    try:
        while True:
            pygame.event.pump()

            # Print the state of each axis
            for i in range(joystick.get_numaxes()):
                axis_value = joystick.get_axis(i)
                print(f"Axis {i}: {axis_value:.2f}")

            # Print the state of each button
            for i in range(joystick.get_numbuttons()):
                button_state = joystick.get_button(i)
                print(f"Button {i}: {button_state}")

            # Print the state of each hat
            for i in range(joystick.get_numhats()):
                hat_state = joystick.get_hat(i)
                print(f"Hat {i}: {hat_state}")

            print("\n---\n")

            pygame.time.wait(10)  # Delay to reduce console output frequency

    except KeyboardInterrupt:
        print("Program terminated.")

    finally:
        pygame.quit()

if __name__ == "__main__":
    print_controller_inputs()
