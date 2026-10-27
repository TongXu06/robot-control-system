


print("Robot Control System Started!")

class Robot:

    def __init__(self, name):
        self.name = name

        self.joints = {
            "shoulder": 30,
            "elbow": 60,
            "wrist": 90
        }

        self.history = []

    def move_joint(self, joint, step):

        if joint not in self.joints:
            print("Invalid joint!")
            return

        current_angle = self.joints[joint]
        next_angle = current_angle + step

        if not 0 <= next_angle <= 180:
            print("Danger! Movement rejected!")
            return

        self.joints[joint] = next_angle

        record = {
            "joint": joint,
            "before": current_angle,
            "after": next_angle
        }

        self.history.append(record)

        print("Movement successful!")

    def show_state(self):
        print("Robot:", self.name)

        for joint, angle in self.joints.items():
            print(joint, ":", angle)

    def show_history(self):
        print("\nRobot Motion History")

        if len(self.history) == 0:
            print("No movements recorded.")
            return

        for i, record in enumerate(self.history, start=1):
            print(
                i,
                record["joint"],
                ":",
                record["before"],
                "->",
                record["after"]
            )

    def reset(self):
        for joint in self.joints:
            self.joints[joint] = 0

        print("Robot reset!")

robot1 = Robot("Robot A")

while True:

    print("\n===== Robot Control System =====")
    print("1. Move Joint")
    print("2. Show Status")
    print("3. Show History")
    print("4. Reset Robot")
    print("5. Exit")

    choice = input("Choose an option: ")

    if choice == "1":

        joint = input("Choose joint:")

        try:
            step = float(input("Rotation angle:"))

        except ValueError:
            print("Please enter a valid number!")
            continue

        robot1.move_joint(joint, step)

    elif choice == "2":
        robot1.show_state()

    elif choice == "3":
        robot1.show_history()

    elif choice == "4":
        robot1.reset()
        robot1.show_state()

    elif choice == "5":
        print("Goodbye!")
        break

    else:
        print("Invalid option!")
