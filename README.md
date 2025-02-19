# SMART #6932 REEFSCAPE-2025
#### This code is a fork of Gene Panov's (username epanov1602) [CommandRevSwerve](https://github.com/epanov1602/CommandRevSwerve) Repository. We take no credit for the docs or the framework. Guides are below for how to properly add subsystems to the repository.

## Requirements
- Robotpy (latest)
- Latest modules in pyproject.toml (done with `py -m pip install robotpy[all]`)
- Python 3.13+
- WPILib 2025.2.1

## How to add any manner of subsystems (if threre isn't one already):
(These also work just fine on tank/mechanum drivetrains)
- [Adding an Arm Subsystem](docs/Adding_Arm.md)
- [Adding a Camera Subsystem](docs/Adding_Camera.md)
- [Adding an Elevator Subsystem](docs/Adding_Elevator.md)
- [Adding an Intake Subsystem](docs/Adding_Intake.md)
- [Adding a Shooter Subsystem](docs/Adding_Shooter.md)
- [Adding a Localizer by reusing PhotonVision cameras](docs/Adding_Localizer.md)
- [Adding Blinkin LED Strips Standalone](docs/Adding_Blinkin_LED_Strip.md)
- [Adding Blinkin LED Strip as part of Intake](docs/Adding_Blinkin_LED_Strip_into_Intake.md)
- [Adding Trajectory Picker](docs/Adding_TrajectoryPicker.md) (https://www.youtube.com/watch?v=eYY4uIxZJlo)

## Adding complex behaviors: commands
- [Autonomous Driving and Aiming](docs/Command_Driving_Aiming.md)
- [Add "Pick Up" Command](docs/Command_PickUp.md)
- [Add "Shoot" Command](docs/Command_Shoot.md)

## Step-by-Step Walkthrough by Gene Panov
https://www.youtube.com/watch?v=K2Aj0S4-aKI
