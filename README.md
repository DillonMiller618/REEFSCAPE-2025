# SMART #6932 REEFSCAPE-2025
#### This code is a fork of [SwervePy](https://github.com/EWall25/swervepy) and Gene Panov's [CommandRevSwerve](https://github.com/epanov1602/CommandRevSwerve/tree/main) repositories. We take no credit for any of the swerve or docs.

## Requirements
- Robotpy (latest)
- Latest modules in pyproject.toml (done with `py -m pip install robotpy[all]`)
- Python 3.13+
- WPILib 2025.2.1

## How to use, connect and deploy to an FRC bot
Make sure as (as a preemptive step) you are using the most recent version of WPILib's VSCode. Then, make sure you have the FRC Driver Station Application, downloadable through [Ni's FRC Game Tools](https://www.ni.com/en/support/downloads/drivers/download.frc-game-tools.html). Once you have these applications open and ready, follow the steps below:
1) Power on the FRC bot you intend to connect to by flipping its breaker.
2) Connect to the RoboRIO through either:
    - A wireless connection, provided it's connected to a radio. After around a minute of being powered on, a network should show up, typically with the team name, and some identifying factor about the robot (for example, its name).
    - A USB-A to USB-B (not micro-b) cable, directly to the RoboRIO. This is the preferred method of communication at competitions while in pits, and for any robot that will not be physically moving during testing.
3) At this point, you would be able to enable and disable the robot, as well as upload code to the robot. Enable/disable controls are in the FRC Driver Station application, and uploading takes place in WPILib VSCode.
    - *Note: Uploading through WPILib's command palette only works for Java and C++, not Python.*
4) To deploy (upload) program code, run a command in the terminal:  ```py -3 -m robotpy deploy```
    - *Note: Unless you have Unit Tests for your program, you can add --skip-tests to avoid it checking for them.*'
5) You're done! There is more to the process of writing code for an FRC bot, but I'll leave it here. This is just to tell you how to upload and connect to a robot, not how to write the code itself. There are good guides out there for Python code in FRC, but there is a lot of trial and error involved. I wish you the best with using this code, and be sure to let me know about all the dumb mistakes I've made when I wrote this.

## How to add any manner of subsystems (if threre isn't one already):
*Note: not all of these subsystems have been tested! You will definitely run into bugs and might not be able to use them!*
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

## Contact information
dillonmiller618@gmail.com
