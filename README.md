# 9668_Reefscape

### Malfunctionz - West High School Robotics Club
#### West High School, Knoxville, TN

---

## Description

This repository contains the robot code for the West High School Robotics Club's entry in the 2025 FIRST Robotics Competition (FRC) game, **"Reefscape."** 

The project builds on the [REV MAXSwerve Java Template](https://github.com/REVrobotics/MAXSwerve-Java-Template/), designed for an FRC swerve drive train using REV MAXSwerve Modules.

---

## Getting Started

### Prerequisites

Install the 2025 WPILib VS Code environment:
1. Follow the instructions [here](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html).
2. Download `WPILib_Windows-2025.1.1.ISO`.
3. Right-click the ISO file and select "Mount" as a disk.
4. Run `D:\WPILibInstaller.exe`.
5. Install for all users and download for this computer only.
6. After completing the installation, eject the ISO by right-clicking the `D:` drive and selecting "Eject."

### Setting Up the Project

1. Open 2025 WPILib VS Code.
2. Fork a copy of the main repository in GitHub
   - Go to https://github.com/rrmcmurry/9668_Reefscape
   - Click on the button that says Fork in the upper right corner
   - This should make a copy of 9668_Reefscape in your own GitHub account.
3. Clone your forked GitHub repository locally in VS Code:
   - On the Welcome Tab under "Start" look for "Clone Git Repository" and select it
   - https://github.com/YOURUSERNAME/9668_Reefscape.git <- Type this in but use your github username   
4. Verify the setup:
   - Check for "Build Successful" in the Terminal.
5. Navigate to the project files:
   - `9668_REEFSCAPE/src/main/java/frc/robot`
   - Focus on modifying `Robot.java`, `Constants.java`, and any files under `subsystems`.

### Workflow 

- **Fetch upstream and merge** 
  - Fetch updates from the main repository before you begin coding 
- **Code & Test:** 
  - Ensure your project compiles and builds successfully before pushing changes
- **Commit** 
  - Use the VS Code Source control area to commit and push your changes back to your own GitHub repository.
- **Pull Requests:**
  - Create a Pull Request when you want me to pull your changes into the main repository


---

## Objectives

### Initial Setup
- [x] Update to 2025 Base Code
- [x] Confirm code runs on the 9668_Swerve robot.
- [ ] Validate AprilTag detection using a [parallel processor](https://github.com/rrmcmurry/WestPi/).
- [ ] Confirm NetworkTables-based control commands work as expected.

### AprilTag-Based Navigation
- [ ] Implement robot alignment to AprilTags.
- [ ] Add team selection (Red/Blue) for AprilTag sets.
- [ ] Enable teleautonomous target selection using the Xbox controller:
	- Reef Right
	- Reef Left
	- Processor
	- Intake
- [ ] Define AprilTag locations and sets.
- [ ] Use AprilTags to autonomously navigate to:
	- Reef Right
	- Reef Left
	- Processor
	- Intake
- [ ] Determine camera configuration:
	- Single static
	- Swivel mounted
	- Turret mounted
	- Dual static

### Mechanisms

#### Elevator Subsystem
- [ ] Implement commands for predefined positions:
	- Stowed
	- Level 1 (L1)
	- Level 2 (L2)
	- Level 3 (L3)
	- Level 4 (L4)
	- Up
	- Down
- [ ] Implement a linear magnetic encoder or continuous string potentiometer to sense elevator position for PID control

#### Arm Rotation Subsystem
- [ ] Thru-Bore encoder for measuring rotation position for PID control

#### Coral Intake Subsystem
- [ ] Add commands:
	- Intake
	- Output
- [ ] Sensor for confirming intake 

#### Algae Subsystem
- [ ] Add commands:
	- Intake
	- Output


### Fine-Tuning
- [ ] Tune scoring positions for Coral placement.
- [ ] Investigate autonomous detection of Algae.

---

Contributers:
Robby McMurry


