// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.NetworkController;



public class Robot extends TimedRobot {

  // Drive command variables
  Double strafe;
  Double forward;
  Double rotate;
  boolean fieldRelative;
  boolean rateLimit;
  boolean teleautonomous;
  boolean humandriver;
  Integer elevatorlevel;
  

  // The robot's subsystems
  private final DriveSubsystem swerveDrive = new DriveSubsystem();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final CoralSubsystem coral = new CoralSubsystem();
  private final AlgaeSubsystem algae = new AlgaeSubsystem();

  // The driver's controller
  private final XboxController controller = new XboxController(OIConstants.kDriverControllerPort);
  

  NetworkTable GameManager;
  NetworkTableEntry nthumandriver;
  NetworkController autoController;
  

  @Override
  public void robotInit() {
    //autoController.Initialize();
    swerveDrive.setPose(0,0,0);
    elevator.init();
    elevator.goToLevel(0);
    nthumandriver = NetworkTableInstance.getDefault().getTable("GameManager").getEntry("HumanDriver");
  }

  @Override
  public void robotPeriodic() {
    swerveDrive.periodic();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {  
    // Initially using field relative with rate limits
    elevator.init();
    fieldRelative = true;
    rateLimit = false;    
    nthumandriver.setBoolean(false);
    
  }

  @Override
  public void autonomousPeriodic() {

    // Get control values from network tables
    strafe = MathUtil.applyDeadband(autoController.leftJoyX.get(), OIConstants.kDriveDeadband);
    forward = MathUtil.applyDeadband(autoController.leftJoyY.get(), OIConstants.kDriveDeadband);
    rotate = MathUtil.applyDeadband(autoController.rightJoyX.get(), OIConstants.kDriveDeadband);

    // Send control values to swerve drive
    swerveDrive.drive(forward, strafe, rotate, fieldRelative, rateLimit);

  }

  @Override
  public void teleopInit() {
    // Initially using field relative with rate limits
    elevator.init();
    fieldRelative = true;
    rateLimit = false;
    teleautonomous = false;
    nthumandriver.setBoolean(true);    
    elevatorlevel = 0;
  }

  @Override
  public void teleopPeriodic() {
    
    
    if (controller.getAButton()) {         
      elevator.lower();  
    } else if (controller.getYButton()) { 
      elevator.raise();   
    } else {
      elevator.stop();
    }
    if (controller.getAButtonPressed()) {
      elevatorlevel -= 1;  
    }
    
    if (controller.getYButtonPressed()) {
      elevatorlevel += 1;  
    }


    elevatorlevel = Math.abs(elevatorlevel % 5);
    
    // elevator.goToCoralLevel(elevatorlevel);
    
    if (controller.getBButton()) {
      coral.wristraise();
    }
    else if (controller.getXButton()) {
      coral.wristlower();
    }
    else {
      coral.wriststop();
    }

    if (controller.getRightTriggerAxis() > 0.05) {
      algae.wristraise();
    }
    else if (controller.getLeftTriggerAxis() > 0.05) {
      algae.wristlower();
    }
    else {
      algae.wriststop();
    }



    if (controller.getRightBumperButton()) {
      algae.intake();
      coral.intake();
      
    }
    else if (controller.getLeftBumperButton()) {
      algae.outtake();
      coral.outtake();
    }
    else {
      algae.stop();
      coral.stop();
    }
     
    // Back button - Toggles field relative    
    if (controller.getBackButtonPressed()) {
      fieldRelative = !fieldRelative;      
    }
    
    // Start button - Toggles autonomous mode
    if (controller.getStartButtonPressed()) {      
      //teleautonomous = !teleautonomous;
    }

    // Get control values from the controller and apply speed limit and deadband
    strafe = MathUtil.applyDeadband(controller.getLeftX() * OIConstants.kDriverSpeedLimit, OIConstants.kDriveDeadband);
    forward = MathUtil.applyDeadband(-controller.getLeftY() * OIConstants.kDriverSpeedLimit, OIConstants.kDriveDeadband);
    rotate = MathUtil.applyDeadband(controller.getRightX() * OIConstants.kDriverRotationLimit, OIConstants.kDriveDeadband);

    // Evaluate whether a driver is driving and publish to networktables
    humandriver = !(strafe == 0 && forward == 0 && rotate == 0);
    nthumandriver.setBoolean(humandriver);

    // If a human isn't currently driving and we're in teleautonomous mode
    if (teleautonomous && !humandriver) {
      // Use controller values from network tables 
      //strafe = autoController.leftJoyX.get();
      //forward = autoController.leftJoyY.get();
      //rotate = autoController.rightJoyX.get();
    }
    

    // Send controller values to swerve drive
    swerveDrive.drive(forward, strafe, rotate, fieldRelative, rateLimit);

  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}