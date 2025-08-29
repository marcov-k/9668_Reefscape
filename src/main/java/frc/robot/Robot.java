// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.DPadHelper;
import frc.utils.Common;
import edu.wpi.first.wpilibj.DigitalInput;



public class Robot extends TimedRobot {

  // Time  
  long elapsedTime;
  long startTime;
  long algaeTime;
  long coralTime;

  // Swerve Drive 
  Double strafe;
  Double forward;
  Double rotate;
  boolean fieldRelative;
  boolean rateLimit;

  // Elevator 
  Integer elevatorlevel;

  // Subsystems
  private final DriveSubsystem swerveDrive = new DriveSubsystem();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final CoralSubsystem coral = new CoralSubsystem();
  private final AlgaeSubsystem algae = new AlgaeSubsystem();

  // Auto
  boolean shootingAlgae;

  // Controller
  private final XboxController controller = new XboxController(OIConstants.kDriverControllerPort);
  DPadHelper dPad = new DPadHelper(controller);


  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
  
  /* ROBOT INIT */
  @Override
  public void robotInit() { 
    swerveDrive.zeroHeading();   
    swerveDrive.setPose(0,0,180);
    elevator.init(); 
  }

  /* ROBOT PERIODIC */
  @Override
  public void robotPeriodic() {
    swerveDrive.periodic();
    elevator.robotPeriodic();
    coral.robotPeriodic();
    algae.robotPeriodic();
  }

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

  /* TELEOP INIT */
  @Override
  public void teleopInit() {    
    elevator.init();
    fieldRelative = false;
    rateLimit = false;
    elevator.manualcontrol = true;
    coral.manualcontrol = true;
    algae.manualcontrol = true;
  }

  /* TELEOP PERIODIC */
  @Override
  public void teleopPeriodic() {

    // Y & A Buttons - manual control elevator Up and Down
    if (controller.getYButton()) {         
      elevator.raise();} 
    else if (controller.getAButton()) { 
      elevator.lower();} 
    else if (elevator.manualcontrol) {
      elevator.stop();}

    
    // Bumpers - manually control Algae Wrist
    if (controller.getRightBumperButton()) {
        algae.wristraise(); 
        algae.manualcontrol = true;}  
    else if (controller.getLeftBumperButton()) {
        algae.wristlower();
        algae.manualcontrol = true; } 
    else {      
      if (algae.manualcontrol) {
        algae.wriststop(); }}  
     
    if (controller.getBButtonPressed())
    {
        algae.manualcontrol = false;
        algae.ballDemo();
    }

    // Triggers - control intake and outtake based on Mode
    if (controller.getRightTriggerAxis() > 0.05) {  
        coral.outtake();
        algae.outtake(); 
      } 
    else if (controller.getLeftTriggerAxis() > 0.05) {
        coral.intake();
        algae.intake();
       } 
    else {
      algae.stop();
      coral.stop(); }
  

    // Back button - Zero Heading
    if (controller.getBackButtonPressed()) {
      swerveDrive.zeroHeading(); }

      
    // Start button - Toggles field relative
    if (controller.getStartButtonPressed()) {            
      fieldRelative = !fieldRelative; }

    // Get control values from the controller, apply speed limits and deadband
    strafe = MathUtil.applyDeadband(controller.getLeftX() * OIConstants.kDriverSpeedLimit * elevator.elevatorspeedlimiter, OIConstants.kDriveDeadband);
    forward = MathUtil.applyDeadband(-controller.getLeftY() * OIConstants.kDriverSpeedLimit * elevator.elevatorspeedlimiter, OIConstants.kDriveDeadband);
    rotate = MathUtil.applyDeadband(controller.getRightX() * OIConstants.kDriverRotationLimit, OIConstants.kDriveDeadband);



    // Send values to swerve drive    
    swerveDrive.drive(forward, strafe, rotate, fieldRelative, rateLimit);

    algae.teleopPeriodic();
  }

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

  /* TEST INIT */
  @Override
  public void testInit() { }

  /* TEST PERIODIC */
  @Override
  public void testPeriodic() {}
}