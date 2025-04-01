// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.DPadHelper;


public class Robot extends TimedRobot {

  // Auto time  
  long elapsedTime;
  long autonomousDuration = 15000;
  long startTime;

  // Drive command variables
  Double strafe;
  Double forward;
  Double rotate;
  boolean fieldRelative;
  boolean rateLimit;

  // Elevator variables
  Integer elevatorlevel;
  boolean CoralModeTrueAlgaeModeFalse;

  // The robot's subsystems
  private final DriveSubsystem swerveDrive = new DriveSubsystem();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final CoralSubsystem coral = new CoralSubsystem();
  private final AlgaeSubsystem algae = new AlgaeSubsystem();

  // PhotonVision 
  PhotonCamera camera = new PhotonCamera("FrontLeftCamera");
  boolean targetVisible;
  double targetYaw;
  int tagid;
  int bestTarget;
  double largestArea;
  double targetRange;

  // The driver's controller
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
    elevator.periodic();
  }

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

  /* AUTONOMOUS INIT */
  @Override
  public void autonomousInit() {  
    elevator.init();
    algae.init();
    coral.init();
    fieldRelative = false;
    rateLimit = false;    
    startTime = System.currentTimeMillis();
  }

  /* AUTONOMOUS PERIODIC */
  @Override
  public void autonomousPeriodic() {

    forward = 0.0;
    strafe = 0.0;
    rotate = 0.0; 
    targetYaw = 0.0;
    targetRange = 0.0;
    elapsedTime = System.currentTimeMillis() - startTime;
    
    // Drive forward for two seconds
    if (elapsedTime < 2000) {
      forward = 0.05; }
    
    // For the next 5 seconds rotate until you see a Reef AprilTag, then rotate and drive towards it
    else if (elapsedTime < 7000) {      
      targetVisible = false;      
      largestArea = 0.0;
      forward = 0.0;
      rotate = 0.0;
      var results = camera.getAllUnreadResults();

      if (!results.isEmpty()){
        // Get the most recent frame
        var result = results.get(results.size() - 1);
        if (result.hasTargets()) {
            // At least one AprilTag was seen by the camera
            for (var target : result.getTargets()) {
                tagid = target.getFiducialId();
                // If the tag is a reef tag
                if ((tagid > 6 && tagid < 11) || (tagid > 17 && tagid < 21)) {
                  // Find the closest reef tag (takes up the largest area of screen)
                  if (target.getArea() > largestArea) {
                    largestArea = target.getArea();
                    bestTarget = tagid;
                    targetYaw = target.getYaw();
                    targetRange = PhotonUtils.calculateDistanceToTargetMeters(Constants.PhotonVisionConstants.kCameraHeight, Constants.PhotonVisionConstants.kReefAprilTagHeight, Units.degreesToRadians(0.0), Units.degreesToRadians(target.getPitch()));
                    targetVisible = true;}}}}}

      if (targetVisible) {
        // forward = (targetRange - Constants.PhotonVisionConstants.kReefAprilTagDistance) * 0.03;
        rotate = targetYaw / 15; }}
    
    swerveDrive.drive(forward, strafe, rotate, fieldRelative, rateLimit);
  }

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

  /* TELEOP INIT */
  @Override
  public void teleopInit() {
    // Initially using field relative with rate limits
    elevator.init();
    elevatorlevel = 0;
    fieldRelative = true;
    rateLimit = false;
  }

  /* TELEOP PERIODIC */
  @Override
  public void teleopPeriodic() {
    
    // A and Y Button - manually control elevator
    if (controller.getAButton()) {         
      elevator.lower(); } 
    else if (controller.getYButton()) { 
      elevator.raise(); } 
    else {
      elevator.stop(); }

    
    // B and X Button - manually control coral wrist
    if (controller.getBButton()) {
      coral.wristraise(); } 
    else if (controller.getXButton()) {
      coral.wristlower(); } 
    else {
      coral.wriststop(); }


    // Triggers - control intake and outtake
    if (controller.getRightTriggerAxis() > 0.05) {  
      //algae.intake();
      coral.intake(); } 
    else if (controller.getLeftTriggerAxis() > 0.05) {
      //algae.outtake();
      coral.outtake(); } 
    else {
      //algae.stop();
      coral.stop(); }


    // Bumpers - manually control Algae Wrist
    if (controller.getRightBumperButton()) {
      algae.wristraise(); } 
    else if (controller.getLeftBumperButton()) {
      algae.wristlower(); } 
    else {
      algae.wriststop(); }
    
    
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

    // Send control values to swerve drive
    swerveDrive.drive(forward, strafe, rotate, fieldRelative, rateLimit);
  }

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

  /* TEST INIT */
  @Override
  public void testInit() {
    elevatorlevel = 0;
    CoralModeTrueAlgaeModeFalse = true;
    elevator.motorrunning = true;
  }

  /* TEST PERIODIC */
  @Override
  public void testPeriodic() {
    
    // DPad Left - Select Coral Mode 
    if (dPad.getDPadLeftPressed()) {
      CoralModeTrueAlgaeModeFalse = true;
      algae.fold();
      coral.unfold();}
    // DPad Right - Select Algae Mode 
    else if (dPad.getDPadRightPressed()) {
      CoralModeTrueAlgaeModeFalse = false; 
      algae.unfold();
      coral.fold();}

    // DPad Up - Go up a level
    if (dPad.getDPadUpPressed()) {
      elevatorlevel +=1;            
      elevatorlevel = elevatorlevel % 5; 
      if (CoralModeTrueAlgaeModeFalse) {
        elevator.goToCoralLevel(elevatorlevel); } 
      else {
        elevator.goToAlgaeLevel(elevatorlevel); }}
    // DPad Down - Go down a level
    else if (dPad.getDPadDownPressed()) {
      elevatorlevel -=1;      
      elevatorlevel = elevatorlevel % 5; 
      if (CoralModeTrueAlgaeModeFalse) {
        elevator.goToCoralLevel(elevatorlevel); } 
      else {
        elevator.goToAlgaeLevel(elevatorlevel); }}

    // Y Button - manual control elevator Up
    if (controller.getYButton()) {         
      elevator.raise();} 
    // A Button - manual control elevator Down
    else if (controller.getAButton()) { 
      elevator.lower();} 
    else if (elevator.motorrunning = true) {
      elevator.stop();}
  }
}