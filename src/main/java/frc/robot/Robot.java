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




public class Robot extends TimedRobot {

  // Time  
  long elapsedTime;
  long startTime;

  // Swerve Drive 
  Double strafe;
  Double forward;
  Double rotate;
  boolean fieldRelative;
  boolean rateLimit;

  // Elevator 
  Integer elevatorlevel;
  boolean CoralModeTrueAlgaeModeFalse;

  // Subsystems
  private final DriveSubsystem swerveDrive = new DriveSubsystem();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final CoralSubsystem coral = new CoralSubsystem();
  private final AlgaeSubsystem algae = new AlgaeSubsystem();

  // PhotonVision 
  PhotonCamera camera;
  boolean targetVisible;  
  int tagid;
  int bestTarget;
  double largestArea;
  Transform3d targetTransform;
  Translation3d targetTranslation;
  Rotation3d targetRotation;
  double targetForwardDistance;
  double targetStrafeDistance;
  double targetYaw;
  boolean aligned;

  // Controller
  private final XboxController controller = new XboxController(OIConstants.kDriverControllerPort);
  DPadHelper dPad = new DPadHelper(controller);

  private double clamp(double value, double min, double max, double deadband) {
    if (Math.abs(value) < deadband) return 0;
    else return Math.max(min, Math.min(max, value)); }
  

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
  
  /* ROBOT INIT */
  @Override
  public void robotInit() { 
    swerveDrive.zeroHeading();   
    swerveDrive.setPose(0,0,180);
    elevator.init();
    camera = new PhotonCamera("FrontLeftCamera");
    camera.getLatestResult(); // warm-up   
  }

  /* ROBOT PERIODIC */
  @Override
  public void robotPeriodic() {
    swerveDrive.periodic();
    elevator.periodic();
    coral.periodic();
    algae.periodic();
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
    aligned = false;
  }

  /* AUTONOMOUS PERIODIC */
  @Override
  public void autonomousPeriodic() {
    // Initialize Variables 
    elapsedTime = System.currentTimeMillis() - startTime;
    forward = 0.0;
    strafe = 0.0;
    rotate = 0.0; 
    targetForwardDistance = 0.0;
    targetStrafeDistance = 0.0;
    targetYaw = 0.0;
    targetVisible = false;      
    
    
    // Drive forward for two seconds
    if (elapsedTime < 2000) {
      forward = 0.05; }
    
    // For the next 5 seconds rotate until you see a Reef AprilTag, then rotate and drive towards it
    else if (elapsedTime < 15000 && !aligned) {      
      largestArea = 0.0;
      var results = camera.getAllUnreadResults();

      if (!results.isEmpty()){
        var result = results.get(results.size() - 1);
        if (result.hasTargets()) {
            for (var target : result.getTargets()) {
                tagid = target.getFiducialId();
                // If the tag is a reef tag
                if ((tagid > 6 && tagid < 11) || (tagid > 17 && tagid < 21)) {
                  // Find the closest reef tag (based on largest area of frame) and capture Yaw and Range
                  if (target.getArea() > largestArea) {
                    largestArea = target.getArea();
                    bestTarget = tagid;
                    targetTransform = target.getBestCameraToTarget();
                    targetTranslation = targetTransform.getTranslation();
                    targetRotation = targetTransform.getRotation();
                    targetForwardDistance = targetTranslation.getZ();
                    targetStrafeDistance = targetTranslation.getX();
                    targetYaw = Math.toDegrees(targetRotation.getZ());
                    targetVisible = true;}}}}}

      if (targetVisible) {
        forward = clamp(targetForwardDistance - 0.3, -0.3, 0.3, 0.05);        
        strafe = clamp(targetStrafeDistance, -0.3, 0.3, 0.05);
        rotate = clamp(targetYaw / 30.0, -0.1, 0.1, 0.02); }

      aligned = Math.abs(targetForwardDistance - 0.3) < 0.05 && Math.abs(targetStrafeDistance) < 0.05 && Math.abs(targetYaw) < 3.0; }
    
    else if (aligned) {
      elevator.goToCoralLevel(2);
      coral.scoringpose(); 
      coral.outtake();
    }
    
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
    CoralModeTrueAlgaeModeFalse = false;
    elevator.motorrunning = true;    
  }

  /* TELEOP PERIODIC */
  @Override
  public void teleopPeriodic() {
    
    // DPad Left - Select Coral Mode 
    if (dPad.getDPadLeftPressed()) {
      CoralModeTrueAlgaeModeFalse = true;
      //algae.fold();
      //coral.unfold();
      }
    // DPad Right - Select Algae Mode 
    else if (dPad.getDPadRightPressed()) {
      CoralModeTrueAlgaeModeFalse = false; 
      //algae.unfold();
      //coral.fold();
      }

    // DPad Up - Go up a level
    if (dPad.getDPadUpPressed()) {
      elevatorlevel +=1;    
      elevator.level +=1;        
      elevatorlevel = Math.floorMod(elevatorlevel, 5); 
      
      if (CoralModeTrueAlgaeModeFalse) {
        elevator.goToCoralLevel(elevatorlevel); } 
      else {
        elevator.goToAlgaeLevel(elevatorlevel); } }
    // DPad Down - Go down a level
    else if (dPad.getDPadDownPressed()) {
      elevatorlevel -=1;      
      elevator.level -=1;
      elevatorlevel = Math.floorMod(elevatorlevel, 5); 
      if (CoralModeTrueAlgaeModeFalse) {
        elevator.goToCoralLevel(elevatorlevel); } 
      else {
        elevator.goToAlgaeLevel(elevatorlevel); }}

    // Y Button - manual control elevator Up and Down
    if (controller.getYButton()) {         
      elevator.raise();} 
    else if (controller.getAButton()) { 
      elevator.lower();} 
    else if (elevator.motorrunning) {
      elevator.stop();}

    
    // B and X Button - manually control coral wrist
    if (controller.getBButton()) {
      coral.wristraise(); } 
    else if (controller.getXButton()) {
      coral.wristlower(); } 
    else {
      coral.wriststop(); }

    // Bumpers - manually control Algae Wrist
    if (controller.getRightBumperButton()) {
      algae.wristraise(); } 
    else if (controller.getLeftBumperButton()) {
      algae.wristlower(); } 
    else {
      algae.wriststop(); }

    // Triggers - control intake and outtake
    if (controller.getRightTriggerAxis() > 0.05) {  
      if (CoralModeTrueAlgaeModeFalse) {
        coral.intake(); } 
      else {
        algae.intake(); }} 
    else if (controller.getLeftTriggerAxis() > 0.05) {
      if (CoralModeTrueAlgaeModeFalse) {
        coral.outtake(); } 
      else {
        algae.outtake(); }} 
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

    // Send control values to swerve drive
    swerveDrive.drive(forward, strafe, rotate, fieldRelative, rateLimit);
  }

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

  /* TEST INIT */
  @Override
  public void testInit() { }

  /* TEST PERIODIC */
  @Override
  public void testPeriodic() {}
}