// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ejml.dense.row.decomposition.svd.SafeSvd_DDRM;
import org.photonvision.PhotonCamera;

import com.ctre.phoenixpro.signals.AbsoluteSensorRangeValue;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Swerve.arm;
import frc.robot.autos.Auton;
import frc.robot.autos.SimpleAuto;
import frc.robot.autos.exampleAuto;
import frc.robot.commands.AutoArm;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.aimAtTarget;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Swerve;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Joystick driver = new Joystick(1);
  private final Joystick scoreMatrix = new Joystick(3);
  private final Joystick armController = new Joystick(3);
  //private final SingleJointedArmSim armM = new Single
  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;
  private final int shoulderAxis  =XboxController.Axis.kLeftTrigger.value;
  private final int jointAxis = XboxController.Axis.kRightTrigger.value;
  private final JoystickButton zeroGyro = 
    new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton robotCentric =
    new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton aim = 
  new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  private final JoystickButton light = 
    new JoystickButton(driver, XboxController.Button.kB.value);
  private final JoystickButton absolute = g
    new JoystickButton(driver, XboxController.Button.kA.value);
  private final JoystickButton halfSpeed = 
    new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final Swerve s_Swerve = new Swerve();
  public static final Arm Armm = new Arm();
  private final Claw claw = new Claw();
  private final PhotonCamera cam = new PhotonCamera("Microsoft_LifeCam_HD-3000 (1)"); //NAME CAMERA
  private final PowerDistribution examplePD = new PowerDistribution(1, ModuleType.kRev);
  
  private final aimAtTarget aimCommand = new aimAtTarget(cam, s_Swerve, s_Swerve::getPose);
  private final Auton autonChooser = new Auton(s_Swerve, Armm, claw);


  // Xbox controller
  
  private final JoystickButton High = 
    new JoystickButton(armController, XboxController.Button.kY.value);
  private final JoystickButton Mid = 
    new JoystickButton(armController, XboxController.Button.kB.value);
  private final JoystickButton Retracted = 
    new JoystickButton(armController, XboxController.Button.kX.value);
  // Intake pos
  private final JoystickButton Down = 
    new JoystickButton(armController, XboxController.Button.kStart.value);
  // Shelf pickup
  private final JoystickButton Shelf = 
    new JoystickButton(armController, XboxController.Button.kBack.value);
  private final JoystickButton Outtake = 
    new JoystickButton(armController, XboxController.Button.kRightBumper.value);
  private final JoystickButton Intake = 
    new JoystickButton(armController, XboxController.Button.kLeftBumper.value);
  //private final JoystickButton MoveToAprilTag = new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton reverse = 
    new JoystickButton(armController, XboxController.Button.kA.value);
   
  // Button board 
  /* 
   private final JoystickButton High = 
    new JoystickButton(scoreMatrix, XboxController.Button.kY.value);
  private final JoystickButton Mid = 
    new JoystickButton(scoreMatrix, XboxController.Button.kB.value);
  private final JoystickButton Retracted = 
    new JoystickButton(scoreMatrix, XboxController.Button.kX.value);
  // Intake pos
  private final JoystickButton Down = 
    new JoystickButton(scoreMatrix, XboxController.Button.kStart.value);
  // Shelf pickup
  private final JoystickButton Shelf = 
    new JoystickButton(scoreMatrix, XboxController.Button.kBack.value);
  private final JoystickButton Outtake = 
    new JoystickButton(scoreMatrix, XboxController.Button.kRightBumper.value);
  private final JoystickButton Intake = 
    new JoystickButton(scoreMatrix, XboxController.Button.kLeftBumper.value);
  //private final JoystickButton MoveToAprilTag = new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton reverse = 
    new JoystickButton(scoreMatrix, XboxController.Button.kA.value);
  */
  private final JoystickButton LowRight = new JoystickButton(scoreMatrix, XboxController.Button.kLeftBumper.value);
  private final JoystickButton LowMiddle = new JoystickButton(scoreMatrix, XboxController.Button.kX.value);
  private final JoystickButton LowLeft = new JoystickButton(scoreMatrix, XboxController.Button.kA.value);

  private final JoystickButton MidRight = new JoystickButton(scoreMatrix, XboxController.Button.kY.value); 
  private final JoystickButton MidMiddle = new JoystickButton(scoreMatrix, XboxController.Button.kB.value);
  private final JoystickButton MidLeft = new JoystickButton(scoreMatrix, XboxController.Button.kRightBumper.value);

  private final JoystickButton HighRight = new JoystickButton(scoreMatrix, XboxController.Button.kBack.value);
  private final JoystickButton HighMiddle = new JoystickButton(scoreMatrix, XboxController.Button.kStart.value);
  private final JoystickButton HighLeft = new JoystickButton(scoreMatrix, XboxController.Button.kLeftStick.value);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    s_Swerve.setDefaultCommand(
      new TeleopSwerve(
        s_Swerve,
        () -> -driver.getRawAxis(translationAxis),
        () -> -driver.getRawAxis(strafeAxis),
        () -> -driver.getRawAxis(rotationAxis),
        ()->false,//() -> robotCentric.getAsBoolean(),
        () -> halfSpeed.getAsBoolean()));

    Armm.setDefaultCommand(Armm.JoystickControl(()-> armController.getRawAxis(jointAxis), ()-> armController.getRawAxis(shoulderAxis), () -> reverse.getAsBoolean()));
    
    claw.setDefaultCommand(claw.Stop());
      /*new Intake(
        claw, 
        () -> Intake.getAsBoolean(),
        () -> Outtake.getAsBoolean()
      );*/
    // Configure the button bindings
    configureButtonBindings();
    
  }
  //robotCentric.get();
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    aim.whileTrue(aimCommand);
    //zeroGyro.whenPressed(new InstantCommand(() -> s_Swerve.zeroGyro()));
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    // Reset Arm
    High.onTrue(Armm.moveTo(arm.HIGHGOAL[0], arm.HIGHGOAL[1]));
    Mid.onTrue(Armm.moveTo(arm.MIDGOAL[0], arm.MIDGOAL[1]));
    Retracted.onTrue(Armm.moveTo(arm.RETRACTED[0], arm.RETRACTED[1]));
    Down.onTrue(Armm.moveTo(arm.INTAKE[0], arm.INTAKE[1]));
    Intake.whileTrue(claw.Intake());
    Shelf.onTrue(Armm.moveTo(arm.SHELF[0], arm.SHELF[1]));
    light.onTrue(new InstantCommand(()->{
      if (examplePD.getSwitchableChannel()){ 
        examplePD.setSwitchableChannel(false);
      } else{
        examplePD.setSwitchableChannel(true);
      }
    }));
    Outtake.whileTrue(claw.Outtake());
    //absolute.onTrue(new RunCommand(()->{s_Swerve.setAbsolute();}).withTimeout(1));
    //MoveToAprilTag.onTrue(s_Swerve.moveToTag(new Translation2d(1, 0)));
    // Claw:

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    //return new SequentialCommandGroup(
    //  Armm.moveTo(arm.INTAKE[0], arm.INTAKE[1]).withTimeout(4), claw.Outtake().withTimeout(2), Armm.moveTo(arm.RETRACTED[0], arm.RETRACTED[1]));
    // An ExampleCommand will run in autonomous
    //return new SimpleAuto(s_Swerve, claw, Armm);
    return autonChooser.getSelected();
    //return new exampleAuto(s_Swerve, Armm, claw);
  }
}
