package frc.robot.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Swerve;
import java.util.List;

public class exampleAuto extends SequentialCommandGroup {
  public exampleAuto(Swerve s_Swerve, Arm s_Arm, Claw s_Claw) {
    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics);
            
    config.setReversed(true);
    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(/*new Translation2d(1, 1), new Translation2d(2, -1)*/),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(-6, 0, new Rotation2d(0)),
            config);
            Trajectory exampleTrajectory2 =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(/*new Translation2d(1, 1), new Translation2d(2, -1)*/),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(-6, 0, new Rotation2d(0)),
                config);
    var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController,
            0,
            0,
            Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);
        SwerveControllerCommand swerveControllerCommand1 =
        new SwerveControllerCommand(
            exampleTrajectory2,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);

    addCommands(
        new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
        s_Arm.moveTo(Constants.Swerve.arm.MIDGOAL[0], Constants.Swerve.arm.MIDGOAL[1]).withTimeout(3), 
        s_Claw.Outtake().withTimeout(2),
        s_Claw.Stop() .withTimeout(1),
        s_Arm.moveTo(Constants.Swerve.arm.RETRACTED[0], Constants.Swerve.arm.RETRACTED[1]).withTimeout(2.5),
        swerveControllerCommand,
        new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
        swerveControllerCommand1);
  }
}
