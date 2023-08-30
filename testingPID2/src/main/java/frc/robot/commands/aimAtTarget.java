package frc.robot.commands;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.camera;
import frc.robot.subsystems.*;

public class aimAtTarget extends CommandBase{
    private final PhotonCamera photonCamera;
    private final Swerve s_Swerve;
    private final Supplier<Pose2d> poseProvider;
    private final PIDController omegaController = new PIDController(Constants.AutoConstants.kPThetaController, 0, 0);
    private PhotonTrackedTarget lastTarget;
    public aimAtTarget(
        PhotonCamera photonCamera, 
        Swerve s_Swerve,
        Supplier<Pose2d> poseProvider) {
        this.photonCamera = photonCamera;
        this.s_Swerve= s_Swerve;
        this.poseProvider = poseProvider;

        omegaController.setTolerance(Units.degreesToRadians(1));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(s_Swerve);
    }
    
    @Override
    public void initialize() {
        var roboPose = poseProvider.get();

    }
  
    @Override
    public void execute() {
        var robotPose2d = poseProvider.get();
        var robotPose = 
            new Pose3d(
                robotPose2d.getX(),
                robotPose2d.getY(),
                0.0, 
                new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians() ));
        var lastest = photonCamera.getLatestResult();
        if (lastest.hasTargets()){
            var target = lastest.getBestTarget();
            lastTarget = target;
            var camPose = robotPose.transformBy(Constants.Swerve.camera.ROBOT_TO_CAMERA);
            var camToTarget= target.getBestCameraToTarget();
            var targetPos = camPose.transformBy(camToTarget);
            var goalPose = targetPos.toPose2d();
            System.out.println(robotPose2d.getRotation());
            System.out.println(goalPose.getRotation());
            omegaController.setSetpoint(goalPose.getRotation().getRadians());
        }
        else{
            lastTarget = null;
        }
        if (lastTarget == null){
            s_Swerve.stop();
        }else{
            var omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());
            if (omegaController.atSetpoint()){
                omegaSpeed = 0;
            }
            s_Swerve.setModuleStates(
                Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        0,
                        0, 
                        omegaSpeed, 
                        robotPose2d.getRotation()
                    )
                )
            );
        }
    }
  
    @Override
    public void end(boolean interrupted) {
        s_Swerve.stop();
    }
  
  }
