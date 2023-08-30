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

public abstract class TimedOuttake extends CommandBase{
    /* 
    private double time;
    private Claw claw;
    public TimedOuttake(Claw claw, double time){
        this.time = time;
        this.claw = claw;
        
    }
    
    @Override
    public void execute() {
        claw.Outtake();
    }

    @Override
    protected boolean isFinished() {
        return isTimedOut();
    }


    @Override
    protected void interrupted() {
        throw new UnsupportedOperationException("Not supported yet.");
    }
    */
}