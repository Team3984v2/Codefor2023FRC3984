package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class AutoArm extends CommandBase {
    private final Arm armm;
    private final Rotation2d[] angles;
    public AutoArm(Arm armm, double angleShoulder, double angleJoint){
        this.armm = armm;
        angles = new Rotation2d[2];
        angles[0] = Rotation2d.fromDegrees(angleShoulder);
        angles[1] = Rotation2d.fromDegrees(angleJoint);
    }
    public void initialize(){

    }
    public void execute(){

        armm.GoTo(angles[0], angles[1]);

    }
    public boolean isFinished(){
        return Math.abs(armm.getErrors(angles)[0].getDegrees()) < 1 && Math.abs(armm.getErrors(angles)[1].getDegrees()) < 1;
    }
}
