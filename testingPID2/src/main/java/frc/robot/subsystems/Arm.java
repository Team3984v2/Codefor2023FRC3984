package frc.robot.subsystems;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.opencv.core.Mat;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve.arm;
import frc.robot.Constants.Swerve.arm.Shoulder;


public class Arm extends SubsystemBase{


    private CANSparkMax shoulderMotor, jointMotor; 
    private RelativeEncoder EncoderShoulder, EncoderJoint;
    private ArmFeedforward ShoulderFF, JointFF;
    private SparkMaxPIDController ShoulderPID, JointPID;


    public Arm() {
        // Initialize feedforwards with the Ks Kg and Kv values.
        /* 
        ShoulderFF = new ArmFeedforward(
            arm.Shoulder.Ks, 
            arm.Shoulder.Kg, 
            arm.Shoulder.Kv 
        );
        JointFF = new ArmFeedforward(
            arm.Joint.Ks,
            arm.Joint.Kg, 
            arm.Joint.Kv
        );*/


        // Initialize the Motors
        shoulderMotor = new CANSparkMax(
            arm.Shoulder.rotMotorID, 
            MotorType.kBrushless
        );
        shoulderMotor.restoreFactoryDefaults();
        shoulderMotor.setIdleMode(IdleMode.kBrake);

        jointMotor = new CANSparkMax(
            arm.Joint.rotMotorID, 
            MotorType.kBrushless
        );
        jointMotor.restoreFactoryDefaults();
        jointMotor.setIdleMode(IdleMode.kBrake);

        // Initialize the built in shoulder encoder
        EncoderShoulder = shoulderMotor.getEncoder(
            SparkMaxRelativeEncoder.Type.kHallSensor, 
            42
        );
        // Set Conversion factor for Encoder -> degrees
        EncoderShoulder.setPositionConversionFactor(
            360.0/arm.Shoulder.gearRatio
        );
        // Set Velocity Conversion factor -> degrees/second
        EncoderShoulder.setVelocityConversionFactor(
            (360 / arm.Shoulder.gearRatio) / 60.0 
        );
        // Set the position to zero
        EncoderShoulder.setPosition(0);

        // Initialize the built in joint encoder
        EncoderJoint = jointMotor.getEncoder(
            SparkMaxRelativeEncoder.Type.kHallSensor, 
            42
        );
        // Set Conversion factor for Encoder -> degrees
        EncoderJoint.setPositionConversionFactor(
            360/arm.Joint.gearRatio
        );
        // Set Valocity Conversion factor -> degrees/second
        EncoderShoulder.setVelocityConversionFactor(
            (360 / arm.Shoulder.gearRatio) / 60.0
        );
        // Set position to zero
        EncoderJoint.setPosition(0);


        // Initialize the built in Motor PID controllers
        ShoulderPID = shoulderMotor.getPIDController();
        ShoulderPID.setP(arm.Shoulder.Kp);
        ShoulderPID.setI(arm.Shoulder.Ki);
        ShoulderPID.setD(arm.Shoulder.Kd);

        JointPID = jointMotor.getPIDController();
        JointPID.setP(arm.Joint.Kp);
        JointPID.setI(arm.Joint.Ki);
        JointPID.setD(arm.Joint.Kd);

        // burn flash
        shoulderMotor.burnFlash();
        jointMotor.burnFlash();
        
    }

    // Calculates the angle needed to get to a given point using inverse Kniematics
    public Rotation2d[] getAngles(double x, double y){
        double AngleShoulder;
        double AngleJoint;

        Rotation2d[] angles = new Rotation2d[2];
		double hypotenuse = Math.hypot(x, y);
		AngleJoint = Math.pow(hypotenuse,  2) - Math.pow(arm.Shoulder.Length, 2) - Math.pow(arm.Joint.Length, 2);
		AngleJoint = AngleJoint/(-2*arm.Shoulder.Length*arm.Joint.Length);
		AngleJoint = Math.acos(AngleJoint);
        double theta2 = Math.pow(arm.Joint.Length,2)-Math.pow(arm.Shoulder.Length,2)-Math.pow(hypotenuse,2);
        theta2 = theta2/(-2*arm.Shoulder.Length*hypotenuse);
        double j = Math.atan(y/x);
        double k = theta2-j;
        AngleShoulder = ((Math.PI/2)-k);
        // Put the angle inside a Rotation2d array. 
        angles[0] = new Rotation2d(AngleShoulder * 6.4/4); // TODO Multiply by the gear ratio.

        angles[1] = new Rotation2d(AngleJoint * 3);
        return angles;
    }

    public Rotation2d[] getPos(){
        Rotation2d posShoulder = Rotation2d.fromDegrees(EncoderShoulder.getPosition());
        Rotation2d posJoint = Rotation2d.fromDegrees(EncoderJoint.getPosition()); // returns a value 1/4 of actual angle for some reason
        Rotation2d[] angles = new Rotation2d[]{posShoulder, posJoint};
        return angles;
    }
    public Rotation2d[] getErrors(Rotation2d[] goal){
        Rotation2d[] currPos = new Rotation2d[] {getPos()[0], getPos()[1]};
        Rotation2d[] error = new Rotation2d[] {
            Rotation2d.fromDegrees(currPos[0].getDegrees() - goal[0].getDegrees()), 
            Rotation2d.fromDegrees(currPos[1].getDegrees() - 90/*goal[1].getDegrees()*/)
        };
        return error;
    }
    public void GoTo(Rotation2d ShoulderGoal, Rotation2d JointGoal){
        ShoulderPID.setReference(
            ShoulderGoal.getDegrees(), 
            ControlType.kPosition, 0
            /*ShoulderFF.calculate(ShoulderGoal.getRadians() 
            /* subtract angle offset from horizontal position later *,
             0)*/
        );
        JointPID.setReference(
            JointGoal.getDegrees(), 
            ControlType.kPosition, 0
            /*ShoulderFF.calculate(ShoulderGoal.getRadians() 
            /* subtract angle offset from horizontal position later *,
             0)*/
        );
    }
    public void ManualControl(double Jointval, double Shoulderval, boolean reverse){
        if (Jointval < 0.2){
            Jointval = 0;
        }
        if (reverse){
            Jointval = Jointval * -1;
            Shoulderval = Shoulderval * -1;
        }
        
        jointMotor.set(Jointval * 0.5);
        shoulderMotor.set(Shoulderval * 0.5);

    }
    public Command JoystickControl(DoubleSupplier Joint, DoubleSupplier Shoulder, BooleanSupplier reverse){
        return run(()->ManualControl(Joint.getAsDouble(), Shoulder.getAsDouble(), reverse.getAsBoolean()));
    }

    public void reset(){
        EncoderShoulder.setPosition(0);
        EncoderJoint.setPosition(0);
    }
    
    public Command moveTo(double Sangle, double Jangle){
        Rotation2d[] a = new Rotation2d[]{Rotation2d.fromDegrees(Sangle)/*(49.26)/*getAngles(x, y)[0]*/, Rotation2d.fromDegrees(Jangle)};/*(61.97)};// getAngles(x, y)[1]};*/
        return runOnce(()->{
            SmartDashboard.putNumber("Shoulder Goal", a[0].getDegrees());
            SmartDashboard.putNumber("Joint Goal", a[1].getDegrees());
        }).andThen(run(
            () -> GoTo(
                a[0], a[1]
            )
        ).until(
            ()->(
                Math.abs(getErrors(a)[0].getDegrees()) < 1 
                && Math.abs(getErrors(a)[1].getDegrees()) < 1
            ))
        );
    }
    public void periodic(){
        SmartDashboard.putNumber("ShoulderPos", getPos()[0].getDegrees());
        SmartDashboard.putNumber("JointPos", getPos()[1].getDegrees());
    }

}
