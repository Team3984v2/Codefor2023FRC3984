package frc.robot.subsystems;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Claw extends SubsystemBase{
    private Spark rightMotor;
    private Spark leftMotor;
    private boolean in;
    private boolean neutral;
    public Claw() {
        rightMotor = new Spark(8);
        leftMotor= new Spark(9);
        leftMotor.setInverted(true);
        rightMotor.setInverted(false);
        in = false;
        neutral = true;
    }

    public Command Intake(){ 
  
        in = true;
        neutral = false;
        return run(()->{      rightMotor.set(1);
            leftMotor.set(1);});
    }
    public Command Outtake(){ 

        in = false;
        neutral = false;
        return run(()->{      rightMotor.set(-0.5);
            leftMotor.set(-0.5);});
    }
    public Command Stop(){
        return run(()->{      rightMotor.set(0);
            leftMotor.set(0);});
    }
    public Command ManualClaw(BooleanSupplier in, BooleanSupplier out){
        if (in.getAsBoolean()){
            //System.out.println("INN!!!");
            return run(()->{ Intake(); System.out.println("A");});//{rightMotor.set(-1); leftMotor.set(1);});
        }
        else if (out.getAsBoolean()){
            return run(()-> {rightMotor.set(1); leftMotor.set(-1);});
        }
        /*else{
            return run(()->{rightMotor.stopMotor(); leftMotor.stopMotor();});
        }*/
        return run(()->{System.out.println("NOTHING!!!!!!!!");});
    }
    public void periodic(){
        if (neutral){
            SmartDashboard.putString("Claw State", "Neutral");
        }
        else{
            if (in == true){
                SmartDashboard.putString("Claw State", "In");
            }
            else if (in == false){
                SmartDashboard.putString("ClawState", "Out");
            }

        }
    }
    /*public void getNewPoint(){
        goal[0] = goal[0] + 0.05;
        goal[1] = goal[0] + 0.05;
    }*/
}
