package frc.robot.autos;

import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.*;


public final class Auton {
    private final Swerve s_Swerve;
    private final Arm s_Arm;
    private final Claw s_Claw;

    private final HashMap<String, Command> eventMap;
    SendableChooser<Command> autonChooser;
    private final SwerveAutoBuilder autonBuilder;
    public Auton(Swerve s_Swerve, Arm s_Arm, Claw s_Claw){
        this.s_Swerve = s_Swerve;
        this.s_Arm = s_Arm;
        this.s_Claw = s_Claw;
        eventMap = new HashMap<>();
        setMarkers();
        autonBuilder = new SwerveAutoBuilder(
            s_Swerve::getPose, 
            s_Swerve::resetOdometry, 
            new PIDConstants(Constants.AutoConstants.kPXController, 0, 0), 
            new PIDConstants(Constants.AutoConstants.kPThetaController, 0, 0), 
            s_Swerve::setModuleStates, 
            eventMap, 
            true,
            s_Swerve);
        autonChooser = new SendableChooser<Command>();
        autonChooser.setDefaultOption("Score 1", scoreOne());
        autonChooser.addOption("Score 1, Mobility", score1Mobility());
        autonChooser.addOption("Score 1, Mid Balance", score1Balance());
        autonChooser.addOption("Score 2, No-Cable Mobility", score2NoCable());
        autonChooser.addOption("Score 2, No-Cable Balance", score2NoCableBalance());
        autonChooser.addOption("Score 1, Grab 1, No-Cable Balnance", score1Grab1NoCableBalance());
        autonChooser.addOption("Test Backwards", back());
        SmartDashboard.putData(autonChooser);
    }
    private void setMarkers(){
        eventMap.put("Wait", new WaitCommand(1));
        eventMap.put("Score Mid", s_Arm.moveTo(Constants.Swerve.arm.MIDGOAL[0], Constants.Swerve.arm.MIDGOAL[1]).withTimeout(2)
                                            .andThen(s_Claw.Outtake().withTimeout(3))
                                            .andThen(s_Claw.Stop().withTimeout(0.1))
                                            /* .andThen(s_Arm.moveTo(Constants.Swerve.arm.RETRACTED[0], Constants.Swerve.arm.RETRACTED[1]).withTimeout(2)*/);
        eventMap.put("Retract",s_Arm.moveTo(Constants.Swerve.arm.RETRACTED[0], Constants.Swerve.arm.RETRACTED[1]).withTimeout(2));
        eventMap.put("Score High", s_Arm.moveTo(Constants.Swerve.arm.HIGHGOAL[0], Constants.Swerve.arm.HIGHGOAL[1]).withTimeout(2)
                                            .andThen(s_Claw.Outtake().withTimeout(3))
                                            .andThen(s_Claw.Stop().withTimeout(0.1))
                                            /* .andThen(s_Arm.moveTo(Constants.Swerve.arm.RETRACTED[0], Constants.Swerve.arm.RETRACTED[1]).withTimeout(2)*/);
       /*  eventMap.put("Score High", s_Arm.moveTo(Constants.Swerve.arm.HIGHGOAL[0], Constants.Swerve.arm.HIGHGOAL[1]).withTimeout(2)
                                            .andThen(s_Claw.Outtake().withTimeout(1))
                                            .andThen(s_Claw.Stop().withTimeout(0.1))
                                            .andThen(s_Arm.moveTo(Constants.Swerve.arm.RETRACTED[0], Constants.Swerve.arm.RETRACTED[1])));*/
        eventMap.put("Intake Position", s_Arm.moveTo(Constants.Swerve.arm.INTAKE[0], Constants.Swerve.arm.INTAKE[1]).withTimeout(2));
        eventMap.put("Intake Activate", s_Claw.Intake().withTimeout(3));
        eventMap.put("Retract", s_Arm.moveTo(Constants.Swerve.arm.RETRACTED[0], Constants.Swerve.arm.RETRACTED[1]));
        eventMap.put("Eject", s_Claw.Outtake().withTimeout(1));
    }
    public Command getSelected(){
        return autonChooser.getSelected();
    }
    public Command scoreOne(){
        return s_Arm.moveTo(Constants.Swerve.arm.INTAKE[0], Constants.Swerve.arm.INTAKE[1]).withTimeout(2).andThen(s_Claw.Outtake().withTimeout(2)).andThen(s_Claw.Stop().withTimeout(0.1)).andThen(s_Arm.moveTo(Constants.Swerve.arm.RETRACTED[0], Constants.Swerve.arm.RETRACTED[1]).withTimeout(2));
    }
    public Command score1Mobility(){
        return autonBuilder.fullAuto(PathPlanner.loadPath("Score1Mobility", Constants.AutoConstants.constraints));
    }
    public Command score1Balance(){
        return autonBuilder.fullAuto(PathPlanner.loadPath("Score1Balance", Constants.AutoConstants.constraints));
    }
    public Command score2NoCable(){
        return autonBuilder.fullAuto(PathPlanner.loadPath("Score2NoCableMobility", Constants.AutoConstants.constraints));
    }
    public Command score2NoCableBalance(){
        return autonBuilder.fullAuto(PathPlanner.loadPath("Score2NoCableBalance", Constants.AutoConstants.constraints));
    }
    public Command score1Grab1NoCableBalance(){
        return autonBuilder.fullAuto(PathPlanner.loadPath("Score1Grab1Balance", Constants.AutoConstants.constraints));
    }
    public Command back(){
        return autonBuilder.fullAuto(PathPlanner.loadPath("TestTranslationalPath", Constants.AutoConstants.constraints));
    }
    
}
