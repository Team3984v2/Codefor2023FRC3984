package frc.robot.subsystems;
import java.io.IOException;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
//import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.wpilibj.SPI;
//import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.Mod0;
import frc.robot.Constants.Swerve.Mod1;
import frc.robot.Constants.Swerve.Mod2;
import frc.robot.Constants.Swerve.Mod3;
import frc.robot.autos.Auton;
public class Swerve extends SubsystemBase {
  //private final Pigeon2 gyro1;
  ADXRS450_Gyro gyro;
  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] mSwerveMods;
  private PhotonCamera cam;
  private Field2d field;
  private AprilTagFieldLayout layout;
  private double previousTimeStamp;

  public Swerve() {
    //gyro1 = new Pigeon2(Constants.Swerve.pigeonID);
    gyro = new ADXRS450_Gyro();
    gyro.calibrate();
    //gyro1.configFactoryDefault();
    //SwerveModulePosition
    zeroGyro();
    mSwerveMods =
    new SwerveModule[] {
      new SwerveModule(0, Constants.Swerve.Mod0.constants),
      new SwerveModule(1, Constants.Swerve.Mod1.constants),
      new SwerveModule(2, Constants.Swerve.Mod2.constants),
      new SwerveModule(3, Constants.Swerve.Mod3.constants)
    };
    swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getPositions());
    //swerveOdometryVision = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getYaw(), getPositions(), new Pose2d());
    //swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), new SwerveModulePosition[]{}, getPose());
    //swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw());

    cam = new PhotonCamera("Microsoft_LifeCam_HD-3000 (1)"); //NAME CAMERA
    
    try {
      layout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
    } catch(IOException e) {
      System.out.println("April Tag Field Layout not Found");
    }
    field = new Field2d();
    SmartDashboard.putData("Field", field);
  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getYaw())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }
  public void setModuleStates(ChassisSpeeds chassisSpeeds){
    SwerveModuleState[] desiredStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);//TODO NEED TO WORK ONnnjnijni
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }
  public void XLock(){
    SwerveModuleState[] desiredStates = new SwerveModuleState[4];
    desiredStates[0] = new SwerveModuleState(0, new Rotation2d(Mod0.angleOffset.getDegrees() + 45));
    desiredStates[1] = new SwerveModuleState(0, new Rotation2d(Mod1.angleOffset.getDegrees() - 45));
    desiredStates[2] = new SwerveModuleState(0, new Rotation2d(Mod2.angleOffset.getDegrees() - 45));
    desiredStates[3] = new SwerveModuleState(0, new Rotation2d(Mod3.angleOffset.getDegrees() + 45));
    setModuleStates(desiredStates);
  }
  public void stop(){
    setModuleStates(Constants.Swerve.swerveKinematics.toSwerveModuleStates(new ChassisSpeeds()));
  }
  public void setAbsolute() {
    //SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }
  // Aims torwards a April Tag
  /* 
  public void turnToTarget(){
    var lastestTarget = cam.getLatestResult();
    if (lastestTarget.hasTargets()){
      Pose3d tagPos = layout.getTagPose(lastestTarget.getBestTarget().getFiducialId()).get();
      Rotation2d target = PhotonUtils.getYawToPose(getPose(), tagPos.toPose2d());
      
      //return runOnce()
    }
    else{
      System.out.println("NO TARGETS FOUND");
    }
    PIDController thetaController = new PIDController(Constants.AutoConstants.kPThetaController, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    Rotation2d targetYaw = PhotonUtils.getYawToPose(getPose(), getPose());
    Translation2d toTarget = new Translation2d(0, targetYaw);
    SwerveModuleState[] states = Constants.Swerve.swerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(0,0, thetaController.calculate(previousTimeStamp), getYaw()));
    //Trajectory g = new TrajectoryGenerator(){}
  }*/
   

  public Pose2d getPose() {
    //return swerveOdometryVision.getEstimatedPosition();
    
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
    //swerveOdometryVision.resetPosition(getYaw(), getPositions(), pose);
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods){ 
      positions[mod.moduleNumber] = mod.getPoset();
    }
    return positions;
  }

  public void zeroGyro() {
    //FIXME ?
    gyro.reset();
    //gyro.setYaw(0);
  }

  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees( gyro.getAngle() *( Constants.Swerve.invertGyro? 1: -1));
    /*return (Constants.Swerve.invertGyro)//FIXME ??
    
        ? Rotation2d.fromDegrees(360 - gyro.getAngle()/*gyro.getYaw()*)
        : Rotation2d.fromDegrees(gyro.getAngle()/*getYaw()*);*/
  }

  /* X out, Y to the left */
  /*public Command moveToPose(Supplier<Pose2d> poseSupplier, Translation2d Offset){
    PIDController yPID = new PIDController(0, 0, 0);
    PIDController xPID = new PIDController(0, 0, 0);
    PIDController thetaPID = new PIDController(0, 0, 0);
    thetaPID.enableContinuousInput(0, 2*Math.PI);
    yPID.setTolerance(1);
    xPID.setTolerance(1);
    thetaPID.setTolerance(1);
    return runOnce(()->{
      Translation2d offset = Offset;
      Pose2d pose = poseSupplier.get();
      Rotation2d targetRot = pose.getRotation();
      offset = Offset.rotateBy(targetRot);
      Translation2d targetTranslation = pose.getTranslation();
      Translation2d offseted = targetTranslation.plus(Offset);
      field.getObject("CurrTarget").setPose(new Pose2d(offseted, targetRot));

      xPID.setSetpoint(offseted.getX());
      yPID.setSetpoint(offseted.getY());

      thetaPID.setSetpoint(targetRot.minus(Rotation2d.fromDegrees(180)).getRadians());

    }).andThen(run(
      () -> {
        drive(new Translation2d(
        xPID.calculate(swerveOdometryVision.getEstimatedPosition().getX()),
        yPID.calculate(swerveOdometryVision.getEstimatedPosition().getY())),
        thetaPID.calculate(swerveOdometryVision.getEstimatedPosition().getRotation().getRadians()),
        true, 
        false
        );
      }
    )).until(()-> xPID.atSetpoint() && yPID.atSetpoint() && thetaPID.atSetpoint()
    ).andThen(()->{
      xPID.close();
      yPID.close();
      thetaPID.close();
    });
    
  }*/
  /* 
  public int getClosestTag(){
    var lastest = cam.getLatestResult();
    if (lastest.hasTargets()){
      return lastest.getBestTarget().getFiducialId();
    }
    else{
      return -1;
    }
  }
  public Command moveToTag(int id, Translation2d offset){
    return moveToPose(() -> layout.getTagPose(id).get().toPose2d(), offset);
  }
  public Command moveToTag(Translation2d offset){
    var lastest = cam.getLatestResult();
    if (lastest.hasTargets()){
      return moveToPose(() -> layout.getTagPose(lastest.getBestTarget().getFiducialId()).get().toPose2d(), offset);
    }
    else{
      return null;
    }
  }*/

  /* 
  public Command alignWNearestRight(int id){
    var lastest = cam.getLatestResult();
    if (lastest.hasTargets()){
      Pose3d tagPos = layout.getTagPose(lastest.getBestTarget().getFiducialId()).get();
      //Pose3d TransPos = new Pose3d(tagPos.getX() + 0.5, tagPos.getY(), tagPos.getZ() + 1, tagPos.getRotation());
      return moveToPose(()->tagPos.toPose2d(), Constants.AutoConstants.GOALRIGHT); //TODO
    }

    return (new SequentialCommandGroup());
  }
  public Command alignWNearestLeft(int id){
    var lastest = cam.getLatestResult();
    if (lastest.hasTargets()){
      Pose3d tagPos = layout.getTagPose(lastest.getBestTarget().getFiducialId()).get();
      //Pose3d TransPos = new Pose3d(tagPos.getX() + 0.5, tagPos.getY(), tagPos.getZ() + 1, tagPos.getRotation());
      return moveToPose(()->tagPos.toPose2d(), Constants.AutoConstants.GOALLEFT); //TODO
    }

    return new SequentialCommandGroup();
  }
  public Command alignWNearestMiddle(int id){
    var lastest = cam.getLatestResult();
    if (lastest.hasTargets()){
      Pose3d tagPos = layout.getTagPose(lastest.getBestTarget().getFiducialId()).get();
      //Pose3d TransPos = new Pose3d(tagPos.getX() + 0.5, tagPos.getY(), tagPos.getZ() + 1, tagPos.getRotation());
      return moveToPose(()->tagPos.toPose2d(), Constants.AutoConstants.GOALMIDDLE); //TODO
    }

    return new SequentialCommandGroup();
  }*/



  @Override
  public void periodic() {
    /* 
    var lastest = cam.getLatestResult();
    var time = lastest.getTimestampSeconds();
    if (time != previousTimeStamp && lastest.hasTargets()){
      previousTimeStamp = time;
      var target = lastest.getBestTarget();
      var targetID = target.getFiducialId();
      Optional<Pose3d> pos = layout == null ? Optional.empty(): layout.getTagPose(targetID);
      if (pos.isPresent() && targetID >= 0 && target.getPoseAmbiguity() <= 0.2){
        var targetPos = pos.get();
        Transform3d camToPos = target.getBestCameraToTarget();
        Pose3d camPos = targetPos.transformBy(camToPos);
        Pose3d visionMeasurement = camPos.transformBy(/*TODOcamToPos);

     //   swerveOdometryVision.addVisionMeasurement(visionMeasurement.toPose2d(), time);
     // }
      
    //}*/

    swerveOdometry.update(getYaw(), getPositions());
    field.setRobotPose(getPose());

    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
      SmartDashboard.putNumber("Mod" + mod.moduleNumber + " DrivePos", mod.getPosets());
        }
  }
}
