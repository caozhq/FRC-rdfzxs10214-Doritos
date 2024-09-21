// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// import sth
package com.team5449.frc2024;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.sql.Driver;
import java.util.function.BooleanSupplier;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.conduit.schema.Joystick;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.team5449.frc2024.Constants.Ports;
// import com.team5449.frc2024.autos.autocommands.WaitCommand;
import com.team5449.frc2024.autos.autocommands.AutoMidCommand;
import com.team5449.frc2024.autos.autocommands.AutoQ1Q3;
import com.team5449.frc2024.commands.AmpCommand;
import com.team5449.frc2024.commands.ArmPoseCommand;
import com.team5449.frc2024.commands.AutoAlign;
import com.team5449.frc2024.commands.ClimbCommand;
import com.team5449.frc2024.commands.DefaultDriveCommand;
import com.team5449.frc2024.commands.IntakeCommand;
import com.team5449.frc2024.commands.OrientToTargetCommand;
import com.team5449.frc2024.commands.OuttakeCommand;
import com.team5449.frc2024.commands.RotateCommand;
import com.team5449.frc2024.commands.ShootCommand;
import com.team5449.frc2024.commands.ShootWithTrajectory;
import com.team5449.frc2024.commands.ArmPoseCommand.ArmSystemState;
import com.team5449.frc2024.subsystems.CalcRotationWithUnitCircleData;
import com.team5449.frc2024.subsystems.drive.DrivetrainSubsystem;
import com.team5449.frc2024.subsystems.drive.GyroIOPigeon;
import com.team5449.frc2024.subsystems.drive.SwerveModuleIOFalconPro;
import com.team5449.frc2024.subsystems.score.Arm;
import com.team5449.frc2024.subsystems.score.Climber;
import com.team5449.frc2024.subsystems.score.Intake;
import com.team5449.frc2024.subsystems.score.Shooter;
import com.team5449.frc2024.subsystems.vision.Led;
import com.team5449.frc2024.subsystems.vision.LimeLight;
import com.team5449.frc2024.subsystems.vision.Led.Color;
import com.team5449.frc2024.subsystems.vision.VisionIO;
import com.team5449.frc2024.subsystems.vision.VisionIOLimelight;
import com.team5449.frc2024.subsystems.vision.VisionSubsystem;
import com.team5449.lib.util.ControllerUtil;
import com.team5449.lib.util.GeomUtil;
import com.team5449.lib.util.TimeDelayedBoolean;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {

  private final DrivetrainSubsystem drivetrainSubsystem;
  private final VisionSubsystem vision;
  private final Shooter shooter;
  private final Intake intake;
  private final Arm arm;
  private final Climber climber;
  private final LimeLight mLimelight;

  private final ArmPoseCommand armPoseCommand;

  private final SlewRateLimiter xLimiter = new SlewRateLimiter(2);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(2);
  private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(3);

  private TimeDelayedBoolean resetGyroBoolean = new TimeDelayedBoolean();
  public final PS5Controller mDriverController = new PS5Controller(0);
  // public final ControllerUtil mDriverControllerU = new ControllerUtil(mDriverController);
  public final XboxController mOperatorController = new XboxController(1);
  // public final ControllerUtil mOperatorControllerU = new ControllerUtil(mOperatorController);
  
  //private final edu.wpi.first.wpilibj.Joystick mDriverJoystick = new edu.wpi.first.wpilibj.Joystick(0);
  //public final CommandXboxController mOperatorController = new CommandXboxController(1);

  public static final DigitalInput noteStored = new DigitalInput(0);

  private final SendableChooser<Command> mAutoChooser;

  private final OrientToTargetCommand mOrientToTargetCommand;
  private final AutoAlign mAutoAlignCommand;

  private final CalcRotationWithUnitCircleData mCircleData;

  private final RotateCommand mRotateCommand;

  private final GyroIOPigeon mPigeon;

  public BooleanSupplier conditionShoot = ControllerUtil.toCond(Constants.ControlConds.shoot);
  public BooleanSupplier conditionIntake = ControllerUtil.toCond(Constants.ControlConds.intake);
  //public Trigger RLst = new Trigger(() -> (mOperatorController.getRightBumper() ^ mOperatorController.getLeftBumper())).debounce(0.1);
  public BooleanSupplier conditionReload = ControllerUtil.toCond(Constants.ControlConds.reload);//() -> mOperatorController.getXButton() && !RLst.getAsBoolean() && !mOperatorController.getRightBumper();
  public BooleanSupplier conditionGoAMP = ControllerUtil.toCond(Constants.ControlConds.amp);
  public BooleanSupplier conditionOverShoot = () -> {return mDriverController.getPOV() == 270;};//ControllerUtil.toCond(Constants.ControlConds.overshoot);//new Trigger(RLst).negate().and(mOperatorController::getRightBumper).and(mOperatorController::getXButton);

  public RobotContainer() {

    
    vision = new VisionSubsystem(new VisionIO[]{
      new VisionIOLimelight("limelight", new Transform3d())
    });

    mPigeon = new GyroIOPigeon();
    drivetrainSubsystem = new DrivetrainSubsystem(
      mPigeon,
      new SwerveModuleIOFalconPro(Ports.kFrontLeftMotorId, Ports.kFrontLeftAziId, Ports.kFrontLeftEncoderId, Ports.kCANBusFDName, Constants.kFrontLeftEncoderOffset, false),
      new SwerveModuleIOFalconPro(Ports.kFrontRightMotorId, Ports.kFrontRightAziId, Ports.kFrontRightEncoderId, Ports.kCANBusFDName, Constants.kFrontRightEncoderOffset, false),
      new SwerveModuleIOFalconPro(Ports.kBackLeftMotorId, Ports.kBackLeftAziId, Ports.kBackLeftEncoderId, Ports.kCANBusFDName, Constants.kBackLeftEncoderOffset, true),
      new SwerveModuleIOFalconPro(Ports.kBackRightMotorId, Ports.kBackRightAziId, Ports.kBackRightEncoderId, Ports.kCANBusFDName, Constants.kBackRightEncoderOffset, true),
      vision
    );

    arm = new Arm();
    climber=new Climber();
    armPoseCommand = new ArmPoseCommand(arm, drivetrainSubsystem);
    arm.setDefaultCommand(armPoseCommand);


    mCircleData = new CalcRotationWithUnitCircleData(mOperatorController::getRightX, ()->-mOperatorController.getRightY(), drivetrainSubsystem.getHeading().getRadians(), 0.5);
    mRotateCommand = new RotateCommand(drivetrainSubsystem, mCircleData::calculate,0.25);

    drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
      drivetrainSubsystem,
      () -> -adjustJoystickValue(xLimiter.calculate(mDriverController.getLeftY())) * drivetrainSubsystem.getMaxVelocityMetersPerSec() * (Robot.isRedAlliance()?-1:1),
      () -> -adjustJoystickValue(yLimiter.calculate(mDriverController.getLeftX())) * drivetrainSubsystem.getMaxVelocityMetersPerSec() * (Robot.isRedAlliance()?-1:1),
      () -> -adjustJoystickValue(omegaLimiter.calculate(mDriverController.getRightX())) * drivetrainSubsystem.getMaxAngularVelocityRadPerSec()/* + mRotateCommand.calcRotVel()*/,
      () -> mDriverController.getOptionsButtonPressed() && ControllerUtil.toCond(Constants.ControlConds.secondContext).getAsBoolean()==false,
      () -> {boolean reset = resetGyroBoolean.update(mDriverController.getCrossButton(), 0.2);if(reset){mCircleData.reset();}return reset;}));
      drivetrainSubsystem.setPathAuto();

    new Trigger(ControllerUtil.toCond(Constants.ControlConds.ToggleSlowMode)).onTrue(new InstantCommand(() -> drivetrainSubsystem.isSlowMode=!drivetrainSubsystem.isSlowMode));

    shooter = Shooter.getInstance();
    intake = Intake.getInstance();

    mOrientToTargetCommand = new OrientToTargetCommand(drivetrainSubsystem, vision);
    mAutoAlignCommand = new AutoAlign(drivetrainSubsystem, vision);

    mLimelight = new LimeLight(new Pose3d(new Translation3d(0.37425, -0.508, /*-0.079*/-0.22),new Rotation3d(Math.toRadians(15), 0, 0)));

    pathPlannerRegisterCommand();

    mAutoChooser = new SendableChooser<>();//AutoBuilder.buildAutoChooser();

    mAutoChooser.addOption("AMid", new AutoMidCommand(intake, shooter, armPoseCommand, drivetrainSubsystem));
    mAutoChooser.addOption("AQ1Q3", new AutoQ1Q3(intake, shooter, armPoseCommand, drivetrainSubsystem));

    SmartDashboard.putData("Builder Auto Chooser", mAutoChooser);

    mAutoChooser.onChange((mCommand) -> System.out.println(mCommand.getName()));

    configureBindings();
  }

  private static double adjustJoystickValue(double value) {
      value = MathUtil.applyDeadband(value, 0.05);
      value = Math.copySign(value * value, value);
      return value;
  }

  private void configureBindings() {
    
    //BooleanSupplier conditionHasTarget = ()->mColorSensor.getTarget()==new Constants.checkTarget[]{Constants.checkTarget.HASTARGET};

    new Trigger(conditionShoot).onTrue(new InstantCommand(() -> armPoseCommand.setPose(ArmSystemState.SHOOTING))).whileTrue(new ShootCommand(shooter, armPoseCommand, () -> armPoseCommand.getArmState() == ArmSystemState.SHOOTING, 50));//.whileTrue(mAutoAlignCommand);//.o
    new Trigger(ControllerUtil.toCond(Constants.ControlConds.forceShoot)).whileTrue(new InstantCommand(() -> shooter.transit(1)));//.whileTrue(mAutoAlignCommand);//.o
    // new Trigger(conditionShoot).whileTrue(new ShootWithTrajectory(shooter, armPoseCommand, () -> {
    //   Pose3d mPose3d = mLimelight.getPose3DBot();/*new Translation2d(1.43+5, -0.3)*/
    //   return new Translation2d(mPose3d.getZ()+1.43, /*-mPose3d.getY()-0.3*/2.6);
    // }));

    // BooleanSupplier trgRestore=() -> {return conditionIntake.getAsBoolean() && armPoseCommand.getArmState() != ArmSystemState.INTAKE;};
    // // Trigger trgRestoreTimeout=new Trigger(trgRestore).debounce(0.2);
    // BooleanSupplier bRestore = () -> {
    //   boolean a=trgRestore.getAsBoolean();
    //   // boolean b=trgRestoreTimeout.getAsBoolean();
    //   boolean v = a/* && !b*/;
    //   SmartDashboard.putBoolean("trgRestore", a);
    //   // SmartDashboard.putBoolean("trgRestoreTimeout", b);
    //   SmartDashboard.putBoolean("bRestore", v);
    //   return v;
    // };
    new Trigger(/*bRestore*/conditionIntake).and(() -> armPoseCommand.getArmState() != ArmSystemState.INTAKE).onTrue(new InstantCommand(() -> armPoseCommand.setPose(ArmSystemState.INTAKE)));
    new Trigger(conditionIntake).and(() -> armPoseCommand.getArmState() == ArmSystemState.INTAKE).toggleOnTrue(new IntakeCommand(shooter, intake, armPoseCommand, false));
    // new Trigger(conditionIntake).whileTrue(new IntakeCommand(shooter, intake, armPoseCommand, false));

    new Trigger(ControllerUtil.toCond(Constants.ControlConds.forceIntake)).whileTrue(new IntakeCommand(shooter, intake, armPoseCommand, true));

    new Trigger(conditionReload).onTrue(new InstantCommand(() -> armPoseCommand.setPose(ArmSystemState.OUTTAKE))).whileTrue(new OuttakeCommand(shooter, intake));

    new Trigger(conditionOverShoot).whileTrue(new ShootWithTrajectory(shooter, armPoseCommand, () -> {
      final Pose2d robotPose = drivetrainSubsystem.getPose();
      final boolean onBlueAlliance = robotPose.getX()<5.838526;
      final boolean onRedAlliance = robotPose.getX()>10.685367;
      SmartDashboard.putBoolean("Drive/onBlueAlliance", onBlueAlliance);
      SmartDashboard.putBoolean("Drive/onRedAlliance", onRedAlliance);
      if((onBlueAlliance && Robot.isRedAlliance()) || (onRedAlliance && !Robot.isRedAlliance()))
      {
        SmartDashboard.putBoolean("Drive/onOppositeAlliance", onRedAlliance);
        return new Translation2d(6, 0);
      }
      return new Translation2d(robotPose.getTranslation().getDistance(GeomUtil.GetStageTranslation().toTranslation2d()) - 0.6 -  1 - 1, 1.8);
    }));//.onTrue(new InstantCommand(() -> armPoseCommand.setPose(ArmSystemState.OVERSHOOT))).whileTrue(new ShootCommand(shooter, armPoseCommand, () -> armPoseCommand.getArmState() == ArmSystemState.OVERSHOOT, 50));

    new Trigger(conditionGoAMP).whileTrue(new AmpCommand(shooter, armPoseCommand, () -> armPoseCommand.getArmState() == ArmSystemState.AMP, -30, false));

    // new Trigger(() -> mOperatorController.getLeftTriggerAxis() == 1).onTrue(new InstantCommand(() -> armPoseCommand.setPose(ArmSystemState.PRECLIMB)));
    // new Trigger(() -> mOperatorController.getRightTriggerAxis() == 1).onTrue(new InstantCommand(() -> armPoseCommand.setPose(ArmSystemState.CLIMB)));

    new Trigger(ControllerUtil.toCond(Constants.ControlConds.scalestring1)).whileTrue(new ClimbCommand(climber, 0.7));

    new Trigger(ControllerUtil.toCond(Constants.ControlConds.forceShoot)).onTrue(new InstantCommand(() -> shooter.transit(0.5))).onFalse(new InstantCommand(() -> shooter.transit(0)));

    new Trigger(ControllerUtil.toCond(Constants.ControlConds.scalestring2)).whileTrue(new ClimbCommand(climber, -0.7));
    // new Trigger(() -> mOperatorController.getPOV() == 180).onTrue(new InstantCommand(()->armPoseCommand.setPose(ArmSystemState.TRAP)));
    // new Trigger(() -> mOperatorController.getPOV() == 0).whileTrue(new AmpCommand(shooter, armPoseCommand, ()->true, -60, true));
    // new Trigger(() -> mOperatorController.getPOV() == 270).onTrue(new InstantCommand(()->armPoseCommand.setPose(ArmSystemState.PRETRAP)).alongWith(
    //   new InstantCommand(()->shooter.setOpenLoop(-0.2, false))))
    //   .onFalse(new InstantCommand(()->shooter.setOpenLoop(0, false)));

    new Trigger(ControllerUtil.toCond(Constants.ControlConds.AutoAlignStage)).toggleOnTrue(mAutoAlignCommand);

    new Trigger(noteStored::get).onTrue(Commands.waitSeconds(0.5).raceWith(
      new InstantCommand(() -> {
        mDriverController.setRumble(RumbleType.kBothRumble, 0.5);
        mOperatorController.setRumble(RumbleType.kBothRumble, 0.5);
        System.out.println("Rumble");
      })
    ).andThen(
      new InstantCommand(() -> {
        mDriverController.setRumble(RumbleType.kBothRumble, 1.0);
        mOperatorController.setRumble(RumbleType.kBothRumble, 1.0);
        Timer.delay(0.5);
        mDriverController.setRumble(RumbleType.kBothRumble, 0.0);
        mOperatorController.setRumble(RumbleType.kBothRumble, 0.0);
      })
    ));

    new Trigger(ControllerUtil.toCond(Constants.ControlConds.ClkwRotatePos90Deg)).onTrue(new InstantCommand(() -> drivetrainSubsystem.offsetHeading(Math.PI/2)));
    new Trigger(ControllerUtil.toCond(Constants.ControlConds.CounterClkwRotatePos90Deg)).onTrue(new InstantCommand(() -> drivetrainSubsystem.offsetHeading(-Math.PI/2)));

    new Trigger(ControllerUtil.toCond(Constants.ControlConds.offsetArmUp)).onTrue(new InstantCommand(() -> armPoseCommand.offsetBy(0.001)));
    new Trigger(ControllerUtil.toCond(Constants.ControlConds.offsetArmDown)).onTrue(new InstantCommand(() -> armPoseCommand.offsetBy(-0.001)));
    new Trigger(ControllerUtil.toCond(Constants.ControlConds.ResetArmOffset)).onTrue(new InstantCommand(armPoseCommand::resetOffset));
  }


  

  public Command getAutonomousCommand() {
    final Command nowSelected = mAutoChooser.getSelected();
    String name = nowSelected.getName();
    if(!nowSelected.getName().equals("InstantCommand") && !nowSelected.getName().equals("AutoMidCommand"))
    {
      JSONParser parser = new JSONParser();
      try{
        Object obj = parser.parse(new FileReader("/home/lvuser/deploy/pathplanner/autos/"+name+".auto"));
        JSONObject autoJsonObj = (JSONObject)obj;
        Pose2d startPose = AutoBuilder.getStartingPoseFromJson((JSONObject)autoJsonObj.get("startingPose"));
        System.out.println("Inital heading: "+startPose.getRotation());
        drivetrainSubsystem.resetHeading(DriverStation.getAlliance().get()==Alliance.Blue ? -startPose.getRotation().getRadians() : -Math.PI-startPose.getRotation().getRadians());
        //TODO: test if works
      }
      catch (FileNotFoundException e) {
        e.printStackTrace();
      }
      catch (IOException e) {
        e.printStackTrace();
      }
      catch (ParseException e) {
        e.printStackTrace();
      }
    }
    return nowSelected;
  }

  public DrivetrainSubsystem getDrivetrainSubsystem(){
    return drivetrainSubsystem;
  }

  private final TimeDelayedBoolean mNoteOutHelper = new TimeDelayedBoolean();
  private final BooleanSupplier delayedNoteOut = () -> {
    final boolean ans = mNoteOutHelper.update(!noteStored.get(), 0.2);
    /*if(!noteStored.get())
    {
      System.out.println("Shoot in SENSOR in "+Timer.getFPGATimestamp());
    }*/
    return ans;
  };
  private final BooleanConsumer mPrintNote = (e) -> {
    /*if(e){
      System.out.println("Shoot in MOTOR in "+Timer.getFPGATimestamp());
    }*/
  };
  private void pathPlannerRegisterCommand(){
    // NamedCommands.registerCommand("Intake", new WaitCommand(new IntakeCommand(shooter, intake), 10, noteStored::get).alongWith(new InstantCommand(() -> armPoseCommand.setPose(ArmSystemState.INTAKE))));
    NamedCommands.registerCommand("Intake", new IntakeCommand(shooter, intake, armPoseCommand, false));
    NamedCommands.registerCommand("NearShoot", Commands.waitSeconds(10).raceWith(new ShootCommand(shooter, armPoseCommand, () -> armPoseCommand.getArmState() == ArmSystemState.AUTOSHOOT, 65, mPrintNote)).until(delayedNoteOut).alongWith(new InstantCommand(() -> armPoseCommand.setAutoShootPosition(0.235))));
    NamedCommands.registerCommand("MiddleShoot", Commands.waitSeconds(10).raceWith(new ShootCommand(shooter, armPoseCommand, () -> armPoseCommand.getArmState() == ArmSystemState.AUTOSHOOT, 75, mPrintNote)). until(delayedNoteOut).alongWith(new InstantCommand(() -> armPoseCommand.setAutoShootPosition(0.22))));
    NamedCommands.registerCommand("FarShoot", Commands.waitSeconds(10).raceWith(new ShootCommand(shooter, armPoseCommand, () -> armPoseCommand.getArmState() == ArmSystemState.AUTOSHOOT, 80, mPrintNote)).until(delayedNoteOut).alongWith(new InstantCommand(() -> armPoseCommand.setAutoShootPosition(0.185))));
    NamedCommands.registerCommand("Arm Down", new InstantCommand(() -> armPoseCommand.setPose(ArmSystemState.ARMDOWN)));
  }

}
