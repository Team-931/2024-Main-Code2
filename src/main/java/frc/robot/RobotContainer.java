// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;
import java.util.Optional;
import java.util.function.IntSupplier;
import java.util.function.UnaryOperator;

import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Shooter shooter = new Shooter();
  private final Intake intake = new Intake();
  private final Arm arm = new Arm();
  private final Climber climber = new Climber();

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandGenericHID opStick =
      new CommandGenericHID(OperatorConstants.opStickPort);

  double inputScale(double input, int scale) {
        double temp = input;
        for(int i = 0; i < scale; i++) {
          temp *= Math.abs(temp);
        }
        return(temp);
      }
  double circularScale(double in) {
  boolean isNegative = in < 0;
  double out;
  if (isNegative) {
    out = (1 - ((1 - DriveConstants.kMinSpeedMultiplier) * m_driverController.getLeftTriggerAxis())) * (Math.sqrt(1 - Math.pow(in, 2)) - 1);
  }
  else {
    out = (1 - ((1 - DriveConstants.kMinSpeedMultiplier) * m_driverController.getLeftTriggerAxis())) * (-Math.sqrt(1 - Math.pow(in, 2)) + 1);
  }
  return(out);
}

/** first part of {@code circularScale} used in {@code m_robotDrive.setDefaultCommand}
 * <p> {@code circularScale(in) == manualScale() * circularScale1(in) * in;}
 */
  double manualScale() {
    return (1 - ((1 - DriveConstants.kMinSpeedMultiplier) * m_driverController.getLeftTriggerAxis()));
  }

/** second part of {@code circularScale} used in {@code m_robotDrive.setDefaultCommand}
 * <p> {@code circularScale(in) == manualScale() * circularScale1(in) * in;}
 * <p> also deals with case {@code in > 1} but assumes {@code in >= 0}
 */
  double circularScale1(double in) {
    return in>1? 1/in: in / (1 + Math.sqrt(1 - in*in));
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    autoChooser.addOption("example", 0); //autoMaker.swerveControllerCommand(AutoMaker.exampleTrajectory)
    autoChooser.addOption("to speaker", 1);
    autoChooser.addOption("From Center Two Notes", 2);
    autoChooser.addOption("From Center Three Notes", 3);
    SmartDashboard.putData(autoChooser);
    SmartDashboard.setDefaultNumber("starting x", 0);
    SmartDashboard.setPersistent("starting x");
    SmartDashboard.setDefaultNumber("starting y", 0);
    SmartDashboard.setPersistent("starting y");
    // Configure default[\] commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> {
              double scale = manualScale(),
              yVal = -MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.kDriveDeadband),
              xVal = -MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.kDriveDeadband),
              turnVal = -MathUtil.applyDeadband(m_driverController.getRightX(), OperatorConstants.kDriveDeadband),
              turnScale = circularScale1(Math.abs(turnVal)),
              linScale = circularScale1(Math.sqrt(xVal*xVal+yVal*yVal));
              m_robotDrive.drive(
                (yVal) * scale * linScale,  
                (xVal) * scale * linScale,
                (turnVal) * scale * turnScale,
                true, false);},
            m_robotDrive));
            
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */

  private void configureButtonBindings() {
    m_driverController.rightBumper().whileTrue(m_robotDrive.run(m_robotDrive :: setX));
        

    m_driverController.y().onTrue(m_robotDrive.runOnce((m_robotDrive :: zeroHeading)));
        
      final EventLoop eventLoop = CommandScheduler.getInstance().getDefaultButtonLoop();
      /**
       * Creates a binding that periodically receives a state request (in the form of an integer) from input,
       * and if it is a change of state, records it and optionally dispatches a {@code Command}.
       * (Recommendation: return a negative to prevent a {@code Command})
       * @param input a function-like object which returns an {@code int} based on any info it wants
       * @param output a set of {@link Command}s to dispatch based on the state (state 0 first),
       * If the state {@code int} is {@code < 0} or {@code >= output.length}, no {@code Command} is dispatched.
       */
      final class MultiBind {
        MultiBind(IntSupplier input, Command... output){
          eventLoop.bind(new Runnable() {
            int prevInput;
            final int len = output.length;
            public void run() {
              int newInput = input.getAsInt();
              if (newInput != prevInput) {
                prevInput = newInput;
                if (0 <= newInput && newInput < len)
                output[newInput].schedule();
              }
            }
          });
        }
      }
    /* y axis: forward and reverse shooter hold */
    new MultiBind (
      new IntSupplier() {
          boolean haveJustGottenNote =false;
        
      public int getAsInt()  {
        var val = opStick.getRawAxis(OperatorConstants.intakeAxis);
        if (val > OperatorConstants.intakeTheshhold) if (!haveJustGottenNote)
          if (shooter.sensorOff() ) return 1;
          else {
            haveJustGottenNote = true;
            return 3;
          } else;
        else {
          haveJustGottenNote = false;
          if (val < -OperatorConstants.intakeTheshhold) return 2;
        }
        return 0;
      }}, 
      shooter.holdCommand(0) .andThen(intake.runcommand(0), rumble(false)),

      shooter.holdCommand(ShooterConstants.holdFwd)
          .andThen(intake.runIf(.3, arm::atBottom), rumble(false)),
      shooter.holdCommand(ShooterConstants.holdRvs)
          .andThen(intake.runIf(-.3, arm::atBottom), rumble(false)),
    // rumble if the line break senses a "note"
      rumble(true)
              .andThen(
                shooter.holdCommand(ShooterConstants.holdRvs*.25),
                new WaitUntilCommand(shooter::sensorOff),
                shooter.holdCommand(0))
      );
    
    
    /* y button: shooter shoot */
      opStick.button(1)
                  .onTrue(shooter.shootCommand(1)
                      .andThen(shooter.holdCommand((ShooterConstants.holdFwd)*2))) 
                  .onFalse(shooter.shootCommand(0)
                      .andThen(shooter.holdCommand(0)));
      opStick.button(5)  .onTrue(shooter.shootCommand(1))
                                .onFalse(shooter.shootCommand(0));
      opStick.button(6)  .onTrue(shooter.holdCommand(ShooterConstants.holdFwd))
                                .onFalse(shooter.holdCommand(0));
      /* a button: arm up */
      opStick.button(2) .onTrue(arm.upCmd(true));
      opStick.button(7) .onTrue(arm.upCmd(false));

      /* x button: release climber */
      opStick.button(3) .and(opStick.button(4).negate()) .onTrue(climber.topOrBottomCommand(true))
        .or (
      /* b button: retract climber and stop */
      opStick.button(4) .and(opStick.button(3).negate()) .onTrue(climber.topOrBottomCommand(false)) 
        )
        .or (
      opStick.button(3) .and(opStick.button(4)) .onTrue(climber.windDownCommand())
        )
                          .onFalse(climber.stayPutCommand());

      new Trigger(() -> climber.currentHigh(true))
                  .onTrue(climber.runOnce(() -> climber.stayPut1(true)));
      new Trigger(() -> climber.currentHigh(false))
                  .onTrue(climber.runOnce(() -> climber.stayPut1(false)));

    
  }

  private Timer rumbleTimer = new Timer();

  private Command rumble(boolean b) {
    if (b) rumbleTimer.restart();
        var xbox = m_driverController.getHID();
    return Commands.runOnce(() -> xbox.setRumble(RumbleType.kBothRumble, b? 1: 0));
  }

  SendableChooser<Integer> autoChooser = new SendableChooser<>();
  private final class AutoMaker {
    
    static Rotation2d finalAngle = Rotation2d.fromDegrees(90);
    static Pose2d finalPose = new Pose2d();
    // Create config for trajectory
    static TrajectoryConfig config = new TrajectoryConfig(
          AutoConstants.kMaxSpeedMetersPerSecond,
          AutoConstants.kMaxAccelerationMetersPerSecondSquared)
          // Add kinematics to ensure max speed is actually obeyed
          .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.(divide by 1.85)
    static Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(/* new Translation2d(1, -1), new Translation2d(2, 1) */),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, finalAngle),
         config);

         final Translation2d rectCtr = new Translation2d(DriveConstants.kWheelBase,DriveConstants.kTrackWidth).div(2);
    Trajectory[] toSpeaker(double x, double y)  {
    var retval = new Trajectory[2];
    
    var initAngle = Rotation2d.fromDegrees(0);
    var init = maybeReflect.apply(new Translation2d (x, y) .plus(rectCtr .rotateBy(initAngle)));
    double speakerAngle = AutoConstants.speakerAngle;
    Translation2d speakerTranslation2d;
    Pose2d leavePose;
    List<Translation2d> leaveWaypt;

    if (y < AutoConstants.centerToCloseNote.getY()) {
      speakerAngle = - AutoConstants.speakerAngle;
      speakerTranslation2d = AutoConstants.speakerLeft;
      leavePose = AutoConstants.leaveLeft;
      leaveWaypt = List.of(/* AutoConstants.waypointLeft */);
    }
    else {
      speakerAngle = AutoConstants.speakerAngle;
      speakerTranslation2d = AutoConstants.speakerRight;
      leavePose = AutoConstants.leaveRight;
      leaveWaypt = List.of();
    }
    var endAngle = Rotation2d.fromDegrees(speakerAngle);
    var end = maybeReflect.apply(speakerTranslation2d .plus(rectCtr .rotateBy(endAngle)));
         endAngle = endAngle.times(allianceSign);
         finalPose = new Pose2d(end, endAngle);
    var finalPose1 = new Pose2d(end, endAngle.plus(AutoConstants.piRot));
         finalAngle = AutoConstants.piRot.minus(endAngle);
    leavePose = new Pose2d(maybeReflect.apply(leavePose.getTranslation() .plus(rectCtr .rotateBy(leavePose.getRotation()))), leavePose.getRotation());
         var initPose = new Pose2d(init, initAngle);
         m_robotDrive.resetOdometry(initPose);
    
      retval[0] = TrajectoryGenerator.generateTrajectory(
        initPose,
        List.of(),
        finalPose1,
        config);
      retval[1] = TrajectoryGenerator.generateTrajectory(
        finalPose,
       leaveWaypt,
       leavePose, config);
    return retval;
    }

  
    
    Trajectory []fromCenterToCloseNote(double x, double y, Pose2d destNote){
      Trajectory[] retVal = new Trajectory[2];
      var fromCenterToCloseNoteStartAngle = Rotation2d.fromDegrees(0);
      var fromCenterToCloseNoteStartPos = maybeReflect.apply(new Translation2d(x, y).plus(rectCtr.rotateBy(fromCenterToCloseNoteStartAngle)));
      var fromCenterToCloseNoteEndAngle = destNote.getRotation();
      var fromCenterToCloseNoteEndPos = maybeReflect.apply(destNote.getTranslation().plus(rectCtr.rotateBy(fromCenterToCloseNoteEndAngle)));
      List<Translation2d> waypts = /* destNote == AutoConstants.centerToLeftNote ?List.of(AutoConstants.leftExtra.plus(rectCtr.rotateBy(fromCenterToCloseNoteStartAngle))) : */List.of();
      fromCenterToCloseNoteEndAngle = fromCenterToCloseNoteEndAngle.times(allianceSign);
      finalPose = new Pose2d(fromCenterToCloseNoteEndPos, fromCenterToCloseNoteEndAngle);
      finalAngle = AutoConstants.piRot.minus(fromCenterToCloseNoteEndAngle);
      var initPose = new Pose2d(fromCenterToCloseNoteStartPos, fromCenterToCloseNoteStartAngle);
      m_robotDrive.resetOdometry(initPose);
      retVal[0] = TrajectoryGenerator.generateTrajectory(
        initPose,
        waypts, 
        finalPose, 
        config);
        config.setReversed(true);
      retVal[1] = TrajectoryGenerator.generateTrajectory(
        finalPose,
        List.of(), 
        initPose, 
        config);
        config.setReversed(false);
      return retVal;
    }


   static ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    
    {
      thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }
    
    SwerveControllerCommand swerveControllerCommand(Trajectory trajectory) {
    // Reset odometry to the starting pose of the trajectory.
    return new SwerveControllerCommand(
        trajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);
    }
    SwerveControllerCommand swerveControllerAngleRequestCommand(Trajectory trajectory, Rotation2d endRotation2d) {
    // Reset odometry to the starting pose of the trajectory.
    //Rotation2d angleChange = endRotation2d .minus(trajectory.getInitialPose().getRotation());
    return new SwerveControllerCommand(
        trajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        () -> endRotation2d,
        m_robotDrive::setModuleStates,
        m_robotDrive);
    }
  }
  AutoMaker autoMaker = new AutoMaker();
  int allianceSign;
  UnaryOperator<Translation2d> maybeReflect = (t -> new Translation2d(t.getX(), t.getY() * allianceSign));
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // - if blue, + if red
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // - if blue, + if red
    allianceSign = alliance.isPresent() && alliance.get() == Alliance.Blue ? -1: 1;
    
    // Comment later
    Integer choice = autoChooser.getSelected();
    if (choice == null) return null;
    switch (choice) {
      case 2: // two notes
        var indexStage = autoMaker.fromCenterToCloseNote(
          SmartDashboard.getNumber("starting x", 0), 
          SmartDashboard.getNumber("starting y", 0),
          AutoConstants.centerToCloseNote);
        return new SequentialCommandGroup(
          shooter.shootCommand(1),
      new ParallelCommandGroup(
        new WaitUntilCommand(shooter::shootFastEnough),
        new WaitCommand(1)
        ),
          shooter.holdCommand(ShooterConstants.holdFwd),
          new WaitCommand(1), // Could we wait for shooter::sensorOff, instead?
          shooter.shootCommand(0),
          shooter.holdCommand(0), 

        new ParallelCommandGroup(
            autoMaker.swerveControllerCommand(indexStage[0]),
            shooter.holdCommand(ShooterConstants.holdFwd)
            .andThen(intake.runIf(.3, arm::atBottom), 
            new ParallelRaceGroup(new WaitUntilCommand(()->!shooter.sensorOff()), new WaitCommand(indexStage[0].getTotalTimeSeconds() + 1.5)), 
            shooter.holdCommand(0), 
            intake.runcommand(0))
            ),

          
         new ParallelCommandGroup
            (autoMaker.swerveControllerCommand(indexStage[1]),
         shooter.shootCommand(1)),
          new WaitUntilCommand(shooter::shootFastEnough),
          shooter.holdCommand(ShooterConstants.holdFwd),
          new WaitCommand(1), // Could we wait for shooter::sensorOff, instead?
          shooter.shootCommand(0),
          shooter.holdCommand(0)
        )
;
  case 3: //Three notes
        indexStage = autoMaker.fromCenterToCloseNote(
          SmartDashboard.getNumber("starting x", 0), 
          SmartDashboard.getNumber("starting y", 0),
          AutoConstants.centerToCloseNote);
        var indexStage2  = autoMaker.fromCenterToCloseNote(
          SmartDashboard.getNumber("starting x", 0), 
          SmartDashboard.getNumber("starting y", 0),
          AutoConstants.centerToLeftNote);
        return new SequentialCommandGroup(
          shooter.shootCommand(1),
      new ParallelCommandGroup(
        new WaitUntilCommand(shooter::shootFastEnough),
        new WaitCommand(1)
        ),
          shooter.holdCommand(ShooterConstants.holdFwd),
          new WaitCommand(1), // Could we wait for shooter::sensorOff, instead?
          shooter.shootCommand(0),
          shooter.holdCommand(0), 

        new ParallelCommandGroup(
            autoMaker.swerveControllerCommand(indexStage[0]),
            shooter.holdCommand(ShooterConstants.holdFwd)
            .andThen(intake.runIf(.3, arm::atBottom), 
            new ParallelRaceGroup(new WaitUntilCommand(()->!shooter.sensorOff()), new WaitCommand(indexStage[0].getTotalTimeSeconds() + 1.5)), 
            shooter.holdCommand(0), 
            intake.runcommand(0))
            ),

          
         new ParallelCommandGroup
            (autoMaker.swerveControllerCommand(indexStage[1]),
         shooter.shootCommand(1)),
          new WaitUntilCommand(shooter::shootFastEnough),
          shooter.holdCommand(ShooterConstants.holdFwd),
          new WaitCommand(1), // Could we wait for shooter::sensorOff, instead?
          shooter.shootCommand(0),
          shooter.holdCommand(0) ,
        new ParallelCommandGroup(
            autoMaker.swerveControllerCommand(indexStage2[0]),
            shooter.holdCommand(ShooterConstants.holdFwd)
            .andThen(intake.runIf(.3, arm::atBottom), 
            new ParallelRaceGroup(new WaitUntilCommand(()->!shooter.sensorOff()), new WaitCommand(indexStage[0].getTotalTimeSeconds() + 1.5)), 
            shooter.holdCommand(0), 
            intake.runcommand(0))
            ),

          
         new ParallelCommandGroup
            (autoMaker.swerveControllerCommand(indexStage2[1]),
         shooter.shootCommand(1)),
          new WaitUntilCommand(shooter::shootFastEnough),
          shooter.holdCommand(ShooterConstants.holdFwd),
          new WaitCommand(1), // Could we wait for shooter::sensorOff, instead?
          shooter.shootCommand(0),
          shooter.holdCommand(0) 
        )
;
      case 1: // one note from side
      indexStage = autoMaker.toSpeaker(
          SmartDashboard.getNumber("starting x", 0), 
          SmartDashboard.getNumber("starting y", 0));

        return new SequentialCommandGroup(
      new ParallelCommandGroup(
        autoMaker.swerveControllerAngleRequestCommand(indexStage[0], AutoMaker.finalPose.getRotation()),
        shooter.shootCommand(1)
      ),
      new Rotator(AutoMaker.finalAngle),
      //m_robotDrive.runOnce(() -> m_robotDrive.drive(0, 0, 0, false, false)),
      new WaitUntilCommand(shooter::shootFastEnough),
      shooter.holdCommand(ShooterConstants.holdFwd),
      new WaitCommand(1), // Could we wait for shooter::sensorOff, instead?
      shooter.shootCommand(0),
      shooter.holdCommand(0),
      autoMaker.swerveControllerCommand(indexStage[1]),
      new Rotator(AutoConstants.piRot)
      );

      case 0:
      default:
        break;
    }
    return autoMaker.swerveControllerCommand(AutoMaker.exampleTrajectory)
      .andThen(new Rotator(AutoMaker.finalAngle));
  }

  class Rotator extends Command {
    Rotation2d target;
    double speed;
    Rotator (Rotation2d targetRotation2d) {
      target = targetRotation2d;
    }
    @Override
    public void initialize() {
      speed = .1 * Math.signum(target.minus(Rotation2d.fromDegrees(m_robotDrive.getHeading())).getRadians() );
    }
    @Override
    public void execute() {
      m_robotDrive.drive(0, 0, speed, false, false);
    }

    @Override
    public boolean isFinished() {
      return speed * target.minus(Rotation2d.fromDegrees(m_robotDrive.getHeading())).getRadians() <= 0;
    }
    @Override
    public void end(boolean interrupted) {
      m_robotDrive.drive(0, 0, 0, false, false);
    }
  }
}
