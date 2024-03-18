// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class FalconMotorConstants {
    public static final double kFreeSpeedRpm = 6380, 
    stallCurrent = 257 /* Amp */,
    stallTorque = 4.69 /* Newton-m */;
  }
  public static class OperatorConstants {
    public static final int opStickPort = 1;
    public static final int intakeAxis = 1;
    public static final double intakeTheshhold = .25;
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  
  }
  public static class ClimberConstants {
  
    public static final int leftID = 11, rightID = 12;
    public static final double gearbox = 12, gearing = gearbox / .75 /* in. diam */ / Math.PI;
    /** The maxHeight is physically 19.5 in. but the gearing is too large 
     * at the bottom: the diameter increases as the rope winds up.
     */
    public static final double maxHeight = 13.5 /* inch */;
    public static final double currentHigh = .95 * FalconMotorConstants.stallCurrent /* Amp */ / 12 /* Volt */;
    public static final double pulleyDiam = 1.25, robotWeight = 100, gravity = 9.80665 /* m/s^2*/,
          supportTorque = Units.inchesToMeters(pulleyDiam) / 2 * Units.lbsToKilograms(robotWeight) / 2 * gravity / gearbox,
          supportVoltage = supportTorque / FalconMotorConstants.stallTorque * 12 /* V */;
    public static final double kP = 2;
  }
  
  public static class IntakeConstants {
  
    
   public static final int intakeID = 15;
  }
  public static class  ArmConstants {
    public static final double gearing = 5 /* sprocket */ * 100 /* gearbox */ /1;
    public static final int ArmID = 10,
          boreEncoderID = 0;
    public static final double rampTime = .25 /* sec */;
    public static final double holdAt0 = .4 /* volts */; // test Voltage to stay up at moment arm horizontal
    public static final double boreOffset = .662;
    public static final double lowerLimit = -14. /*degrees */ / 360 /* degrees / rotation */+ .07;
    public static final double upperLimit = .31 /* rotation */;
    /** full power at 1/5 of full range i. e. >= 3 inches to go */
    public static final double kP = 12. /* volts */ / (upperLimit - lowerLimit) /* rotation */ * 4.5;
    public static final double kD = /* kP * 0.04 */0;
    public static final double kV = 3;
  }
  public static class ShooterConstants {
    public static final double shootGearing = 2./3;
    
    public static final double shootTopSpd = -120*shootGearing,
      shootBottomSpd = 27.5 / 80 * shootTopSpd,
      intakeRotationalSpeedReverse = 0.3, intakeRotationalSpeed = -0.3, holdFwd = 1, holdRvs = -1;

    public static final int shootBottomID = 22, shootTopID = 23,
      holdBackID = 20, holdFrontID = 21,
      sensorID = 0;
    public static final double RpM2RpS = 1.0/60;
  }
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kMinSpeedMultiplier = 0.5;

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(25.5); //values have been adjusted for current bot
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26);  //values have been adjusted for current bot
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    //Falcon CAN IDs
    //All CAN IDs set for current bot
    public static final int kFrontLeftDrivingCanId = 1; 
    public static final int kRearLeftDrivingCanId = 7;
    public static final int kFrontRightDrivingCanId = 3;
    public static final int kRearRightDrivingCanId = 5;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftTurningCanId = 2;
    public static final int kRearLeftTurningCanId = 8;
    public static final int kFrontRightTurningCanId = 4;
    public static final int kRearRightTurningCanId = 6;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = FalconMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = ((45.0 * 22) / (kDrivingMotorPinionTeeth * 15))/ kWheelCircumferenceMeters;
    public static final double kDriveWheelFreeSpeedMps = (kDrivingMotorFreeSpeedRps )
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedMps;
    public static final double kDrivingP = 0.9 * kDrivingFF;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

  
    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  

  public static final class AutoConstants {
    public static final double distanceFudge = 1,
              inverseFudge = 1 / distanceFudge;
    public static final double speakerAngle = 60;
    public static final Translation2d speakerRight = new Translation2d(.90, 3.13);
    public static final double allianceZoneLimit = Units.inchesToMeters(76.1);
    public static final double centerStartX = 0.9;
    public static final double centerStartY = 2.66;
    public static final Translation2d centerToCenterNote = new Translation2d(2.8-.66, 2.66);
    public static final Translation2d noteToCenterSpeaker = new Translation2d(1.1 - 0.66, 2.65);
    public static final Translation2d centerToLeftNote = new Translation2d(2.6 - 0.66, 0.95);
    public static final Translation2d leftNoteToCenter = new Translation2d(1.11 - 0.66, 2.67);
    public static final double leftStartX = 0.3;
    public static final double leftStartY = 1.6;
    public static final Translation2d leaveLeftSide1 = new Translation2d(0.8, 1.0);
    public static final Translation2d leaveLeftSide2 = new Translation2d(1.8, 1.01);

    public static final double kMaxSpeedMetersPerSecond = 3.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    
  }

}
