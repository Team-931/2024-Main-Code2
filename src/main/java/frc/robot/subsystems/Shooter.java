// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.AnalogInput;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter object.
   * Note that these Slot0Configs are specific to the {@link VelocityVoltage}
   * control method. To use {@link VelocityDutyCycle} divide by 12 V.
   * @see shootVelocityVoltage
   * @see VelocityVoltage
   */
  public Shooter() {
    Slot0Configs pid = new Slot0Configs();
    pid.kP = .11;
    pid.kI = .5;
    pid.kD = .0001;
    pid.kV = .12;
    shootBottom.getConfigurator().apply(pid);
    shootTop.getConfigurator().apply(pid);
  }

  /**
   * command factory method controlling the note shooting motors.
   *
   * @param s - "power" to pass on
   * @return a command
   */
  public Command shootCommand(double s) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
          setShoot(s);
        });
  }

/**
   * command factory method controlling the note holding motors.
   *
   * @param h - "power" to pass on
   * @return a command
   */
  public Command holdCommand(double h) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
          setHold(h);
          /* SmartDashboard.putNumber("holding:", h); */
        });
  }

/**
   * command factory method controlling the back holding motor {@bold only}.
   *
   * @param h - "power" to pass on
   * @return a command
   */
  public Command holdbackCommand(double h) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
          holdIng = h;
          /* SmartDashboard.putNumber("holding:", h); */
        });
  }

    /**
   * tests the line break sensor
   * @return true if line is not broken
   */
  public boolean sensorOff() {
    // Query some boolean state, such as a digital sensor.
    return sensor.getVoltage() <= 2;
  }

  public boolean shootFastEnough() {
    return shVel.getValueAsDouble() > .95 * ShooterConstants.shootBottomSpd;
  }

    /** This method will be called once per scheduler run
     * (every 20 ms).
     */
    @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //shootBottom.set(ShooterConstants.shootBottomSpd * shootIng);
    //shootTop.set(ShooterConstants.shootTopSpd * shootIng);
    armIntakeControl.set(ShooterConstants.intakeRotationalSpeedReverse * holdIng);
    floorIntakeControl.set(ShooterConstants.intakeRotationalSpeed * holdIngFront);
     /*     {
      if (periodicdelay > 0) --periodicdelay;
      else {
        periodicdelay = 10;
        SmartDashboard.putString("shooter velocity", shVel.refresh().toString());
        SmartDashboard.putNumber("hold velocity (RpS)", holdEnc.getVelocity());
        SmartDashboard.putBoolean("sensor", sensor.getVoltage() >= 2);
        //SmartDashboard.putNumber("sensor Voltage", sensor.getVoltage());
      }
    }
 */}

  public void simulationInit() {
    REVPhysicsSim.getInstance().addSparkMax(armIntakeControl, DCMotor.getNEO(1));
    REVPhysicsSim.getInstance().addSparkMax(floorIntakeControl, DCMotor.getNEO(1));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    REVPhysicsSim.getInstance().run();
  }

  private double holdIng = 0, holdIngFront = 0;

  public void setHold(double h) {
    holdIng = h;
    holdIngFront = h;
  }
  
  /** for control of shootTop and shootBottom */
  static final VelocityVoltage shootVelocityVoltage
     = new VelocityVoltage (0) .withSlot(0) /* .withOverrideBrakeDurNeutral(true) */;

  public void setShoot(double s) {
    shootTop.setControl(shootVelocityVoltage.withVelocity(s*ShooterConstants.shootTopSpd)
        /* .withFeedForward(Math.signum(s)) */);
    shootBottom.setControl(shootVelocityVoltage.withVelocity(s*ShooterConstants.shootBottomSpd)
        /* .withFeedForward(-Math.signum(s)) */);
  }
//motors
  private final TalonFX shootTop = new TalonFX(ShooterConstants.shootTopID),
   shootBottom = new TalonFX(ShooterConstants.shootBottomID);
  private final CANSparkMax floorIntakeControl = new CANSparkMax(ShooterConstants.holdFrontID, MotorType.kBrushless),
   armIntakeControl = new CANSparkMax(ShooterConstants.holdBackID, MotorType.kBrushless);
  private final AnalogInput sensor = new AnalogInput(ShooterConstants.sensorID);
/**  used only in periodic() */
  //private int periodicdelay = 0;
  private final StatusSignal<Double> shVel = shootBottom.getVelocity();
  private final RelativeEncoder holdEnc = armIntakeControl.getEncoder();
  {
    holdEnc.setVelocityConversionFactor(ShooterConstants.RpM2RpS);
  }
 }
