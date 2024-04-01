package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm  extends SubsystemBase{
    public Arm() {
        boreEncoder.setDistancePerRotation(-1);
        boreEncoder.setPositionOffset(ArmConstants.boreOffset + ArmConstants.lowerLimit);
        var mctrl = motor.getConfigurator();
        var sensConfigs = new FeedbackConfigs();
        sensConfigs.SensorToMechanismRatio = ArmConstants.gearing;
        mctrl.apply(sensConfigs);
        var pid = new Slot0Configs()
             .withKP(ArmConstants.kP) 
             .withKG(ArmConstants.holdAt0)
             .withKI(ArmConstants.kD)
             .withKV(ArmConstants.kV)
             .withGravityType(GravityTypeValue.Arm_Cosine);
        mctrl.apply(pid);
        var out = new MotorOutputConfigs()
             .withInverted(InvertedValue.CounterClockwise_Positive)
             .withNeutralMode(NeutralModeValue.Brake);
        mctrl.apply(out);
        mctrl.setPosition(ArmConstants.lowerLimit);
        var limitCfg = new HardwareLimitSwitchConfigs() 
            .withReverseLimitType(ReverseLimitTypeValue.NormallyOpen)
            .withReverseLimitEnable(true);
        mctrl.apply(limitCfg);
        var motMag = new MotionMagicConfigs() .withMotionMagicAcceleration(2) . withMotionMagicCruiseVelocity(20);
        mctrl.apply(motMag);
        mctrl.apply(new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(ArmConstants.rampTime));
    }

    @Override
    public void periodic() {
        if (periodicdelay > 0) --periodicdelay;
      else {
        periodicdelay = 10;
        double ang = boreEncoder.getDistance();
        motor.setPosition(ang);
        //boolean motorStopping;
        if(angleOut.Position == ArmConstants.lowerLimit && ang < ArmConstants.lowerLimit + .005)
            {motor.stopMotor(); /* motorStopping = true; */}
            //else motorStopping = false;
/*         SmartDashboard.putBoolean("motor stopping", motorStopping);
        SmartDashboard.putString("arm angle", angle.refresh().toString());
        SmartDashboard.putNumber("arm angle 2", ang);
        SmartDashboard.putString("armVoltage", voltage.refresh().toString());
        SmartDashboard.putString("limit", limit.refresh().toString());
 */        
      }
    }

    public double getAngle() {
        return angle.refresh().getValueAsDouble();
    }
    public boolean atBottom() {
        return getAngle() < ArmConstants.lowerLimit + 1./72;
    }
    public Command upCmd(boolean b) {
        return runOnce(() -> {gotoAngle(b ? ArmConstants.upperLimit: ArmConstants.lowerLimit);});
    }
    public Command shootPosCommand(boolean b){
        return runOnce(()-> {gotoAngle(b ? ArmConstants.upperLimit*0.2465 : ArmConstants.lowerLimit);});
    }
    public Command shootFarCommand(boolean b){
        return runOnce(()-> {gotoAngle(b ? ArmConstants.upperLimit * 0.45 : ArmConstants.lowerLimit);});
    }
    public void gotoAngle(double angle) {
        angle = Math.max(angle, ArmConstants.lowerLimit);
        angle = Math.min(angle, ArmConstants.upperLimit);
        //SmartDashboard.putNumber("desired angle", angle);
        motor.setControl(angleOut.withPosition(angle));
    }
    private final TalonFX motor = new TalonFX(ArmConstants.ArmID);
    private final StatusSignal<Double> angle = motor.getPosition(), voltage = motor.getMotorVoltage();
    private final StatusSignal<ReverseLimitValue> limit = motor.getReverseLimit();
    private final MotionMagicVoltage angleOut = new MotionMagicVoltage(0) .withSlot(0);

    private final DutyCycleEncoder boreEncoder = new DutyCycleEncoder(ArmConstants.boreEncoderID);
    /**  used only in periodic() */
    private int periodicdelay = 0;
}
