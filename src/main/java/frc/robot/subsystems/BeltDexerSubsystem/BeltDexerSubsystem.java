package frc.robot.subsystems.BeltDexerSubsystem;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BeltDexerConstants;

public class BeltDexerSubsystem extends SubsystemBase {
    private SparkMax motorA;
    private SparkMax motorB;
    private RelativeEncoder encoderA;
    private RelativeEncoder encoderB;
    private SparkClosedLoopController pidA;
    private SparkClosedLoopController pidB;

    public BeltDexerSubsystem() {
        motorA = new SparkMax(BeltDexerConstants.MotorcanIDA, SparkLowLevel.MotorType.kBrushless);
        motorB = new SparkMax(BeltDexerConstants.MotorcanIDB, SparkLowLevel.MotorType.kBrushless);
        SparkMaxConfig configA = new SparkMaxConfig();
        SparkMaxConfig configB = new SparkMaxConfig();
        configA.inverted(BeltDexerConstants.invertA);
        configB.inverted(BeltDexerConstants.invertB);
        configA.idleMode(SparkBaseConfig.IdleMode.kCoast);
        configB.idleMode(SparkBaseConfig.IdleMode.kCoast);
        configA.smartCurrentLimit(40);
        configB.smartCurrentLimit(40);
        configA.encoder
                .positionConversionFactor(360 / BeltDexerConstants.gearRatioA)
                .velocityConversionFactor((360.0/60.0) / BeltDexerConstants.gearRatioA);
        configB.encoder
                .positionConversionFactor(360 / BeltDexerConstants.gearRatioB)
                .velocityConversionFactor((360.0/60.0) / BeltDexerConstants.gearRatioB);
        configA.closedLoop
                .pid(BeltDexerConstants.MotorAP, BeltDexerConstants.MotorAI, BeltDexerConstants.MotorAD);
        configB.closedLoop
                .pid(BeltDexerConstants.MotorBP, BeltDexerConstants.MotorBI, BeltDexerConstants.MotorBD);
        motorA.configure(configA, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorB.configure(configB, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoderA = motorA.getEncoder();
        encoderB = motorB.getEncoder();
        pidA = motorA.getClosedLoopController();
        pidB = motorB.getClosedLoopController();
    }

    public void setMotorAPosition(double position) { // in rotations according to setSetpoint, but will be converted to degrees by the encoder conversion factor maybe, idk, maybe not
        pidA.setSetpoint(position, SparkBase.ControlType.kPosition);
    }

    public void setMotorBPosition(double position) {
        pidB.setSetpoint(position, SparkBase.ControlType.kPosition);
    }

    public void setMotorAVelocity(double velocity) {
        pidA.setSetpoint(velocity, SparkBase.ControlType.kVelocity);
    }

    public void setMotorBVelocity(double velocity) {
        pidB.setSetpoint(velocity, SparkBase.ControlType.kVelocity);
    }

    public void setMotorAPower(double power) {
        motorA.set(power);
    }

    public void setMotorBPower(double power) {
        motorB.set(power);
    }

    public void stop() {
        motorA.stopMotor();
        motorB.stopMotor();
    }

    public Command runDexer() {
    return Commands.startEnd(
        () -> {
            motorA.set(0.3);
            motorB.set(0.3);
        },
        () -> {
            motorA.stopMotor();
            motorB.stopMotor();
        },
        this
    );
}
}
