package frc.robot.subsystems.BeltDexerSubsystem;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.BeltDexerConstants;;

public class BeltDexerSubsystem {
    private SparkMax motorA;
    private SparkMax motorB;
    private RelativeEncoder encoderA;
    private RelativeEncoder encoderB;
    private SparkClosedLoopController pidA;
    private SparkClosedLoopController pidB;

    public BeltDexerSubsystem() {
        motorA = new SparkMax(BeltDexerConstants.MotorcanIDA, SparkLowLevel.MotorType.kBrushless);
        motorB = new SparkMax(BeltDexerConstants.MotorcanIDB, SparkLowLevel.MotorType.kBrushless);
        encoderA = motorA.getEncoder();
        encoderB = motorB.getEncoder();
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
    }
}
