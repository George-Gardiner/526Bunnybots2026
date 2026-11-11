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

public class Slapdown extends SubsystemBase {
    private SparkMax motorA;
    private SparkMax motorB;
    private RelativeEncoder encoderA;
    private RelativeEncoder encoderB;
    private SparkClosedLoopController pidA;
    private SparkClosedLoopController pidB;

    public enum PivotState {
        RAISING,
        AGITATING,
        LOWERING
    }

    
}
