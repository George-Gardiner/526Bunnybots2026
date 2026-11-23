package frc.robot.subsystems.BeltDexerSubsystem;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SlapdownConstants;

public class SlapdownSubsystem extends SubsystemBase {
    private final SparkMax pivotMotorLeft;
    // private final SparkMax pivotMotorRight;
    private final SparkMax motorRoller;
    private final SparkClosedLoopController pidPivotLeft;
    private final SparkMaxConfig pivotMotorConfigLeft;
    // private final SparkMaxConfig pivotMotorConfigRight;
    private final SparkMaxConfig rollerMotorConfig;    
    private boolean isRollerEnabled;
    private final SparkClosedLoopController pidRoller;

    public SlapdownSubsystem() {
        pivotMotorLeft = new SparkMax(SlapdownConstants.pivotMotorLeftID, SparkLowLevel.MotorType.kBrushless);
        // pivotMotorRight = new SparkMax(SlapdownConstants.pivotMotorRightID, SparkLowLevel.MotorType.kBrushless);
        motorRoller = new SparkMax(SlapdownConstants.rollerMotorID, SparkLowLevel.MotorType.kBrushless);
        pivotMotorConfigLeft = new SparkMaxConfig();
        // pivotMotorConfigRight = new SparkMaxConfig();
        rollerMotorConfig = new SparkMaxConfig();
        pivotMotorConfigLeft.inverted(SlapdownConstants.invertPivot);
        pivotMotorConfigLeft.idleMode(IdleMode.kCoast);
        pivotMotorConfigLeft.smartCurrentLimit(SlapdownConstants.pivotCurrentLimit);
        pivotMotorConfigLeft.closedLoop.maxMotion.cruiseVelocity(100);
        pivotMotorConfigLeft.closedLoop.pid(SlapdownConstants.pivotP, SlapdownConstants.pivotI, SlapdownConstants.pivotD);

        rollerMotorConfig.inverted(SlapdownConstants.invertRoller);
        rollerMotorConfig.idleMode(IdleMode.kCoast);
        rollerMotorConfig.closedLoop.pid(SlapdownConstants.rollerP, SlapdownConstants.rollerI, SlapdownConstants.rollerD);
        rollerMotorConfig.smartCurrentLimit(SlapdownConstants.rollerCurrentLimit);

        // pivotMotorConfigRight.follow(pivotMotorLeft, true);

        pivotMotorLeft.configure(pivotMotorConfigLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // pivotMotorRight.configure(pivotMotorConfigRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorRoller.configure(rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        pidPivotLeft = pivotMotorLeft.getClosedLoopController();

        pidRoller = motorRoller.getClosedLoopController();

        SmartDashboard.setDefaultBoolean("Is Intake Lowering/Lowered?", false);
        SmartDashboard.setDefaultBoolean("Intake/Intake Running", false);
        SmartDashboard.setDefaultBoolean("Intake/Intake Reversed", false);
  }

  @Override
  public void periodic() {
    ;
  }

  public void toggleRoller() {
    setRoller(!isRollerEnabled);
  }

  public void setRoller(boolean enabled) {
    if (enabled) {
      pidRoller.setSetpoint(SlapdownConstants.rollerVelocity, SparkBase.ControlType.kVelocity);
    }
    else {
      stopRoller();
    }
    isRollerEnabled = enabled;
    SmartDashboard.putBoolean("Intake/Intake Running", isRollerEnabled);
  }

  //make this later
  public void setRollerReversed(boolean enabled) {
    if (enabled) {
      pidRoller.setSetpoint(-SlapdownConstants.rollerVelocity, SparkBase.ControlType.kVelocity);
    }
    else {
      stopRoller();
    }
    isRollerEnabled = enabled;
    SmartDashboard.putBoolean("Intake/Intake Running", isRollerEnabled);
  }

  public void setPivotAngle(double angle) {
    pidPivotLeft.setSetpoint(angle, ControlType.kMAXMotionPositionControl);
  }

  public void stop() {
    setRoller(false);
    stopPivot();
  }

  public void stopPivot() {
    pivotMotorLeft.stopMotor();
    // pivotMotorRight.stopMotor();
  }

  public void stopRoller() {
    motorRoller.stopMotor();
  }

  public Command intakeDownCommand() {
    return startEnd(
            () -> {
              setPivotAngle(SlapdownConstants.pivotLoweredAngle);
              setRoller(true);
            },
            () -> {
              setPivotAngle(SlapdownConstants.pivotRaisedAngle);
              setRoller(false);
            }
    );
  }

  public Command intakeMidCommand() {
    return startEnd(
            () -> {
              setPivotAngle(SlapdownConstants.pivotL1Angle);
            },
            () -> {
              setPivotAngle(SlapdownConstants.pivotRaisedAngle);
            }
    );
  }
  
  public Command rollerIntakeCommand() {
    return startEnd(
      () -> setRoller(true), 
      () -> setRoller(false)
    );
  }

  public Command rollerOutakeCommand() {
    return startEnd(
      () -> setRollerReversed(true), 
      () -> setRollerReversed(false)
    );
  }

  @Override
  public void simulationPeriodic() {

  }
} 
