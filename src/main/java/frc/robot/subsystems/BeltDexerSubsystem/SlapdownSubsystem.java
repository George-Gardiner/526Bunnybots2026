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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SlapdownConstants;

public class SlapdownSubsystem extends SubsystemBase {
    private final SparkMax pivotMotorLeft;
    private final SparkMax pivotMotorRight;
    private final SparkMax motorRoller;
    private final SparkClosedLoopController pidPivotLeft;
    private final SparkClosedLoopController rollerPid;
    private final SparkMaxConfig pivotMotorConfigLeft;
    private final SparkMaxConfig pivotMotorConfigRight;
    private final SparkMaxConfig rollerMotorConfig;    
    private boolean isRollerEnabled;
    AbsoluteEncoder throughBore;
    RelativeEncoder encoderLeft;

    public SlapdownSubsystem() {
        pivotMotorLeft = new SparkMax(SlapdownConstants.pivotMotorLeftID, SparkLowLevel.MotorType.kBrushless);
        pivotMotorRight = new SparkMax(SlapdownConstants.pivotMotorRightID, SparkLowLevel.MotorType.kBrushless);
        motorRoller = new SparkMax(SlapdownConstants.rollerMotorID, SparkLowLevel.MotorType.kBrushless);
        encoderLeft = pivotMotorLeft.getEncoder();
        throughBore = pivotMotorLeft.getAbsoluteEncoder();
        pivotMotorConfigLeft = new SparkMaxConfig();
        pivotMotorConfigRight = new SparkMaxConfig();
        rollerMotorConfig = new SparkMaxConfig();
        pivotMotorConfigLeft.inverted(SlapdownConstants.invertPivot);
        pivotMotorConfigLeft.idleMode(IdleMode.kCoast);
        pivotMotorConfigLeft.smartCurrentLimit(SlapdownConstants.pivotCurrentLimit);
        pivotMotorConfigLeft.encoder.positionConversionFactor(360.0/96);
        pivotMotorConfigLeft.absoluteEncoder.positionConversionFactor(360);
        pivotMotorConfigLeft.absoluteEncoder.inverted(true);
        pivotMotorConfigLeft.closedLoop.pid(SlapdownConstants.pivotP, SlapdownConstants.pivotI, SlapdownConstants.pivotD);
        pivotMotorConfigLeft.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        //pivotMotorConfigLeft.closedLoop.maxMotion.cruiseVelocity(1500).maxAcceleration(3000);
        
        rollerMotorConfig.inverted(SlapdownConstants.invertRoller);
        rollerMotorConfig.idleMode(IdleMode.kCoast);
        rollerMotorConfig.closedLoop.pid(SlapdownConstants.rollerP, SlapdownConstants.rollerI, SlapdownConstants.rollerD);
        rollerMotorConfig.smartCurrentLimit(SlapdownConstants.rollerCurrentLimit);
        // isRollerEnabled = false;
        pivotMotorConfigRight.follow(pivotMotorLeft, true);
        rollerMotorConfig.encoder.velocityConversionFactor(3);

        pivotMotorLeft.configure(pivotMotorConfigLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivotMotorRight.configure(pivotMotorConfigRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorRoller.configure(rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pidPivotLeft = pivotMotorLeft.getClosedLoopController();

        rollerPid = motorRoller.getClosedLoopController();

        setPivotAngle(SlapdownConstants.pivotStowAngle);

        SmartDashboard.setDefaultBoolean("Intake/Intake Running", false);
        SmartDashboard.setDefaultBoolean("Intake/Intake Reversed", false);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake/Intake Pivot Position Absolute", throughBore.getPosition());
    SmartDashboard.putNumber("Intake/Intake Pivot Position Relative", encoderLeft.getPosition());
    SmartDashboard.putNumber("Left current:", pivotMotorLeft.getOutputCurrent());
    SmartDashboard.putNumber("Right current", pivotMotorRight.getOutputCurrent());
    SmartDashboard.putNumber("Intake/Intake Velocity", motorRoller.getEncoder().getVelocity());
    SmartDashboard.putNumber("Intake/Intake setpoint", rollerPid.getSetpoint());
  }

  public void toggleRoller() {
    setRoller(!isRollerEnabled);
  }

  public void setRoller(boolean enabled) {
    if (enabled) {
      rollerPid.setSetpoint(SlapdownConstants.rollerVelocity, SparkBase.ControlType.kVelocity);
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
      rollerPid.setSetpoint(-SlapdownConstants.rollerVelocity, SparkBase.ControlType.kVelocity);
    }
    else {
      stopRoller();
    }
    isRollerEnabled = enabled;
    SmartDashboard.putBoolean("Intake/Intake Running", isRollerEnabled);
  }

  public void setPivotAngle(double angle) {
    pidPivotLeft.setSetpoint(angle, ControlType.kPosition);
  }

  public void stop() {
    //setRoller(false);
    stopPivot();
  }

  public void stopPivot() {
    pivotMotorLeft.stopMotor();
  }

  public void stopRoller() {
    motorRoller.stopMotor();
  }

  public double getAngle() {
    return throughBore.getPosition();
  }

  public Command stowCommand() {
    return runOnce (
            () -> {
                setPivotAngle(Constants.SlapdownConstants.pivotStowAngle);
            }
    );
  }
  
  public Command l1Command() {
    return runOnce (
            () -> {
                setPivotAngle(Constants.SlapdownConstants.pivotL1Angle);
            }
    );
  }

  
  public Command transferCommand() {
    return runOnce (
            () -> {
                setPivotAngle(Constants.SlapdownConstants.pivotTransferAngle);
            }
    );
  }

  
  public Command groundCommand() {
    return runOnce (
            () -> {
                setPivotAngle(Constants.SlapdownConstants.pivotGroundAngle);
            }
    );
  }

  public Command reverseRollerCommand() {
    return run (
            () -> {
                setPivotAngle(Constants.SlapdownConstants.pivotGroundAngle);
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
