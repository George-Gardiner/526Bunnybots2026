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
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;

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
    private double lastPivotFeedforward = 0;

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
        pivotMotorConfigLeft.encoder.positionConversionFactor(SlapdownConstants.pivotRelativeEncoderPositionConversionFactor); //gear ratio was 96 is now 60
        pivotMotorConfigLeft.encoder.velocityConversionFactor(SlapdownConstants.pivotRelativeEncoderVelocityConversionFactor);
        pivotMotorConfigLeft.absoluteEncoder.positionConversionFactor(SlapdownConstants.pivotAbsoluteEncoderPositionConversionFactor);
        pivotMotorConfigLeft.absoluteEncoder.inverted(true);
        pivotMotorConfigLeft.absoluteEncoder.averageDepth(SlapdownConstants.pivotAbsEncoderAverageDepth);
        pivotMotorConfigLeft.closedLoop.pid(SlapdownConstants.pivotP, SlapdownConstants.pivotI, SlapdownConstants.pivotD);
        pivotMotorConfigLeft.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        //pivotMotorConfigLeft.closedLoop.maxMotion.cruiseVelocity(1500).maxAcceleration(3000);
         
        rollerMotorConfig.inverted(SlapdownConstants.invertRoller);
        rollerMotorConfig.idleMode(IdleMode.kCoast);
        rollerMotorConfig.closedLoop.pid(SlapdownConstants.rollerP, SlapdownConstants.rollerI, SlapdownConstants.rollerD);
        rollerMotorConfig.smartCurrentLimit(SlapdownConstants.rollerCurrentLimit);
        // isRollerEnabled = false;
        pivotMotorConfigRight.follow(pivotMotorLeft, true);
        rollerMotorConfig.encoder.velocityConversionFactor(SlapdownConstants.rollerRelativeEncoderVelocityConversionFactor);

        pivotMotorLeft.configure(pivotMotorConfigLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivotMotorRight.configure(pivotMotorConfigRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorRoller.configure(rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pidPivotLeft = pivotMotorLeft.getClosedLoopController();

        rollerPid = motorRoller.getClosedLoopController();

        //setPivotAngle(SlapdownConstants.pivotStowAngle);

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
    SmartDashboard.putNumber("Intake/pivot setPoint", pidPivotLeft.getSetpoint());
    SmartDashboard.putNumber("Intake/Intake pivot feedforward", lastPivotFeedforward);

    // Pivot telemetry (AdvantageScope: /SmartDashboard/Pivot/*). Compare Position against
    // "Intake Pivot Position Relative" to see the drivetrain slack.
    SmartDashboard.putNumber("Pivot/Position", throughBore.getPosition());
    SmartDashboard.putNumber("Pivot/Setpoint", pidPivotLeft.getSetpoint());
    SmartDashboard.putNumber("Pivot/AppliedVolts", pivotMotorLeft.getAppliedOutput() * pivotMotorLeft.getBusVoltage());
    SmartDashboard.putNumber("Pivot/FeedforwardVolts", lastPivotFeedforward);
    SmartDashboard.putNumber("Pivot/VelocityDegPerSec", encoderLeft.getVelocity() * 360.0 / 60.0);
    SmartDashboard.putNumber("Pivot/LeftCurrent", pivotMotorLeft.getOutputCurrent());
    SmartDashboard.putNumber("Pivot/RightCurrent", pivotMotorRight.getOutputCurrent());
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
    double currentAngle = throughBore.getPosition();
    double feedForward = -Math.cos((currentAngle - SlapdownConstants.horizontalAngle)/SlapdownConstants.pivotAbsoluteEncoderPositionConversionFactor * 2 * Math.PI) * SlapdownConstants.pivotKcos;
    // Static-friction kick toward the setpoint; P alone is too weak to break the ~0.2-0.35 V friction band
    double error = angle - currentAngle;
    if (Math.abs(error) > SlapdownConstants.pivotKsDeadband) {
      feedForward += Math.signum(error) * SlapdownConstants.pivotKs;
    }
    lastPivotFeedforward = feedForward;
    pidPivotLeft.setSetpoint(angle, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedForward, ArbFFUnits.kVoltage);
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
    return run (
            () -> {
                setPivotAngle(Constants.SlapdownConstants.pivotStowAngle);
            }
    );
  }
  
  public Command l1Command() {
    return run (
            () -> {
                setPivotAngle(Constants.SlapdownConstants.pivotL1Angle);
            }
    );
  }

  
  public Command transferCommand() {
    return run(
            () -> {
                setPivotAngle(Constants.SlapdownConstants.pivotTransferAngle);
            }
    );
  }

  
  public Command groundCommand() {
    return run (
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
