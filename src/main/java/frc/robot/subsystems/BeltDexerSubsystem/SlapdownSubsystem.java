package frc.robot.subsystems.BeltDexerSubsystem;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SlapdownConstants;

public class SlapdownSubsystem extends SubsystemBase {
    private final SparkMax motorPivot;
    private final SparkMax motorRoller;
    private final SparkClosedLoopController pidPivot;
    private final SparkMaxConfig pivotMotorConfig;
    private final SparkMaxConfig rollerMotorConfig;    
    private boolean isRollerEnabled;
    
    public enum PivotState {
        RAISING,
        LOWERING
    }
    public PivotState pivotState = PivotState.RAISING;

    public SlapdownSubsystem() {
        motorPivot = new SparkMax(SlapdownConstants.pivotMotorID, SparkLowLevel.MotorType.kBrushless);
        motorRoller = new SparkMax(SlapdownConstants.rollerMotorID, SparkLowLevel.MotorType.kBrushless);
        pivotMotorConfig = new SparkMaxConfig();
        rollerMotorConfig = new SparkMaxConfig();

        pivotMotorConfig.inverted(SlapdownConstants.invertPivot);
        pivotMotorConfig.idleMode(IdleMode.kCoast);
        pivotMotorConfig.smartCurrentLimit(SlapdownConstants.rollerCurrentLimit);
        pivotMotorConfig.closedLoop.pid(SlapdownConstants.pivotP, SlapdownConstants.pivotI, SlapdownConstants.pivotD);

        rollerMotorConfig.inverted(SlapdownConstants.invertRoller);
        rollerMotorConfig.idleMode(IdleMode.kCoast);
        rollerMotorConfig.smartCurrentLimit(SlapdownConstants.rollerCurrentLimit);

        motorPivot.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorRoller.configure(rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pidPivot = motorPivot.getClosedLoopController();

        SmartDashboard.setDefaultString("Intake/Pivot State", pivotState.toString());
        SmartDashboard.setDefaultBoolean("Intake/Intake Running", false);
        SmartDashboard.setDefaultBoolean("Intake/Intake Reversed", false);
  }

  @Override
  /* Periodically raises/lowers the pivot depending on its current state. Will not run if in lowered/lowering state. */
  public void periodic() {
    
  }

  public void setPivotState(PivotState newState){
    pivotState = newState;
    SmartDashboard.putString("Intake/Pivot State", pivotState.toString());
    switch (newState){
      case RAISING:
        setPivotAngle(SlapdownConstants.pivotRaisedAngle);
        break;
      case LOWERING:
        setPivotAngle(SlapdownConstants.pivotLoweringAngle);
        break;
      default:
        break;
    }
  }

  public void toggleRoller() {
    setRoller(!isRollerEnabled);
  }

  public void setRoller(boolean enabled) {
    if (enabled) {
      motorRoller.set(5); //change ts
    } 
    else {
      stopRoller();
    }
    isRollerEnabled = enabled;
    SmartDashboard.putBoolean("Intake/Intake Running", isRollerEnabled);
  }

  //make this later
  public void setRollerReversed(boolean enabled) {

  }

  public void setPivotAngle(double angle) {
    pidPivot.setSetpoint(angle, ControlType.kPosition);
  }

  public void stop() {
    setRoller(false);
    stopPivot();
  }

  /* Set the pivot motor speed to 0. */
  public void stopPivot() {
    motorPivot.stopMotor();
  }

  /* Set the roller motor speed to 0. */
  public void stopRoller() {
    motorRoller.stopMotor();
  }

  /**
   * Enable the intake roller. If the intake is raised, it will lower and then start.
   *
   * @return a command to toggle the intake
   */
  public Command intakeCommand() {
    // Inline construction of command goes here.
    return startEnd(
            () -> {
              if (pivotState == PivotState.LOWERING) {
                setRoller(true);
              }
              else {
                setRoller(true);
                setPivotState(PivotState.LOWERING);
              }
            },
            () -> setRoller(false));
  }

  /**
   * Toggle the intake roller. If the intake is raised, it will lower and then start.
   *
   * @return a command to toggle the intake
   */
  public Command toggleIntakeCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          if (pivotState == PivotState.LOWERING) {
            toggleRoller();
          }
          else {
            setRoller(true);
            setPivotState(PivotState.LOWERING);
          }
        });
  }
  
  /**
   * Reverse the intake roller while held
   *
   * @return a command to reverse the intake
   */
  public Command reverseIntakeCommand() {
    return startEnd(
      () -> setRollerReversed(true), 
      () -> setRollerReversed(false)
    );
  }

  public Command resetIntakeCommand() {
    // Subsystem: RunOnce implicitly requires this subsystem.
    return runOnce(
        () -> {
          setRoller(false);
          setPivotState(PivotState.RAISING);
        });
  }

  @Override
  public void simulationPeriodic() {

  }
}
