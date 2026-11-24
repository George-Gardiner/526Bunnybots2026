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
import frc.robot.Constants;
import frc.robot.Constants.SlapdownConstants;

public class SlapdownSubsystem extends SubsystemBase {
    private final SparkMax motorPivotLeft;
    private final SparkMax motorPivotRight;
    private final SparkMax motorRoller;
    private final SparkClosedLoopController pidPivotLeft;
    private final SparkClosedLoopController pidPivotRight;
    private final SparkMaxConfig pivotMotorConfigLeft;
    private final SparkMaxConfig pivotMotorConfigRight;
    private final SparkMaxConfig  rollerMotorConfig;    
    private boolean isRollerEnabled;
    AbsoluteEncoder throughBore;
    RelativeEncoder encoderLeft;

    public SlapdownSubsystem() {
        motorPivotLeft = new SparkMax(SlapdownConstants.pivotMotorLeftID, SparkLowLevel.MotorType.kBrushless);
        motorPivotRight = new SparkMax(SlapdownConstants.pivotMotorRightID, SparkLowLevel.MotorType.kBrushless);
        motorRoller = new SparkMax(SlapdownConstants.rollerMotorID, SparkLowLevel.MotorType.kBrushless);
        encoderLeft = motorPivotLeft.getEncoder();
        pivotMotorConfigLeft = new SparkMaxConfig();
        pivotMotorConfigRight = new SparkMaxConfig();
        rollerMotorConfig = new SparkMaxConfig();
        try {
           throughBore = motorPivotLeft.getAbsoluteEncoder();
        } 
        catch (IllegalStateException e) {
          System.out.println(e.getMessage());
        }
        pivotMotorConfigLeft.inverted(SlapdownConstants.invertPivotLeft);
        pivotMotorConfigLeft.idleMode(IdleMode.kCoast);
        pivotMotorConfigLeft.smartCurrentLimit(SlapdownConstants.pivotCurrentLimit);
        pivotMotorConfigLeft.closedLoop.pid(SlapdownConstants.pivotP, SlapdownConstants.pivotI, SlapdownConstants.pivotD);
        pivotMotorConfigLeft.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        //pivotMotorConfigLeft.closedLoop.maxMotion.cruiseVelocity(1500).maxAcceleration(3000);

        rollerMotorConfig.inverted(SlapdownConstants.invertRoller);
        rollerMotorConfig.idleMode(IdleMode.kCoast);
        rollerMotorConfig.smartCurrentLimit(SlapdownConstants.rollerCurrentLimit);

        pivotMotorConfigRight.follow(motorPivotLeft, true);

        motorPivotLeft.configure(pivotMotorConfigLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorPivotRight.configure(pivotMotorConfigRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorRoller.configure(rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        pidPivotLeft = motorPivotLeft.getClosedLoopController();
        pidPivotRight = motorPivotRight.getClosedLoopController();

        SmartDashboard.setDefaultBoolean("Is Intake Lowering/Lowered?", false);
        SmartDashboard.setDefaultBoolean("Intake/Intake Running", false);
        SmartDashboard.setDefaultBoolean("Intake/Intake Reversed", false);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pivot Angle Absolute:", throughBore.getPosition());
    SmartDashboard.putNumber("Pivot Angle Relative:", encoderLeft.getPosition());
    SmartDashboard.putNumber("Left current:", motorPivotLeft.getOutputCurrent());
    SmartDashboard.putNumber("Right current", motorPivotRight.getOutputCurrent());
  }

  public void movePivot(boolean isLowering) {
   
  }

  public void setRoller(boolean enabled) {
    if (enabled) {
      motorRoller.set(SlapdownConstants.rollerDutyCycleSpeed); //change ts
    } 
    else {  
      stopRoller();
    }
    SmartDashboard.putBoolean("Intake/Intake Running", isRollerEnabled);
  }

  //make this later
  public void setRollerReversed(boolean enabled) {
    motorPivotLeft.set(-SlapdownConstants.rollerDutyCycleSpeed);
  }

  public void setPivotAngle(double angle) {
    pidPivotLeft.setSetpoint(angle, ControlType.kPosition);
  }

  public void stop() {
    setRoller(false);
    stopPivot();
  }

  public void stopPivot() {
    motorPivotLeft.stopMotor();
    motorPivotRight.stopMotor();
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
                setPivotAngle(Constants.SlapdownConstants.pivotl1Angle);
            }
    );
  }

  
  public Command transferCommand() {
    return run (
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

  
  public Command reverseIntakeCommand() {
    return startEnd(
      () -> setRollerReversed(true), 
      () -> setRollerReversed(false)
    );
  }


  @Override
  public void simulationPeriodic() {

  }
}
