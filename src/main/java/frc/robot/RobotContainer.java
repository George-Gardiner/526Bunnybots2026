package frc.robot;
import edu.wpi.first.wpilibj2.command.button.CommandStadiaController;
//testing testing Josie was here
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.BeltDexerSubsystem.BeltDexerSubsystem;
import yams.mechanisms.swerve.utility.SwerveInputStream;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer
{

  final CommandXboxController driverXbox = new CommandXboxController(0);

  private final SwerveDriveSubsystem swerve = new SwerveDriveSubsystem();
  private final BeltDexerSubsystem indexer = new BeltDexerSubsystem();

  private final SwerveInputStream driveAngularVelocity =
      // swerve.getAngularVelocityStream(
      //           driverXbox::getLeftY,
      //           () -> driverXbox.getLeftX(),
      //           () -> driverXbox.getRawAxis(2))
      //       .withAllianceRelativeControl();
      SwerveInputStream.of(swerve.getSwerveDrive(), () -> driverXbox.getLeftY(), () -> driverXbox.getLeftX()).withControllerRotationAxis(() -> -driverXbox.getRightX())
      .withAllianceRelativeControl()
      .deadband(0.1);

  public RobotContainer()
  {
    configureBindings();
  }

  private void configureBindings()
  {
    RobotModeTriggers.test().whileTrue(Commands.idle(swerve).withName("Live Tuning"));
    // set a default command. This is the command that runs if no other command is set
    swerve.setDefaultCommand(swerve.drive(driveAngularVelocity));
  //indexer.setDefaultCommand(Commands.run(() -> { indexer.setMotorAPower(0); indexer.setMotorBPower(0); }, indexer));
    // When a or y button is pressed, add these command to the command sceduler
    driverXbox.y().onTrue(swerve.zeroGyro()); // swerve.zeroGyro() returns a command
    driverXbox.a().toggleOnTrue(indexer.runDexer());
    driverXbox.b().onTrue(swerve.setWheelsZero());
  }
}