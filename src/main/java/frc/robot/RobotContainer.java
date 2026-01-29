// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera1;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.goals.RobotGoals;
import frc.robot.goals.RobotGoalsBehavior;
import frc.robot.operator.OperatorIntent;
import frc.robot.state.MatchState;
import frc.robot.subsystems.climber.ClimberBehavior;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.IndexerBehavior;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeBehavior;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterBehavior;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.AllEvents;
import frc.robot.util.GoalBehavior;
import frc.robot.util.SubsystemBehavior;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final AprilTagVision vision;

  // Subsystems
  private final Drive drive;
  private final double DRIVE_SPEED = 0.55;
  private final double ANGULAR_SPEED = 0.55;

  private final IntakeSubsystem intake;
  private final IndexerSubsystem indexer;
  private final ClimberSubsystem climber;
  private final ShooterSubsystem shooter;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController characterizeController = new CommandXboxController(4);

  // Reactive architecture components
  private final OperatorIntent operatorIntent;
  private final MatchState matchState;
  private final RobotGoals robotGoals;

  private boolean m_teleopInitialized = false;
  private AutoCommandManager autoCommandManager;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    CANBus canbus = new CANBus("rio");
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        // TODO add TalonFX
        intake =
            new IntakeSubsystem(
                new IntakeIO() {

                  @Override
                  public void setIntakerTarget(Voltage target) {}

                  @Override
                  public void stop() {}

                  @Override
                  public void updateInputs(IntakeInputs input) {}
                });
        climber =
            new ClimberSubsystem(
                new ClimberIO() {

                  @Override
                  public void setClimberHeight(Distance target) {}

                  @Override
                  public void updateInputs(ClimberInputs input) {}
                });
        shooter = new ShooterSubsystem(new ShooterIOTalonFX(20, 10, 14, canbus));
        indexer =
            new IndexerSubsystem(new IndexerIOTalonFX(67, canbus)); // TODO: find real motor ID

        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));
        vision =
            new AprilTagVision(
                drive::setPose,
                drive::addVisionMeasurementAutoAlign,
                new VisionIOLimelight(camera0Name, drive::getRotation),
                new VisionIOLimelight(camera1Name, drive::getRotation));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision =
            new AprilTagVision(
                drive::setPose,
                drive::addVisionMeasurementAutoAlign,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
        intake = new IntakeSubsystem(new IntakeIOSim());
        indexer = new IndexerSubsystem(new IndexerIOSim());
        climber =
            new ClimberSubsystem(
                new ClimberIOSim(
                    new ElevatorSim(
                        LinearSystemId.createElevatorSystem(
                            DCMotor.getKrakenX60Foc(2),
                            Pounds.of(45).in(Kilograms),
                            Inches.of(ClimberSubsystem.SPOOL_RADIUS).in(Meters),
                            ClimberSubsystem.REDUCTION),
                        DCMotor.getKrakenX60Foc(2),
                        Inches.of(0).in(Meters),
                        Inches.of(32).in(Meters),
                        true,
                        Inches.of(0).in(Meters))));
        shooter = new ShooterSubsystem(new ShooterIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision =
            new AprilTagVision(
                drive::setPose,
                drive::addVisionMeasurementAutoAlign,
                new VisionIO() {},
                new VisionIO() {});
        intake = new IntakeSubsystem(null);
        indexer = new IndexerSubsystem(new IndexerIOTalonFX(0, canbus)); // TODO: find real motor ID
        climber = new ClimberSubsystem(null);
        shooter = new ShooterSubsystem(null);
        break;
    }

    autoCommandManager = new AutoCommandManager(drive);

    // Initialize reactive architecture
    operatorIntent = new OperatorIntent(0);
    matchState = new MatchState();
    robotGoals = new RobotGoals();

    // Create goal behaviors (wires operator intent → robot goals)
    new RobotGoalsBehavior(robotGoals);
    new IndexerBehavior(indexer);
    new IntakeBehavior(intake);
    new ShooterBehavior(shooter);
    new ClimberBehavior(climber);

    // TODO (students): Create subsystem behaviors here, e.g.:
    // new intakeBehavior(intake);
    // new DriveBehavior(drive);

    // Configure all behaviors
    GoalBehavior.configureAll(operatorIntent);
    SubsystemBehavior.configureAll(
        new AllEvents(robotGoals, matchState, indexer, shooter, intake, climber));

    // Configure the button bindings
    configureButtonBindings();
    configureCharacterizationButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY() * DRIVE_SPEED,
            () -> -controller.getLeftX() * DRIVE_SPEED,
            () -> -controller.getRightX() * ANGULAR_SPEED));

    // // Lock to 0° when A button is held
    // controller
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    controller.a().whileTrue(shooter.shooterCommand()).whileFalse(shooter.idleCommand());
    operatorIntent.wantsToClimbL1().whileTrue(climber.goToL1Command());
    operatorIntent.wantsToClimbL2().whileTrue(climber.goToL2Command());
    operatorIntent.wantsToClimbL3().whileTrue(climber.goToL3Command());

    // Reset gyro to 0° when B button is pressed
    // controller
    //     .b()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     drive.setPose(
    //                         new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
    //                 drive)
    //             .ignoringDisable(true));
  }

  public void configureCharacterizationButtonBindings() {
    characterizeController
        .back()
        .and(characterizeController.y())
        .whileTrue(drive.sysIdDynamic(Direction.kForward));
    characterizeController
        .back()
        .and(characterizeController.x())
        .whileTrue(drive.sysIdDynamic(Direction.kReverse));
    characterizeController
        .start()
        .and(characterizeController.y())
        .whileTrue(drive.sysIdQuasistatic(Direction.kForward));
    characterizeController
        .start()
        .and(characterizeController.x())
        .whileTrue(drive.sysIdQuasistatic(Direction.kReverse));

    characterizeController
        .povUp()
        .whileTrue(DriveCommands.wheelRadiusCharacterization(drive))
        .onFalse(DriveCommands.brakeDrive(drive));

    characterizeController
        .a()
        .onTrue(
            new InstantCommand(
                () -> {
                  SignalLogger.setPath("/media/sda1/logs");
                  // SignalLogger.enableAutoLogging(true);
                  SignalLogger.start();
                  System.out.println("Started Logger");
                }));
    characterizeController
        .b()
        .onTrue(
            new InstantCommand(
                () -> {
                  SignalLogger.stop();
                  System.out.println("Stopped Logger");
                }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command autoCommand = autoCommandManager.getAutonomousCommand();
    // turnoff updating odometry based on april tags
    vision.enableUpdateOdometryBasedOnApriltags();
    if (autoCommand != null) {
      // tell vision autonomous path is updated
      vision.updateAutonomous();
    }
    return autoCommand;
  }

  public void teleopInit() {
    if (!this.m_teleopInitialized) {
      vision.updateStartingPosition();
      vision.enableUpdateOdometryBasedOnApriltags();
      m_teleopInitialized = true;
      // TODO uncomment
      //   SignalLogger.setPath("/media/sda1/");
      //   SignalLogger.start();
    }
  }
}
