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
import static frc.robot.subsystems.vision.VisionConstants.questCamName;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera1;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.state.GameData;
import frc.robot.state.MatchState;
import frc.robot.subsystems.climber.ClimberBehavior;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.hood.HoodBehavior;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.hood.HoodIOTalonFX;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.indexer.IndexerBehavior;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeBehavior;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterBehavior;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretBehavior;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretIOTalonFX;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.vision.VisionIOQuest;
import frc.robot.util.AllEvents;
import frc.robot.util.GoalBehavior;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.SubsystemBehavior;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.littletonrobotics.junction.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Set to true when Testing Individual subsystems
  // This should stay false otherwise
  private static final boolean ISTESTING = false;

  private final AprilTagVision vision;

  // Subsystems
  private final Drive drive;
  private SwerveDriveSimulation driveSimulation = null;
  private final double DRIVE_SPEED = 0.55;
  private final double ANGULAR_SPEED = 0.55;

  private final IntakeSubsystem intake;
  private final IndexerSubsystem indexer;
  private final ClimberSubsystem climber;
  private final ShooterSubsystem shooter;
  private final TurretSubsystem turret;
  private final HoodSubsystem hood;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController testController = new CommandXboxController(3);
  private final CommandXboxController characterizeController = new CommandXboxController(4);

  // Reactive architecture components
  private final OperatorIntent operatorIntent;
  private final MatchState matchState;
  private final RobotGoals robotGoals;
  private final GameData gameData;

  private boolean m_teleopInitialized = false;
  private AutoCommandManager autoCommandManager;

  final LoggedTunableNumber setIndexerVelocity =
      new LoggedTunableNumber("RobotState/Indexer/setVelocity", 2);
  final LoggedTunableNumber setTurretAngle =
      new LoggedTunableNumber("RobotState/Turret/setAngle", 45);
  final LoggedTunableNumber setShooterSpeed =
      new LoggedTunableNumber("RobotState/Shooter/setSpeed", 87);
  final LoggedTunableNumber setIntakeVolts =
      new LoggedTunableNumber("RobotState/Intake/setVolts", 2);
  final LoggedTunableNumber setIntakeExtenderVolts =
      new LoggedTunableNumber("RobotState/IntakeExtender/setVolts", 2);
  final LoggedTunableNumber setHoodAngle = new LoggedTunableNumber("RobotState/Hood/setAngle", 45);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    CANBus rioCanbus = new CANBus("rio");
    CANBus upperCanbus = new CANBus("upperCAN");
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
                new ModuleIOTalonFX(TunerConstants.BackRight),
                (robotPose) -> {});
        intake = new IntakeSubsystem(new IntakeIOTalonFX(1, 2, 3, upperCanbus));
        climber = new ClimberSubsystem(new ClimberIO() {}); // TODO: Implement Climber
        shooter = new ShooterSubsystem(new ShooterIOTalonFX(4, 5, 6, 10, upperCanbus));
        indexer = new IndexerSubsystem(new IndexerIOTalonFX(8, 9, upperCanbus));
        turret =
            new TurretSubsystem(
                new TurretIOTalonFX(7, 1, 2, upperCanbus),
                drive::getAutoAlignPose,
                new Pose2d(0.0, 0.0, new Rotation2d()));
        hood = new HoodSubsystem(new HoodIOTalonFX(11, upperCanbus));

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
                new VisionIOLimelight(camera1Name, drive::getRotation),
                new VisionIOQuest(drive::getAutoAlignPose, questCamName));
        break;

      case SIM:
        driveSimulation =
            new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.overrideInstance(new Arena2026Rebuilt(false));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOSim(driveSimulation.getModules()[0]),
                new ModuleIOSim(driveSimulation.getModules()[1]),
                new ModuleIOSim(driveSimulation.getModules()[2]),
                new ModuleIOSim(driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);
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
        turret =
            new TurretSubsystem(
                new TurretIOSim(), drive::getAutoAlignPose, new Pose2d(0.0, 0.0, new Rotation2d()));
        hood = new HoodSubsystem(new HoodIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                (robotPose) -> {});
        vision =
            new AprilTagVision(
                drive::setPose,
                drive::addVisionMeasurementAutoAlign,
                new VisionIO() {},
                new VisionIO() {});
        intake = new IntakeSubsystem(new IntakeIO() {});
        indexer = new IndexerSubsystem(new IndexerIO() {});
        climber = new ClimberSubsystem(new ClimberIO() {});
        shooter = new ShooterSubsystem(new ShooterIO() {});
        turret =
            new TurretSubsystem(
                new TurretIO() {}, drive::getAutoAlignPose, new Pose2d(0.0, 0.0, new Rotation2d()));
        hood = new HoodSubsystem(new HoodIO() {});
        break;
    }

    autoCommandManager = new AutoCommandManager(drive, RobotGoals.getInstance());

    // Initialize reactive architecture
    operatorIntent = OperatorIntent.getInstance(0);
    matchState = MatchState.getInstance();
    robotGoals = RobotGoals.getInstance();
    gameData = GameData.getInstance();

    // Create goal behaviors (wires operator intent → robot goals)
    new RobotGoalsBehavior(robotGoals);
    new IndexerBehavior(indexer);
    new IntakeBehavior(intake);
    new ShooterBehavior(shooter);
    new ClimberBehavior(climber);
    new HoodBehavior(hood);
    new TurretBehavior(turret);

    // TODO (students): Create subsystem behaviors here, e.g.:
    // new intakeBehavior(intake);
    // new DriveBehavior(drive);

    // Configure all behaviors
    GoalBehavior.configureAll(operatorIntent);

    // Configure the button bindings
    configureButtonBindings(ISTESTING);
    configureCharacterizationButtonBindings();

    // TODO Examples of use of GameData in Behaviors (REMOVE Once find a use or merged into master
    // and used in Behaviors)
    gameData
        .isActiveShootingTrigger()
        .onTrue(new InstantCommand(() -> Logger.recordOutput("ACTIVESHOOTING", true)))
        .onFalse(new InstantCommand(() -> Logger.recordOutput("ACTIVESHOOTING", false)));
    gameData
        .m_state
        .is(GameData.GameDataStates.SHIFT1)
        .onTrue(new InstantCommand(() -> Logger.recordOutput("S1", true)))
        .onFalse(new InstantCommand(() -> Logger.recordOutput("S1", false)));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings(boolean isTesting) {
    if (isTesting) {
      configureTestButtonBindings();
    } else {
      SubsystemBehavior.configureAll(
          new AllEvents(robotGoals, matchState, gameData, indexer, shooter, intake, climber, hood));
      gameData.isActiveShootingTrigger();
    }
    // Reset gyro / odometry
    final Runnable resetOdometry =
        Constants.currentMode == Constants.Mode.SIM
            ? () -> drive.setPose(driveSimulation.getSimulatedDriveTrainPose())
            : () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY() * DRIVE_SPEED,
            () -> -controller.getLeftX() * DRIVE_SPEED,
            () -> -controller.getRightX() * ANGULAR_SPEED));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Maple-Sim Button Bindings
    // // Spawns Fuel
    // controller
    //     .povUp()
    //     .onTrue(
    //         Commands.runOnce(
    //             () ->
    //                 SimulatedArena.getInstance()
    //                     .addGamePieceProjectile(
    //                         new RebuiltFuelOnFly(
    //                             driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
    //                             new Translation2d(0.4, 0),
    //
    // driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
    //                             driveSimulation.getSimulatedDriveTrainPose().getRotation(),
    //                             Meters.of(1.35),
    //                             MetersPerSecond.of(1.5),
    //                             Degrees.of(-60)))));

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

  public void configureTestButtonBindings() {
    intake.setTestingState();
    shooter.setTestingState();
    indexer.setTestingState();
    turret.setTestingState();
    climber.setTestingState();
    hood.setTestingState();
    testController
        .a()
        .whileTrue(indexer.getNewSetIndexerVelocityCommand(setIndexerVelocity))
        .whileFalse(indexer.getNewSetIndexerVelocityCommand(() -> 0.0));
    testController
        .x()
        .whileTrue(turret.getNewSetTurretAngleCommand(setTurretAngle))
        .whileFalse(turret.getNewSetTurretAngleCommand(() -> 0.0));
    testController
        .b()
        .whileTrue(shooter.getNewSetShooterSpeedCommand(setShooterSpeed))
        .whileFalse(shooter.getNewSetShooterSpeedCommand(() -> 0.0));
    testController
        .y()
        .whileTrue(intake.getNewSetIntakeVoltsCommand(setIntakeVolts))
        .whileFalse(intake.getNewSetIntakeVoltsCommand(() -> 0.0));
    testController
        .povUp()
        .whileTrue(intake.getNewSetIntakeExtenderVoltsCommand(setIntakeExtenderVolts))
        .whileFalse(
            intake.getNewSetIntakeExtenderVoltsCommand(
                () -> -2.0)); // Default value for intake extender volts
    testController
        .povRight()
        .whileTrue(hood.getNewSetHoodAngleCommand(setHoodAngle))
        .whileFalse(hood.getNewSetHoodAngleCommand(() -> 0.0));
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

  public void resetSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    // drive.setPose(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
    if (DriverStation.isDisabled()) {
      // Disable AprilTags when disabled
      vision.disableUpdateOdometryBasedOnApriltags();
    }
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
  }

  public void loggingPeriodic() {
    if (Constants.currentMode != Constants.Mode.SIM) return;
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
  }
}
