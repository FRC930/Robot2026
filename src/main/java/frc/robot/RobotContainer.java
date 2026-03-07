// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static frc.robot.subsystems.vision.VisionConstants.backLeftCamera;
import static frc.robot.subsystems.vision.VisionConstants.frontLeftCamera;
import static frc.robot.subsystems.vision.VisionConstants.frontLeftForwardCamera;
import static frc.robot.subsystems.vision.VisionConstants.frontRightCamera;
import static frc.robot.subsystems.vision.VisionConstants.questCamName;
import static frc.robot.subsystems.vision.VisionConstants.robotToBackLeftCamera;
import static frc.robot.subsystems.vision.VisionConstants.robotToFrontLeftCamera;
import static frc.robot.subsystems.vision.VisionConstants.robotToFrontLeftForwardCamera;
import static frc.robot.subsystems.vision.VisionConstants.robotToFrontRightCamera;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
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
import frc.robot.aiming.AimingBehavior;
import frc.robot.aiming.AimingConstants;
import frc.robot.aiming.AimingService;
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
import frc.robot.subsystems.drive.DriveBehavior;
import frc.robot.subsystems.drive.DriveZoneTracker;
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
import frc.robot.util.HighFrequencyLoop;
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
  // 3D simulated visualization of 3d model
  private RobotVisualization robotVisualization = RobotVisualization.instance();

  // Set to true when Testing Individual subsystems
  // This should stay false otherwise
  private static final boolean ISTESTING = false;

  private final AprilTagVision vision;

  // Subsystems
  private final Drive drive;
  public static final SwerveDriveSimulation driveSimulation =
      new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));

  private final double REG_DRIVE_SPEED = 0.9;
  private final double REG_ANGULAR_SPEED = 0.75;

  private final double SLOW_DRIVE_SPEED = 0.5;

  private final IntakeSubsystem intake;
  private final IndexerSubsystem indexer;
  private final ClimberSubsystem climber;
  private final ShooterSubsystem shooter;
  private final TurretSubsystem turret;
  private final HoodSubsystem hood;
  private final AimingService aimingService;
  private final DriveZoneTracker driveZoneTracker;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController testController = new CommandXboxController(3);
  private final CommandXboxController characterizeController = new CommandXboxController(4);

  // Reactive architecture components
  private final OperatorIntent operatorIntent;
  private final MatchState matchState;
  private final RobotGoals robotGoals;

  private boolean m_teleopInitialized = false;
  private AutoCommandManager autoCommandManager;

  final LoggedTunableNumber setIndexerVelocity =
      new LoggedTunableNumber("RobotTesting/Indexer/setVelocity", 50.0);
  final LoggedTunableNumber setFeederVelocity =
      new LoggedTunableNumber("RobotTesting/Feeder/setVelocity", 50.0);
  final LoggedTunableNumber setTurretAngle =
      new LoggedTunableNumber("RobotTesting/Turret/setAngle", 45.0);
  final LoggedTunableNumber setShooterSpeed =
      new LoggedTunableNumber("RobotTesting/Shooter/setSpeed", 87);
  final LoggedTunableNumber setIntakeRPM =
      new LoggedTunableNumber("RobotTesting/Intake/setRPM", 1000);
  final LoggedTunableNumber setIntakeExtenderUp =
      new LoggedTunableNumber("RobotTesting/IntakeExtender/setAngleUP", 105.0);
  final LoggedTunableNumber setIntakeExtenderDown =
      new LoggedTunableNumber("RobotTesting/IntakeExtender/setAngleDOWN", 0.0);
  final LoggedTunableNumber setHoodAngle =
      new LoggedTunableNumber("RobotTesting/Hood/setAngle", 45.0);

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    // Initialize reactive architecture
    operatorIntent = OperatorIntent.getInstance(0);
    matchState = MatchState.getInstance();
    robotGoals = RobotGoals.getInstance();

    CANBus rioCanbus = new CANBus("rio");
    CANBus upperCanbus = new CANBus("upperCanbus");
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
        aimingService = new AimingService(drive::getLatestSnapshot);
        driveZoneTracker = new DriveZoneTracker(drive::getAutoAlignPose, drive::getChassisSpeeds);
        intake = new IntakeSubsystem(new IntakeIOTalonFX(1, 2, 3, upperCanbus));
        // intake = new IntakeSubsystem(new IntakeIO() {});

        climber = new ClimberSubsystem(new ClimberIO() {}); // TODO: Implement Climber

        shooter =
            new ShooterSubsystem(
                new ShooterIOTalonFX(4, 5, 6, 10, upperCanbus), aimingService::getShooterRPM);
        // shooter = new ShooterSubsystem(new ShooterIO() {}, aimingService::getShooterRPM);

        indexer = new IndexerSubsystem(new IndexerIOTalonFX(8, 9, upperCanbus));
        // indexer = new IndexerSubsystem(new IndexerIO() {});

        turret =
            new TurretSubsystem(
                new TurretIOTalonFX(7, 1, 2, upperCanbus), aimingService::getTurretAngleDeg);
        // turret = new TurretSubsystem(new TurretIO() {}, aimingService::getTurretAngleDeg);

        hood =
            new HoodSubsystem(new HoodIOTalonFX(11, upperCanbus), aimingService::getHoodAngleDeg);
        // hood = new HoodSubsystem(new HoodIO() {}, aimingService::getHoodAngleDeg);

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
                new VisionIOLimelight(frontLeftCamera, drive::getRotation),
                new VisionIOLimelight(frontRightCamera, drive::getRotation),
                new VisionIOLimelight(frontLeftForwardCamera, drive::getRotation),
                new VisionIOLimelight(backLeftCamera, drive::getRotation),
                new VisionIOQuest(drive::getAutoAlignPose, questCamName));
        break;

      case SIM:
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
                new VisionIOPhotonVisionSim(
                    frontLeftCamera, robotToFrontLeftCamera, drive::getPose),
                new VisionIOPhotonVisionSim(
                    frontRightCamera, robotToFrontRightCamera, drive::getPose),
                new VisionIOPhotonVisionSim(
                    frontLeftForwardCamera, robotToFrontLeftForwardCamera, drive::getPose),
                new VisionIOPhotonVisionSim(backLeftCamera, robotToBackLeftCamera, drive::getPose));
        aimingService = new AimingService(drive::getLatestSnapshot);
        driveZoneTracker = new DriveZoneTracker(drive::getPose, drive::getChassisSpeeds);
        intake = new IntakeSubsystem(new IntakeIOSim(driveSimulation));
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
        shooter = new ShooterSubsystem(new ShooterIOSim(), aimingService::getShooterRPM);
        turret = new TurretSubsystem(new TurretIOSim(), aimingService::getTurretAngleDeg);
        hood = new HoodSubsystem(new HoodIOSim(), aimingService::getHoodAngleDeg);
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
        aimingService = new AimingService(drive::getLatestSnapshot);
        driveZoneTracker = new DriveZoneTracker(drive::getAutoAlignPose, drive::getChassisSpeeds);
        intake = new IntakeSubsystem(new IntakeIO() {});
        indexer = new IndexerSubsystem(new IndexerIO() {});
        climber = new ClimberSubsystem(new ClimberIO() {});
        shooter = new ShooterSubsystem(new ShooterIO() {}, aimingService::getShooterRPM);
        turret = new TurretSubsystem(new TurretIO() {}, aimingService::getTurretAngleDeg);
        hood = new HoodSubsystem(new HoodIO() {}, aimingService::getHoodAngleDeg);
        break;
    }

    // Start 250Hz control threads for REAL and SIM (not REPLAY)
    if (Constants.currentMode != Constants.Mode.REPLAY) {
      double freq = AimingConstants.AIMING_FREQUENCY;

      new HighFrequencyLoop("AimingThread", freq, aimingService::computeAimingSolution).start();

      new HighFrequencyLoop(
              "TurretThread",
              freq,
              () -> {
                if (turret.shouldThreadCommand()) {
                  double angle = MathUtil.clamp(aimingService.getTurretAngleDeg(), -180.0, 180.0);
                  turret.getIO().setTarget(angle);
                }
              })
          .start();

      new HighFrequencyLoop(
              "ShooterThread",
              freq,
              () -> {
                if (shooter.shouldThreadCommand()) {
                  double rpm = aimingService.getShooterRPM();
                  if (rpm < AimingConstants.SHOOTER_MIN_RPM) {
                    rpm = shooter.getPrespinSetpoint();
                  }
                  shooter.getIO().setShooterTarget(RPM.of(rpm));
                }
              })
          .start();

      new HighFrequencyLoop(
              "HoodThread",
              freq,
              () -> {
                if (hood.shouldThreadCommand()) {
                  // Transform angle from 0 degrees vertical to 0 degree horizontal so like tranform
                  // the angle by 90 degrees
                  double hoodAngle = 90.0 - aimingService.getHoodAngleDeg();
                  hood.getIO().setHoodTarget(Degrees.of(hoodAngle));
                }
              })
          .start();
    }

    autoCommandManager = new AutoCommandManager(drive, RobotGoals.getInstance());

    // Create goal behaviors (wires operator intent → robot goals)
    new RobotGoalsBehavior(robotGoals);
    new DriveBehavior(drive);
    new IndexerBehavior(indexer);
    new IntakeBehavior(intake);
    new ShooterBehavior(shooter);
    new ClimberBehavior(climber);
    new HoodBehavior(hood);
    new TurretBehavior(turret);
    new AimingBehavior(aimingService);

    // Configure all behaviors
    GoalBehavior.configureAll(operatorIntent);

    // Configure the button bindings
    configureButtonBindings(ISTESTING);
    configureCharacterizationButtonBindings();
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
          new AllEvents(
              robotGoals,
              matchState,
              indexer,
              shooter,
              intake,
              climber,
              hood,
              driveZoneTracker,
              aimingService));
    }
    // Reset gyro / odometry
    final Runnable resetOdometry =
        Constants.currentMode == Constants.Mode.SIM
            ? () -> drive.setPose(driveSimulation.getSimulatedDriveTrainPose())
            : () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));

    // Effective speed limit = min(operator slow button, goal-based limit from DriveBehavior)
    drive.setOperatorSpeedLimit(REG_DRIVE_SPEED);
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY() * drive.getEffectiveSpeedLimit(),
            () -> -controller.getLeftX() * drive.getEffectiveSpeedLimit(),
            () -> -controller.getRightX() * REG_ANGULAR_SPEED));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Slow button sets operator speed limit directly
    controller
        .leftTrigger()
        .whileTrue(
            Commands.startEnd(
                () -> drive.setOperatorSpeedLimit(SLOW_DRIVE_SPEED),
                () -> drive.setOperatorSpeedLimit(REG_DRIVE_SPEED)));

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
    // testController
    //     .a()
    //     .whileTrue(indexer.getNewSetIndexerVelocityCommand(setIndexerVelocity))
    //     .whileFalse(new InstantCommand(() -> indexer.stop()));
    // testController
    //     .b()
    //     .whileTrue(indexer.getNewSetFeederVelocityCommand(setFeederVelocity))
    //     .whileFalse(new InstantCommand(() -> indexer.stop()));
    // testController
    //     .x()
    //     .whileTrue(turret.getNewSetTurretAngleCommand(setTurretAngle))
    //     .whileFalse(turret.getNewSetTurretAngleCommand(() -> 0.0));
    // testController
    //     .b()
    //     .whileTrue(shooter.getNewSetShooterSpeedCommand(setShooterSpeed))
    //     .whileFalse(new InstantCommand(() -> shooter.stop()));
    // testController
    //     .b()
    //     .whileTrue(intake.getNewSetIntakeVelocityCommand(setIntakeRPM))
    //     .whileFalse(new InstantCommand(() -> intake.stop()));
    // testController
    //     .a()
    //     .whileTrue(
    //         intake.getNewSetIntakeExtenderAngleCommand(
    //             () -> Degrees.of(setIntakeExtenderUp.get()), false))
    //     .whileFalse(
    //         intake.getNewSetIntakeExtenderAngleCommand(
    //             () -> Degrees.of(setIntakeExtenderDown.get()), false));
    // testController
    //     .a()
    //     .povDown()
    //     .whileTrue(indexer.getNewSetFeederVelocityCommand(setFeederVelocity))
    //     .whileFalse(new InstantCommand(() -> indexer.stop()));
    // testController
    //     .x()
    //     .whileTrue(turret.getNewSetTurretAngleCommand(setTurretAngle))
    //     .whileFalse(new InstantCommand(() -> turret.stop()));
    // testController
    //     .leftBumper()
    //     .whileTrue(
    //         new RepeatCommand(
    //             new InstantCommand(
    //                 () -> {
    //                   if (new Vector2(testController.getRightX(), testController.getRightY())
    //                           .getMagnitudeSquared()
    //                       >= 0.25) {
    //                     double ROT_CONST = 0.5;
    //                     turret.setPosition(
    //                         Math.toDegrees(
    //                                 Math.atan2(
    //                                     -testController.getRightX(),
    // -testController.getRightY()))
    //                             * ROT_CONST);
    //                   }
    //                 })));
    // testController
    //     .b()
    //     .whileTrue(shooter.getNewSetShooterSpeedCommand(setShooterSpeed))
    //     .whileFalse(new InstantCommand(() -> shooter.stop()));
    // testController
    //     .y()
    //     .whileTrue(intake.getNewSetIntakeVelocityCommand(setIntakeRPM))
    //     .whileFalse(new InstantCommand(() -> intake.stop()));
    // testController
    //     .povUp()
    //     .whileTrue(intake.getNewSetIntakeExtenderVoltsCommand(setIntakeExtenderVolts, false))
    //     .whileFalse(
    //         intake.getNewSetIntakeExtenderVoltsCommand(
    //             setIntakeExtenderVolts, true)); // Default value for intake extender volts
    // testController
    //     .b()
    //     .whileTrue(hood.getNewSetHoodAngleCommand(setHoodAngle))
    //     .whileFalse(hood.getNewSetHoodAngleCommand(() -> 10.0));
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
    }
  }

  public void resetSimulation() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      // drive.setPose(new Pose2d(3, 3, new Rotation2d()));
      SimulatedArena.getInstance().resetFieldForAuto();
      if (DriverStation.isDisabled()) {
        // Disable AprilTags when disabled
        vision.disableUpdateOdometryBasedOnApriltags();
      }
    }
  }

  public void updateSimulation() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      SimulatedArena.getInstance().simulationPeriodic();
    }
  }

  public void loggingPeriodic() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      Logger.recordOutput(
          "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
      Logger.recordOutput(
          "FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
    }
  }
}
