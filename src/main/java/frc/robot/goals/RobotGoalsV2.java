package frc.robot.goals;

import com.ctre.phoenix6.CANBus;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import frc.robot.Constants;
import frc.robot.operator.OperatorIntentEvents;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.EnumState;
import frc.robot.util.VirtualSubsystem;

/**
 * Wires operator button presses to robot goal state.
 *
 * <p>This is the teleop-specific logic. Autonomous bypasses this and calls RobotGoals.setGoal()
 * directly.
 *
 * <p>TODO (students): Map your button intents to goals here Example:
 * intent.wantsToScore().onTrue(goals.setGoal(RobotGoal.LAUNCHING))
 * .onFalse(goals.setGoal(RobotGoal.IDLE));
 */
public class RobotGoalsV2 {
  private final EnumState<RobotGoal> currentGoal =
      new EnumState<>("RobotGoals/Goal", RobotGoal.IDLE);
  
  ClimberSubsystem climber;
  IntakeSubsystem intake;
  ShooterSubsystem shooter;

  public RobotGoalsV2() {
    switch (Constants.currentMode) {
      case REAL:
        // TODO add TalonFX
        climber = new ClimberSubsystem(null);
        intake = new IntakeSubsystem(null);
        shooter = new ShooterSubsystem(null);
        
        break;
      case SIM:
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
        
        intake = new IntakeSubsystem(new IntakeIOSim());
        shooter = new ShooterSubsystem(new ShooterIOSim());
        break;
      default:
        // un-supported mode so all should be null
        climber = new ClimberSubsystem(null);
        intake = new IntakeSubsystem(null);
        shooter = new ShooterSubsystem(null);
        break;
    }
  }

  public void configure(OperatorIntentEvents intent) {
    intent
        .wantsToScoreTrigger()
        .onTrue(intake.intakeCommand().alongWith(shooter.shooterCommand()))
        .onFalse(intake.idleCommand().alongWith(shooter.idleCommand()));
  }
}
