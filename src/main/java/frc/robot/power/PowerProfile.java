package frc.robot.power;

/** Discrete power allocation profiles with per-subsystem supply current limits (amps). */
public enum PowerProfile {
  NORMAL(60.0, 40.0, 40.0, 80.0, 22.0, 40.0, 40.0, 40.0),
  SHOOTING(40.0, 40.0, 30.0, 60.0, 22.0, 40.0, 40.0, 40.0),
  LOW_VOLTAGE(30.0, 35.0, 20.0, 40.0, 15.0, 25.0, 25.0, 25.0);

  public final double driveSupplyLimit;
  public final double shooterSupplyLimit;
  public final double intakeRollerSupplyLimit;
  public final double intakeExtenderSupplyLimit;
  public final double indexerSupplyLimit;
  public final double feederSupplyLimit;
  public final double turretSupplyLimit;
  public final double hoodSupplyLimit;

  PowerProfile(
      double driveSupply,
      double shooterSupply,
      double intakeRollerSupply,
      double intakeExtenderSupply,
      double indexerSupply,
      double feederSupply,
      double turretSupply,
      double hoodSupply) {
    this.driveSupplyLimit = driveSupply;
    this.shooterSupplyLimit = shooterSupply;
    this.intakeRollerSupplyLimit = intakeRollerSupply;
    this.intakeExtenderSupplyLimit = intakeExtenderSupply;
    this.indexerSupplyLimit = indexerSupply;
    this.feederSupplyLimit = feederSupply;
    this.turretSupplyLimit = turretSupply;
    this.hoodSupplyLimit = hoodSupply;
  }
}
