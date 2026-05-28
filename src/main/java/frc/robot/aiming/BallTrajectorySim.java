package frc.robot.aiming;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.intake.IntakeIOSim;
import java.util.ArrayList;
import java.util.Iterator;
import org.littletonrobotics.junction.Logger;

/**
 * Particle-stream trajectory visualization. Spawns a new projectile at regular intervals, each
 * inheriting the current launch velocity + turret base velocity. All active projectiles are
 * simulated independently with gravity every cycle. The resulting point cloud bends and wobbles as
 * robot velocity changes, showing velocity compensation effects in real time.
 */
public class BallTrajectorySim {

  private static final int MAX_BALLS = 40;
  private static final double DT = 0.02; // Physics step = one robot cycle (20ms)

  private final ArrayList<Projectile> activeBalls = new ArrayList<>();

  /** Publish an empty trajectory when no valid solution exists. */
  public void publishEmpty() {
    // Still update existing balls so they finish their arcs
    updateAll();
    logPositions();
  }

  /**
   * Called every cycle. Spawns a new ball at intervals and updates all active balls.
   *
   * @param turretFieldPos 2D position of turret pivot in field frame (meters)
   * @param turretYawRad field-frame yaw the turret is pointing (radians)
   * @param launchAngleRad elevation angle (radians from horizontal)
   * @param ballSpeed total ball exit speed (m/s)
   * @param turretVelocity velocity of turret pivot in field frame (m/s), inherited by ball
   */
  public void simulate(
      Translation2d turretFieldPos,
      double turretYawRad,
      double launchAngleRad,
      double ballSpeed,
      Translation2d turretVelocity) {
    if (spawnFuelOnGround == true) {
      // Spawn a new ball at regular intervals
      spawnBall(turretFieldPos, turretYawRad, launchAngleRad, ballSpeed, turretVelocity);
    }

    // Advance all active balls by one physics step
    updateAll();

    // Log all positions
    logPositions();
  }

  private void spawnBall(
      Translation2d turretFieldPos,
      double turretYawRad,
      double launchAngleRad,
      double ballSpeed,
      Translation2d turretVelocity) {

    // Compute initial velocity in field frame
    double vHorizontal = ballSpeed * Math.cos(launchAngleRad);
    double vVertical = ballSpeed * Math.sin(launchAngleRad);
    double vxLaunch = vHorizontal * Math.cos(turretYawRad);
    double vyLaunch = vHorizontal * Math.sin(turretYawRad);

    // Ball inherits turret base velocity
    double vx = vxLaunch + turretVelocity.getX();
    double vy = vyLaunch + turretVelocity.getY();
    double vz = vVertical;

    // Cap at max balls (remove oldest)
    if (activeBalls.size() >= MAX_BALLS) {
      activeBalls.remove(0);
    }

    activeBalls.add(
        new Projectile(
            turretFieldPos.getX(),
            turretFieldPos.getY(),
            AimingConstants.TURRET_PIVOT_HEIGHT_METERS,
            vx,
            vy,
            vz));
  }

  private void updateAll() {
    double g = AimingConstants.GRAVITY;
    Iterator<Projectile> it = activeBalls.iterator();
    while (it.hasNext()) {
      Projectile p = it.next();

      // Gravity only (no drag for the stream viz — keeps it fast and clean)
      p.vz -= g * DT;
      p.px += p.vx * DT;
      p.py += p.vy * DT;
      p.pz += p.vz * DT;

      // Remove balls that hit the ground on the way down
      if (p.pz < 0 && p.vz < 0) {
        removeBalls(it, p.px, p.py);
      }
    }
  }

  // Set to false to stop always spawning fuel on ground impact.
  // Call setSpawnFuelOnGround(true) to re-enable.
  public boolean spawnFuelOnGround = false;

  public void setSpawnFuelOnGround(boolean spawn) {
    this.spawnFuelOnGround = spawn;
  }

  public void removeBalls(Iterator<Projectile> it, double x, double y) {
    it.remove();
    IntakeIOSim.spawnFuel(x, y);
  }

  private void logPositions() {
    Translation3d[] positions = new Translation3d[activeBalls.size()];
    for (int i = 0; i < activeBalls.size(); i++) {
      Projectile p = activeBalls.get(i);
      positions[i] = new Translation3d(p.px, p.py, p.pz);
    }
    Logger.recordOutput("Aiming/BallTrajectory", positions);
    Logger.recordOutput("Aiming/spawningFuel", spawnFuelOnGround);
  }

  private static class Projectile {
    double px, py, pz;
    double vx, vy, vz;

    Projectile(double px, double py, double pz, double vx, double vy, double vz) {
      this.px = px;
      this.py = py;
      this.pz = pz;
      this.vx = vx;
      this.vy = vy;
      this.vz = vz;
    }
  }
}
