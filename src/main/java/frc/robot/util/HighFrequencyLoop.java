package frc.robot.util;

/**
 * Generic reusable daemon thread for running a task at a specified frequency. Used for 250Hz aiming
 * computation and motor commands. Modeled after PhoenixOdometryThread's daemon pattern.
 */
public class HighFrequencyLoop extends Thread {

  private final Runnable task;
  private final double frequencyHz;

  public HighFrequencyLoop(String name, double frequencyHz, Runnable task) {
    setName(name);
    setDaemon(true);
    this.task = task;
    this.frequencyHz = frequencyHz;
  }

  @Override
  public void run() {
    while (true) {
      try {
        Thread.sleep((long) (1000.0 / frequencyHz));
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
      task.run();
    }
  }
}
