package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.goals.RobotGoal;
import frc.robot.goals.RobotGoals;
import frc.robot.subsystems.drive.Drive;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.Optional;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoCommandManager {

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private Drive m_drive;

  public AutoCommandManager(Drive drive, RobotGoals goals) {
    configureNamedCommands(drive, goals);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");

    for (String autoName : AutoBuilder.getAllAutoNames()) {
      boolean competitionUsed =
          new File(Filesystem.getDeployDirectory(), "pathplanner/autos/" + autoName + ".compflag")
              .exists();
      boolean inCompetition = false;

      if (inCompetition && !competitionUsed) {
        continue;
      }

      System.out.println("trying " + autoName);

      autoChooser.addOption(autoName, new PathPlannerAuto(autoName));

      try {
        // Validate if path .0 exists
        PathPlannerPath path = PathPlannerPath.fromPathFile(autoName + ".0");
        File f =
            new File(Filesystem.getDeployDirectory(), "pathplanner/autos/" + autoName + ".auto");
        JSONObject autoJson =
            (JSONObject) new JSONParser().parse(new FileReader((f.getAbsoluteFile())));
        boolean resetOdom =
            autoJson.get("resetOdom") != null && (boolean) autoJson.get("resetOdom");
        if (!resetOdom) {
          autoChooser.addOption(autoName, new AutoPathCommand(autoName));
        }
      } catch (FileNotFoundException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      } catch (IOException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      } catch (ParseException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
    }
  }

  public Command getAutonomousCommand() {
    return getAutoWithCurrentPose();
  }

  private void configureNamedCommands(Drive drive, RobotGoals goals) {
    NamedCommands.registerCommand("Intaking", goals.setGoalCommand(RobotGoal.INTAKING));
    NamedCommands.registerCommand("Shooting", goals.setGoalCommand(RobotGoal.SHOOTING));
    NamedCommands.registerCommand("Outtaking", goals.setGoalCommand(RobotGoal.OUTTAKING));
    // TODO: Make only the intake retract
    NamedCommands.registerCommand("Idle", goals.setGoalCommand(RobotGoal.IDLE));
  }

  public Command getAutoWithCurrentPose() {
    Command command = autoChooser.get();
    Command returnCommand = command;

    if (command instanceof AutoPathCommand) {
      AutoPathCommand ppAutoCommand = (AutoPathCommand) command;
      String autoName = ppAutoCommand.m_autoName;
      // TODO detemine if need to prepend (super class with attribute of auto name)
      returnCommand = getToPath(autoName + ".0");
      if (returnCommand != null) {
        returnCommand = returnCommand.andThen(command);
      }
    }
    return returnCommand;
  }

  public Command getToPath(String pathName) {
    Command command = null;
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      Optional<Pose2d> optPath = path.getStartingHolonomicPose();
      if (optPath.isPresent()) {
        // NOTE: Make sure .0 path and starting velocity is > 0.0
        Pose2d pose;
        if (AutoBuilder.shouldFlip()) {
          pose = FlippingUtil.flipFieldPose(optPath.get());
        } else {
          pose = optPath.get();
        }
        command =
            AutoBuilder.pathfindToPose(
                pose, path.getGlobalConstraints(), path.getIdealStartingState().velocity());
      }
    } catch (Exception e) {
      // If we get Exception assume we have no command
      command = null;
    }
    return command;
  }
}
