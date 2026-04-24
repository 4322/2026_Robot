package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.shooter.flywheel.Flywheel;

public class TesterFlywheel extends Command {
  private Flywheel flywheel;
  private String test;
  private Color leaderColorStatus;
  private Color followerColorStatus;
  private String leaderStatus;
  private String followerStatus;

  public TesterFlywheel(Flywheel flywheel, String testName) {
    this.flywheel = flywheel;
    this.test = testName;
  }

  @Override
  public void initialize() {
    leaderColorStatus = Constants.NetworkTables.purple;
    followerColorStatus = Constants.NetworkTables.purple;
    leaderStatus = "";
    followerStatus = "";
    setColorStatus();
    setTextStatus();
    SmartDashboard.putString("Tester/Flywheel", test);
  }

  @Override
  public void execute() {
    if (!flywheel.leaderConnected()) {
      leaderColorStatus = Constants.NetworkTables.red;
      leaderStatus = "Not Connected";
    } else if (!flywheel.leaderRollerAtGoal()) {
      leaderColorStatus = Constants.NetworkTables.orange;
      leaderStatus =
          "Slow by"
              + String.format(
                  "%.1f",
                  100 - flywheel.getLeaderRollerSpeed() / flywheel.getRequestedSetpoint() * 100)
              + "%";
    } else {
      leaderColorStatus = Constants.NetworkTables.green;
      leaderStatus = "Up To Speed";
    }

    if (!flywheel.followerConnected()) {
      followerColorStatus = Constants.NetworkTables.red;
      followerStatus = "Not Connected";
    } else if (!flywheel.followerRollerAtGoal()) {
      followerColorStatus = Constants.NetworkTables.orange;
      followerStatus =
          "Slow by"
              + String.format(
                  "%.1f",
                  100 - flywheel.getFollowerRollerSpeed() / flywheel.getRequestedSetpoint() * 100)
              + "%";
      ;
    } else {
      followerColorStatus = Constants.NetworkTables.green;
      followerStatus = "Up To Speed";
    }

    if (!flywheel.rollersSpinningTogether()) {
      leaderStatus = "Not Spinning Together";
      followerStatus = "Not Spinning Together";
      leaderStatus +=
          "\nLeader at "
              + String.format("%.1f", flywheel.getLeaderRollerSpeed())
              + " RPS\nFollower at "
              + String.format("%.1f", flywheel.getFollowerRollerSpeed())
              + " RPS";
      followerStatus +=
          "\nLeader at "
              + String.format("%.1f", flywheel.getLeaderRollerSpeed())
              + " RPS\nFollower at "
              + String.format("%.1f", flywheel.getFollowerRollerSpeed())
              + " RPS";
      leaderStatus +=
          "\nDiffrence of "
              + String.format(
                  "%.1f", flywheel.getLeaderRollerSpeed() - flywheel.getFollowerRollerSpeed())
              + " RPS";
      followerStatus +=
          "\nDiffrence of "
              + String.format(
                  "%.1f", flywheel.getFollowerRollerSpeed() - flywheel.getLeaderRollerSpeed())
              + " RPS";
    } else {
      followerColorStatus = Constants.NetworkTables.green;
      leaderColorStatus = Constants.NetworkTables.green;
      leaderStatus += "\nFlywheel Spinning Together";
      followerStatus += "\nFlywheel Spinning Together";
    }

    setColorStatus();
    setTextStatus();
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  private void setColorStatus() {
    SmartDashboard.putString(
        Constants.Tester.flywheelColorKeyLeader, leaderColorStatus.toHexString());
    SmartDashboard.putString(
        Constants.Tester.flywheelColorKeyFollower, followerColorStatus.toHexString());
  }

  private void setTextStatus() {
    SmartDashboard.putString(Constants.Tester.flywheelKeyFollower, followerStatus);
    SmartDashboard.putString(Constants.Tester.flywheelKeyLeader, leaderStatus);
  }
}
