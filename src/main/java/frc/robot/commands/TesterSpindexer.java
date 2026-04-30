package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.shooter.spindexer.Spindexer;

public class TesterSpindexer extends Command {
  private Spindexer spindexer;
  private String test;
  private Color leaderColorStatus;
  private Color followerColorStatus;
  private String leaderStatus;
  private String followerStatus;

  public TesterSpindexer(Spindexer spindexer, String testName) {
    this.spindexer = spindexer;
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
    if (!spindexer.leaderConnected()) {
      leaderColorStatus = Constants.NetworkTables.red;
      leaderStatus = "Not Connected";
    } else if (!spindexer.leaderAtGoal()) {
      leaderColorStatus = Constants.NetworkTables.orange;
      leaderStatus =
          "Slow by"
              + String.format(
                  "%.1f",
                  100
                      - Math.abs(spindexer.getLeaderSpeed())
                          / Math.abs(spindexer.getRequestedSetpoint())
                          * 100)
              + "%";
    } else {
      leaderColorStatus = Constants.NetworkTables.green;
      leaderStatus = "Up To Speed";
    }

    if (!spindexer.followerConnected()) {
      followerColorStatus = Constants.NetworkTables.red;
      followerStatus = "Not Connected";
    } else if (!spindexer.followerAtGoal()) {
      followerColorStatus = Constants.NetworkTables.orange;
      followerStatus =
          "Slow by"
              + String.format(
                  "%.1f",
                  100
                      - Math.abs(spindexer.getFollowerSpeed())
                          / Math.abs(spindexer.getRequestedSetpoint())
                          * 100)
              + "%";
      ;
    } else {
      followerColorStatus = Constants.NetworkTables.green;
      followerStatus = "Up To Speed";
    }

    if (!spindexer.spinningTogether()) {
      leaderStatus = "Not Spinning Together";
      followerStatus = "Not Spinning Together";
      leaderStatus +=
          "\nLeader at "
              + String.format("%.1f", spindexer.getLeaderSpeed())
              + " RPS\nFollower at "
              + String.format("%.1f", spindexer.getFollowerSpeed())
              + " RPS";
      followerStatus +=
          "\nLeader at "
              + String.format("%.1f", spindexer.getLeaderSpeed())
              + " RPS\nFollower at "
              + String.format("%.1f", spindexer.getFollowerSpeed())
              + " RPS";
      leaderStatus +=
          "\nDiffrence of "
              + String.format("%.1f", spindexer.getLeaderSpeed() - spindexer.getFollowerSpeed())
              + " RPS";
      followerStatus +=
          "\nDiffrence of "
              + String.format("%.1f", spindexer.getFollowerSpeed() - spindexer.getLeaderSpeed())
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


  private void setColorStatus() {
    SmartDashboard.putString(
        Constants.Tester.spindexerColorKeyLeader, leaderColorStatus.toHexString());
    SmartDashboard.putString(
        Constants.Tester.spindexerColorKeyFollower, followerColorStatus.toHexString());
  }

  private void setTextStatus() {
    SmartDashboard.putString(Constants.Tester.spindexerKeyFollower, followerStatus);
    SmartDashboard.putString(Constants.Tester.spindexerKeyLeader, leaderStatus);
    leaderStatus = "";
    followerStatus = "";
  }
}
