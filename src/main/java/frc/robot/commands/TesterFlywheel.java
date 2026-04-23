package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.shooter.flywheel.Flywheel;

public class TesterFlywheel extends Command {
  private Flywheel flywheel;
  private String test;
  private Color leaderFlywheelColorStatus = Constants.NetworkTables.purple;
  private Color followerFlywheelColorStatus = Constants.NetworkTables.purple;
  private String flywheelLeader;
  private String flywheelFollower;

  public TesterFlywheel(Flywheel flywheel, String testName) {
    this.flywheel = flywheel;
    this.test = testName;
  }

  @Override
  public void initialize() {
    leaderFlywheelColorStatus = Constants.NetworkTables.purple;
    followerFlywheelColorStatus = Constants.NetworkTables.purple;
    SmartDashboard.putString(
        Constants.Tester.FlywheelKeyLeader, Constants.NetworkTables.purple.toHexString());
    SmartDashboard.putString(
        Constants.Tester.FlywheelKeyFollower, Constants.NetworkTables.purple.toHexString());
  }

  @Override
  public void execute() {
    flywheelFollower = " ";
    flywheelLeader = " ";
    if (!flywheel.leaderConnected()) {
      leaderFlywheelColorStatus = Constants.NetworkTables.red;
      flywheelLeader = " Not Connected";
    } else if (!flywheel.leaderRollerAtGoal()) {
      leaderFlywheelColorStatus = Constants.NetworkTables.orange;
      flywheelLeader =
          "Not Spinning at Goal by"
              + +(100 - ((flywheel.getLeaderRollerSpeed() / flywheel.getRequestedSetpoint()) * 100))
              + "%";
    } else {
      leaderFlywheelColorStatus = Constants.NetworkTables.green;
    }

    if (!flywheel.followerConnected()) {
      followerFlywheelColorStatus = Constants.NetworkTables.red;
      flywheelFollower = " Not Connected";
    } else if (!flywheel.followerRollerAtGoal()) {
      followerFlywheelColorStatus = Constants.NetworkTables.orange;
      flywheelFollower =
          "Not Spinning at Goal by"
              + +(100
                  - ((flywheel.getFollowerRollerSpeed() / flywheel.getRequestedSetpoint()) * 100))
              + "%";
      ;
    } else {
      followerFlywheelColorStatus = Constants.NetworkTables.green;
    }

    if (!flywheel.rollersSpinningTogether()) {
      flywheelLeader = " Not Spinning Together";
      flywheelFollower = " Not Spinning Together";
      flywheelLeader =
          flywheelLeader
              + " Leader at "
              + flywheel.getLeaderRollerSpeed()
              + " RPS, Follower at "
              + flywheel.getFollowerRollerSpeed()
              + " RPS";
      flywheelFollower =
          flywheelFollower
              + " Leader at "
              + flywheel.getLeaderRollerSpeed()
              + " RPS, Follower at "
              + flywheel.getFollowerRollerSpeed()
              + " RPS";
      flywheelLeader =
          flywheelLeader
              + "Diffrence of "
              + (flywheel.getLeaderRollerSpeed() - flywheel.getFollowerRollerSpeed())
              + " RPS";
      flywheelFollower =
          flywheelFollower
              + "Diffrence of "
              + (flywheel.getFollowerRollerSpeed() - flywheel.getLeaderRollerSpeed())
              + " RPS";
    } else {
      followerFlywheelColorStatus = Constants.NetworkTables.green;
      leaderFlywheelColorStatus = Constants.NetworkTables.green;
      flywheelLeader = flywheelLeader + " Flywheel Spinning Together";
      flywheelFollower = flywheelFollower + " Flywheel Spinning Together";
    }

    SmartDashboard.putString(
        Constants.Tester.FlywheelColorKeyLeader, leaderFlywheelColorStatus.toHexString());
    SmartDashboard.putString(
        Constants.Tester.FlywheelColorKeyFollower, followerFlywheelColorStatus.toHexString());

    SmartDashboard.putString(
        Constants.Tester.FlywheelKeyFollower,
        SmartDashboard.getString(Constants.Tester.FlywheelKeyFollower, "")
            + test
            + flywheelFollower);
    SmartDashboard.putString(
        Constants.Tester.FlywheelKeyLeader,
        SmartDashboard.getString(Constants.Tester.FlywheelKeyLeader, "") + test + flywheelLeader);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
