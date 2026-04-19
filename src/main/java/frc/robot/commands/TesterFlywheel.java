package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.shooter.flywheel.Flywheel;

public class TesterFlywheel extends Command {
  private Flywheel flywheel;
  private String test;
  private Color currentLeaderFlywheelColorStatus = Constants.NetworkTables.purple;
  private Color currentFollowerFlywheelColorStatus = Constants.NetworkTables.purple;
  private String flywheelLeader = " Flywheel Leader: ";
  private String flywheelFollower = " Flywheel Follower: ";

  public TesterFlywheel(Flywheel flywheel, String test) {
    this.flywheel = flywheel;
    this.test = test;
  }

  @Override
  public void initialize() {
    currentLeaderFlywheelColorStatus = Constants.NetworkTables.purple;
    currentFollowerFlywheelColorStatus = Constants.NetworkTables.purple;
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
      currentLeaderFlywheelColorStatus = Constants.NetworkTables.red;
      flywheelLeader = " Not Connected";
    } else if (!flywheel.leaderRollerAtGoal()) {
      currentLeaderFlywheelColorStatus = Constants.NetworkTables.orange;
      flywheelLeader =
          "Not Spinning at Goal by"
              + +(100 - ((flywheel.getLeaderRollerSpeed() / flywheel.getRequestedSetpoint()) * 100))
              + "%";
    } else {
      currentLeaderFlywheelColorStatus = Constants.NetworkTables.green;
    }

    if (!flywheel.followerConnected()) {
      currentFollowerFlywheelColorStatus = Constants.NetworkTables.red;
      flywheelFollower = " Not Connected";
    } else if (!flywheel.followerRollerAtGoal()) {
      currentFollowerFlywheelColorStatus = Constants.NetworkTables.orange;
      flywheelFollower =
          "Not Spinning at Goal by"
              + +(100
                  - ((flywheel.getFollowerRollerSpeed() / flywheel.getRequestedSetpoint()) * 100))
              + "%";
      ;
    } else {
      currentFollowerFlywheelColorStatus = Constants.NetworkTables.green;
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
      currentFollowerFlywheelColorStatus = Constants.NetworkTables.green;
      currentLeaderFlywheelColorStatus = Constants.NetworkTables.green;
      flywheelLeader = flywheelLeader + " Flywheel Spinning Together";
      flywheelFollower = flywheelFollower + " Flywheel Spinning Together";
    }

    SmartDashboard.putString(
        Constants.Tester.FlywheelColorKeyLeader, currentLeaderFlywheelColorStatus.toHexString());
    SmartDashboard.putString(
        Constants.Tester.FlywheelColorKeyFollower,
        currentFollowerFlywheelColorStatus.toHexString());

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
