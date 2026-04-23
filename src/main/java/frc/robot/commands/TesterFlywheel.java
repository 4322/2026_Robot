package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import org.littletonrobotics.junction.Logger;

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
    Logger.recordOutput("Tester/Flywheel", test);
  }

  @Override
  public void execute() {
    followerStatus = " ";
    leaderStatus = " ";
    if (!flywheel.leaderConnected()) {
      leaderColorStatus = Constants.NetworkTables.red;
      leaderStatus = " Not Connected";
    } else if (!flywheel.leaderRollerAtGoal()) {
      leaderColorStatus = Constants.NetworkTables.orange;
      leaderStatus =
          "Not Spinning at Goal by"
              + +(100 - ((flywheel.getLeaderRollerSpeed() / flywheel.getRequestedSetpoint()) * 100))
              + "%";
    } else {
      leaderColorStatus = Constants.NetworkTables.green;
      leaderStatus = "Up To Speed";
    }

    if (!flywheel.followerConnected()) {
      followerColorStatus = Constants.NetworkTables.red;
      followerStatus = " Not Connected";
    } else if (!flywheel.followerRollerAtGoal()) {
      followerColorStatus = Constants.NetworkTables.orange;
      followerStatus =
          "Not Spinning at Goal by"
              + +(100
                  - ((flywheel.getFollowerRollerSpeed() / flywheel.getRequestedSetpoint()) * 100))
              + "%";
      ;
    } else {
      followerColorStatus = Constants.NetworkTables.green;
      followerStatus = "Up To Speed";
    }

    if (!flywheel.rollersSpinningTogether()) {
      leaderStatus = " Not Spinning Together";
      followerStatus = " Not Spinning Together";
      leaderStatus =
          leaderStatus
              + " Leader at "
              + flywheel.getLeaderRollerSpeed()
              + " RPS, Follower at "
              + flywheel.getFollowerRollerSpeed()
              + " RPS";
      followerStatus =
          followerStatus
              + " Leader at "
              + flywheel.getLeaderRollerSpeed()
              + " RPS, Follower at "
              + flywheel.getFollowerRollerSpeed()
              + " RPS";
      leaderStatus =
          leaderStatus
              + "Diffrence of "
              + (flywheel.getLeaderRollerSpeed() - flywheel.getFollowerRollerSpeed())
              + " RPS";
      followerStatus =
          followerStatus
              + "Diffrence of "
              + (flywheel.getFollowerRollerSpeed() - flywheel.getLeaderRollerSpeed())
              + " RPS";
    } else {
      followerColorStatus = Constants.NetworkTables.green;
      leaderColorStatus = Constants.NetworkTables.green;
      leaderStatus = leaderStatus + " Flywheel Spinning Together";
      followerStatus = followerStatus + " Flywheel Spinning Together";
    }

    SmartDashboard.putString(
        Constants.Tester.FlywheelColorKeyLeader, leaderColorStatus.toHexString());
    SmartDashboard.putString(
        Constants.Tester.FlywheelColorKeyFollower, followerColorStatus.toHexString());
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  private void setColorStatus() {
    SmartDashboard.putString(
        Constants.Tester.FlywheelColorKeyLeader, leaderColorStatus.toHexString());
    SmartDashboard.putString(
        Constants.Tester.FlywheelColorKeyFollower, followerColorStatus.toHexString());
  }

  private void setTextStatus() {
    SmartDashboard.putString(Constants.Tester.FlywheelKeyFollower, followerStatus);
    SmartDashboard.putString(Constants.Tester.FlywheelKeyLeader, leaderStatus);
  }
}
