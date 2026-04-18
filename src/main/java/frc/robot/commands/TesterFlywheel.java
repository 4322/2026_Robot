package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.shooter.flywheel.Flywheel;

public class TesterFlywheel extends Command {
  private Flywheel flywheel;
  private String test;
  private String currentLeaderFlywheelStatusKey =
      "Tester/Shooter/Flywheel/Leader Flywheel Color Status";
  private String currentFollowerFlywheelStatusKey =
      "Tester/Shooter/Flywheel/Follower Flywheel Color Status";
  private Color currentLeaderFlywheelColorStatus = Constants.NetworkTables.purple;
  private Color currentFollowerFlywheelColorStatus = Constants.NetworkTables.purple;
  private String rollerLeader = " Roller Leader: ";
  private String rollerFollower = " Roller Follower: ";

  public TesterFlywheel(Flywheel flywheel, String test) {
    this.flywheel = flywheel;
    this.test = test;
  }

  @Override
  public void initialize() {
    currentLeaderFlywheelColorStatus = Constants.NetworkTables.purple;
    currentFollowerFlywheelColorStatus = Constants.NetworkTables.purple;
    SmartDashboard.putString(
        currentLeaderFlywheelStatusKey, Constants.NetworkTables.purple.toHexString());
    SmartDashboard.putString(
        currentFollowerFlywheelStatusKey, Constants.NetworkTables.purple.toHexString());
  }

  @Override
  public void execute() {
    rollerFollower = " ";
    rollerLeader = " ";
    if (!flywheel.leaderConnected()) {
      currentLeaderFlywheelColorStatus = Constants.NetworkTables.red;
      rollerLeader = " Not Connected";
    } else if (!flywheel.leaderRollerAtGoal()) {
      currentLeaderFlywheelColorStatus = Constants.NetworkTables.orange;
      rollerLeader =
          "Not Spinning at Goal by"
              + +(100 - ((flywheel.getLeaderRollerSpeed() / flywheel.getRequestedSetpoint()) * 100))
              + "%";
    } else {
      currentLeaderFlywheelColorStatus = Constants.NetworkTables.green;
    }

    if (!flywheel.followerConnected()) {
      currentFollowerFlywheelColorStatus = Constants.NetworkTables.red;
      rollerFollower = " Not Connected";
    } else if (!flywheel.followerRollerAtGoal()) {
      currentFollowerFlywheelColorStatus = Constants.NetworkTables.orange;
      rollerFollower =
          "Not Spinning at Goal by"
              + +(100
                  - ((flywheel.getFollowerRollerSpeed() / flywheel.getRequestedSetpoint()) * 100))
              + "%";
      ;
    } else {
      currentFollowerFlywheelColorStatus = Constants.NetworkTables.green;
    }

    if (!flywheel.rollersSpinningTogether()) {
      rollerLeader = " Not Spinning Together";
      rollerFollower = " Not Spinning Together";
      rollerLeader =
          rollerLeader
              + " Leader at "
              + flywheel.getLeaderRollerSpeed()
              + " RPS, Follower at "
              + flywheel.getFollowerRollerSpeed()
              + " RPS";
      rollerFollower =
          rollerFollower
              + " Leader at "
              + flywheel.getLeaderRollerSpeed()
              + " RPS, Follower at "
              + flywheel.getFollowerRollerSpeed()
              + " RPS";
      rollerLeader =
          rollerLeader
              + "Diffrence of "
              + (flywheel.getLeaderRollerSpeed() - flywheel.getFollowerRollerSpeed())
              + " RPS";
      rollerFollower =
          rollerFollower
              + "Diffrence of "
              + (flywheel.getFollowerRollerSpeed() - flywheel.getLeaderRollerSpeed())
              + " RPS";
    } else {
      currentFollowerFlywheelColorStatus = Constants.NetworkTables.green;
      currentLeaderFlywheelColorStatus = Constants.NetworkTables.green;
      rollerLeader = rollerLeader + " Rollers Spinning Together";
      rollerFollower = rollerFollower + " Rollers Spinning Together";
    }

    SmartDashboard.putString(
        currentLeaderFlywheelStatusKey, currentLeaderFlywheelColorStatus.toHexString());
    SmartDashboard.putString(
        currentFollowerFlywheelStatusKey, currentFollowerFlywheelColorStatus.toHexString());

    SmartDashboard.putString(
        currentFollowerFlywheelStatusKey,
        SmartDashboard.getString(currentFollowerFlywheelStatusKey, "") + test + rollerFollower);
    SmartDashboard.putString(
        currentLeaderFlywheelStatusKey,
        SmartDashboard.getString(currentLeaderFlywheelStatusKey, "") + test + rollerLeader);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
