package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;

public class DeletePrevStringTester extends Command {

  public DeletePrevStringTester() {}

  @Override
  public void initialize() {
    SmartDashboard.putString(Constants.Tester.DriveKeyFL, " ");
    SmartDashboard.putString(Constants.Tester.DriveKeyFR, " ");
    SmartDashboard.putString(Constants.Tester.DriveKeyBL, " ");
    SmartDashboard.putString(Constants.Tester.DriveKeyBR, " ");
    SmartDashboard.putString(Constants.Tester.TurnKeyFL, " ");
    SmartDashboard.putString(Constants.Tester.TurnKeyFR, " ");
    SmartDashboard.putString(Constants.Tester.TurnKeyBL, " ");
    SmartDashboard.putString(Constants.Tester.TurnKeyBR, " ");
    SmartDashboard.putString(Constants.Tester.RollerKeyLeader, " ");
    SmartDashboard.putString(Constants.Tester.RollerKeyFollower, " ");
    SmartDashboard.putString(Constants.Tester.FlywheelKeyLeader, " ");
    SmartDashboard.putString(Constants.Tester.FlywheelKeyFollower, " ");
  }
}
