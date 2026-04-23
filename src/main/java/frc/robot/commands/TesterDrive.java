package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Drive;

public class TesterDrive extends Command {
  private Drive drive;
  private String currentFLStatus = "Front Left:";
  private String FRStatus = "Front Right:";
  private String BLStatus = "Back Left:";
  private String BRStatus = "Back Right:";
  private Color FLColorStatus = new Color(0, 0, 0);
  private Color FRColorStatus = new Color(0, 0, 0);
  private Color BLColorStatus = new Color(0, 0, 0);
  private Color BRColorStatus = new Color(0, 0, 0);
  private String test;
  private String driveConnectionMessage = " Drive Not Connected";

  // If Not working = color
  // If working = green
  // If not connected = red
  // If too slow = orange
  // If pulling too much current = blue;
  // still running Tests = purple
  public TesterDrive(Drive drive, String test) {
    this.drive = drive;
    this.test = test;
  }

  @Override
  public void initialize() {
    this.FLColorStatus = Constants.NetworkTables.purple;
    this.FRColorStatus = Constants.NetworkTables.purple;
    this.BLColorStatus = Constants.NetworkTables.purple;
    this.BRColorStatus = Constants.NetworkTables.purple;
    SmartDashboard.putString(Constants.Tester.DriveKeyFL, this.FLColorStatus.toHexString());
    SmartDashboard.putString(Constants.Tester.DriveKeyFR, this.FRColorStatus.toHexString());
    SmartDashboard.putString(Constants.Tester.DriveKeyBL, this.BLColorStatus.toHexString());
    SmartDashboard.putString(Constants.Tester.DriveKeyBR, this.BRColorStatus.toHexString());
  }

  @Override
  public void execute() {

    if (!drive.isDriveConnected(0)) {
      this.FLColorStatus = Constants.NetworkTables.red;
      this.currentFLStatus = "FL " + driveConnectionMessage;
    } else if (!drive.isDriveCorrectSpeed(0)) {
      this.FLColorStatus = Constants.NetworkTables.orange;
      this.currentFLStatus =
          "FL "
              + " Too Slow by "
              + (100 - ((drive.getModuleVelocity(0) / drive.requestedSpeed) * 100))
              + "% ";
    } else {
      this.FLColorStatus = Constants.NetworkTables.green;
    }

    if (!drive.isDriveConnected(1)) {
      this.FRColorStatus = Constants.NetworkTables.red;
      this.FRStatus = "FR " + driveConnectionMessage;
    } else if (!drive.isDriveCorrectSpeed(1)) {
      this.FRColorStatus = Constants.NetworkTables.orange;
      this.FRStatus =
          "FR "
              + " Too Slow by "
              + (100 - ((drive.getModuleVelocity(1) / drive.requestedSpeed) * 100))
              + "%";
    } else {
      this.FRColorStatus = Constants.NetworkTables.green;
    }

    if (!drive.isDriveConnected(2)) {
      this.BLColorStatus = Constants.NetworkTables.red;
      this.BLStatus = "BL " + driveConnectionMessage;
    } else if (!drive.isDriveCorrectSpeed(2)) {
      this.BLColorStatus = Constants.NetworkTables.orange;
      this.BLStatus =
          "BL "
              + " Too Slow by "
              + (100 - ((drive.getModuleVelocity(2) / drive.requestedSpeed) * 100))
              + "%";

    } else {
      this.BLColorStatus = Constants.NetworkTables.green;
    }

    if (!drive.isDriveConnected(3)) {
      this.BRColorStatus = Constants.NetworkTables.red;
      this.BRStatus = driveConnectionMessage;
    } else if (!drive.isDriveCorrectSpeed(3)) {
      this.BRColorStatus = Constants.NetworkTables.orange;
      this.BRStatus =
          "BR "
              + " Too Slow by "
              + (100 - ((drive.getModuleVelocity(3) / drive.requestedSpeed) * 100))
              + "%";
    } else {
      this.BRColorStatus = Constants.NetworkTables.green;
    }

    SmartDashboard.putString(Constants.Tester.DriveColorKeyFL, this.FLColorStatus.toHexString());
    SmartDashboard.putString(Constants.Tester.DriveColorKeyFR, this.FRColorStatus.toHexString());
    SmartDashboard.putString(Constants.Tester.DriveColorKeyBL, this.BLColorStatus.toHexString());
    SmartDashboard.putString(Constants.Tester.DriveColorKeyBR, this.BRColorStatus.toHexString());

    SmartDashboard.putString(
        Constants.Tester.DriveKeyFL,
        SmartDashboard.getString(Constants.Tester.DriveKeyFL, "")
            + test
            + this.currentFLStatus);
    SmartDashboard.putString(
        Constants.Tester.DriveKeyFR,
        SmartDashboard.getString(Constants.Tester.DriveKeyFR, "")
            + test
            + this.FRStatus);
    SmartDashboard.putString(
        Constants.Tester.DriveKeyBL,
        SmartDashboard.getString(Constants.Tester.DriveKeyBL, "")
            + test
            + this.BLStatus);
    SmartDashboard.putString(
        Constants.Tester.DriveKeyBR,
        SmartDashboard.getString(Constants.Tester.DriveKeyBR, "")
            + test
            + this.BRStatus);
  }

  @Override
  public void end(boolean interrupted) {}
}
