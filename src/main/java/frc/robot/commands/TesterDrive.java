package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Drive;

public class TesterDrive extends Command {
  private Drive drive;
  private String currentFLStatus = "Front Left:";
  private String currentFRStatus = "Front Right:";
  private String currentBLStatus = "Back Left:";
  private String currentBRStatus = "Back Right:";
  private Color currentFLColorStatus = new Color(0, 0, 0);
  private Color currentFRColorStatus = new Color(0, 0, 0);
  private Color currentBLColorStatus = new Color(0, 0, 0);
  private Color currentBRColorStatus = new Color(0, 0, 0);
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
    this.currentFLColorStatus = Constants.NetworkTables.purple;
    this.currentFRColorStatus = Constants.NetworkTables.purple;
    this.currentBLColorStatus = Constants.NetworkTables.purple;
    this.currentBRColorStatus = Constants.NetworkTables.purple;
    SmartDashboard.putString(Constants.Tester.DriveKeyFL, this.currentFLColorStatus.toHexString());
    SmartDashboard.putString(Constants.Tester.DriveKeyFR, this.currentFRColorStatus.toHexString());
    SmartDashboard.putString(Constants.Tester.DriveKeyBL, this.currentBLColorStatus.toHexString());
    SmartDashboard.putString(Constants.Tester.DriveKeyBR, this.currentBRColorStatus.toHexString());
  }

  @Override
  public void execute() {

    if (!drive.isDriveConnected(0)) {
      this.currentFLColorStatus = Constants.NetworkTables.red;
      this.currentFLStatus = "FL " + driveConnectionMessage;
    } else if (!drive.isDriveCorrectSpeed(0)) {
      this.currentFLColorStatus = Constants.NetworkTables.orange;
      this.currentFLStatus =
          "FL "
              + " Too Slow by "
              + (100 - ((drive.getModuleVelocity(0) / drive.requestedSpeed) * 100))
              + "% ";
    } else {
      this.currentFLColorStatus = Constants.NetworkTables.green;
    }

    if (!drive.isDriveConnected(1)) {
      this.currentFRColorStatus = Constants.NetworkTables.red;
      this.currentFRStatus = "FR " + driveConnectionMessage;
    } else if (!drive.isDriveCorrectSpeed(1)) {
      this.currentFRColorStatus = Constants.NetworkTables.orange;
      this.currentFRStatus =
          "FR "
              + " Too Slow by "
              + (100 - ((drive.getModuleVelocity(1) / drive.requestedSpeed) * 100))
              + "%";
    } else {
      this.currentFRColorStatus = Constants.NetworkTables.green;
    }

    if (!drive.isDriveConnected(2)) {
      this.currentBLColorStatus = Constants.NetworkTables.red;
      this.currentBLStatus = "BL " + driveConnectionMessage;
    } else if (!drive.isDriveCorrectSpeed(2)) {
      this.currentBLColorStatus = Constants.NetworkTables.orange;
      this.currentBLStatus =
          "BL "
              + " Too Slow by "
              + (100 - ((drive.getModuleVelocity(2) / drive.requestedSpeed) * 100))
              + "%";

    } else {
      this.currentBLColorStatus = Constants.NetworkTables.green;
    }

    if (!drive.isDriveConnected(3)) {
      this.currentBRColorStatus = Constants.NetworkTables.red;
      this.currentBRStatus = driveConnectionMessage;
    } else if (!drive.isDriveCorrectSpeed(3)) {
      this.currentBRColorStatus = Constants.NetworkTables.orange;
      this.currentBRStatus =
          "BR "
              + " Too Slow by "
              + (100 - ((drive.getModuleVelocity(3) / drive.requestedSpeed) * 100))
              + "%";
    } else {
      this.currentBRColorStatus = Constants.NetworkTables.green;
    }

    SmartDashboard.putString(Constants.Tester.DriveColorKeyFL, this.currentFLColorStatus.toHexString());
    SmartDashboard.putString(Constants.Tester.DriveColorKeyFR, this.currentFRColorStatus.toHexString());
    SmartDashboard.putString(Constants.Tester.DriveColorKeyBL, this.currentBLColorStatus.toHexString());
    SmartDashboard.putString(Constants.Tester.DriveColorKeyBR, this.currentBRColorStatus.toHexString());

    SmartDashboard.putString(
        Constants.Tester.DriveKeyFL,
        SmartDashboard.getString(Constants.Tester.DriveKeyFL, "")
            + test
            + this.currentFLStatus);
    SmartDashboard.putString(
        Constants.Tester.DriveKeyFR,
        SmartDashboard.getString(Constants.Tester.DriveKeyFR, "")
            + test
            + this.currentFRStatus);
    SmartDashboard.putString(
        Constants.Tester.DriveKeyBL,
        SmartDashboard.getString(Constants.Tester.DriveKeyBL, "")
            + test
            + this.currentBLStatus);
    SmartDashboard.putString(
        Constants.Tester.DriveKeyBR,
        SmartDashboard.getString(Constants.Tester.DriveKeyBR, "")
            + test
            + this.currentBRStatus);
  }

  @Override
  public void end(boolean interrupted) {}
}
