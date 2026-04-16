

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Drive;

public class TesterTurn extends Command{
    private Drive drive;
    private String currentFLStatus = "Front Left:";
  private String currentFRStatus = "Front Right:";
  private String currentBLStatus = "Back Left:";
  private String currentBRStatus = "Back Right:";
  private String test;
  private double numberTest;
  private String  frontLeftKey = "Tester/Drive/Turn/FL Turn Color Status";
  private String  frontRightKey = "Tester/Drive/Turn/FR Turn Color Status";
 private String  backLeftKey = "Tester/Drive/Turn/BL Turn Color Status";
 private String  backRightKey = "Tester/Drive/Turn/BR Turn Color Status";
  private String  turnConnectionMessage = " Turn Not Connected";

  // If Not working = color
  // If working = green
  // If not connected = red
  // If too slow = orange
  // If pulling too much current = blue;
  // still running Tests = purple
    public TesterTurn(Drive drive, String test) {
    this.drive = drive;
    this.test = test;
    }
    
    @Override
    public void initialize() {
        SmartDashboard.putString(
              frontLeftKey,
              Constants.NetworkTables.red.kMediumPurple.toHexString());
        SmartDashboard.putString(
              frontRightKey,
              Constants.NetworkTables.red.kMediumPurple.toHexString());
        SmartDashboard.putString(
              backLeftKey,
              Constants.NetworkTables.red.kMediumPurple.toHexString());
        SmartDashboard.putString(
              backRightKey,
              Constants.NetworkTables.red.kMediumPurple.toHexString());
          
          this.currentFLStatus = this.currentFLStatus + " Running Test " + test;
          this.currentFRStatus = this.currentFRStatus + " Running Test " + test;
          this.currentBLStatus = this.currentBLStatus + " Running Test " + test;
          this.currentBRStatus = this.currentBRStatus + " Running Test " + test;
          this.numberTest = 0;
    }

    @Override
    public void execute() {
          this.currentFLStatus = this.currentFLStatus + " #" + (numberTest +  1);
          this.currentFRStatus = this.currentFRStatus + " #" + (numberTest +  1);
          this.currentBLStatus = this.currentBLStatus + " #" + (numberTest +  1);
          this.currentBRStatus = this.currentBRStatus + " #" + (numberTest +  1);

            if (!drive.isTurnConnected(0)) {
            SmartDashboard.putString(
                frontLeftKey,
                Constants.NetworkTables.red.toHexString());
            this.currentFLStatus = this.currentFLStatus + turnConnectionMessage;
          } else if (!drive.isCorrectAngleSpeed(0)) {
              SmartDashboard.putString(
                  frontLeftKey,
                  Constants.NetworkTables.red.kOrange.toHexString());
              this.currentFLStatus = this.currentFLStatus + " Incorrect Angle Speed by " + (100 - ((drive.getModuleAngle(0) / drive.anglePerSecondRequested) * 100)) + "%";
          } else {
            SmartDashboard.putString(
                frontLeftKey,
                Constants.NetworkTables.green.toHexString());
          }


          if (!drive.isTurnConnected(1)) {
            SmartDashboard.putString(
                frontRightKey,
                Constants.NetworkTables.red.toHexString());
            this.currentFRStatus = this.currentFRStatus + turnConnectionMessage;
          } else if (!drive.isCorrectAngleSpeed(1)) {
              SmartDashboard.putString(
                  frontRightKey,
                  Constants.NetworkTables.red.kOrange.toHexString());
              this.currentFRStatus = this.currentFRStatus + " Incorrect Angle Speed by " + (100 - ((drive.getModuleAngle(1) / drive.anglePerSecondRequested) * 100)) + "%";
          } else {
            SmartDashboard.putString(
                frontRightKey,
                Constants.NetworkTables.green.toHexString());
          }

          if (!drive.isTurnConnected(2)) {
            SmartDashboard.putString(
                backLeftKey,
                Constants.NetworkTables.red.kBlack.toHexString());
            this.currentBLStatus = this.currentBLStatus + turnConnectionMessage;
            }  else if(!drive.isCorrectAngleSpeed(2)) {
              SmartDashboard.putString(
                  backLeftKey,
                  Constants.NetworkTables.red.kOrange.toHexString());
              this.currentBLStatus = this.currentBLStatus + " Incorrect Angle Speed by " + (100 - ((drive.getModuleAngle(2) / drive.anglePerSecondRequested) * 100)) + "%";

          } else {
            SmartDashboard.putString(
                backLeftKey,
                Constants.NetworkTables.green.toHexString());
          }
          
          if (!drive.isTurnConnected(3)) {
            SmartDashboard.putString(
                backRightKey,
                Constants.NetworkTables.red.kBlack.toHexString());
            this.currentBRStatus = this.currentBRStatus + turnConnectionMessage;
          } else if (!drive.isCorrectAngleSpeed(3)) {
              SmartDashboard.putString(
                  backRightKey,
                  Constants.NetworkTables.red.kOrange.toHexString());
              this.currentBRStatus = this.currentBRStatus + " Incorrect Angle Speed by " + (100 - ((drive.getModuleAngle(3) / drive.anglePerSecondRequested) * 100)) + "%";
          } else {
                        SmartDashboard.putString(
                backRightKey,
                Constants.NetworkTables.green.toHexString());
          }
          SmartDashboard.putString("Tester/Drive/Turn/Front Turn Left Status", this.currentFLStatus);
          SmartDashboard.putString("Tester/Drive/Turn/Front Turn Right Status", this.currentFRStatus);
          SmartDashboard.putString("Tester/Drive/Turn/Back Turn Left Status", this.currentBLStatus);
          SmartDashboard.putString("Tester/Drive/Turn/Back Turn Right Status", this.currentBRStatus);
          this.currentFLStatus = "Front Left:";
          this.currentFRStatus = "Front Right:";
          this.currentBLStatus = "Back Left:";
          this.currentBRStatus = "Back Right:";
    }
    
    @Override
    public void end(boolean interrupted) {
    }
    
}
