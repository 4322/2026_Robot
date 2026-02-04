package frc.robot.subsystems.shooter.turret;

import java.time.Period;

import frc.robot.constants.Constants;

public class Turret {
  private TurretIO io;
  private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  public enum turretState{
    IDLE,
    SET_TURRET_ANGLE
  }
  public turretState state = turretState.IDLE;
  public Turret(TurretIO io) {
    this.io = io;
  }
  public void periodic(){
    switch (Constants.turretMode){
      case DISABLED ->{}
      case TUNING ->{}
      case DRIVE_TUNING ->{}
      case NORMAL ->{
        switch (state){
          case IDLE ->{}
          case SET_TURRET_ANGLE ->{}
        }
      }
    }
  }
  
public void setAngle(double angle, boolean safeToUnwind){}
public void isAtGoal(){}
public void preemptiveUnwind(){
  //io.setAngle();
}
public void setBrakeMode(Boolean mode){}

}
