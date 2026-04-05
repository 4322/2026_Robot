/*
 * File: Shooter.cc
 * Minimal base shooter subsystem for an FRC robot (C++).
 *
 * Place this file in:
 *   /c:/Users/super/Clockwork 2026/2026_Robot/src/main/java/frc/robot/subsystems/shooter/Shooter.cc
 *
 * This is a lightweight starting point: add motor controllers, sensors,
 * and commands as needed.
 */


#include <algorithm>;

namespace frc::robot::subsystems {

class Shooter : public frc2::SubsystemBase {
public:
    Shooter() = default;
    ~Shooter() override = default;

    // Set shooter output in [-1.0, 1.0]. Replace with proper motor-control code.
    void SetSpeed(double percent) {
        percent = std::clamp(percent, -1.0, 1.0);
        m_speed = percent;
        // TODO: drive motor controller here, e.g. m_motor.Set(percent);
        frc::SmartDashboard::PutNumber("Shooter/Speed", m_speed);
    }

    // Stop the shooter.
    void Stop() {
        SetSpeed(0.0);
    }

    // Periodic called once per scheduler run.
    void Periodic() override {
        // Add periodic monitoring or closed-loop control here.
        frc::SmartDashboard::PutBoolean("Shooter/Enabled", m_speed != 0.0);
    }

private:
    double m_speed{0.0};

    // TODO: add motor controllers and sensors as members, e.g.
    // rev::CANSparkMax m_motor{CAN_ID, rev::CANSparkMax::MotorType::kBrushless};
};

}  // namespace frc::robot::subsystems