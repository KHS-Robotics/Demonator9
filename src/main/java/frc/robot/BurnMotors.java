package frc.robot;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class BurnMotors {
    static class Hood {
        public Hood(int id) {
            CANSparkMax unburned = new CANSparkMax(id, MotorType.kBrushless);
            this.CANSparkMax();
        }
    }
}