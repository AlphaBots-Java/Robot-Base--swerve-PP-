package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class CapSubsystem extends SubsystemBase{
    private final TalonFX capKraken44 = new TalonFX(0, "canBUS");
    private final CANcoder capThroughBourne = new CANcoder(1, "canBUS");

    private final PIDController capPID = new PIDController(0.0,0.0,0.0);

    public CapSubsystem(){

    }

    public void setCapPositionDegrees(double degrees){
        double CapDegrees = (capThroughBourne.getPosition().getValueAsDouble() * 360) * (1/ShooterConstants.kCapStep);
        double CAPVoltage = capPID.calculate(CapDegrees ,degrees);

        capKraken44.setVoltage(MathUtil.clamp(CAPVoltage, -12, 12));
    }
}
