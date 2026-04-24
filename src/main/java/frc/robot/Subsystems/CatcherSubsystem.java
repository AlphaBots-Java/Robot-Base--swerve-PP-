package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CatcherSubsystem extends SubsystemBase {

    private static final int EXTENSOR_ID = 51;
    private static final int CATCHER_ID = 52;
    private static final int CANCODER_ID = 50;
    private static final String CAN_BUS = "BallSystemCAN";

    private static final double EXTENDED_POS = 2.8;
    private static final double RETRACTED_POS = 0.2;

    private static final double CATCH_SPEED = 0.9;

    private final TalonFX extensor = new TalonFX(EXTENSOR_ID, CAN_BUS);
    private final TalonFX catcher = new TalonFX(CATCHER_ID, CAN_BUS);
    private final CANcoder encoder = new CANcoder(CANCODER_ID, CAN_BUS);

    private final PIDController extenderPID = new PIDController(2.3, 0.5, 0.2);

    private boolean extended = false;

    private enum CatchState {
        IDLE,
        INTAKE,
        EXPEL
    }

    private CatchState catchState = CatchState.IDLE;

    public CatcherSubsystem() {
        extenderPID.setTolerance(0.05);
    }

    // ================= EXTENSION =================

    public void toggleExtended() {
        extended = !extended;
    }

    public void setExtended(boolean value) {
        extended = value;
    }

    // ================= CATCH STATE =================

    public void toggleCatch() {
        if (catchState == CatchState.IDLE || catchState == CatchState.EXPEL) {
            catchState = CatchState.INTAKE;
        } else {
            catchState = CatchState.IDLE;
        }
    }

    public void setIntake() {
        catchState = CatchState.INTAKE;
    }

    public void setExpel() {
        catchState = CatchState.EXPEL;
    }

    public void stop() {
        catchState = CatchState.IDLE;
    }

    // ================= CONTROL =================

    private void updateExtension() {
        double setpoint = extended ? EXTENDED_POS : RETRACTED_POS;

        double error = -encoder.getPosition().getValueAsDouble();
        double output = extenderPID.calculate(error, setpoint);

        extensor.setVoltage(output);
    }

    private void updateCatcher() {
        switch (catchState) {
            case INTAKE -> catcher.set(-CATCH_SPEED);
            case EXPEL -> catcher.set(CATCH_SPEED);
            case IDLE -> catcher.set(0);
        }
    }

    // ================= DEBUG =================

    private void debug() {
        SmartDashboard.putNumber("Catcher/Encoder", encoder.getPosition().getValueAsDouble());
        SmartDashboard.putBoolean("Catcher/Extended", extended);
        SmartDashboard.putString("Catcher/State", catchState.name());
    }

    // ================= PERIODIC =================

    @Override
    public void periodic() {
        debug();

        if (DriverStation.isEnabled()) {
            updateExtension();
            updateCatcher();
        } else {
            catchState = CatchState.IDLE;
            catcher.set(0);
        }
    }
}