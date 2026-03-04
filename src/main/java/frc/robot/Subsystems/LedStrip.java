package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.CapSubsystem;
import frc.robot.Subsystems.LimeLightSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;

public class LedStrip extends SubsystemBase {
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;
    private int m_rainbowFirstPixelHue;
    private int m_alphabotsPhase; // Para controlar a oscilação da cor Alphabots
    private final LimeLightSubsystem m_limelight;
    private final ShooterSubsystem m_shooter;
    private final CapSubsystem m_cap;
    private int m_blinkCounter = 0;
    private final int BLINK_PERIOD = 25;

    // Tolerâncias para considerar o robô "pronto"
    private static final double TX_TOLERANCE = 0.2; // Graus de desvio horizontal do Limelight
    private static final double RPM_TOLERANCE = 0.0; // RPM de diferença aceitável
    private static final double ANGLE_TOLERANCE = 1.0; // Graus de diferença aceitável para o cap

    public LedStrip(LimeLightSubsystem limelight, ShooterSubsystem shooter, CapSubsystem cap) {
        // Porta PWM 9 - Deve ser uma porta PWM no RIO
        m_led = new AddressableLED(9);

        // Define o comprimento da fita (ex: 60 LEDs)
        m_ledBuffer = new AddressableLEDBuffer(60);
        m_led.setLength(m_ledBuffer.getLength());

        // Define os dados iniciais e inicia a saída
        m_led.setData(m_ledBuffer);
        m_led.start();

        m_alphabotsPhase = 0; // Inicializa a fase da cor Alphabots
        m_limelight = limelight;
        m_shooter = shooter;
        m_cap = cap;
    }

    @Override
    public void periodic() {
        // Envia os dados do buffer para a fita a cada ciclo
        m_led.setData(m_ledBuffer);

        // Lógica de "Pronto para Atirar"
        // Só pisca verde se o shooter estiver ativo (setpoint > 0)
        if (m_shooter.getCurrentSetpoint() > 0) {
            boolean hasTarget = LimeLightSubsystem.hasTarget();
            double distance = LimeLightSubsystem.DistanceToTarget();
            boolean capExtended = m_cap.isExtended(); // VERIFICAÇÃO ADICIONADA: O cap está estendido?

            // Verifica se há alvo, distância válida e cap estendido
            if (hasTarget && distance > 0 && capExtended) {
                double tx = LimeLightSubsystem.getTx();
                // double targetRpm = m_shooter.getRpmForDistance(distance);
                double currentRpm = ShooterSubsystem.getShooterVelocityRPM();
                double targetAngle = m_cap.getAngleForDistance(distance);
                double currentAngleDegrees = m_cap.getStepDegrees(); // Ângulo atual do cap em graus

                // Verifica alinhamento e tolerâncias de velocidade/ângulo
                boolean aligned = Math.abs(tx) < TX_TOLERANCE;
                // boolean rpmReached = Math.abs(targetRpm - currentRpm) < RPM_TOLERANCE;
                boolean angleReached = Math.abs(targetAngle - currentAngleDegrees) < ANGLE_TOLERANCE;

                // if (aligned && rpmReached && angleReached) {
                //     ledBrightGreenBlink(); // Todas as condições atendidas: pisca verde
                // } else {
                //     ledAlphabots(); // Não pronto, mas shooter ativo
                // }
            } else {
                ledAlphabots(); // Sem alvo, distância inválida ou cap não estendido, mas shooter ativo
            }
        } else {
            ledAlphabots(); // Shooter não está ativo, mostra padrão Alphabots
        }
    }

    /**
     * Define toda a fita para uma cor RGB específica.
     */
    public void setColor(int r, int g, int b) {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }
    }

    
    public void ledRainbow() {
        /*
         * Cria um efeito de arco-íris em movimento.
         * Deve ser chamado repetidamente (ex: dentro de um comando).
         */
        // Para cada pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Calcula o Hue (matiz)
            final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
            // Define o valor HSV (Hue, Saturation, Value)
            m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Incrementa o hue inicial para fazer o arco-íris se mover
        m_rainbowFirstPixelHue += 3;
        m_rainbowFirstPixelHue %= 180;
    }

    public void ledBlue() {
        setColor(0, 0, 255);
    }

    public void ledRed() {
        setColor(255, 0, 0);
    }

    public void ledGreen() {
        setColor(0, 255, 0);
    }

    public void ledPurple() {
        setColor(128, 0, 128);
    }

    public void ledYellow() {
        setColor(255, 255, 0);
    }

    public void ledOff() {
        setColor(0, 0, 0);
    }

    public void ledAlphabots() {
        /*
         * Cria um efeito de cor "Alphabots" que oscila intercalando as cores #01532F e #0EA23A.
         * Deve ser chamado repetidamente (ex: dentro de um comando).
         */
        // Cores Alphabots em RGB
        final int r1 = 1, g1 = 83, b1 = 47;   // #01532F
        final int r2 = 14, g2 = 162, b2 = 58; // #0EA23A

        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Alterna as cores com base no índice do pixel e na fase atual
            if (((i + m_alphabotsPhase) % 2) == 0) {
                m_ledBuffer.setRGB(i, r1, g1, b1);
            } else {
                m_ledBuffer.setRGB(i, r2, g2, b2);
            }
        }
        m_alphabotsPhase = (m_alphabotsPhase + 1) % 2; // Inverte a fase para a próxima chamada
    }

    /**
     * Faz a fita de LED piscar em verde brilhante.
     * Deve ser chamado repetidamente (ex: dentro de um comando).
     */
    public void ledBrightGreenBlink() {
        if (m_blinkCounter < BLINK_PERIOD) {
            setColor(0, 255, 0); // Verde Brilhante
        } else {
            setColor(0, 0, 0); // Desligado
        }
        m_blinkCounter = (m_blinkCounter + 1) % (BLINK_PERIOD * 2); // Cicla o contador
    }
}