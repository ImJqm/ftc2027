package org.firstinspires.ftc.teamcode.pedro;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


@Configurable
public class Flywheel {

    public enum State {
        SPINUP,
        READY,
        RECOVERING
    }

    public static double P = 120;
    public static double I = 0.0;
    public static double D = 0.1;
    public static double F = 13.0;

    public static double dipThreshold = 60.0;
    public static double recoverTolerance = 20.0;
    public static double spinupTolerance = 40.0;
    public static double boostGain = 10.0;
    public static double boostMax = 600.0;
    public static double ballDip = 70.0;

    private static final double TICKS_PER_REV = 28.0;

    public final double maxOmega = 6000 / 60.0 * TICKS_PER_REV;

    public DcMotorEx motor0;

    public DcMotorEx motor1;

    private double appliedP = Double.NaN;
    private double appliedI = Double.NaN;
    private double appliedD = Double.NaN;
    private double appliedF = Double.NaN;

    private State state = State.SPINUP;
    private double target = 0.0;
    private double command = 0.0;
    private double boost = 0.0;
    private double velocity0 = 0.0;
    private double velocity1 = 0.0;
    private double currentDip = 0.0;
    private double lastDip = 0.0;
    private int shotCount = 0;

    public void init(HardwareMap hardwareMap) {
        motor0 = hardwareMap.get(DcMotorEx.class, "flywheel0");
        motor1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        motor0.setDirection(DcMotorSimple.Direction.REVERSE);

        motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        applyPIDF();
    }

    public void update(double targetTPS) {
        if (P != appliedP || I != appliedI || D != appliedD || F != appliedF) {
            applyPIDF();
        }

        velocity0 = motor0.getVelocity();
        velocity1 = motor1.getVelocity();

        if (targetTPS != target) {
            target = targetTPS;
            state = State.SPINUP;
            currentDip = 0.0;
        }

        double error = target - getVelocity();

        if (target <= 0.0) {
            state = State.SPINUP;
        } else {
            switch (state) {
                case SPINUP:
                    if (Math.abs(error) < spinupTolerance) {
                        state = State.READY;
                    }
                    break;
                case READY:
                    if (error > dipThreshold) {
                        state = State.RECOVERING;
                        currentDip = error;
                        shotCount++;
                    }
                    break;
                case RECOVERING:
                    currentDip = Math.max(currentDip, error);
                    if (error < recoverTolerance) {
                        lastDip = currentDip;
                        currentDip = 0.0;
                        state = State.READY;
                    }
                    break;
            }
        }

        boost = state == State.RECOVERING ? Math.min(boostMax, boostGain * Math.max(currentDip, ballDip)) : 0.0;
        command = target + boost;

        motor0.setVelocity(command);
        motor1.setVelocity(command);
    }

    public void applyPIDF() {
        PIDFCoefficients coefficients = new PIDFCoefficients(P, I, D, F);
        motor0.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
        motor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
        appliedP = P;
        appliedI = I;
        appliedD = D;
        appliedF = F;
    }

    public void setPidf(double P, double I, double D, double F) {
        Flywheel.P = P;
        Flywheel.I = I;
        Flywheel.D = D;
        Flywheel.F = F;
        applyPIDF();
    }

    public void resetShotTracking() {
        shotCount = 0;
        lastDip = 0.0;
        currentDip = 0.0;
    }

    public boolean isReady() {
        return state == State.READY;
    }

    public State getState() {
        return state;
    }

    public double getTarget() {
        return target;
    }

    public double getCommand() {
        return command;
    }

    public double getBoost() {
        return boost;
    }

    public double getVelocity() {
        return (velocity0 + velocity1) / 2.0;
    }

    public double getVelocity0() {
        return velocity0;
    }

    public double getVelocity1() {
        return velocity1;
    }

    public double getCurrentDip() {
        return currentDip;
    }

    public double getLastDip() {
        return lastDip;
    }

    public int getShotCount() {
        return shotCount;
    }

}
