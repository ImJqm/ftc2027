package org.firstinspires.ftc.teamcode.pedro;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Flywheel {

    public enum PIDFINCREMENT {
        P {
            @Override
            public PIDFINCREMENT nextState() {
                return I;
            }
        },
        I {
            @Override
            public PIDFINCREMENT nextState() {
                return D;
            }
        },
        D {
            @Override
            public PIDFINCREMENT nextState() {
                return F;
            }
        },

        F {
            @Override
            public PIDFINCREMENT nextState() {return P;}
        };

        public abstract PIDFINCREMENT nextState() ;
    }
    public PIDFINCREMENT pidf;
    public Double increment = 0.1;
    public Double P = 1.0;
    public Double I = 0.0;
    public Double D = 1.0;
    public Double F = 1.0;
    private final Double TICKS_PER_REV = 28.0;

    public final Double maxOmega = 6000 / 60.0 * TICKS_PER_REV;

    public DcMotorEx motor0;

    public DcMotorEx motor1;

    public void init(HardwareMap hardwareMap) {

        pidf = PIDFINCREMENT.P;

        motor0 = hardwareMap.get(DcMotorEx.class, "flywheel0");
        motor1 = hardwareMap.get(DcMotorEx.class, "flywheel1");

        motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void update(Double targetTPS) {
        motor0.setVelocity(targetTPS);
        motor1.setVelocity(targetTPS);
    }

    public void applyPIDF() {
        motor0.setVelocityPIDFCoefficients(P, I, D, F);
        motor1.setVelocityPIDFCoefficients(P, I, D, F);
    }

    public void setPIDFIncrement(double inc) {
        increment = inc;
    }

    public void changeCurrent(double sign) {
        switch (pidf) {
            case P:
                P+=sign*increment;
                break;
            case I:
                I+=sign*increment;
                break;
            case D:
                D+=sign*increment;
                break;
            case F:
                F+=sign*increment;
                break;
        }

        applyPIDF();
    }

    public void nextState() {
        pidf = pidf.nextState();
    }

}
