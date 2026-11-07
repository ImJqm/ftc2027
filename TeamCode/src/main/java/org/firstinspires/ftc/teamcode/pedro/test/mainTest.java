package org.firstinspires.ftc.teamcode.pedro.test;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="mainTest")
public class mainTest extends OpMode {

    //Declare motors
    DcMotor motorFrL;
    DcMotor motorFrR;
    DcMotor motorBL;
    DcMotor motorBR;

    DcMotor transfer0;
    DcMotor transfer1;
    DcMotor turret0;
    DcMotor turret1;

    boolean turretOn = false;

    boolean transferOn = false;

    double turretPower = 0.2;

    double transferPower = 0.2;

    Servo stopperServo;
    //Declare IMU
    IMU imu;

    @Override
    public void init() {
        //Instatiate motors
        motorFrL = hardwareMap.get(DcMotor.class,"lf");
        motorFrR = hardwareMap.get(DcMotor.class,"rf");
        motorBL = hardwareMap.get(DcMotor.class,"lr");
        motorBR = hardwareMap.get(DcMotor.class,"rr ");

        transfer0 = hardwareMap.get(DcMotor.class, "transfer0");
        transfer1 = hardwareMap.get(DcMotor.class, "transfer1");
        transfer1.setDirection(DcMotorSimple.Direction.REVERSE);

        turret0 = hardwareMap.get(DcMotor.class, "flywheel0");
        turret1 = hardwareMap.get(DcMotor.class, "flywheel1");
        turret0.setDirection(DcMotorSimple.Direction.REVERSE);

        //These are backwards for a mechanum drivetrain
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrL.setDirection(DcMotorSimple.Direction.REVERSE);

        //Resist pushing
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //init IMU
        imu = hardwareMap.get(IMU.class, "imu");

        stopperServo = hardwareMap.get(Servo.class, "stopper");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT ));
        imu.initialize(parameters);

    }

    @Override
    public void loop() {

        //Get Controller Input
        double y = -gamepad2.left_stick_y;
        double x = gamepad2.left_stick_x;
        double rx = gamepad2.right_stick_x;

        //Pressing Plus Resets the Zero Point
        if (gamepad2.options) {
            imu.resetYaw();
        }

        if (gamepad2.aWasPressed()) {
            turretOn = !turretOn;
        }

        if (gamepad2.rightBumperWasPressed()) {
            turretPower+=0.05;
            if (turretPower>1.0) {
                turretPower = 1.0;
            }
        }
        if (gamepad2.leftBumperWasPressed()) {
            turretPower-=0.05;
            if (turretPower<0.0) {
                turretPower = 0.0;
            }
        }
        if (gamepad2.dpadLeftWasPressed()) {
            transferPower+=0.05;
            if (transferPower>1.0) {
                transferPower = 1.0;
            }
        }
        if (gamepad2.dpadRightWasPressed()) {
            transferPower-=0.05;
            if (transferPower<0.0) {
                transferPower = 0.0;
            }
        }


        //Get Bot Angle
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botHeading) - y* Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX*1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        motorFrL.setPower(frontLeftPower);
        motorFrR.setPower(frontRightPower);
        motorBL.setPower(backLeftPower);
        motorBR.setPower(backRightPower);

        transfer0.setPower(gamepad2.right_trigger_pressed ? transferPower : 0.0);
        transfer1.setPower(gamepad2.right_trigger_pressed ? transferPower: 0.0);

        turret0.setPower(turretOn ? turretPower : 0.0);
        turret1.setPower(turretOn ? turretPower: 0.0);

        telemetry.addData("turretPower:", turretPower);
        telemetry.addData("transferPower:", transferPower);
        telemetry.addData("turretOn:", turretOn);
    }
}
