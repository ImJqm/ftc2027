package org.firstinspires.ftc.teamcode.pedro.test;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.DataStorage;
import org.firstinspires.ftc.teamcode.pedro.Flywheel;

@TeleOp(name = "turretTest")
public class turretTest extends OpMode {
    private Follower follower;

    private double angleOffset = 0.0;

    boolean turretOn = false;
    double turretPower = 0.55;

    private Servo turretServo;

    private double servoBlockerStart = 0.0;
    private double servoBlockerStop = 0.5;

    Flywheel flywheel = new Flywheel();

    DcMotor transfer;
    DcMotor intake;

    boolean reverse = false;

    boolean shooting = false;



    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.update();

        //flywheel.init(hardwareMap);

        transfer = hardwareMap.get(DcMotor.class, "transfer0");
        intake = hardwareMap.get(DcMotor.class, "transfer1");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheel.init(hardwareMap);

        turretServo = hardwareMap.get(Servo.class, "turretServo");
        turretServo.setPosition(0.15);
    }

    @Override
    public void start() {
        follower.setPose(DataStorage.autonomousEndPose);
    }

    @Override
    public void loop() {

        turretServo.setPosition(0.15);

        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad2.left_stick_y,
                -gamepad2.left_stick_x,
                -gamepad2.right_stick_x,
                follower.pose().heading(),
                -angleOffset
        );

        ManualDrive.driveOrHold(follower, powers);

        //this SHOULD theoretically reset the mechanum drice train but I can't test it rn so ig well never know
        if (gamepad2.startWasPressed()) {
            angleOffset = follower.pose().heading();
        }

        follower.update();


        //Telemetry
        Pose pose = follower.pose();
        telemetry.addData("X", pose.x());
        telemetry.addData("Y", pose.y());
        telemetry.addData("Heading", pose.heading());

        //Experimental!
        //This should theretically always point a servo towards the middle of the feild to its best ability
        //This kinda assumes that Servo.setPosition(0.0) is the center of the feild
        //I need to doulbe check field geonetry this is most likely totally wrong
        //telemetry.addData("Experimental Angle From Center of Feild", CoolMath.coolArctan(72, 72, pose.x(), pose.y()));


        if (gamepad2.aWasPressed()) {
            turretOn = !turretOn;
        }

        if (gamepad2.rightBumperWasPressed()) {
            turretPower+=0.01;
            if (turretPower>1.0) {
                turretPower = 1.0;
            }
        }
        if (gamepad2.leftBumperWasPressed()) {
            turretPower-=0.01;
            if (turretPower<0.0) {
                turretPower = 0.0;
            }
        }


        if (gamepad2.xWasPressed()) {
            shooting = !shooting;
        }

        reverse = gamepad2.left_trigger_pressed;

        if (reverse) {
            transfer.setDirection(DcMotorSimple.Direction.REVERSE);
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            transfer.setDirection(DcMotorSimple.Direction.FORWARD);
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        if (gamepad2.dpadLeftWasPressed()) {
            flywheel.setPIDFIncrement(flywheel.increment-0.01);
        }

        if (gamepad2.dpadRightWasPressed()) {
            flywheel.setPIDFIncrement(flywheel.increment+0.01);
        }

        if (gamepad2.dpadUpWasPressed()) {
            flywheel.changeCurrent(1.0);
        }
        if (gamepad2.dpadDownWasPressed()) {
            flywheel.changeCurrent(-1.0);
        }

        if (gamepad2.bWasPressed()) {
            flywheel.nextState();
        }

        if (shooting) {
            transfer.setDirection(DcMotorSimple.Direction.FORWARD);
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
            transfer.setPower(1.0);
            intake.setPower(1.0);
        } else {
            transfer.setPower(0.0);
            intake.setPower(gamepad2.right_trigger_pressed || gamepad2.left_trigger_pressed? 0.9 : 0 );
        }

        flywheel.update(turretOn ? 1.0*turretPower*flywheel.maxOmega : 0.0);

        telemetry.addData("turretPower", turretPower);
        telemetry.addData("Shooting", shooting);
        telemetry.addData("Currently Changing", flywheel.pidf);
        telemetry.addData("P", flywheel.P);
        telemetry.addData("I", flywheel.I);
        telemetry.addData("D", flywheel.D);
        telemetry.addData("F", flywheel.F);
        telemetry.addData("Increment Value", flywheel.increment);
        telemetry.addData("turretPower:", turretPower);
        telemetry.addData("Target Speed (Ticks Per Second)", turretOn ? 1.0*turretPower*flywheel.maxOmega : 0.0);
        telemetry.addData("Current Speed: (Ticks Per Second)", flywheel.getVelocity());

    }
}
