package org.firstinspires.ftc.teamcode.pedro.teleop;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive.*;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.CoolMath;
import org.firstinspires.ftc.teamcode.pedro.DataStorage;

@TeleOp(name = "pedroTeleOP")
public class pedroTeleOp extends OpMode {
    private Follower follower;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.update();
    }

    @Override
    public void start() {
        follower.setPose(DataStorage.autonomousEndPose);
    }

    @Override
    public void loop() {
        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad2.left_stick_y,
                gamepad2.left_stick_x,
                gamepad2.right_stick_x,
                follower.pose().heading()
        );

        ManualDrive.driveOrHold(follower, powers);

        //this SHOULD theoretically reset the mechanum drice train but I can't test it rn so ig well never know
        if (gamepad2.startWasPressed()) {
            Pose repose = new Pose(follower.pose().x(), follower.pose().y(), 0);
            follower.setPose(repose);
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
        telemetry.addData("Experimental Angle From Center of Feild", CoolMath.coolArctan(72, 72, pose.x(), pose.y()));

    }
}
