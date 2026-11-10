package org.firstinspires.ftc.teamcode.pedro.auto;

import static com.pedropathing.api.Paths.*;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.DataStorage;
import org.firstinspires.ftc.teamcode.pedro.Flywheel;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "redBasic")
public class redBasic extends OpMode {

    private Follower follower;


    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56.4631, 8.4813, 270);
    private final Pose shoot = poseFactory.of(56.2323, 22.9885, 270);
    private final Pose park = poseFactory.of(13.7241, 94.4762, 89.9307);
    private final Pose parkControlPoint = poseFactory.of(13.4548, 49.7151, 0);

    private com.pedropathing.paths.Path startShoot;
    private Path shootPark;

    private DcMotor turret0;

    private DcMotor turret1;

    DcMotor transfer0;
    DcMotor transfer1;

    Timer powerUpTimer;

    Timer shootTimer;

    Servo turretServo;

    Flywheel flywheel = new Flywheel();
    private double turretPower = 0.66;


    @Override
    public void init() {

        Scheduler.reset();

        follower = Constants.create(hardwareMap);

        initPaths();

        follower.setPose(start);

        turret0 = hardwareMap.get(DcMotor.class, "flywheel0");
        turret1 = hardwareMap.get(DcMotor.class, "flywheel1");
        turret0.setDirection(DcMotorSimple.Direction.REVERSE);

        transfer0 = hardwareMap.get(DcMotor.class, "transfer0");
        transfer1 = hardwareMap.get(DcMotor.class, "transfer1");
        transfer1.setDirection(DcMotorSimple.Direction.REVERSE);

        turretServo = hardwareMap.get(Servo.class, "turretServo");
        turretServo.setPosition(0.15);

        powerUpTimer = new Timer();
        shootTimer = new Timer();

        flywheel.init(hardwareMap);
        flywheel.setPidf(1.355, 0.0, 0.02, 8.5);
    }

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        turretServo.setPosition(0.15);
        follower.update();
        Scheduler.execute();
    }

    @Override
    public void stop() {
        DataStorage.autonomousEndPose = follower.pose();
    }

    public void initPaths() {
        startShoot = Paths.line(start, shoot).linear(start, shoot);
        shootPark = Paths.curve(shoot, parkControlPoint, park).tangent();
    }

    private Command autoRoutine() {
        return sequential(
            follow(follower, startShoot),
            powerUp(),
            shoot(),
            follow(follower, shootPark)
        );
    }

    private Command powerUp() {
        return Command.build()
                .setStart(() -> {
                    powerUpTimer.reset();
                    flywheel.update(1.0 * turretPower * flywheel.maxOmega);
                })
                .setExecute(() -> {
                    flywheel.update(1.0 * turretPower * flywheel.maxOmega);
                })
                .setDone(() -> {
                    return powerUpTimer.get(TimeUnit.SECONDS) > 2;
                });
    }

    private Command shoot() {
        return Command.build()
                .setStart(() -> {
                    shootTimer.reset();
                    flywheel.update(1.0 * turretPower * flywheel.maxOmega);
                    transfer0.setPower(0.8);
                    transfer1.setPower(0.8);
                })
                .setExecute(() -> {
                    flywheel.update(1.0 * turretPower * flywheel.maxOmega);
                })
                .setDone(() -> {
                    return shootTimer.get(TimeUnit.SECONDS) > 4;
                })
                .setEnd(endCondition -> {
                    turret0.setPower(0.0);
                    turret1.setPower(0.0);
                    transfer0.setPower(0.0);
                    transfer1.setPower(0.0);
                    flywheel.update(0.0);
                });
    }


}
