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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.DataStorage;

@Autonomous(name = "redBasic")
public class redBasic extends OpMode {

    private Follower follower;


    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(8.8333, 22.6379, 90);
    private final Pose shoot = poseFactory.of(56.2323, 22.9885, 90);
    private final Pose park = poseFactory.of(13.7241, 94.4762, 89.9307);
    private final Pose parkControlPoint = poseFactory.of(13.4548, 49.7151, 0);

    private com.pedropathing.paths.Path startShoot;
    private Path shootPark;

    @Override
    public void init() {

        Scheduler.reset();

        follower = Constants.create(hardwareMap);

        initPaths();

        follower.setPose(start);
    }

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
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
            waitMs(5000),
            follow(follower, shootPark)
        );
    }


}
