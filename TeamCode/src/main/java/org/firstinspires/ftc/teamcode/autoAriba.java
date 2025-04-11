package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.constants.FConstants;
import org.firstinspires.ftc.teamcode.constants.LConstants;

@Autonomous(name = "Ariba", group = "Examples")
public class autoAriba extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(0, 71, Math.toRadians(0));

    private final Pose Clip = new Pose(10, 71, Math.toRadians(0));

    private final Pose posClip = new Pose(10, 10, Math.toRadians(0));

    private final Pose specimen = new Pose(40, 10, Math.toRadians(0));

    //private final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0));

    //private final Pose parkPose = new Pose(60, 98, Math.toRadians(90));

    //private final Pose parkControlPose = new Pose(60, 98, Math.toRadians(90));

    private Path scorePreload, park, clipa;
    private PathChain clipada, grabPickup2, grabPickup3, aposClip, scorePickup2, scorePickup3;
    private PathChain traj1;
    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(Clip)));
        scorePreload.setTangentHeadingInterpolation();

        clipa = new Path(new BezierLine(new Point(Clip), new Point(posClip)));
        clipa.setConstantHeadingInterpolation(Math.toRadians(0));

        traj1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(Clip)))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(new Point(Clip), new Point(posClip)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(new Point(posClip), new Point(specimen)))
                .setTangentHeadingInterpolation()
                .build();



        clipada = follower.pathBuilder()
                .addPath(new BezierLine(new Point(Clip), new Point(posClip)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        aposClip = follower.pathBuilder()
                .addPath(new BezierLine(new Point(posClip), new Point(specimen)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        //grabPickup2 = follower.pathBuilder()
                //.addPath(new BezierLine(new Point(Clip), new Point(specimen)))
                //.setLinearHeadingInterpolation(Clip.getHeading(), specimen.getHeading())
                //.build();

        //scorePickup2 = follower.pathBuilder()
                //.addPath(new BezierLine(new Point(specimen), new Point(Clip)))
                //.setLinearHeadingInterpolation(specimen.getHeading(), Clip.getHeading())
                //.build();

        //grabPickup3 = follower.pathBuilder()
                //.addPath(new BezierLine(new Point(Clip), new Point(pickup3Pose)))
                //.setLinearHeadingInterpolation(Clip.getHeading(), pickup3Pose.getHeading())
                //.build();

        //scorePickup3 = follower.pathBuilder()
                //.addPath(new BezierLine(new Point(pickup3Pose), new Point(Clip)))
                //.setLinearHeadingInterpolation(pickup3Pose.getHeading(), Clip.getHeading())
                //.build();

        //park = new Path(new BezierCurve(new Point(Clip), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
        //park.setLinearHeadingInterpolation(Clip.getHeading(), parkPose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(traj1);
                setPathState(3);
                break;
            case 1:

                if(!follower.isBusy()) {

                    follower.followPath(clipa);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {

                    follower.followPath(aposClip);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {

                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {

        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
    }
}
