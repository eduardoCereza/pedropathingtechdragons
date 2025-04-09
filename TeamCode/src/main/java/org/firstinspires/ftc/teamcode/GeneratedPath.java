package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.constants.FConstants;
import org.firstinspires.ftc.teamcode.constants.LConstants;

@Autonomous
public class GeneratedPath {
    public GeneratedPath(){
        PathBuilder builder = new PathBuilder();

        builder
                .addPath(
                        new BezierLine(
                                new Point(5, 72, Point.CARTESIAN),
                                new Point(45.300, 72.220, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        new BezierLine(
                                new Point(45.300, 72.220, Point.CARTESIAN),
                                new Point(33.000, 53.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation();
    }

}
