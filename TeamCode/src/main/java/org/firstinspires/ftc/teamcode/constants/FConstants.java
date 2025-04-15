package org.firstinspires.ftc.teamcode.constants;

import com.pedropathing.localization.*;
import com.pedropathing.follower.*;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.DRIVE_ENCODERS;

        FollowerConstants.leftFrontMotorName = "leftFront";
        FollowerConstants.leftRearMotorName = "leftRear";
        FollowerConstants.rightFrontMotorName = "rightFront";
        FollowerConstants.rightRearMotorName = "rightRear";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.REVERSE;

        FollowerConstants.mass = 13;

        FollowerConstants.xMovement = 59.68591114417795;
        FollowerConstants.yMovement = 60.409023240213436;

        FollowerConstants.forwardZeroPowerAcceleration = -104.677768;
        FollowerConstants.lateralZeroPowerAcceleration = -560.371639;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.5,0,0.0,0.2);
        FollowerConstants.useSecondaryTranslationalPID = true;

        FollowerConstants.headingPIDFCoefficients.setCoefficients(0.5,0,0,0.2);
        FollowerConstants.useSecondaryHeadingPID = true;

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.1,0,0,0.6,0);
        FollowerConstants.useSecondaryDrivePID = true;

        FollowerConstants.zeroPowerAccelerationMultiplier = 1;
        FollowerConstants.centripetalScaling = 0.0005;

        FollowerConstants.pathEndTimeoutConstraint = 500;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;
    }
}
