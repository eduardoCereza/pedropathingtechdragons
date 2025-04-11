package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.constants.FConstants;
import org.firstinspires.ftc.teamcode.constants.LConstants;


/**
 * This is an example teleop that showcases movement and robot-centric driving.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 12/30/2024
 */


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOperado")
public class TeleOp_Mundial extends OpMode {
    private Follower follower;
    PController pController;
    DcMotorEx slide;
    private final Pose startPose = new Pose(0,0,0);

    /** This method is call once when init is played, it initializes the follower **/
    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        slide = hardwareMap.get(DcMotorEx.class, "gobilda");
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {
        if (gamepad1.right_trigger > 0){
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            follower.update();
            telemetry.addLine("Modo Normal - Joystick 1");
        }else if (gamepad1.left_trigger > 0){
            follower.setTeleOpMovementVectors(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, true);
            follower.update();
            telemetry.addLine("Modo Reverso - Joystick 1");
        }

        moveSlide();

        /* Telemetry Outputs of our Follower */
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

        /* Update Telemetry to the Driver Hub */
        telemetry.update();

    }

    //TODO: Mover Slide
    public void moveSlide(){
        double minPower =0.01;
        double maxPower =0.5;
        float j = gamepad1.left_stick_y;
        int limitMax = -3200;

        pController = new PController(1);
        pController.setInputRange(50, limitMax);
        pController.setOutputRange(minPower, maxPower);

        double powerA = minPower + pController.getComputedOutput(slide.getCurrentPosition());
        double powerB = minPower - pController.getComputedOutput(slide.getCurrentPosition());

        if (j > 0){
            slide.setPower(powerA);
        } else if (j < 0) {
            slide.setPower(-powerA);
        } else {
            slide.setPower(powerB);
        }

        if (slide.getCurrentPosition() > limitMax){
            slide.setPower(powerB);
        }
    }

    //TODO: Mover base do atuador
    public void moveBase(){

    }

    //Todo: Mover servo
    public void moveServo(){

    }
}