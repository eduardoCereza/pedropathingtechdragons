package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "teste atuador")
public class teste_atuado extends OpMode {
    DcMotor armMotorL, armMotorR;
    public void init(){
        armMotorL = hardwareMap.get(DcMotorEx.class, "armmotorleft");
        armMotorR = hardwareMap.get(DcMotorEx.class, "armmotorright");
        armMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    public void loop(){
        telemetry.addData("Posição Left: ", armMotorL.getCurrentPosition());
        telemetry.addData("Posição Right: ", armMotorR.getCurrentPosition());

        if (gamepad2.a){
            armMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }
        telemetry.update();
    }
}
