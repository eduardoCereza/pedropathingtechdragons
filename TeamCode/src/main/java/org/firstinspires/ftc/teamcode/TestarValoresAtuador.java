package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Teste de Atuadores")
public class TestarValoresAtuador extends OpMode {

    DcMotor armL, armR;

    public void init(){
        armL = hardwareMap.get(DcMotor.class, "armmotorleft");
        armR = hardwareMap.get(DcMotor.class, "armmotorright");
        armL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void loop(){
        if (gamepad2.right_stick_y < 0){
            armR.setPower(0.5);
            armL.setPower(0.5);
        } else if (gamepad2.right_stick_y > 0) {
            armR.setPower(-0.1);
            armL.setPower(-0.1);
        }
        telemetry.addData("Pos Left: ", armL.getCurrentPosition());
        telemetry.addData("Pos Right: ", armR.getCurrentPosition());
        telemetry.addData("Joy valor:  ", gamepad2.right_stick_y);
        telemetry.update();
    }
}
