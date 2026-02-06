package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class Spinnyyyyy extends LinearOpMode {
    private DcMotorEx spindexer;
    @Override
    public void runOpMode() throws InterruptedException {
        spindexer = hardwareMap.get(DcMotorEx.class, "spindexer");
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while (opModeIsActive())
        {
            if (gamepad1.y)
            {
                spindexer.setPower(0.5);
            }
            else if (gamepad1.a)
            {
                spindexer.setPower(0.2);
            }
            else if (gamepad1.b)
            {
                spindexer.setPower(0.8);
            }
            else if (gamepad1.x)
            {
                spindexer.setPower(-0.4);
            }
            else {
                spindexer.setPower(0);
            }
        }

    }
}
