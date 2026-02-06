package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class AnalogTest extends LinearOpMode {
    private AnalogInput encoder;
    private DcMotorEx spindexer;
    @Override
    public void runOpMode() throws InterruptedException {
        // Get analog port instance from hardwareMap
        encoder = hardwareMap.get(AnalogInput.class, "encoder");
        spindexer=hardwareMap.get(DcMotorEx.class, "spindexer");
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        while (opModeIsActive())
        {
            // Simple position return
            double position = encoder.getVoltage() / 3.2 * 360;

// Position return with adjustable offset
// Offset must be positive, otherwise the modulo (%) operation breaks
//
// If you want to use a negative offset, just add some multiple of 360 so
// that the offset evaluates to be positive.
            double offset = -123.4 + 360;
            double offsetPosition = (encoder.getVoltage() / 3.2 * 360 + offset) % 360;

            if (gamepad1.right_bumper)
            {
                spindexer.setPower(0.2);
            }
            else {
                spindexer.setPower(0);

            }


            telemetry.addData("Voltage: ", encoder.getVoltage());
            telemetry.addData("Offset Position: ", offsetPosition);
            telemetry.update();
        }
    }
}
