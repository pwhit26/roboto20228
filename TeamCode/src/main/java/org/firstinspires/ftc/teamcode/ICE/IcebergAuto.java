package org.firstinspires.ftc.teamcode.ICE;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
//START AT THE MIDDLE OF THE BLUE GOAL
    public class IcebergAuto extends OpMode {
    public ElapsedTime runtime = new ElapsedTime();
    long startTime = 0;
    int pathState= 0;
    private Limelight3A limelight;
    int aimSettleCount = 0;
    double servoPos;
    long aimStartMs = 0;

    long sequenceStartTime = 0;

    private double cameraHeightM = 0.27;      // set your camera height (m)
    private double tagHeightM = 0.80;         // set your tag center height (m)
    private double cameraMountPitchDeg = 25.0; // camera tilt up (+deg)
    //Motors
    private DcMotorEx turret, intake, transferR, transferL;

    //Servos
    private Servo popup, angleTurret0, angleTurret1, turnTurret;
    private CRServo spinny;

    //Follower
    private Follower follower;

    //Points
    private Pose IceBerg, shoot, grabBalls;
    private PathChain shoot1, grab1, shoot2;

    //Booleans
    boolean aimActive = false;
    boolean popSequenceActive = false;
    boolean popSequenceComplete = true;

    @Override
    public void init() {
        //Initialize
        turret=hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake=hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferR=hardwareMap.get(DcMotorEx.class, "transferR");
        transferR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferL=hardwareMap.get(DcMotorEx.class, "transferL");
        transferL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servoPos = turnTurret.getPosition() * 360;

        popup=hardwareMap.get(Servo.class, "popup");
        popup.setPosition(0.14);
        angleTurret0 = hardwareMap.get(Servo.class, "angleTurret0");
        angleTurret0.setPosition(0.04);
        angleTurret1 = hardwareMap.get(Servo.class, "angleTurret1");
        angleTurret1.setPosition(0.95);
        spinny=hardwareMap.get(CRServo.class, "spinny");
        turnTurret=hardwareMap.get(Servo.class, "turnTurret");
        turnTurret.scaleRange(0, 1);
        turnTurret.setPosition(0);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        if (limelight != null) {
            limelight.pipelineSwitch(1);
            limelight.start();
        }




        //Pedro Pathing Visualizer Values
        IceBerg = new Pose(22.1, 125.1, Math.toRadians(325));
        shoot = new Pose(57.7, 87.8, Math.toRadians(180));
        grabBalls = new Pose(15, 86.3, Math.toRadians(-178));

        //Our field zero values
        //IceBerg = new Pose(0, 0, Math.toRadians(325));
        //shoot = new Pose(40, 38, Math.toRadians(180));
        //grabBalls = new Pose(42, -4, Math.toRadians(-178));

        follower= Constants.createFollower(hardwareMap);
        follower.setStartingPose(IceBerg);

        //Build paths
        shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(IceBerg, shoot))
                .setLinearHeadingInterpolation(IceBerg.getHeading(), shoot.getHeading())
                .build();
        grab1 = follower.pathBuilder()
                .addPath(new BezierLine(shoot, grabBalls))
                .setTangentHeadingInterpolation()
                .build();
        shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(grabBalls, shoot))
                .setLinearHeadingInterpolation(grabBalls.getHeading(), shoot.getHeading())
                .build();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void correct() {
        if (limelight != null) {
            LLResult ll = limelight.getLatestResult();
            double txDeg = 0.0;
            double tyDeg = 0.0;
            double ta = 0.0;
            boolean llValid = false;
            if (ll != null) {
                txDeg = ll.getTx();
                tyDeg = ll.getTy();
                ta = ll.getTa();
                llValid = ll.isValid();
            }

            if (llValid) {

                    telemetry.addData("LL","x - aprilTag");
                    double epsStartDeg = 1.0;
                    aimActive = Math.abs(txDeg) > epsStartDeg;
                    if (aimActive) {
                        aimSettleCount = 0;
                        aimStartMs = System.currentTimeMillis();
                    }

                if (aimActive) {
                    //double kP = 0.015;
                    //double minPower = 0.12;
                    //double rxAuto;
                    double epsDriveDeg = 1.0;
                    if (Math.abs(txDeg) > epsDriveDeg) {
                        //rxAuto = kP * txDeg;
                        servoPos = servoPos + txDeg;
                        turnTurret.setPosition(servoPos/360);
                        //if (Math.abs(rxAuto) < minPower) {
                        //rxAuto = Math.copySign(minPower, rxAuto);
                        //}
                    } //else {
                    //rxAuto = 0.0;
                    //}
                    //if (rxAuto > 0.6) rxAuto = 0.6;
                    //if (rxAuto < -0.6) rxAuto = -0.6;
                    //rx = rxAuto;

                    double epsDeg = 1.0;
                    if (Math.abs(txDeg) <= epsDeg) {
                        aimSettleCount++;
                    } else {
                        aimSettleCount = 0;
                    }
                    if (aimSettleCount >= 5) {
                        aimActive = false;
                    }
                }
        }}}


        @Override
    public void loop() {
        //Switch Cases
        long elapsedTime = System.currentTimeMillis() - startTime;
        switch (pathState)
        {
            case 0: //start first path
                follower.followPath(shoot1);
                if (elapsedTime >= 3500) {
                    pathState++;
                    aimActive=true;
                    startTime = System.currentTimeMillis();
                }
                telemetry.addData("Status", "Starting first path");
                break;
            case 1: //correct to goal
                if (!follower.isBusy())
                {
                    correct();
                }
                pathState++;
                break;
            case 2: //shoot 2 preload
                if (!follower.isBusy())
                {
                    transferL.setPower(0.3);
                    transferR.setPower(0.3);
                    spinny.setPower(0.7);
                    turret.setPower(1);
                    if (elapsedTime>=6500)
                    {
                        pathState++;
                        startTime= System.currentTimeMillis();
                    }
                }
                break;
            case 3: //shoot 3rd preload
                popup.setPosition(0.105);
                if (elapsedTime >= 7500) {
                    pathState++;
                    //sequenceStartTime = System.currentTimeMillis();
                }
                break;
            case 4: //finish shooting 3rd preload
                popup.setPosition(0.14);
                popSequenceComplete = true;
                popSequenceActive = false;
                pathState++;
                //sequenceStartTime = 0;
                break;
            case 5: //turn off shooting
                transferL.setPower(0);
                transferR.setPower(0);
                spinny.setPower(0);
                turret.setPower(0);
                pathState++;
                break;
            case 6: //grab balls
                intake.setPower(1);
                transferL.setPower(0.3);
                transferR.setPower(0.3);
                follower.followPath(grab1);
                if (elapsedTime>=10000)
                {
                    pathState++;
                    startTime=System.currentTimeMillis();
                }
                break;
            case 7: //move to shoot
                intake.setPower(0);
                transferL.setPower(0.2);
                transferR.setPower(0.2);
                follower.followPath(shoot2);
                if (elapsedTime>=13000)
                {
                    pathState++;
                    startTime=System.currentTimeMillis();
                }
                break;
            case 8: //correct to AprilTag
                if (!follower.isBusy())
                {
                    correct();
                }
                pathState++;
                break;
            case 9: //shoot 2 balls
                if (!follower.isBusy())
                {
                    transferL.setPower(0.3);
                    transferR.setPower(0.3);
                    spinny.setPower(0.7);
                    turret.setPower(1);
                    if (elapsedTime>=6500)
                    {
                        pathState++;
                        startTime= System.currentTimeMillis();
                    }
                }
                break;
            case 10: //shoot last ball
                popup.setPosition(0.105);
                if (elapsedTime >= 7500) {
                    pathState++;
                    //sequenceStartTime = System.currentTimeMillis();
                }
                break;
            case 11: //finish shooting last ball
                popup.setPosition(0.14);
                popSequenceComplete = true;
                popSequenceActive = false;
                pathState++;
                //sequenceStartTime = 0;
                break;
            case 12: //turn off
                transferL.setPower(0);
                transferR.setPower(0);
                spinny.setPower(0);
                turret.setPower(0);
                intake.setPower(0);
                pathState++;
                break;
            case 13: //finish
                terminateOpModeNow();
        }

    }

}
