
package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class test extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Declare motors
        DcMotor bl = hardwareMap.dcMotor.get("bl");
        DcMotor br = hardwareMap.dcMotor.get("br");
        DcMotor fl = hardwareMap.dcMotor.get("fl");
        DcMotor fr = hardwareMap.dcMotor.get("fr");
        DcMotor rightOuttake = hardwareMap.dcMotor.get("rout");
        DcMotor leftOuttake = hardwareMap.dcMotor.get("lout");
        DcMotor intake = hardwareMap.dcMotor.get("intake");

        // Declare servos
        Servo BLservo = hardwareMap.servo.get("BLservo");
        Servo BRservo = hardwareMap.servo.get("BRservo");
        Servo FRservo = hardwareMap.servo.get("FRservo");
        Servo uptake = hardwareMap.servo.get("uptake");

        // Motor directions
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        rightOuttake.setDirection(DcMotorSimple.Direction.REVERSE);
        leftOuttake.setDirection(DcMotorSimple.Direction.FORWARD);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double currentpos = intake.getCurrentPosition();
        double targetPos = 0.0;
        boolean sixtyplus = false;
        boolean sixtyminus = false;
        boolean onetwentyplus = false;
        boolean onetwentyminus = false;
        double spindexPWR = 0;
        boolean isMoving = false;

        //------------------------------------------------------
        // ENCODER CONSTANTS FOR SPINDEXER
        //------------------------------------------------------
        double encoderTicks = 8192;
        // IMPORTANT: must use floating point here
        double spindexerGR = 170.0 / 24.0;
        double ticksPerRev = encoderTicks * spindexerGR;
        double ticksPerDeg = ticksPerRev / 360.0;

        int spindexerTarget = -1;

        // IMU setup
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);

        // Smoothing timer for CR servos
        ElapsedTime servoTimer = new ElapsedTime();
        final int UPDATE_MS = 25;

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.options) {
                imu.resetYaw();
            }




            if (Math.abs(gamepad2.right_stick_x) > 0.05) {
                BRservo.setPosition((double) 1/3);
                BLservo.setPosition((double) 1/3);
                FRservo.setPosition((double) 1/3);
                //spindexerTarget = -1;   // cancel automatic mode
            }




            // Intake
            if (gamepad2.left_bumper) {
                intake.setPower(1);
            }
            else if (gamepad2.right_bumper) {
                intake.setPower(-1);
            }
            else {
                intake.setPower(0);
            }


            double  outP = 0.8*gamepad2.left_stick_y;
            if (gamepad2.x) {
                rightOuttake.setPower(0.8*outP);
                leftOuttake.setPower(0.8*outP);
            } else {
                rightOuttake.setPower(outP);
                leftOuttake.setPower(outP);
            }

            if (gamepad2.triangle) {
                uptake.setPosition(0.5);

            } else {
                uptake.setPosition(0);
            }


//            if (isMoving) {
//                boolean targetReached;
//                int currentIntakePos = intake.getCurrentPosition();
//
//                // Determine if the target has been reached based on the direction of travel
//                if (spindexPWR > 0) {
//                    // Positive movement: Target reached if current position is >= target
//                    targetReached = (currentIntakePos >= currentpos);
//                } else { // spindexPWR < 0
//                    // Negative movement: Target reached if current position is <= target
//                    targetReached = (currentIntakePos <= currentpos);
//                }
//
//                if (!targetReached) {
//                    // Keep running the servos in the set direction
//                    BRservo.setPower(spindexPWR);
//                    BLservo.setPower(spindexPWR);
//                    FRservo.setPower(spindexPWR);
//                } else {
//                    // Target reached: STOP the servos and reset the state
//                    BRservo.setPower(0);
//                    BLservo.setPower(0);
//                    FRservo.setPower(0);
//                    isMoving = false;
//                    spindexPWR = 0;
//                }
//            } else {
//                // If no movement is commanded, ensure the servos are off
//                BRservo.setPower(0);
//                BLservo.setPower(0);
//                FRservo.setPower(0);
//            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX *= 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            fl.setPower((rotY + rotX - rx) / denominator);
            bl.setPower((rotY - rotX - rx) / denominator);
            fr.setPower((rotY + rotX + rx) / denominator);
            br.setPower((rotY - rotX + rx) / denominator);


            telemetry.addData("Encoder Position", intake.getCurrentPosition());
            telemetry.addData("Target", spindexerTarget);
            telemetry.update();
        }
    }
}