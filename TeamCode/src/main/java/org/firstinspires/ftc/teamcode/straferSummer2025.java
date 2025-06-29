package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;



@TeleOp

public class straferSummer2025 extends LinearOpMode {
        private void setMotorPower( double pFL, double pBL, double pFR, double pBR){
            hardwareMap.dcMotor.get("frontLeft").setPower(pFL);
            hardwareMap.dcMotor.get("backLeft").setPower(pBL);
            hardwareMap.dcMotor.get("frontRight").setPower(pFR);
            hardwareMap.dcMotor.get("backRight").setPower(pBR);
        }

        private void setHardware() {
            hardwareMap.dcMotor.get("frontLeft").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hardwareMap.dcMotor.get("backLeft").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hardwareMap.dcMotor.get("frontRight").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hardwareMap.dcMotor.get("backRight").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            hardwareMap.dcMotor.get("frontLeft").setDirection(DcMotorSimple.Direction.REVERSE);
            hardwareMap.dcMotor.get("backLeft").setDirection(DcMotorSimple.Direction.REVERSE);
            hardwareMap.dcMotor.get("frontRight").setDirection(DcMotorSimple.Direction.FORWARD);
            hardwareMap.dcMotor.get("backRight").setDirection(DcMotorSimple.Direction.FORWARD);


        }

        private Servo initializeServo(){
            Servo servo = hardwareMap.servo.get("claw");
            servo.setPosition(0.35);
            return(servo);
        }

        private IMU initializeIMU(){
            IMU imu = hardwareMap.get(IMU.class, "imu");
            // Adjust the orientation parameters to match your robot
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
            // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
            imu.initialize(parameters);
            return imu;
        }



    @Override
        public void runOpMode() throws InterruptedException {

            setHardware();

            IMU imu=initializeIMU();

            Servo servo = initializeServo();

            DistanceSensor  backRightDistance = hardwareMap.get(DistanceSensor.class, "backRightDistance");
            DistanceSensor  backLeftDistance = hardwareMap.get(DistanceSensor.class, "backLeftDistance");

            waitForStart();

            if (isStopRequested()) return;

            final double tgtDistance = 4;
    //        final double allowedError = 1;
            double motorPower = 0;
            double i = 0;
            double previous_error = 0;
            double prev_time = 0;
            final double k_p = 0.3; //change later
            final double k_i = 0; //change later
            final double k_d = 0; //change later
            final double max_i = 1; //change later

            final double open_pos = 0.35;
            final double close_pos = 0.2;

            ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            runtime.reset();
            double period=100;//time in millisecond
            while (opModeIsActive()) {

                double current_time = runtime.time();

                double leftDistance = backLeftDistance.getDistance(DistanceUnit.INCH);
                double rightDistance = backRightDistance.getDistance(DistanceUnit.INCH);

                telemetry.addData("left range", leftDistance);
                telemetry.addData("right range", rightDistance);

                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                telemetry.addData("bot heading", botHeading);


                if (gamepad1.a) {
                    telemetry.addData("a is pressed", "");

                    double delta_time = current_time - prev_time;
                    double avgDistance = (leftDistance + rightDistance) / 2;
                    double current_error = tgtDistance - avgDistance;

                    double p = k_p * current_error;
                    i += k_i * (current_error * (delta_time));

                    if (i > max_i){
                        i = max_i;
                    }
                    else if (i < -max_i){
                        i = -max_i;
                    }

                    double d = k_d * (current_error - previous_error) / delta_time;

                    motorPower = p + i + d;
                    setMotorPower(motorPower,motorPower,motorPower,motorPower);

                    previous_error=current_error;
                    prev_time = current_time;

//                    if (error > allowedError) {
//                        setMotorPower(-motorPower,-motorPower,-motorPower,-motorPower);
//                    } else if (error < -allowedError) {
//                        setMotorPower(motorPower,motorPower,motorPower,motorPower);
//                    }

                } else if (gamepad1.options) {
                    imu.resetYaw();
                } else if (gamepad1.b) {
                    servo.setPosition(open_pos);
                } else if (gamepad1.x) {
                    servo.setPosition(close_pos);
                }
                else {
                    double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                    double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
                    double rx = gamepad1.right_stick_x;


                    // Rotate the movement direction counter to the bot's rotation
                    double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                    double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                    telemetry.addData("rot X", rotX);
                    telemetry.addData("rot Y", rotY);
                    telemetry.update();

                    // Denominator is the largest motor power (absolute value) or 1
                    // This ensures all the powers maintain the same ratio,
                    // but only if at least one is out of the range [-1, 1]
                    double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                    double frontLeftPower = (rotY + rotX + rx) / denominator;
                    double backLeftPower = (rotY - rotX + rx) / denominator;
                    double frontRightPower = (rotY - rotX - rx) / denominator;
                    double backRightPower = (rotY + rotX - rx) / denominator;
                    setMotorPower( frontLeftPower,backLeftPower,frontRightPower, backRightPower );

                }

                double new_time = runtime.time();
                while( new_time-current_time < period){
                    Thread.yield();
                    new_time = runtime.time();
                }

//
//
//                if (gamepad1.a) {
//                    telemetry.addData("a is pressed","");
//                    double tgtDistance = 4;
//                    double allowedError = 1;
//                    double motorPower = 0.3;
//                    while (true){
//                        double leftDistance = backLeftDistance.getDistance(DistanceUnit.INCH);
//                        double rightDistance = backRightDistance.getDistance(DistanceUnit.INCH);
//                        double avgDistance = (leftDistance + rightDistance)/2;
//                        double error = avgDistance-tgtDistance;
//                        if (error>allowedError) {
//                            setPowerAll(-motorPower,frontLeft, frontRight, backLeft, backRight);
//                        } else if (error<-allowedError) {
//                            setPowerAll(motorPower,frontLeft, frontRight, backLeft, backRight);
//                        }
//                        else {
//                            setPowerAll(0,frontLeft, frontRight, backLeft, backRight);
//                            break;
//                        }
//
//                    }
//                }





            }
        }
    }

