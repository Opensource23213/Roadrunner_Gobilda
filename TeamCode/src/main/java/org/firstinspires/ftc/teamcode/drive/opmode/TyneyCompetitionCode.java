package org.firstinspires.ftc.teamcode.drive.opmode;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;
import static java.lang.Math.abs;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.sensors;

/*
  This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
  the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
  of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
  class is instantiated on the Robot Controller and executed.

  This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
  It includes all the skeletal structure that all linear OpModes contain.

  Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
  Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TyneyCompetitionCode", group="ABC Opmode")
//@Disabled
public class TyneyCompetitionCode extends LinearOpMode {


    // Declare OpMode members.
     IMU imu;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor front_left = null;
    private DcMotor front_right = null;
    private DcMotor rear_left = null;
    private DcMotor rear_right = null; //also has encoder for vertical deadwheel

    private DcMotor Arm1 = null;

    private DcMotor Arm2 = null;

    private Servo elbow1;
    private Servo wristy;
    private Servo mustaches;
    private Servo ilifty;
    private Servo grabby;

    private CRServo spinny;
    private Servo flippy;
    private DcMotor ArmPos = null;

    private Servo lifty = null;
    private Servo shooty;
    private DistanceSensor sensorDistance;
    private DistanceSensor sensorDistance2;
    @Override
    public void runOpMode() {

        // Hardware Maps
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        // Retrieve and initialize the IMU.
        imu = hardwareMap.get(IMU.class, "imu");
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        rear_left = hardwareMap.get(DcMotor.class, "rear_left");
        rear_right = hardwareMap.get(DcMotor.class, "rear_right");
        Arm1 = hardwareMap.get(DcMotor.class, "Arm1");
        Arm2 = hardwareMap.get(DcMotor.class, "Arm2");
        elbow1 = hardwareMap.get(Servo.class, "elbow1");
        grabby = hardwareMap.get(Servo.class, "grabby");
        wristy = hardwareMap.get(Servo.class, "wristy");
        spinny = hardwareMap.get(CRServo.class, "spinny");
        flippy = hardwareMap.get(Servo.class, "flippy");
        ArmPos = hardwareMap.get(DcMotor.class, "ArmPos");
        shooty = hardwareMap.get(Servo.class, "shooty");
        lifty = hardwareMap.get(Servo.class, "lifty");
        mustaches = hardwareMap.get(Servo.class, "mustaches");
        ilifty = hardwareMap.get(Servo.class, "lifty2");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensorDistance");
        sensorDistance2 = hardwareMap.get(DistanceSensor.class, "sensorDistance2");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;
        Rev2mDistanceSensor sensorTimeOfFlight2 = (Rev2mDistanceSensor) sensorDistance2;
        double serAdjust = 3;

        //Arm_Encoder = new Encoder(hardwareMap.get(DcMotorEx.class, "parallelEncoder"));

        // Stuff like set direction
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        front_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        rear_left.setDirection(DcMotor.Direction.REVERSE);
        rear_right.setDirection(DcMotor.Direction.FORWARD);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rear_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rear_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm1.setDirection(DcMotor.Direction.FORWARD);
        Arm2.setDirection(DcMotor.Direction.REVERSE);
        Arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbow1.setDirection(Servo.Direction.REVERSE);
        lifty.setDirection(Servo.Direction.REVERSE);
        float wristOut = (float) ((float) .11 + serAdjust/355);
        float wristIn = (float) ((float) .6 + serAdjust/355);
        wristy.setPosition(wristIn);
        double grabIn = 1;
        double grabOut = .85;
        grabby.setPosition(grabIn);
        float servoDegree = 1 / 355;
        spinny.setDirection(CRServo.Direction.FORWARD);  //intake
        double pow = 0;
        flippy.setPosition(.5); //Pixel flipper servo
        shooty.setPosition(.5);
        double reInit = 0;
        double armInit = 2;
        lifty.setPosition(0);
        mustaches.setPosition(.28
        );
        ilifty.setPosition(.5);
        double counter8 = 1;


        // Define hub orientation
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        // Initialize the IMU with this mounting orientation
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        telemetry.addData("Hub orientation", "Logo=%s   USB=%s\n ", logoDirection, usbDirection);
        // Wait for the game to start (driver presses PLAY)
        // Buttons for gamepad1 are capitalized while gamepad2 buttons aren't
        waitForStart();
        runtime.reset();



        // Random Bustle of Variables for buttons and other stuff
        double ticks = 22.76; // ticks per degree in encoder
        float deadcirc = (float) 4.329; // Dead wheel circumference in inches
        double ticksinch = ticks * 83.16;
        double ticksfoot = ticksinch * 12;
        double max;
        double power_level = .8;
        double Pi = 3.1415926 / 2;
        int Drive_Mode = 2;
        double dis2 = 1;
        int xPress1 = 1;
        int xpress = 1;
        int aPress = 2;
        int apress = 1;
        int bPress = 1;
        int bpress = 1;
        int yPress = 1;
        int ypress = 1;
        int logic = 1;
        int Logic = 1;
        int reset = 1;
        double position = 0;
        double serPosition = .91;
        int counter = 0;
        int grabOpen = 0;
        int counter2 = 0;
        int counter3 = 0;
        double elbowTyney = 0; //266
        double wristTyney = 158;
        double shoulderTyney = 89;
        double aPress2 = 2;
        double flippydoo = .5;
        double pickflip = .2 ;
        shooty.setPosition(.5);
        double armAngle = 0;
        double shootcount = 0;
        double pixelbut = 0;
        double hey = 1;
        double xopen = 0;
        double ymove = 1;
        double count = 1;
        ElapsedTime timer = new ElapsedTime(SECONDS);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double dis = (sensorDistance.getDistance(DistanceUnit.INCH) + sensorDistance2.getDistance(DistanceUnit.INCH))/2;
            // servo position in degrees
            double elbowDegree = elbow1.getPosition() * 355;
            double wristDegree = wristy.getPosition() * 355 + serAdjust;

            //Variables Like Arm and Yaw
            // Setup a variable for each drive wheel to save power level for telemetry
            double frontleftPower;
            double frontrightPower;
            double rearleftPower;
            double rearrightPower;
            if(gamepad2.ps){
                ArmPos.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armInit = 1;
                position = 0;
            }
            if(armInit == 2){
                armAngle = ArmPos.getCurrentPosition() / ticks - 23;
            }else {
                armAngle = ArmPos.getCurrentPosition() / ticks;
            }


            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y / (gamepad1.left_trigger + 1);  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x / (gamepad1.left_trigger + 1);
            double yaw = gamepad1.right_stick_x / (gamepad1.left_trigger + 1);

            // joysticks on gamepad2 for arm
            double arm = gamepad2.left_stick_y/1.5; //power reduced to 50%
            double elbow = gamepad2.right_stick_y;

            // Eliminate stick drift
            if (gamepad2.left_stick_y < .05 && gamepad2.left_stick_y > -.05) {
                arm = 0;
            }

            // Retrieve Rotational Angles and Velocities
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

            // Check to see if heading reset is requested
            // Reset Field Centric

            if (gamepad1.ps) {
                telemetry.addData("Yaw", "Resetting\n");
                imu.initialize(new IMU.Parameters(orientationOnRobot));
                imu.resetDeviceConfigurationForOpMode();
                imu.resetYaw();
            } else {
                telemetry.addData("Yaw", "Press Middle Button (Logic) on Gamepad to reset\n");
            }
            if(gamepad1.right_trigger > .4 && gamepad1.a){
                if(shootcount < 2){
                    shootcount += 1;
                }else{
                    shooty.setPosition(.75);
                }
                lifty.setPosition(.2);
            }else{
                shooty.setPosition(.5);
                lifty.setPosition(0);
                shootcount = 0;
            }
            //Set Drive Mode
           /* if (gamepad1.x && xPress1 == 1) {  // Field Centric
                Drive_Mode = 1;
                xPress1 = 2;
            } /*else if (gamepad1.x && xPress1 == 1) {  // POV
                Drive_Mode = 0;
                xPress1 = 1;
            }*/


            //deactivate and activate Intake
            if (gamepad1.a) {
                ilifty.setPosition(.5);
                flippydoo = pickflip;
                spinny.setPower(0);
                yPress = 1;
                mustaches.setPosition(.332);
            }
            if (gamepad1.y && gamepad1.left_trigger > .4){
                mustaches.setPosition(mustaches.getPosition() + .01);
            }
            if (gamepad1.right_trigger > .4 && gamepad1.y) {
                flippydoo = .44;
                spinny.setPower(-1);
            } else if (gamepad1.y){
                spinny.setPower(1);
                flippydoo = .475;
                if (hey == 1){
                    if(yPress == 1){
                        ilifty.setPosition(.3);
                        mustaches.setPosition(.68);
                        yPress = 2;
                        hey = 2;
                    }else{
                        mustaches.setPosition(.42);
                        count = 2;
                        yPress = 1;
                        hey = 2;
                    }
                }
            }
            if (count >= 2 && count <= 18){
                count += 1;
            }else if(count >= 19){
                mustaches.setPosition(.28);
                ilifty.setPosition(.5);
                count = 1;
            }
            if (!gamepad1.y){
                hey = 1;
            }
            flippy.setPosition(flippydoo); //Moves flipper to position set on line 222 or 226

            // Field Centric Calculations
            double yaw_rad = orientation.getYaw(AngleUnit.RADIANS) + Pi;
            double temp = axial * Math.sin(yaw_rad) + lateral * Math.cos(yaw_rad);
            lateral = -axial * Math.cos(yaw_rad) + lateral * Math.sin(yaw_rad);
            //double temp = axial * Math.cos(yaw_rad) + lateral * Math.sin(yaw_rad);
            //lateral = -axial * Math.sin(yaw_rad) + lateral * Math.cos(yaw_rad);
            axial = temp;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = (axial + lateral + yaw) * power_level;
            double rightFrontPower = (axial - lateral - yaw) * power_level;
            double leftBackPower = (axial - lateral + yaw) * power_level;
            double rightBackPower = (axial + lateral - yaw) * power_level;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(abs(leftFrontPower), abs(rightFrontPower));
            max = Math.max(max, abs(leftBackPower));
            max = Math.max(max, abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Send calculated power to wheels


            if(grabOpen == 2) {
                dis2 = dis + .35;
                grabOpen = 3;
            }

            // D pad used to move the robot
            if(xPress1 == 1 && bPress == 1){
                if(grabOpen == 3 && ymove == 1) {
                    if (grabOpen == 3) {
                        if (dis2 > dis) {
                            front_left.setPower(-.2);
                            front_right.setPower(-.2);
                            rear_left.setPower(-.2);
                            rear_right.setPower(-.2);
                        } else {
                            grabOpen = 4;
                        }
                    }
                } else if (grabOpen == 3 && ymove == 2) {
                    grabOpen = 4;
                } else if(gamepad1.dpad_left){
                    if (ymove == 1) {
                        if (gamepad1.right_trigger > .4) {
                            front_left.setPower(-.71);
                            front_right.setPower(.71);
                            rear_left.setPower(.7);
                            rear_right.setPower(-.7);
                        } else {
                            front_left.setPower(-.31);
                            front_right.setPower(.31);
                            rear_left.setPower(.3);
                            rear_right.setPower(-.3);
                        }
                    }else{
                        if (gamepad1.right_trigger > .4) {
                            front_left.setPower(.7);
                            front_right.setPower(-.7);
                            rear_left.setPower(-.8);
                            rear_right.setPower(.8);
                        }else {
                            front_left.setPower(.3);
                            front_right.setPower(-.3);
                            rear_left.setPower(-.4);
                            rear_right.setPower(.4);
                        }
                    }
                }else if(gamepad1.dpad_right){
                    if(ymove == 1) {
                        if (gamepad1.right_trigger > .4) {
                            front_left.setPower(.71);
                            front_right.setPower(-.71);
                            rear_left.setPower(-.7);
                            rear_right.setPower(.7);
                        } else {
                            front_left.setPower(.31);
                            front_right.setPower(-.31);
                            rear_left.setPower(-.3);
                            rear_right.setPower(.3);
                        }
                    }else {
                        if (gamepad1.right_trigger > .4) {
                            front_left.setPower(-.7);
                            front_right.setPower(.7);
                            rear_left.setPower(.8);
                            rear_right.setPower(-.8);
                        } else {
                            front_left.setPower(-.3);
                            front_right.setPower(.3);
                            rear_left.setPower(.4);
                            rear_right.setPower(-.4);

                        }
                    }
                }else if(gamepad1.dpad_up){
                    if (gamepad1.right_trigger > .4) {
                        front_left.setPower(.7);
                        front_right.setPower(.7);
                        rear_left.setPower(.7);
                        rear_right.setPower(.7);
                    }else{
                        front_left.setPower(.3);
                        front_right.setPower(.3);
                        rear_left.setPower(.3);
                        rear_right.setPower(.3);
                    }
                }else if(gamepad1.dpad_down) {
                    if (gamepad1.right_trigger > .4) {
                        front_left.setPower(-.7);
                        front_right.setPower(-.7);
                        rear_left.setPower(-.7);
                        rear_right.setPower(-.7);
                    } else {
                        front_left.setPower(-.3);
                        front_right.setPower(-.3);
                        rear_left.setPower(-.3);
                        rear_right.setPower(-.3);
                    }
                }else{
                    front_left.setPower(leftFrontPower);
                    front_right.setPower(rightFrontPower);
                    rear_left.setPower(leftBackPower);
                    rear_right.setPower(rightBackPower);
                }
            }




            //Arm code Shoulder

            if (gamepad2.x) {
                if (gamepad2.left_trigger > .4) {  //Pick Pixels
                    xpress = 2;
                    grabby.setPosition(grabIn);
                    wristy.setPosition((float) ((float) .67));
                    flippydoo = .23;
                    flippy.setPosition(.23);

                } else {  //Gripper open and close
                    grabby.setPosition(grabIn);
                    grabOpen = 1;
                    counter2 = 0;

                }
            }
            if (grabOpen == 1){
                if (!gamepad2.x){
                    xopen += 1;
                    grabOpen = 2;
                }
            }
            if (gamepad2.a) { //Low score position
                // Pick
                if(gamepad2.left_trigger > .4 && gamepad2.a){
                    pixelbut = 1;
                }
                else if(gamepad2.right_trigger > .4 && gamepad2.a){
                    pixelbut = 2;
                }
                else if(pixelbut == 0){
                    xopen = 0;
                    ymove = 1;
                    ypress = 1;
                    position = -7;
                    apress = 2;
                }
            }
            if (pixelbut == 1 && !gamepad2.a){
                position += 5;
                pixelbut = 0;
            }
            if (pixelbut == 2 && !gamepad2.a){
                position -= 5;
                pixelbut = 0;
            }
            if (gamepad2.b) { // Pounce position
                ypress = 1;
                ymove = 1;
                xopen = 0;
                grabby.setPosition(grabIn);
                serPosition = .91;
                wristy.setPosition((float) ((float) .67));
                grabby.setPosition(1);
                position = -10;

                bpress = 2;

            }
            if (gamepad2.y) {
                // Play
                position = 125;
                ymove = 2;
                ypress = 2;
            }
            // pick pixels
            else if (xpress == 2) {
                flippy.setPosition(.20);
                timer.reset();
                xpress = 3;
                counter3 = 0;
            } else if (xpress == 3 && timer.time() > .25) {
                serPosition = 1;
                xpress = 4;
                counter3 = 0;
            } else if (xpress == 4 && timer.time() > .55) {
                grabby.setPosition(grabOut);
                xpress = 5;
                counter3 = 0;
            } else if (xpress == 5 && timer.time() > .75) {
                flippydoo = .5;
                flippy.setPosition(flippydoo);
                xpress = 6;
                counter3 = 0;
            } else if (xpress == 6 && timer.time()>.85) {
                serPosition = .91;
                xpress = 1;
                counter3 = 0;
                wristy.setPosition(wristIn);
            }
            if (grabOpen == 4) {
                grabby.setPosition(grabOut);
                grabOpen = 5;
            }
            // kinematics for a
            if (gamepad2.right_bumper) {
                wristy.setPosition(0);
                grabby.setPosition(grabIn);
            }else{
                if (armAngle > -10 && ypress != 3 && xpress < 2 && serPosition == 0) { // Wrist 0 = Out
                    wristy.setPosition((20 + serAdjust + -5 * 1.4 - armAngle * 1.4) / 355);
                    arm *= .5;
                }
            }

            // kinematics for y
            if (ypress == 3 && armAngle >= -10 && xpress < 2) {
                // serPosition = (elbowTyney + shoulderTyney * 13 - armAngle * 13) / 355;  //elbow
                wristy.setPosition((wristTyney + serAdjust + 99 * 1.2 - armAngle * 1.2) / 355);  //wrist
                arm /= 3;
            }
            if (ypress != 3 && xpress >= 2) {
                // serPosition = (elbowTyney + shoulderTyney * 13 - armAngle * 13) / 355;  //elbow
                wristy.setPosition((float) ((float) .67));
            }
            // b movement
            if (bpress == 2 && counter < 15) {
                counter += 1;
            } else if (apress == 3 && counter < 15) {
                counter += 1;
            } else {
                counter = 0;
                bpress = 1;
                if (apress == 3) {
                    position = -5 ;
                    apress = 1;
                }
                if (gamepad2.left_stick_y > .2 || gamepad2.left_stick_y < -.2) {
                    Arm1.setPower(arm);
                    Arm2.setPower(arm);
                    position = armAngle;
                } else {
                    if (position < armAngle + 1 && position > armAngle - 1) {// Stop arm movement within a 4 degree range
                        Arm1.setPower(0);
                        Arm2.setPower(0);
                        if (apress == 2) {
                            serPosition = 0;
                            wristy.setPosition(wristOut);
                            apress = 3;
                        }

                    } else if (position > armAngle + 11 || position < armAngle - 11) {//  Far and fast arm move into position within an infinite range
                        if (position < armAngle) {
                            Arm1.setPower(1);
                            Arm2.setPower(1);
                        }
                        if (position > armAngle) {
                            Arm1.setPower(-1);
                            Arm2.setPower(-1);
                        }
                        if (ypress == 2) {
                            serPosition = elbowTyney;
                            wristy.setPosition(wristTyney + serAdjust/355);
                            ypress = 3;
                        }
                    } else { //Close and slow arm move into position if arm is in a 16 degree range
                        if (position < armAngle) {
                            Arm1.setPower(.2);
                            Arm2.setPower(.2);
                        }
                        if (armAngle < 10){
                            if (position > armAngle) {
                                Arm1.setPower(-.1);
                                Arm2.setPower(-.1);
                            }
                        }
                        if (position > armAngle) {
                            Arm1.setPower(-.2);
                            Arm2.setPower(-.2);
                        }

                    }
                }
            }
            // Arm code Elbow

            elbow1.setPosition(serPosition);


            // Telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addLine("");
            telemetry.addLine("Arm Values:");
            telemetry.addLine("");
            telemetry.addData("Shoulder Arm", "Angle: " + armAngle);
            telemetry.addData("Elbow Joint", "Value: " + elbow);
            telemetry.addData("Wrist", wristy.getPosition() * 355 + serAdjust);
            telemetry.addLine("");
            telemetry.addLine("Drive Train Values:");
            telemetry.addLine("");
            telemetry.addData("Forward Distance: ", dis);
            telemetry.addData("Front left/Right: ", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right: ", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Spinny Speed: ", "%4.2f", leftFrontPower);
            telemetry.addLine("");
            telemetry.addLine("Angles:");
            telemetry.addLine("");
            telemetry.addData("Yaw (Z): ", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Pitch (X): ", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll (Y):  ", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
            telemetry.update();

        }
    }
}



