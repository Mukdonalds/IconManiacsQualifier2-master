/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.HardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Thread.sleep;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */

/*
CONTROL HUB

DC MOTORS

0 - frontLeft (front left motor)
1 - backLeft (back left motor)
2 - backRight (back right motor)
3 - frontRight (front right motor)

SERVOS

0 - claw (Servo on arm)
1 - claw2 (2nd Servo on arm)

*/

    /*
    EXPANSION HUB

    DC MOTORS

    0 - arm
    1 - conveyorBelt
    2 - shooter
    3 - intake

     */

public class IMDriveBot

{
    /* Public OpMode members. */
    // DCMotors and Servos declaration
    public DcMotor  frontLeft   = null;
    public DcMotor  frontRight  = null;
    public DcMotor  backLeft = null;
    public DcMotor  backRight = null;
    public Servo claw = null;
    public Servo claw2 = null;
    public static final double SERVO_HOME =  0.0 ;

    // initial positions of the servos used in the program
    public static final double clawOpen = 11.5;
    public static final double clawClose = 0.0;
    public static double clawPOS = clawOpen;


    /* local OpMode members. */
    HardwareMap hwMap =  null;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors\
        frontLeft = hwMap.get(DcMotor.class, "frontLeft");
        backLeft = hwMap.get(DcMotor.class, "backLeft");
        frontRight = hwMap.get(DcMotor.class, "frontRight");
        backRight = hwMap.get(DcMotor.class, "backRight");

        // Set all motors to zero power
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

        // sets zeroPowerBehavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Sets motor directions
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // sets enconder mode to run with encoder
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        claw = hwMap.get(Servo.class, "claw");
        claw2 = hwMap.get(Servo.class, "claw2");

        //platform.setPosition(SERVO_HOME);
        claw.setPosition(SERVO_HOME);
        claw2.setPosition(SERVO_HOME);


    }

    public void powerOff(){
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
    }

    public void moveForward(double power, double distance){ //power from -1 to 1 and distance measured in inches
        // the motor encoders will be reset
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double circumference = 3.14*3.937; //pi*diameter
        double rotationsRequired = distance/circumference; //inputed distance divided by the circumference of the mecanum wheels
        int encoderTargetPOS = (int)(rotationsRequired*537.6); // rotations needed * ticks per count on the gobilda motor
        //the encoder target position which was calculated above is input into the motors below
        frontLeft.setTargetPosition(encoderTargetPOS);
        backLeft.setTargetPosition(encoderTargetPOS);
        frontRight.setTargetPosition(encoderTargetPOS);
        backRight.setTargetPosition(encoderTargetPOS);


        // sets the motors to the respective power that was input
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);
        //the mode is set to run to the target position that was set above
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(frontLeft.isBusy() || backRight.isBusy() || frontRight.isBusy() || backLeft.isBusy()){
            //wait for the motors to reach the target position
         //   telemetry.addData("Status: ","Robot is traveling " + distance + "units" + " to target position");
          //  telemetry.update();
        }

        powerOff();

    }

    public void turnLeft(double power, double distance){ //power from -1 to 1 and distance measured in inches
        // the motor encoders will be reset
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double circumference = 3.14*3.937; //pi*diameter
        double rotationsRequired = distance/circumference; //inputed distance divided by the circumference of the mecanum wheels
        int encoderTargetPOS = (int)(rotationsRequired*537.6); // rotations needed * ticks per count on the gobilda motor
        //the encoder target position which was calculated above is input into the motors below
        frontLeft.setTargetPosition(encoderTargetPOS);
        backLeft.setTargetPosition(encoderTargetPOS);
        frontRight.setTargetPosition(-encoderTargetPOS);
        backRight.setTargetPosition(-encoderTargetPOS);


        // sets the motors to the respective power that was input
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);
        //the mode is set to run to the target position that was set above
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(frontLeft.isBusy() || backRight.isBusy() || frontRight.isBusy() || backLeft.isBusy()){
            //wait for the motors to reach the target position
            //   telemetry.addData("Status: ","Robot is traveling " + distance + "units" + " to target position");
            //  telemetry.update();
        }

        powerOff();

    }

    public void turnRight(double power, double distance){ //power from -1 to 1 and distance measured in inches
        // the motor encoders will be reset
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double circumference = 3.14*3.937; //pi*diameter
        double rotationsRequired = distance/circumference; //inputed distance divided by the circumference of the mecanum wheels
        int encoderTargetPOS = (int)(rotationsRequired*537.6); // rotations needed * ticks per count on the gobilda motor
        //the encoder target position which was calculated above is input into the motors below
        frontLeft.setTargetPosition(-encoderTargetPOS);
        backLeft.setTargetPosition(-encoderTargetPOS);
        frontRight.setTargetPosition(encoderTargetPOS);
        backRight.setTargetPosition(encoderTargetPOS);


        // sets the motors to the respective power that was input
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);
        //the mode is set to run to the target position that was set above
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(frontLeft.isBusy() || backRight.isBusy() || frontRight.isBusy() || backLeft.isBusy()){
            //wait for the motors to reach the target position
            //   telemetry.addData("Status: ","Robot is traveling " + distance + "units" + " to target position");
            //  telemetry.update();
        }

        powerOff();

    }

    public void moveBackward(double power, double distance){ //power from -1 to 1 and distance measured in inches
        // the motor encoders will be reset
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double circumference = 3.14*3.937; //pi*diameter
        double rotationsRequired = distance/circumference; //inputed distance divided by the circumference of the mecanum wheels
        int encoderTargetPOS = (int)(rotationsRequired*537.6); // rotations needed * ticks per count on the gobilda motor
        //the encoder target position which was calculated above is input into the motors below
        frontLeft.setTargetPosition(-encoderTargetPOS);
        backLeft.setTargetPosition(-encoderTargetPOS);
        frontRight.setTargetPosition(-encoderTargetPOS);
        backRight.setTargetPosition(-encoderTargetPOS);


        // sets the motors to the respective power that was input
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);
        //the mode is set to run to the target position that was set above
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(frontLeft.isBusy() || backRight.isBusy() || frontRight.isBusy() || backLeft.isBusy()){
            //wait for the motors to reach the target position
            //   telemetry.addData("Status: ","Robot is traveling " + distance + "units" + " to target position");
            //  telemetry.update();
        }

        powerOff();

    }

    public void strafeRight(double power, double distance){ //power from -1 to 1 and distance measured in inches
        // the motor encoders will be reset
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double circumference = 3.14*3.937; //pi*diameter
        double rotationsRequired = distance/circumference; //inputed distance divided by the circumference of the mecanum wheels
        int encoderTargetPOS = (int)(rotationsRequired*537.6); // rotations needed * ticks per count on the gobilda motor
        //the encoder target position which was calculated above is input into the motors below
        frontLeft.setTargetPosition(encoderTargetPOS);
        backLeft.setTargetPosition(-encoderTargetPOS);
        frontRight.setTargetPosition(-encoderTargetPOS);
        backRight.setTargetPosition(encoderTargetPOS);


        // sets the motors to the respective power that was input
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);
        //the mode is set to run to the target position that was set above
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(frontLeft.isBusy() || backRight.isBusy() || frontRight.isBusy() || backLeft.isBusy()){
            //wait for the motors to reach the target position
            //   telemetry.addData("Status: ","Robot is traveling " + distance + "units" + " to target position");
            //  telemetry.update();
        }

        powerOff();

    }

    public void strafeLeft(double power, double distance){ //power from -1 to 1 and distance measured in inches
        // the motor encoders will be reset
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double circumference = 3.14*3.937; //pi*diameter
        double rotationsRequired = distance/circumference; //inputed distance divided by the circumference of the mecanum wheels
        int encoderTargetPOS = (int)(rotationsRequired*537.6); // rotations needed * ticks per count on the gobilda motor
        //the encoder target position which was calculated above is input into the motors below
        frontLeft.setTargetPosition(-encoderTargetPOS);
        backLeft.setTargetPosition(encoderTargetPOS);
        frontRight.setTargetPosition(encoderTargetPOS);
        backRight.setTargetPosition(-encoderTargetPOS);


        // sets the motors to the respective power that was input
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);
        //the mode is set to run to the target position that was set above
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(frontLeft.isBusy() || backRight.isBusy() || frontRight.isBusy() || backLeft.isBusy()){
            //wait for the motors to reach the target position
            //   telemetry.addData("Status: ","Robot is traveling " + distance + "units" + " to target position");
            //  telemetry.update();
        }

        powerOff();

    }
}