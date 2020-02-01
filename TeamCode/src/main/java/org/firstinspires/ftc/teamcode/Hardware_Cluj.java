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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
public class Hardware_Cluj
{
    /* Public OpMode members. */
    /** Motoare de deplasare**/

    public DcMotor LeftBackMotor = null;
    public DcMotor LeftFrontMotor = null;
    public DcMotor RightBackMotor = null;
    public DcMotor RightFrontMotor = null;

    /** Servo pentru brat **/

    public Servo servoExtindere = null ;
    public Servo servoPrindereCub = null;

    /**Servo pentru tava*/
    public Servo servoTavaStanga=null;
    public Servo servoTavaDreapta=null;

    /**Motoare slidere*/
    public DcMotor leftSliderMotor=null;
    public DcMotor rightSliderMotor=null;

    /**MOTOARE INTAKE*/
    public DcMotor leftIntakeMotor=null;
    public DcMotor rightIntakeMotor=null;


    public static final double      PRINDERE_INITIAL      =  0.25 ;
    public static final double      PRINDERE_COMPLETA = 1;
    public static final double      TAVA_JOS =  0.75 ;
    public static final double      TAVA_SUS   = 0.25 ;
    public static final double      DriveValue = 2.43;
    public static final double      TurnValue = 2;
    public static final double      StrafeValue = 2;
    public static final double      PullValue = 125;
    public static final double      COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder 1440 tetrix
    public static final double      DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    public static final double      WHEEL_DIAMETER_MM   = 4.0 * 25.4;     // For figuring circumference
    public static final double     COUNTS_PER_MM         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_MM * 3.1415);
    public static final double      DRIVE_SPEED = 1;
    public static final double      TURN_SPEED = 0.5;
    public static final double      PULL_SPEED = 0.1;
    public static final double PozitieInitial=0.27;

    /* local OpMode members. */
    HardwareMap HWM_Cluj  =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Hardware_Cluj(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        HWM_Cluj = ahwMap;

        /** MOTOARE DE DEPLASARE **/
        LeftBackMotor  = HWM_Cluj.get(DcMotor.class, "LeftBackMotor");
        LeftFrontMotor = HWM_Cluj.get(DcMotor.class, "LeftFrontMotor");
        RightBackMotor = HWM_Cluj.get(DcMotor.class,"RightBackMotor");
        RightFrontMotor = HWM_Cluj.get(DcMotor.class,"RightFrontMotor");

        /**MOTOARE INTAKE*/
        leftIntakeMotor=HWM_Cluj.get(DcMotor.class,"leftIntakeMotor");
        rightIntakeMotor=HWM_Cluj.get(DcMotor.class,"rightIntakeMotor");


        /** Motoare pentru brat **/
        leftSliderMotor = HWM_Cluj.get(DcMotor.class, "leftSliderMotor");
        rightSliderMotor = HWM_Cluj.get(DcMotor.class, "rightSliderMotor");

        /** Servo-uri pentru brat **/
        servoExtindere = HWM_Cluj.get(Servo.class, "servoExtindere");
        servoPrindereCub = HWM_Cluj.get(Servo.class, "servoPrindereCub");

        /**Servo-uri tava*/

        servoTavaStanga=HWM_Cluj.get(Servo.class,"servoTavaStanga");
        servoTavaDreapta=HWM_Cluj.get(Servo.class,"servoTavaDreapta");

        /** MOTOARE DE DEPLASARE **/
        LeftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        LeftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        RightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        RightBackMotor.setDirection(DcMotor.Direction.FORWARD);




        /** Servo-uri pentru brat **/
        servoTavaDreapta.setDirection(Servo.Direction.FORWARD);
        servoTavaStanga.setDirection(Servo.Direction.FORWARD);

        /** Servo-uri pentru intake **/

        servoExtindere.setDirection(Servo.Direction.FORWARD);
        servoPrindereCub.setDirection(Servo.Direction.FORWARD);

      /**ROTITE INTAKE**/
      leftIntakeMotor.setDirection(DcMotor.Direction.FORWARD);
      rightIntakeMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        /** MOTOARE DE DEPLASARE **/
        LeftBackMotor.setPower(0);
        LeftFrontMotor.setPower(0);
        RightBackMotor.setPower(0);
        RightFrontMotor.setPower(0);


        /** Servo-uri pentru brat **/
        servoExtindere.setPosition(0.5);
        servoPrindereCub.setPosition(0.2);

        /** Servo-uri pentru intake **/
        servoTavaStanga.setPosition(0);
        servoTavaDreapta.setPosition(0);

        /**Rotite intake*/
        leftIntakeMotor.setPower(0);
        rightIntakeMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        /** Motoare **/
        LeftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



    }
}

