package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
@TeleOp(name = "Wrist")
public class Wrist extends LinearOpMode {


    GeneralHardwareMap gHMap = new GeneralHardwareMap(this);


public int pos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        //gHMap.initRANDOMOTOR("slideAngle");
        //gHMap.initRANDOMOTOR2("slideLength");
        gHMap.init("Kineses");
        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.right_trigger>0.1){pos--;}
            if(gamepad1.left_trigger>0.1){pos++;}
            gHMap.zero3.setPosition(pos*0.001);
            gHMap.zero4.setPosition(pos*0.001);

            if (gamepad1.cross || gamepad1.square) {
                gHMap.bone1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                gHMap.bone2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                gHMap.bone3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            telemetry.addData("motor1", gHMap.bone1.getCurrentPosition());
            telemetry.addData("motor3", gHMap.bone3.getCurrentPosition());
            telemetry.addData("Intended POS", pos*0.001);
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}




