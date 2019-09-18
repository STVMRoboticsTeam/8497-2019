package net.camfeezel.robotics;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(group = "Sensor Tests", name = "01-Distance")
public class SensorTest1 extends LinearOpMode {

    private Rev2mDistanceSensor sensorDistance00;
    private Rev2mDistanceSensor sensorDistance18;
    private Rev2mDistanceSensor sensorDistance27;

    private ColorSensor sensorColorDown;

    private ModernRoboticsI2cGyro sensorGyro;


    private ElapsedTime timer1 = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        sensorDistance00 = hardwareMap.get(Rev2mDistanceSensor.class, "dist00");
        sensorDistance18 = hardwareMap.get(Rev2mDistanceSensor.class, "dist18");
        sensorDistance27 = hardwareMap.get(Rev2mDistanceSensor.class, "dist27");

        sensorColorDown = hardwareMap.colorSensor.get("colorDown");

        // GYRO Init and Calibration
        sensorGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        telemetry.log().add("DO NOT MOVE ROBOT! Gyro Calibrating...");
        sensorGyro.calibrate();
        timer1.reset();
        while (!isStopRequested() && sensorGyro.isCalibrating())  {
            telemetry.addData("Gyro Calibration", "%s", Math.round(timer1.seconds())%2==0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }
        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated.");
        telemetry.clear(); telemetry.update();

        waitForStart();
        telemetry.log().clear();
        double initialDist00 = sensorDistance00.getDistance(DistanceUnit.CM);
        double initialDist18 = sensorDistance18.getDistance(DistanceUnit.CM);
        double initialDist27 = sensorDistance27.getDistance(DistanceUnit.CM);
        double initialHeading = sensorGyro.getHeading();
        while(opModeIsActive()) {
            float zAngle = sensorGyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            telemetry.addData("Heading", "%3d", sensorGyro.getHeading());
            telemetry.addData("Integ. Z", "%3d", sensorGyro.getIntegratedZValue());
            telemetry.addData("Angle", "%3d", String.format("%.3f", zAngle));

            double cd00 = sensorDistance00.getDistance(DistanceUnit.CM);
            double cd18 = sensorDistance18.getDistance(DistanceUnit.CM);
            double cd27 = sensorDistance27.getDistance(DistanceUnit.CM);
            telemetry.addData("Distance-000", "%d cm", cd00);
            telemetry.addData("Distance-180", "%d cm", cd18);
            telemetry.addData("Distance-270", "%d cm", cd27);

            telemetry.addLine("Color")
                    .addData("R", sensorColorDown.red())
                    .addData("G", sensorColorDown.green())
                    .addData("B", sensorColorDown.blue());
            telemetry.update();


        }
    }

    /**
     *
     * @param x speed in the 90/270 degrees direction. -1 to 1
     * @param y speed in the 0/180 degrees direction. -1 to 1
     * @param rot rotational speed, positive means positive degrees.
     */
    private void setVelocity(float x, float y, float rot) {
        float frFin = 0f;
        float flFin = 0f;
        float brFin = 0f;
        float blFin = 0f;

        x = Range.clip(x, -1, 1);
        y = Range.clip(y, -1, 1);

        // fl
        if(y > 0) {
            // TODO check what happens on zeros. for FL its defo wrong
            if(x >= 0) {
                flFin = x * y;
                frFin = -((x * 2) - 1) * y;
            }
        }
    }

}
