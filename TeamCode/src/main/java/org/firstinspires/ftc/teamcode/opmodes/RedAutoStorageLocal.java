package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.libs.Globals.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.libs.AutoImport;

@Autonomous(name="RedAutoStorageLocal", group="redAuto")
public class RedAutoStorageLocal extends AutoImport {

    public RedAutoStorageLocal() { super(60, -35, 75, 170, 173, 170); }

    public void runOpMode() {
        super.runOpMode();

        if (opModeIsActive()) {
            // Goes to spinner and does spinny
            slauto.drive(50, -60, -90, 0.75, 4000, this, true, true);
            setSpinny(true, 1000);

            // Goes to the shipping hub and delivers based on the team element position
            // 1 is added to elementPosition because height's 0 is ground level, not the first layer
            setArm(elementPosition + 1, 1);
            sleep(1000);
            slauto.drive(25, -60, -90, 0.75, 0, this, false, true);
            slauto.drive(25, -33, -90, 0.75, 0, this, true, false);
            deposit(true);
            sleep(1000);
            //runIntake(1, 1000);
            //sleep(1000);
            slauto.drive(25, -35, -90, 1, 0, this, false, false);
            slauto.drive(25, -33, -90, 1, 0, this, false, false);
            deposit(false);

            // Parks in storage
            slauto.drive(23, -55, 0, 0.75, this);
            slauto.drive(42, -60, 0, 0.75, this);

            // Lowers arm
            setArm(0, 1);
            while (armY.isBusy()){
                sleep(10);
            }

            stopCamera();
        }
    }
}