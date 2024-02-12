package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private double closedValue;
    private double openedValue;
    private Servo s;

    public Claw (Servo servo, double closedValue, double openedValue)
    {
        s = servo;
        setOpenedValue(openedValue);
        setClosedValue(closedValue);
    }

    public void setOpenedValue(double openedValue) {
        this.openedValue = openedValue;
    }

    public void setClosedValue(double closedValue) {
        this.closedValue = closedValue;
    }

    public double getOpenedValue() {
        return openedValue;
    }

    public double getClosedValue() {
        return closedValue;
    }

    public void open()
    {
        s.setPosition(openedValue);
    }

    public void close()
    {
        s.setPosition(closedValue);
    }
}
