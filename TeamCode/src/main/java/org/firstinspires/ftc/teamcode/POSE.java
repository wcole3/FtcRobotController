package org.firstinspires.ftc.teamcode;

public enum POSE {
    BACK(1.0, 0.0),
    TOUCHDOWN(0.5, 0.6),
    LUNGE(0.2, 0.9);

    private final double rightPos;
    private final double leftPos;

    POSE(double left, double right){
        rightPos = right;
        leftPos = left;
    }

    public double getRightArmPos(){
        return rightPos;
    }

    public double getLeftArmPos(){
        return leftPos;
    }
}
