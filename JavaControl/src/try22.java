// File:          JavaController.java
// Date:          2020/04/03
// Description:
// Author:
// Modifications:

// You may need to add other webots classes such as
//  import com.cyberbotics.webots.controller.DistanceSensor;

import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;
import java.util.*;
import java.math.BigDecimal;
import java.util.Arrays;

// Here is the main class of your controller.
// This class defines how to initialize and how to run your controller.
public class try22 {
    // 单腿杆件长
    public static final double L1 = 0.05;
    public static final double L2 = 0.3;
    public static final double L3 = 0.32; // 加上足端 半球半径0.02

    // 定义过渡坐标系相对于机身的位置
    public  static double xRef = 0.4;
    public  static double yRef = 0.18;
    public  static double zRef = -0.05;

    public static final double PI = 3.14;

    // 定义摆动腿足端轨迹
    public static double xTrajectorySwing;
    public static double yTrajectorySwing;
    public static double zTrajectorySwing;

    // 定义支撑腿足端轨迹
    public double xTrajectorySupport;
    public double yTrajectorySupport;
    public double zTrajectorySupport;

    // 定义摆动周期
    public static double Tm = 2;

    // 定义步长、步高
    public static double s = 0.1;
    public static double h = 0.2;


    public static void main(String[] args) {
//        double[] angle = new double[]{0,0.262,-0.524}; //初始位置

//        Robot robot = new Robot();
//        int timeStep = (int) Math.round(robot.getBasicTimeStep());
//        // 初始化电机 按对角腿控制分为两组，motors1的值与motors2一致
//        Motor[] motors1 = new Motor[6];
//        Motor[] motors2 = new Motor[6];
//        motors1[0] = robot.getMotor("RF1_motor");
//        motors1[1] = robot.getMotor("RF2_motor");
//        motors1[2] = robot.getMotor("RF3_motor");
//        motors1[3] = robot.getMotor("LF1_motor");
//        motors1[4] = robot.getMotor("LF2_motor");
//        motors1[5] = robot.getMotor("LF3_motor");
//
//        motors2[0] = robot.getMotor("LH1_motor");
//        motors2[1] = robot.getMotor("LH2_motor");
//        motors2[2] = robot.getMotor("LH3_motor");
//        motors2[3] = robot.getMotor("RH1_motor");
//        motors2[4] = robot.getMotor("RH2_motor");
//        motors2[5] = robot.getMotor("RH3_motor");
//
//        while (robot.step(timeStep) != -1){
//            double t = robot.getTime();
//        }

        // Test测试！
        double t = 0.2;
        for(int i=1; i<20; i++){
            t = t+i;
            System.out.println(Arrays.toString(joint(t)));
        }


    }

    /**
     * 支撑腿轨迹规划 并输出关节角度 过渡坐标系下
     * @param t 实时时间
     **/
    static double[] joint(double t){
        double[] jointResult = new double[6];
        xTrajectorySwing = s*(t/Tm-1/(2*PI)*Math.sin((2*PI)*t/Tm))-s/2;
        yTrajectorySwing = 0;
        if(t>=0 && t<Tm/2)
            zTrajectorySwing = 2*h*(t/Tm-1/(4*PI)*Math.sin((4*PI)*t/Tm));
        if(t>=Tm/2 && t<Tm)
            zTrajectorySwing = 2*h*(1-t/Tm+1/(4*PI)*Math.sin((4*PI)*t/Tm));
        xTrajectorySwing = BigDecimal.valueOf(xTrajectorySwing).setScale(5,BigDecimal.ROUND_HALF_UP).doubleValue();
        yTrajectorySwing = BigDecimal.valueOf(yTrajectorySwing).setScale(5,BigDecimal.ROUND_HALF_UP).doubleValue();
        zTrajectorySwing = BigDecimal.valueOf(zTrajectorySwing).setScale(5,BigDecimal.ROUND_HALF_UP).doubleValue();
        jointResult = jointAngleRF(xTrajectorySwing,yTrajectorySwing,zTrajectorySwing);
        return jointResult;
    }

    /**
     * 逆运动学解算 右前腿 机身坐标系下
     * @param p1,p2,p3 x,y,z方向过渡坐标系下位移
     *
     */
    static double[] jointAngleRF (double p1,double p2,double p3){
        p1 = p1 - xRef;
        p2 = -p2 + yRef;
        p3 = p3 + zRef;
        double[] result = new double[6];
        double x = Math.atan((-p2+ yRef)/(p3- zRef));
        double a = L1 +p3*Math.cos(x)- zRef *Math.cos(x)-p2*Math.sin(x)+ yRef *Math.sin(x);
        double z = Math.acos((Math.pow((p1- xRef),2)+Math.pow(a,2)-Math.pow(L2,2)-Math.pow(L3,2))/(2*L2*L3));
        double b = L2*Math.sin(z)+p1- xRef;
        double y ;
        // b存在为零的情况
        if(b == 0)
            y = 0;
        else {
            y = 2 * Math.atan((a + Math.sqrt(Math.pow(a, 2) - b * L3 * Math.sin(z) + b * (p1 - xRef))) / b);
        }

//        result[0] = x;
//        result[1] = y;
//        result[2] = z;
//
//        result[3] = x;
//        result[4] = y;
//        result[5] = z;
//        result[0] = BigDecimal.valueOf(x).setScale(3,BigDecimal.ROUND_HALF_UP).doubleValue();
//        result[1] = BigDecimal.valueOf(y).setScale(3,BigDecimal.ROUND_HALF_UP).doubleValue();
//        result[2] = BigDecimal.valueOf(z).setScale(3,BigDecimal.ROUND_HALF_UP).doubleValue();
//
//        result[3] = BigDecimal.valueOf(x).setScale(3,BigDecimal.ROUND_HALF_UP).doubleValue();
//        result[4] = BigDecimal.valueOf(y).setScale(3,BigDecimal.ROUND_HALF_UP).doubleValue();
//        result[5] = BigDecimal.valueOf(z).setScale(3,BigDecimal.ROUND_HALF_UP).doubleValue();
        return result;
    }

    /**
     * 逆运动学解算 左前腿
     * @param p1,p2,p3 x,y,z方向
     *
     */
    static double[] jointAngleLF (double p1,double p2,double p3){
        yRef = -yRef;
        double[] result = new double[6];
        double x = Math.atan((-p2+ yRef)/(p3- zRef));
        double a = L1 +p3*Math.cos(x)- zRef *Math.cos(x)-p2*Math.sin(x)+ yRef *Math.sin(x);
        double z = Math.acos((Math.pow((p1- xRef),2)+Math.pow(a,2)-Math.pow(L2,2)-Math.pow(L3,2))/(2*L2*L3));
        double b = L2*Math.sin(z)+p1- xRef;
        double y ;
        // b存在为零的情况
        if(b == 0)
            y = 0;
        else {
            y = 2 * Math.atan((a + Math.sqrt(Math.pow(a, 2) - b * L3 * Math.sin(z) + b * (p1 - xRef))) / b);
        }

        result[0] = BigDecimal.valueOf(x).setScale(3,BigDecimal.ROUND_HALF_UP).doubleValue();
        result[1] = BigDecimal.valueOf(y).setScale(3,BigDecimal.ROUND_HALF_UP).doubleValue();
        result[2] = BigDecimal.valueOf(z).setScale(3,BigDecimal.ROUND_HALF_UP).doubleValue();

        result[3] = BigDecimal.valueOf(x).setScale(3,BigDecimal.ROUND_HALF_UP).doubleValue();
        result[4] = BigDecimal.valueOf(y).setScale(3,BigDecimal.ROUND_HALF_UP).doubleValue();
        result[5] = BigDecimal.valueOf(z).setScale(3,BigDecimal.ROUND_HALF_UP).doubleValue();
        return result;
    }
}
