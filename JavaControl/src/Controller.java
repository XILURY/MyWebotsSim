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
public class Controller {
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

    public static double t;

    public static void main(String[] args) {
//        double[] angle = new double[]{0,0.262,-0.524}; //初始位置

//       Robot robot = new Robot();
//       int timeStep = (int) Math.round(robot.getBasicTimeStep());
        // 初始化电机 按对角腿控制分为两组，motors1的值与motors2一致
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

//        while (robot.step(timeStep) != -1){
//            t = robot.getTime();
//            for(int i=0;i<6;i++){
//                motors1[i].setPosition(joint1(t)[i]);
//                motors2[i].setPosition(joint2(t)[i]);
//            }
//        }

        // Test测试！
        double t = 0.5;
        while(t<20){
            System.out.println(t+"  "+Arrays.toString(joint1(t))+"  "+Arrays.toString(joint2(t)));
            BigDecimal nt = BigDecimal.valueOf(t);
            nt = nt.add(BigDecimal.valueOf(0.5));
            t = nt.doubleValue();

        }
    }

//    /**
//     * 运动学求解 根据关节角度求足端在机身坐标系下的位置 以右前腿为例
//     * @param  angle 以矩阵形式输入关节角度
//     * 验证正确 单腿的
//     */
//    static double[] Kinematics (double[] angle){
//        double[] position = new double[3];
//        double x = angle[0];
//        double y = angle[1];
//        double z = angle[2];
//        BigDecimal p1 = BigDecimal.valueOf(L2*Math.sin(y) + L3*Math.sin(y+z) + xRef).setScale(4,BigDecimal.ROUND_HALF_UP);
//        BigDecimal p2 = BigDecimal.valueOf(Math.sin(x)*(L3*Math.cos(y+z) + L2*Math.cos(y) + L1) + yRef).setScale(4,BigDecimal.ROUND_HALF_UP);;
//        BigDecimal p3 = BigDecimal.valueOf(-(L1*Math.cos(x) + L2*Math.cos(x)*Math.cos(y) + L3*Math.cos(x)*Math.cos(y+z)) + zRef).setScale(4,BigDecimal.ROUND_HALF_UP);;
//        position[0] = p1.doubleValue();
//        position[1] = p2.doubleValue();
//        position[2] = p3.doubleValue();
//        return position;
//    }

//    /**
//     * 逆运动学求解 机身坐标系下的足端位置与关节角的关系?
//     * @param position0 过渡坐标系下的足端位置
//     * 验证正确 单腿的
//     */
//    static double[] InverseKinematics (double[] position0){
//        double p1 = position0[0];
//        double p2 = position0[1];
//        double p3 = position0[2];
//
//        double x = Math.atan((-p2+ yRef)/(p3- zRef));
//        double a = L1 +p3*Math.cos(x)- zRef *Math.cos(x)-p2*Math.sin(x)+ yRef *Math.sin(x);
//        double z = Math.acos((Math.pow((p1- xRef),2)+Math.pow(a,2)-Math.pow(L2,2)-Math.pow(L3,2))/(2*L2*L3));
//        double b = L2*Math.sin(z)+p1- xRef;
//        double y ;
//        // b存在为零的情况
//        if(b == 0)
//            y = 0;
//        else {
//            y = 2 * Math.atan((a + Math.sqrt(Math.pow(a, 2) - b * L3 * Math.sin(z) + b * (p1 - xRef))) / b);
//        }
//        double[] result = new double[3];
//        result[0] = BigDecimal.valueOf(x).setScale(3,BigDecimal.ROUND_HALF_UP).doubleValue();
//        result[1] = BigDecimal.valueOf(y).setScale(3,BigDecimal.ROUND_HALF_UP).doubleValue();
//        result[2] = BigDecimal.valueOf(z).setScale(3,BigDecimal.ROUND_HALF_UP).doubleValue();
//        return result;
//    }

    /**
     * 右前腿 先摆动后支撑
     * @param t 实时时间
     **/
    static double[] joint1(double t){
        double[] jointResult = new double[6];
        t = new BigDecimal(t % (2*Tm)).doubleValue();
        if(t<Tm){
            xTrajectorySwing = s*(t/Tm-1/(2*PI)*Math.sin((2*PI)*t/Tm));
            yTrajectorySwing = 0;
            if(t>=0 && t<Tm/2)
                zTrajectorySwing = 2*h*(t/Tm-1/(4*PI)*Math.sin((4*PI)*t/Tm));
            if(t>=Tm/2 && t<Tm)
                zTrajectorySwing = -2*h*(t/Tm-1/(4*PI)*Math.sin((4*PI)*t/Tm))+2*h;
        }else{
            yTrajectorySwing = 0;
            zTrajectorySwing = 0;
            xTrajectorySwing = s - s/(2*PI)*(2*PI*(t-Tm)/Tm-Math.sin(2*PI*(t-Tm)/Tm));
        }

        xTrajectorySwing = BigDecimal.valueOf(xTrajectorySwing).setScale(5,BigDecimal.ROUND_HALF_UP).doubleValue();
        yTrajectorySwing = BigDecimal.valueOf(yTrajectorySwing).setScale(5,BigDecimal.ROUND_HALF_UP).doubleValue();
        zTrajectorySwing = BigDecimal.valueOf(zTrajectorySwing).setScale(5,BigDecimal.ROUND_HALF_UP).doubleValue();
        jointResult = jointAngleRF(xTrajectorySwing,yTrajectorySwing,zTrajectorySwing);
        return jointResult;
    }

    static double[] joint2(double t){
        double[] jointResult = new double[6];
        t = new BigDecimal(t % (2*Tm)).doubleValue();
        if(t<Tm){
            yTrajectorySwing = 0;
            zTrajectorySwing = 0;
            xTrajectorySwing = s - s/(2*PI)*(2*PI*(t-Tm)/Tm-Math.sin(2*PI*(t-Tm)/Tm));
        }else{
            xTrajectorySwing = s*(t/Tm-1/(2*PI)*Math.sin((2*PI)*t/Tm));
            yTrajectorySwing = 0;
            if(t>=0 && t<Tm/2)
                zTrajectorySwing = 2*h*(t/Tm-1/(4*PI)*Math.sin((4*PI)*t/Tm));
            if(t>=Tm/2 && t<Tm)
                zTrajectorySwing = -2*h*(t/Tm-1/(4*PI)*Math.sin((4*PI)*t/Tm))+2*h;
        }

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

        result[0] = x;
        result[1] = y;
        result[2] = z;

        result[3] = x;
        result[4] = y;
        result[5] = z;
        result[0] = BigDecimal.valueOf(x).setScale(3,BigDecimal.ROUND_HALF_UP).doubleValue();
        result[1] = BigDecimal.valueOf(y).setScale(3,BigDecimal.ROUND_HALF_UP).doubleValue();
        result[2] = BigDecimal.valueOf(z).setScale(3,BigDecimal.ROUND_HALF_UP).doubleValue();

        result[3] = BigDecimal.valueOf(x).setScale(3,BigDecimal.ROUND_HALF_UP).doubleValue();
        result[4] = BigDecimal.valueOf(y).setScale(3,BigDecimal.ROUND_HALF_UP).doubleValue();
        result[5] = BigDecimal.valueOf(z).setScale(3,BigDecimal.ROUND_HALF_UP).doubleValue();
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
