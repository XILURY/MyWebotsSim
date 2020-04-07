// 正确的正逆运动学 留存
// File:          JavaController.java
// Date:          2020/04/03
// Description:
// Author:
// Modifications:

// You may need to add other webots classes such as
//  import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;

import java.math.BigDecimal;
import java.util.Arrays;

// Here is the main class of your controller.
// This class defines how to initialize and how to run your controller.
public class InverseKinematics {
    // 单腿杆件长
    public static final double L1 = 0.05;
    public static final double L2 = 0.3;
    public static final double L3 = 0.32; // 加上足端 半球半径0.02

    // 定义过渡坐标系相对于机身的位置
    public static final double X_REF = 0.4;
    public static final double Y_REF = 0.18;
    public static final double Z_REF = -0.05;

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
        double[] angle = new double[]{0,0,0};
        double[] position = new double[]{0.4,0.18,-0.7};
        System.out.println(Arrays.toString(Kinematics(angle)));
//       double[] position = new double[]{0.39428,0.18,-0.69884};
        System.out.println(Arrays.toString(InverseKinematics(position)));


    }

    /**
     * 运动学求解 根据关节角度求足端在机身坐标系下的位置 以右前腿为例
     * @param  angle 以矩阵形式输入关节角度
     * 验证正确 单腿的
     */
    static double[] Kinematics (double[] angle){
        double[] position = new double[3];
        double x = angle[0];
        double y = angle[1];
        double z = angle[2];

        BigDecimal p1 = BigDecimal.valueOf(L2*Math.sin(y) + L3*Math.sin(y+z) + X_REF).setScale(5,BigDecimal.ROUND_HALF_UP);
        BigDecimal p2 = BigDecimal.valueOf(Math.sin(x)*(L3*Math.cos(y+z) + L2*Math.cos(y) + L1) + Y_REF).setScale(5,BigDecimal.ROUND_HALF_UP);;
        BigDecimal p3 = BigDecimal.valueOf(-(L1*Math.cos(x) + L2*Math.cos(x)*Math.cos(y) + L3*Math.cos(x)*Math.cos(y+z)) + Z_REF).setScale(5,BigDecimal.ROUND_HALF_UP);;
        position[0] = p1.doubleValue();
        position[1] = p2.doubleValue();
        position[2] = p3.doubleValue();
        return position;
    }

    /**
     * 逆运动学求解 机身坐标系下的足端位置与关节角的关系?
     * @param position0 过渡坐标系下的足端位置
     * 验证正确 单腿的
     */
    static double[] InverseKinematics (double[] position0){
        double p1 = position0[0];
        double p2 = position0[1];
        double p3 = position0[2];

        double x = Math.atan((-p2+Y_REF)/(p3-Z_REF));
        double a = L1 +p3*Math.cos(x)-Z_REF*Math.cos(x)-p2*Math.sin(x)+Y_REF*Math.sin(x);
        double z = Math.acos((Math.pow((p1-X_REF),2)+Math.pow(a,2)-Math.pow(L2,2)-Math.pow(L3,2))/(2*L2*L3));
        double b = L2*Math.sin(z)+p1-X_REF;
        double y ;
        // b存在为零的情况
        if(b == 0)
            y = 0;
        else {
            y = 2 * Math.atan((a + Math.sqrt(Math.pow(a, 2) - b * L3 * Math.sin(z) + b * (p1 - X_REF))) / b);
        }
        double[] result = new double[3];
        result[0] = BigDecimal.valueOf(x).setScale(3,BigDecimal.ROUND_HALF_UP).doubleValue();
        result[1] = BigDecimal.valueOf(y).setScale(3,BigDecimal.ROUND_HALF_UP).doubleValue();
        result[2] = BigDecimal.valueOf(z).setScale(3,BigDecimal.ROUND_HALF_UP).doubleValue();
        return result;
    }
}
