// File:          JavaController.java
// Date:
// Description:
// Author:
// Modifications:

// You may need to add other webots classes such as
//  import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;
import java.io.IOException;
import java.io.PrintWriter;
import java.math.BigDecimal;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Scanner;

// Here is the main class of your controller.
// This class defines how to initialize and how to run your controller.
public class JavaController {

    public static final double L1 = 0.05;
    public static final double L2 = 0.3;
    public static final double L3 = 0.32; // 加上足端 半球半径0.02

    public  static double xRef = 0.4;
    public  static double yRef = 0.18;
    public  static double zRef = -0.05;

    public static final double PI = 3.14;

    public static double xTrajectorySwing;
    public static double yTrajectorySwing;
    public static double zTrajectorySwing;

    public double xTrajectorySupport;
    public double yTrajectorySupport;
    public double zTrajectorySupport;

    public static double Tm = 2;

    public static double s = 0.1;
    public static double h = 0.2;
    
    public static double t;

    public static void main(String[] args) throws IOException {
        Robot robot = new Robot();
        int timeStep = (int) Math.round(robot.getBasicTimeStep());
        
        Motor[] motors1 = new Motor[6];
        Motor[] motors2 = new Motor[6];
        motors1[0] = robot.getMotor("RF1_motor");
        motors1[1] = robot.getMotor("RF2_motor");
        motors1[2] = robot.getMotor("RF3_motor");
        motors1[3] = robot.getMotor("LH1_motor");
        motors1[4] = robot.getMotor("LH2_motor");
        motors1[5] = robot.getMotor("LH3_motor");

        motors2[0] = robot.getMotor("LF1_motor");
        motors2[1] = robot.getMotor("LF2_motor");
        motors2[2] = robot.getMotor("LF3_motor");
        motors2[3] = robot.getMotor("RH1_motor");
        motors2[4] = robot.getMotor("RH2_motor");
        motors2[5] = robot.getMotor("RH3_motor");
       
        
        Scanner in = new Scanner(Paths.get("out1.txt"),"UTF-8");
        ArrayList<Double> out1 = new ArrayList<>();
        while(in.hasNext()){
            out1.add(Double.valueOf(in.nextLine()));
        }
        out1.trimToSize();
        
        Scanner in2 = new Scanner(Paths.get("out2.txt"),"UTF-8");
        ArrayList<Double> out2 = new ArrayList<>();
        while(in2.hasNext()){
            out2.add(Double.valueOf(in2.nextLine()));
        }
        out2.trimToSize();
        
        Scanner in3 = new Scanner(Paths.get("out3.txt"),"UTF-8");
        ArrayList<Double> out3 = new ArrayList<>();
        while(in3.hasNext()){
            out3.add(Double.valueOf(in3.nextLine()));
        }
        out3.trimToSize();

        while (robot.step(timeStep) != -1){
          for(int i=0;i<out1.size()-1;i++){
               motors1[0].setPosition(out1.get(i)); 
               System.out.println(out1.get(i));
               motors1[1].setPosition(out2.get(i)); 
               System.out.println(out2.get(i));
               motors1[2].setPosition(out3.get(i)); 
               System.out.println(out3.get(i));
          } 
    }
  }
    static double[] joint1(double t){
        double[] jointResult = new double[6];
        t = new BigDecimal(t % (2*Tm)).doubleValue();
        if(t<Tm){
            yTrajectorySwing = s*(t/Tm-1/(2*PI)*Math.sin((2*PI)*t/Tm));
            xTrajectorySwing = 0;
            if(t>=0 && t<Tm/2)
                zTrajectorySwing = 2*h*(t/Tm-1/(4*PI)*Math.sin((4*PI)*t/Tm));
            if(t>=Tm/2 && t<Tm)
                zTrajectorySwing = -2*h*(t/Tm-1/(4*PI)*Math.sin((4*PI)*t/Tm))+2*h;
        }else{
            xTrajectorySwing = 0;
            zTrajectorySwing = 0;
            yTrajectorySwing = s - s/(2*PI)*(2*PI*(t-Tm)/Tm-Math.sin(2*PI*(t-Tm)/Tm));
        }

        // xTrajectorySwing = BigDecimal.valueOf(xTrajectorySwing).setScale(5,BigDecimal.ROUND_HALF_UP).doubleValue();
        // yTrajectorySwing = BigDecimal.valueOf(yTrajectorySwing).setScale(5,BigDecimal.ROUND_HALF_UP).doubleValue();
        // zTrajectorySwing = BigDecimal.valueOf(zTrajectorySwing).setScale(5,BigDecimal.ROUND_HALF_UP).doubleValue();
        jointResult = jointAngleRF(xTrajectorySwing,yTrajectorySwing,zTrajectorySwing);
        return jointResult;
    }

    static double[] joint2(double t){
        double[] jointResult = new double[6];
        t = new BigDecimal(t % (2*Tm)).doubleValue();
        if(t<Tm){
            xTrajectorySwing = 0;
            zTrajectorySwing = 0;
            yTrajectorySwing = s - s/(2*PI)*(2*PI*(t-Tm)/Tm-Math.sin(2*PI*(t-Tm)/Tm));
        }else{
            yTrajectorySwing = s*(t/Tm-1/(2*PI)*Math.sin((2*PI)*t/Tm));
            xTrajectorySwing = 0;
            if(t>=0 && t<Tm/2)
                zTrajectorySwing = 2*h*(t/Tm-1/(4*PI)*Math.sin((4*PI)*t/Tm));
            if(t>=Tm/2 && t<Tm)
                zTrajectorySwing = -2*h*(t/Tm-1/(4*PI)*Math.sin((4*PI)*t/Tm))+2*h;
        }

        // xTrajectorySwing = BigDecimal.valueOf(xTrajectorySwing).setScale(5,BigDecimal.ROUND_HALF_UP).doubleValue();
        // yTrajectorySwing = BigDecimal.valueOf(yTrajectorySwing).setScale(5,BigDecimal.ROUND_HALF_UP).doubleValue();
        // zTrajectorySwing = BigDecimal.valueOf(zTrajectorySwing).setScale(5,BigDecimal.ROUND_HALF_UP).doubleValue();
        jointResult = jointAngleRF(xTrajectorySwing,yTrajectorySwing,zTrajectorySwing);
        return jointResult;
    }


    static double[] jointAngleRF (double p1,double p2,double p3){
        double[] result = new double[6];
        double x = Math.atan((-p2+ yRef)/(p3- zRef));
        double a = L1 +p3*Math.cos(x)- zRef *Math.cos(x)-p2*Math.sin(x)+ yRef *Math.sin(x);
        double z = Math.acos((Math.pow((p1- xRef),2)+Math.pow(a,2)-Math.pow(L2,2)-Math.pow(L3,2))/(2*L2*L3));
        double b = L2*Math.sin(z)+p1- xRef;
        double y ;
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
        // result[0] = BigDecimal.valueOf(x).setScale(3,BigDecimal.ROUND_HALF_UP).doubleValue();
        // result[1] = BigDecimal.valueOf(y).setScale(3,BigDecimal.ROUND_HALF_UP).doubleValue();
        // result[2] = BigDecimal.valueOf(z).setScale(3,BigDecimal.ROUND_HALF_UP).doubleValue();

        // result[3] = BigDecimal.valueOf(x).setScale(3,BigDecimal.ROUND_HALF_UP).doubleValue();
        // result[4] = BigDecimal.valueOf(y).setScale(3,BigDecimal.ROUND_HALF_UP).doubleValue();
        // result[5] = BigDecimal.valueOf(z).setScale(3,BigDecimal.ROUND_HALF_UP).doubleValue();
        return result;
    }

    static double[] jointAngleLF (double p1,double p2,double p3){
        yRef = -yRef;
        double[] result = new double[6];
        double x = Math.atan((-p2+ yRef)/(p3- zRef));
        double a = L1 +p3*Math.cos(x)- zRef *Math.cos(x)-p2*Math.sin(x)+ yRef *Math.sin(x);
        double z = Math.acos((Math.pow((p1- xRef),2)+Math.pow(a,2)-Math.pow(L2,2)-Math.pow(L3,2))/(2*L2*L3));
        double b = L2*Math.sin(z)+p1- xRef;
        double y ;
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

        // result[0] = BigDecimal.valueOf(x).setScale(3,BigDecimal.ROUND_HALF_UP).doubleValue();
        // result[1] = BigDecimal.valueOf(y).setScale(3,BigDecimal.ROUND_HALF_UP).doubleValue();
        // result[2] = BigDecimal.valueOf(z).setScale(3,BigDecimal.ROUND_HALF_UP).doubleValue();

        // result[3] = BigDecimal.valueOf(x).setScale(3,BigDecimal.ROUND_HALF_UP).doubleValue();
        // result[4] = BigDecimal.valueOf(y).setScale(3,BigDecimal.ROUND_HALF_UP).doubleValue();
        // result[5] = BigDecimal.valueOf(z).setScale(3,BigDecimal.ROUND_HALF_UP).doubleValue();
        return result;
    }
  
}