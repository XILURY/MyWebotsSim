// 暂时测试的地方
import java.io.IOException;
import java.io.PrintWriter;
import java.math.BigDecimal;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Scanner;

public class Test {
    public static void main(String[] arg) throws IOException {
        Scanner in = new Scanner(Paths.get("out1.txt"),"UTF-8");
        ArrayList<Double> out1 = new ArrayList<>();
        while(in.hasNext()){
            out1.add(Double.valueOf(in.nextLine()));
        }
        out1.trimToSize();

        Scanner in2 = new Scanner(Paths.get("out2.txt"),"UTF-8");
        ArrayList<Double> out2 = new ArrayList<>();
        while(in.hasNext()){
            out2.add(Double.valueOf(in2.nextLine()));
        }
        out2.trimToSize();

        Scanner in3 = new Scanner(Paths.get("out3.txt"),"UTF-8");
        ArrayList<Double> out3 = new ArrayList<>();
        while(in.hasNext()){
            out3.add(Double.valueOf(in3.nextLine()));
        }
        out3.trimToSize();


            for(int i=0;i<out1.size();i++){
                System.out.println(out1.get(i));
                System.out.println(out2.get(i));
                System.out.println(out3.get(i));
            }
    }

}
