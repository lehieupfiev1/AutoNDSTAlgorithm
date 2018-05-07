/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package autondstalgorithm;

import static autondstalgorithm.SensorUtility.*;
import static autondstalgorithm.SensorUtility.readFile;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.swing.JOptionPane;

/**
 *
 * @author Hieu
 * 
 * 
 */
public class AutoNDSTAlgorithm {

     /**
     * @param args the command line arguments
     */
    public static ArrayList<Double> listTotalTime;
    public static long timeRuning;
    public static long timeRunFindPath;
    public static long timeRunCplex;
    public static long timeRunCoppy;
    public static long timeRunCombine;
    public static double timeLifeOn;
    public static String mPath = "E:\\HIEU\\CAO HOC\\Testcase\\";
    
    public static void main(String[] args) {

        // TODO code application logic here
        //NDSTAlgorithm1 algorithm = new NDSTAlgorithm1();
        NDSTAlgorithm11 algorithm = new NDSTAlgorithm11(); //Tinh all path ngay tu dau
        initData();
        //Chay test case tu 6 den 10
        for (int i = 1; i <= 16; i++) {
            try {
                System.out.println("Test case "+i+"---------------------------");
                //Cai dat ten File
                String filename = "test"+i+".INP";
                
                readFile(mPath+filename); //Add URL sensor file with format (
                long begin = System.currentTimeMillis();
                algorithm.run();
                long end = System.currentTimeMillis();
                timeRuning = end - begin;
                timeLifeOn = algorithm.mTimeLife;

            } catch (IOException ex) {
                Logger.getLogger(AutoNDSTAlgorithm.class.getName()).log(Level.SEVERE, null, ex);
            }
            try {
                writeResultFile(mPath+"NSDTAlgorithm12.txt", i, timeRuning, timeLifeOn); //Url luu file input duoc sinh ra
                resetData();
            } catch (IOException ex) {
                Logger.getLogger(AutoNDSTAlgorithm.class.getName()).log(Level.SEVERE, null, ex);
            }
        }
        System.out.println("Running Finish ");
        JOptionPane.showMessageDialog(null, "NSDT Algorithm run finished !");
        
    }
    
    public static void writeResultFile(String filename, int postion, double timeRuning, double timLife) throws IOException {
        FileWriter fw = new FileWriter(filename, true); //the true will append the new data
        fw.write("Test case : "+ postion+"\n");
        fw.write("Sensor="+mListSensorNodes.size() + "  Target="+mListTargetNodes.size()+ "  Sink="+mListSinkNodes.size()+ "  Rs="+mRsValue +"  Rc="+mRcValue +"  MaxHop="+mMaxHopper+ "  L="+Lvalue+"\n");
        fw.write("TimeFindPath = " + AutoNDSTAlgorithm.timeRunFindPath+" ,Time Run = "+ timeRuning+" , Time Life = "+ timLife+"\n");
        fw.write("\n");
        fw.close();

    }
    
    static void initData() {
        listTotalTime = new ArrayList<>();
    }
    
    static void resetData() {
        listTotalTime.clear();
        mListofListPathTime.clear();
        timeLifeOn = 0;
    }
    
    static double calculateTotalTime() {
        listTotalTime.clear();
        for (int i = 0; i < mListofListPathTime.size(); i++) {
            List<Double> next = mListofListPathTime.get(i);
            listTotalTime.add(calculateTotal(next));
        }

        double minimumTime = Double.MAX_VALUE;
        for (int i = 0; i < listTotalTime.size(); i++) {
            if (minimumTime > listTotalTime.get(i).doubleValue())
            minimumTime = listTotalTime.get(i).doubleValue();
        }
        return minimumTime;

    }
    static double calculateTotalTime2() {
     //------------Algorithm 10----------------------------
        double total =0;
        for (int i = 0; i < mListofListTime.size(); i++) {
            double next = mListofListTime.get(i);
            total += next;
        }
        return total;
    }
    static double calculateTotal(List<Double> list) {
        double result =0;
        for (int i =0; i< list.size();i++) {
            result += list.get(i);
        }
        return result;
    }
    //fw.write("TimeFindPath = " + AutoNDSTAlgorithm.timeRunFindPath+" , Time Cplex ="+ timeRunCplex+" , TimeCombine ="+timeRunCombine+ ", TimeCoppy= "+timeRunCoppy+" ,Time Run = "+ timeRuning+" , Time Life = "+ timLife+"\n");//appends the string to the file

}
