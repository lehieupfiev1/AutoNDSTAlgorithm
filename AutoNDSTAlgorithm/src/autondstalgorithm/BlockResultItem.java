/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package autondstalgorithm;

import java.util.List;

/**
 *
 * @author Hieu
 */
public class BlockResultItem {
    int id;
    int postionI;
    int postionJ;
    List<List<PathItem>> listResultX;
    List<Double> listTime;
    double TotalTime;

    public BlockResultItem() {
    }

    
    public BlockResultItem(int postionI, int postionJ, List<List<PathItem>> listResultX, List<Double> listTime, double TotalTime) {
        this.id =0;
        this.postionI = postionI;
        this.postionJ = postionJ;
        this.listResultX = listResultX;
        this.listTime = listTime;
        this.TotalTime = TotalTime;
    }
    
    
    public BlockResultItem(int id, int postionI, int postionJ, List<List<PathItem>> listResultX, List<Double> listTime, double TotalTime) {
        this.id = id;
        this.postionI = postionI;
        this.postionJ = postionJ;
        this.listResultX = listResultX;
        this.listTime = listTime;
        this.TotalTime = TotalTime;
    }

    public int getId() {
        return id;
    }

    public void setId(int id) {
        this.id = id;
    }

    public int getPostionI() {
        return postionI;
    }

    public void setPostionI(int postionI) {
        this.postionI = postionI;
    }

    public int getPostionJ() {
        return postionJ;
    }

    public void setPostionJ(int postionJ) {
        this.postionJ = postionJ;
    }

    public List<List<PathItem>> getListResultX() {
        return listResultX;
    }

    public void setListResultX(List<List<PathItem>> listResultX) {
        this.listResultX = listResultX;
    }

    public List<Double> getListTime() {
        return listTime;
    }

    public void setListTime(List<Double> listTime) {
        this.listTime = listTime;
    }

    public double getTotalTime() {
        return TotalTime;
    }

    public void setTotalTime(double TotalTime) {
        this.TotalTime = TotalTime;
    }
    
    
    
    
}
