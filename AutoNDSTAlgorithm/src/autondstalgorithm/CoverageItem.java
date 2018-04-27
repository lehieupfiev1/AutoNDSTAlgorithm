/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package autondstalgorithm;

import java.util.ArrayList;
import java.util.List;

/**
 *
 * @author sev_user
 */
public class CoverageItem {
    int id; // Id of Sensor
    List<Integer> listCoverage; // List target that sensor coverage 

    public CoverageItem() {
        this.listCoverage = new ArrayList<>();
    }

    public CoverageItem(int id) {
        this.id = id;
        this.listCoverage = new ArrayList<>();
    }
     
    public CoverageItem(int id, List<Integer> listCoverage) {
        this.id = id;
        this.listCoverage = listCoverage;
    }

    public int getId() {
        return id;
    }

    public void setId(int id) {
        this.id = id;
    }

    public List<Integer> getListCoverage() {
        return listCoverage;
    }

    public void setListCoverage(List<Integer> listCoverage) {
        this.listCoverage = listCoverage;
    }
    
    public void addIdCover(int idC){
        if (this.listCoverage != null) {
            boolean exit = false;
            for (int i =0; i< listCoverage.size(); i++) {
                if (listCoverage.get(i) == idC) {
                    exit = true;
                    break;
                }
            }
            if (!exit) {
                this.listCoverage.add(idC);
            }
        }
    }
    
    public int sizeCoverage() {
        if (this.listCoverage != null) {
            return this.listCoverage.size();
        }
        return 0;
    }
}