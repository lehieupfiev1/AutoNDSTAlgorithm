/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package autondstalgorithm;

/**
 *
 * @author sev_user
 */
public class HeuristicItem {
    int id;
    float value;
    double time;

    public HeuristicItem() {
    }

    public HeuristicItem(int id, float value, double time) {
        this.id = id;
        this.value = value;
        this.time = time;
    }

    public HeuristicItem(int id, float value) {
        this.id = id;
        this.value = value;
        this.time = 0;
    }

    public int getId() {
        return id;
    }

    public void setId(int id) {
        this.id = id;
    }

    public float getValue() {
        return value;
    }

    public void setValue(float value) {
        this.value = value;
    }

    public double getTime() {
        return time;
    }

    public void setTime(double time) {
        this.time = time;
    }
    
}
