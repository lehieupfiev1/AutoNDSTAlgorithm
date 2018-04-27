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
 * @author Hieu
 */
public class CustomPathItem {
    int id;
    List<Integer> listId;
    PathItem pathItem;
    double time;

    public CustomPathItem() {
    }

    public CustomPathItem(PathItem pathItem) {
        this.pathItem = pathItem;
        listId = new ArrayList<>();
        time =0;
    }

    public CustomPathItem(List<Integer> listId, PathItem pathItem) {
        this.listId = listId;
        this.pathItem = pathItem;
        time =0;
    }

    public CustomPathItem(int id, PathItem pathItem, double time) {
        this.id = id;
        this.pathItem = pathItem;
        this.time = time;
    }

    public CustomPathItem(int id, List<Integer> listId, PathItem pathItem, double time) {
        this.id = id;
        this.listId = listId;
        this.pathItem = pathItem;
        this.time = time;
    }

    public int getId() {
        return id;
    }

    public void setId(int id) {
        this.id = id;
    }

    public List<Integer> getListId() {
        return listId;
    }

    public void setListId(List<Integer> listId) {
        this.listId = listId;
    }

    public PathItem getPathItem() {
        return pathItem;
    }

    public void setPathItem(PathItem pathItem) {
        this.pathItem = pathItem;
    }

    public double getTime() {
        return time;
    }

    public void setTime(double time) {
        this.time = time;
    }
    
    public void addTime(double times) {
        synchronized (this) {
           this.time += times;
        }
    }
}
