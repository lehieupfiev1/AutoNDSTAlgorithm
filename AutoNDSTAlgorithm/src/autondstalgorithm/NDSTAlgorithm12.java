/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package autondstalgorithm;

import static autondstalgorithm.SensorUtility.*;
import static autondstalgorithm.SensorUtility.mListSinkNodes;
import static autondstalgorithm.SensorUtility.mListTargetNodes;
import static autondstalgorithm.SensorUtility.mListofListPath;
import ilog.concert.IloException;
import ilog.concert.IloLinearNumExpr;
import ilog.concert.IloNumVar;
import ilog.cplex.IloCplex;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Objects;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 *
 * @author Hieu Coppy from NDST11 Change Algorithm convert
 */
public class NDSTAlgorithm12 {
	public float Distance[][];// Matrix distance between two nodes
	public float MinDistanceSink[];// Matrix distance between two nodes
	public float Target[][];// Target nodes
	public float Point[][];// Total nodes
	public float Sink[][];// Target covering sensors

	public double mTimeLife;
	float Rs, Rc;// Rs and Rt value
	float R; // Tinh phan chia nho nhat
	int MaxHopper;

	List<List<List<PathItem>>> SaveListofListY;
	List<List<List<Double>>> SaveListofListTi;
	List<List<PathItem>> TotalListY;
	List<List<Integer>> SaveListTarget;
	List<List<Integer>> SaveListSensor;
	List<List<Integer>> SaveListSink;
	List<List<PathItem>> resultListY;
	List<List<Double>> resultListTi;
	List<List<Integer>> ListofListNeighbor;
	List<Integer> mListSensor;
	List<Integer> mListTarget;
	List<Integer> mListSink;
	List<Integer> mListSensing;
	List<List<Integer>> mListSensingOfTarget;
	List<CustomPathItem> ListCustomPathItem;
	List<CustomPathItem> mListAllPathItem; // Luu tat ca cac danh sach cau path
	List<List<Integer>> mListPosPathOfTarget; // List danh sach path cuat target trong mListAllPathItem
	List<BlockResultItem> mListBlockResult;
	List<BlockResultItem> mListBlockMaxTime;

	// Result
	List<List<PathItem>> mListResultX;
	List<Double> mListResultT;
	float ListEnergySensor[];
	float ListEnergyUsing[];
	int MAX_INTERGER = 100000000;
	float MAX_FLOAT = 10000000000000.0f;
	float TimeStamp;
	boolean isWidthOptimal = false;
	boolean isHeightOptimal = false;
	boolean isFull = false;

	float Es, Et, Er, Efs, Emp, Do, bit;
	int cnt;
	static int countBlock = 0;

	int K;// Number Sink
	int N;// Number sensor
	int TP; // Total points (Contain Sensor , Sink, Target )
	int T;// Number of Tagert Nodes
	int Anpha; // So lan tang

	int maxBlockPi;
	int maxBlockPj;
	double maxBlockTime;

	public NDSTAlgorithm12() {

	}

	public void run() {

		init();

		readData();
		// Step 1: Find target-covering Sensor
		// FindTargetCoveringSensor();

		// Step 2:

		runAlgorithm();

		long start3 = System.currentTimeMillis();
		CoppyToListSensor();
		long end3 = System.currentTimeMillis();
		// AutoNDSTAlgorithm.timeRunCoppy = end3-start3;
		System.out.println("Time coppy :" + (end3 - start3));
//        
		freeData();
//        
		System.gc();
	}

	public void init() {
		resultListY = new ArrayList<>();
		resultListTi = new ArrayList<>();
		SaveListofListY = new ArrayList<>();
		SaveListofListTi = new ArrayList<>();
		SaveListTarget = new ArrayList<>();
		SaveListSink = new ArrayList<>();
		TotalListY = new ArrayList<>();
		ListofListNeighbor = new ArrayList<>();
		mListSensor = new ArrayList<>();
		mListTarget = new ArrayList<>();
		mListSink = new ArrayList<>();
		mListSensing = new ArrayList<>();
		mListSensingOfTarget = new ArrayList<>();
		ListCustomPathItem = new ArrayList<>();
		mListAllPathItem = new ArrayList<>();
		mListPosPathOfTarget = new ArrayList<>();
		mListResultX = new ArrayList<>();
		mListResultT = new ArrayList<>();
		mListBlockResult = new ArrayList<>();
		maxBlockPi = 0;
		maxBlockPj = 0;
		maxBlockTime = 0;
		mListBlockMaxTime = new ArrayList<BlockResultItem>();
	}

	public void readData() {
		// Read Rs, Rc
		Rs = SensorUtility.mRsValue;
		Rc = SensorUtility.mRcValue;
		mTimeLife = 0;
		MaxHopper = SensorUtility.mMaxHopper;
		R = Rs + MaxHopper * Rc;

		// Read constance Energy : Es, Et,Er,Efs,Emp
		Es = SensorUtility.mEsValue;
		Et = SensorUtility.mEtValue;
		Er = SensorUtility.mErValue;
		Efs = SensorUtility.mEfsValue;
		Emp = SensorUtility.mEmpValue;
		Do = (float) Math.sqrt(Efs / Emp);
		bit = SensorUtility.mBitValue;
		TimeStamp = SensorUtility.mTstamp;
		Anpha = SensorUtility.Lvalue;

		// Read Sensor , Sink, Target
		N = SensorUtility.mListSensorNodes.size();
		T = SensorUtility.mListTargetNodes.size();
		K = SensorUtility.mListSinkNodes.size();
		TP = N + T + K;

		// Add to Total Point;
		Point = new float[TP + 1][2];
		ListEnergySensor = new float[N];
		ListEnergyUsing = new float[N];

		for (int i = 0; i < mListSensorNodes.size(); i++) {
			Point[i][0] = mListSensorNodes.get(i).getX();
			Point[i][1] = mListSensorNodes.get(i).getY();
			// Add Energy for every node
			ListEnergySensor[i] = SensorUtility.mEoValue;
			ListEnergyUsing[i] = 0;
		}

		for (int i = 0; i < mListTargetNodes.size(); i++) {
			Point[N + i][0] = mListTargetNodes.get(i).getX();
			Point[N + i][1] = mListTargetNodes.get(i).getY();
		}
		for (int i = 0; i < mListSinkNodes.size(); i++) {
			Point[N + T + i][0] = mListSinkNodes.get(i).getX();
			Point[N + T + i][1] = mListSinkNodes.get(i).getY();
		}

		// Create matrix distance
		Distance = new float[TP + 1][TP + 1];
		for (int i = 0; i < TP; i++) {
			for (int j = 0; j <= i; j++) {
				if (i == j) {
					Distance[i][j] = 0;
				} else {
					Distance[i][j] = Distance[j][i] = calculateDistance(Point[i][0], Point[i][1], Point[j][0],
							Point[j][1]);
				}
			}
		}

		// Caculate Mindistance form sensor to Sink
		MinDistanceSink = new float[N];
		float min;
		for (int i = 0; i < N; i++) {
			min = MAX_FLOAT;
			for (int j = 0; j < K; j++) {
				if (Distance[i][N + T + j] < min) {
					min = Distance[i][N + T + j];
				}
			}
			MinDistanceSink[i] = min;
		}

		// Init resultListY and resultListTi

		for (int i = 0; i < T; i++) {
			List<PathItem> pathY = new ArrayList<>();
			List<Double> timeY = new ArrayList<>();
			resultListY.add(pathY);
			resultListTi.add(timeY);
		}

	}

	public void freeData() {
		MinDistanceSink = null;
		SaveListofListY = null;
		SaveListofListTi = null;
		SaveListTarget = null;
		SaveListSensor = null;
		Point = null;
		Distance = null;
		SaveListSink = null;
		ListEnergySensor = null;
		TotalListY = null;
		ListofListNeighbor = null;
		mListSensor = null;
		mListSink = null;
		mListTarget = null;
		ListCustomPathItem = null;
		mListSensingOfTarget = null;
		mListSensing = null;
		mListAllPathItem = null;
		mListPosPathOfTarget = null;
		mListResultX = null;
		mListResultT = null;
		mListBlockResult = null;
		mListBlockMaxTime = null;
	}

	public float calculateDistance(float x1, float y1, float x2, float y2) {
		return (float) Math.sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
	}

	boolean checkSensorConnectSink(int sensor, List<Integer> listSink) {
		for (int i = 0; i < listSink.size(); i++) {
			if (Distance[sensor][N + T + listSink.get(i)] <= Rc) {
				return true;
			}
		}
		return false;
	}

	void Finding_CCP(List<Integer> listAllSensor, List<Integer> listTarget, List<Integer> listSink,
			List<List<PathItem>> ListPathY) {
		List<List<List<Integer>>> ListPi = new ArrayList<>();
		List<List<Integer>> ListP = new ArrayList<>();
		List<List<Integer>> ListParent = new ArrayList<>();

		// Check null
		if (listAllSensor.isEmpty() || listTarget.isEmpty() || listSink.isEmpty()) {
			return;
		}
		for (int i = 0; i < listAllSensor.size(); i++) {
			int idSen = listAllSensor.get(i);
			for (int j = 0; j < listTarget.size(); j++) {
				if (Distance[idSen][N + listTarget.get(j)] <= Rs) {
					mListSensing.add(idSen);
					break;
				}
			}
		}

		// Find ListSensor near Sink
		List<Integer> listSensorNear = new ArrayList<>();

		for (int k = 0; k < mListSensing.size(); k++) {
			int idSensing = mListSensing.get(k);
			listSensorNear.clear();
			List<PathItem> listPi = new ArrayList<>();
			// TH sensing là replay
			if (MinDistanceSink[idSensing] <= Rc) {
				List<Integer> list = new ArrayList<>();
				list.add(idSensing);
				PathItem pathItem = new PathItem(list);
				listPi.add(pathItem);
				ListPathY.add(listPi);
				continue;
			}

			// Find list Sensor posible
			for (int j = 0; j < listAllSensor.size(); j++) {
				int id = listAllSensor.get(j);
				if (id != idSensing && Distance[idSensing][id] <= (MaxHopper - 1) * Rc) {
					listSensorNear.add(id);
				}
			}

			// Khoi tao danh sach Pi
			ListP.clear();
			ListParent.clear();
			// System.out.println("Target "+k + " id ="+target);
			//
			List<Integer> listParent1 = new ArrayList<>();
			int num = 0;
			for (int i = 0; i < listSensorNear.size(); i++) {
				if (Distance[idSensing][listSensorNear.get(i)] <= Rc) {
					List<Integer> list = new ArrayList<>();
					list.add(idSensing);
					list.add(listSensorNear.get(i));
					listParent1.add(listSensorNear.get(i));

					if (MinDistanceSink[listSensorNear.get(i)] <= Rc) {
						PathItem p = new PathItem(list);
						listPi.add(p);
					} else {
						ListP.add(list);
						num++;
					}

				}
			}
			for (int j = 0; j < num; j++) {
				ListParent.add(listParent1);
			}

			while (!ListP.isEmpty()) {
				List<Integer> headP = ListP.get(0);
				List<Integer> headParent = ListParent.get(0);
				int lastSensor = headP.get(headP.size() - 1); // Lay phan tu cuoi cung cua head

				if (MinDistanceSink[lastSensor] <= Rc) {
					listPi.add(new PathItem(headP));
					ListP.remove(0);
					ListParent.remove(0);
					continue;
				}

				if (headP.size() == MaxHopper) {
					ListP.remove(0);
					ListParent.remove(0);
				} else {
					List<Integer> listParent = new ArrayList<>();
					for (int j = 0; j < headParent.size(); j++) {
						listParent.add(headParent.get(j));
					}
					int count = 0;
					for (int i = 0; i < listSensorNear.size(); i++) {
						if (lastSensor != listSensorNear.get(i) && Distance[lastSensor][listSensorNear.get(i)] <= Rc) {

							if (!checkPointExitInList(listSensorNear.get(i), headParent)) {
								// Coppy to new Array
								List<Integer> list = new ArrayList<>();
								for (int j = 0; j < headP.size(); j++) {
									list.add(headP.get(j));
								}
								list.add(listSensorNear.get(i));
								listParent.add(listSensorNear.get(i));
								count++;

								// Add list to P
								ListP.add(list);
							}

						}

					}
					for (int j = 0; j < count; j++) {
						ListParent.add(listParent);
					}
					ListP.remove(0);
					ListParent.remove(0);

				}

			}

			ListPathY.add(listPi);

		}

		System.gc();

	}

	void Getting_CCP(List<CustomPathItem> ListAllPathItem, List<Integer> listTarget,
			List<CustomPathItem> ListCustomPath) {
		ListCustomPath.clear();
		boolean Check[] = new boolean[ListAllPathItem.size()];
		for (int i = 0; i < listTarget.size(); i++) {
			int id = listTarget.get(i);
			List<Integer> listPos = mListPosPathOfTarget.get(id);
			for (int j = 0; j < listPos.size(); j++) {
				Check[listPos.get(j)] = true;
			}
		}
		for (int i = 0; i < ListAllPathItem.size(); i++) {
			if (Check[i]) {
				CustomPathItem customPathItem = ListAllPathItem.get(i);
				CustomPathItem newCustomPathItem = new CustomPathItem(customPathItem.getId(),
						customPathItem.getListId(), customPathItem.getPathItem(), 0);

				ListCustomPath.add(newCustomPathItem);
			}
		}
		Check = null;
	}

	boolean checkPointExitInList(int point, List<Integer> listPoint) {
		for (int i = 0; i < listPoint.size(); i++) {
			if (point == listPoint.get(i))
				return true;
		}
		return false;
	}

	int getPointExitInList(int point, List<Integer> listPoint) {
		for (int i = 0; i < listPoint.size(); i++) {
			if (point == listPoint.get(i))
				return i;
		}
		return -1;
	}

	float TranferEnergy(float distance) {
		float result = Et;
		if (distance < Do) {
			result += (Efs * distance * distance);
		} else {
			result += (Emp * distance * distance * distance * distance);
		}
		return result;
	}

	float CaculateEnergyConsume(List<List<Integer>> ListPath, int sensor) {
		List<Integer> path;
		float result = 0;
		for (int i = 0; i < ListPath.size(); i++) {
			path = ListPath.get(i);
			for (int j = 0; j < path.size(); j++) {
				if (j == 0 && sensor == path.get(j)) {
					// TH sensor la sensing node
					result += bit * Es;
					if (path.size() == 1) {
						result += bit * TranferEnergy(MinDistanceSink[sensor]);
					} else {
						result += bit * TranferEnergy(Distance[sensor][path.get(j + 1)]);
					}

					break;
				} else if (sensor == path.get(j)) {
					// TH sensor laf relay node
					result += bit * Er;
					if (j == path.size() - 1) {
						result += bit * TranferEnergy(MinDistanceSink[sensor]);
					} else {
						result += bit * TranferEnergy(Distance[sensor][path.get(j + 1)]);
					}
					break;
				}
			}

		}
		return result;
	}

	int checkExitPathItemInList(PathItem item, List<CustomPathItem> listCustomAllPath) {
		List<Integer> listSensor = item.getPath();
		for (int i = 0; i < listCustomAllPath.size(); i++) {
			CustomPathItem customPathItem = listCustomAllPath.get(i);
			List<Integer> listTempSensor = customPathItem.getPathItem().getPath();
			int count = 0;
			if (listSensor.size() == listTempSensor.size()) {
				for (int j = 0; j < listSensor.size(); j++) {
					if (Objects.equals(listSensor.get(j), listTempSensor.get(j))) {
						count++;
						break;
					}
				}
			}
			if (count == listSensor.size()) {
				return i;
			}
		}
		return -1;
	}

	public void LinearProAlgorithm(List<CustomPathItem> listCustomPath, List<Integer> listSenSor,
			List<Integer> listTarget, double valueE0, boolean isFull) {

		int n = listTarget.size(); // Number target
		int m = listSenSor.size(); // Number sensor
		int Vmax = 0;

		System.out.println();

		if (m == 0 || n == 0) {
			return;
		}

		System.out.println("Total Path " + listCustomPath.size());

		// Check Input
		List<List<Integer>> ListofListPathOfTarget = new ArrayList<>();

		for (int i = 0; i < listTarget.size(); i++) {
			List<Integer> pathOftarget = new ArrayList<>();
			ListofListPathOfTarget.add(pathOftarget);
		}
		// Tim list vitri cua cac path thuoc cac target
		for (int i = 0; i < listCustomPath.size(); i++) {
			List<Integer> listTargetId = listCustomPath.get(i).getListId();
			for (int j = 0; j < listTargetId.size(); j++) {
				for (int k = 0; k < listTarget.size(); k++) {
					if (Objects.equals(listTargetId.get(j), listTarget.get(k))) {
						List<Integer> pathOftarget = ListofListPathOfTarget.get(k);
						pathOftarget.add(i);
						if (!isFull)
							break;
					}
				}

			}
		}

		try {
			// Init model
			IloCplex cplex = new IloCplex();

			// Define variable
			int Max = listCustomPath.size();
			IloNumVar[] t = new IloNumVar[Max];

			for (int j = 0; j < Max; j++) {
				t[j] = cplex.numVar(0, Float.MAX_VALUE);
			}

			// Define Objective
			IloNumVar object = cplex.numVar(0, Float.MAX_VALUE);
			IloLinearNumExpr objective = cplex.linearNumExpr();
			objective.addTerm(1.0, object);

			cplex.addMaximize(objective);

			// Contraint
			// Energy of Sensor <= Eo
			for (int i = 0; i < m; i++) {
				IloLinearNumExpr arrayExpress = cplex.linearNumExpr();
				int sensor = listSenSor.get(i);
				for (int j = 0; j < Max; j++) {
					CustomPathItem customPathItem = listCustomPath.get(j);
					List<Integer> listPa = customPathItem.getPathItem().getPath();
					float value = getEnergyConsumer(listPa, sensor);

					arrayExpress.addTerm(value, t[j]);

				}
				cplex.addLe(arrayExpress, valueE0);
			}

			// Time On of target <= Object
			IloLinearNumExpr[] express = new IloLinearNumExpr[n];
			for (int j = 0; j < n; j++) {
				express[j] = cplex.linearNumExpr();
				List<Integer> listOfPathTarget = ListofListPathOfTarget.get(j);
				for (int k = 0; k < listOfPathTarget.size(); k++) {
					express[j].addTerm(1.0, t[listOfPathTarget.get(k)]);
				}
				cplex.addLe(object, express[j]);

			}

			cplex.setParam(IloCplex.Param.Simplex.Display, 0);

			if (cplex.solve()) {

				// System.out.println("Min time of Target in Block: " + cplex.getObjValue());

				// Reduce variable =0;
				int cnt = 0;
				for (int j = 0; j < listCustomPath.size();) {
					if (cplex.getValue(t[cnt]) > 0) {
						CustomPathItem customPathItem = listCustomPath.get(j);
						customPathItem.setTime(cplex.getValue(t[cnt]));
						j++;
					} else {
						listCustomPath.remove(j);
					}
					cnt++;
				}

				if (isFull) {
					List<List<PathItem>> listPathY = new ArrayList<>();
					List<List<Double>> ListTime = new ArrayList<>();
					for (int i = 0; i < mListTarget.size(); i++) {
						List<PathItem> pathY = new ArrayList<>();
						listPathY.add(pathY);
						List<Double> time = new ArrayList<>();
						ListTime.add(time);
					}
					// Coppy to ListY and ListTime
					for (int i = 0; i < listCustomPath.size(); i++) {
						CustomPathItem customPathItem = listCustomPath.get(i);
						List<Integer> listTagetId = customPathItem.getListId();
						for (int j = 0; j < listTagetId.size(); j++) {
							int id = listTagetId.get(j);
							if (isFull) {
								List<Integer> path = customPathItem.getPathItem().getPath();
								List<Integer> tempPath = new ArrayList<>();
								for (int k = 0; k < path.size(); k++) {
									tempPath.add(path.get(k));
								}
							}
							PathItem pathItem = customPathItem.getPathItem();
							double time = customPathItem.getTime();
							listPathY.get(id).add(pathItem);
							ListTime.get(id).add(time);
						}
					}
				}

				// return cplex.getValue(objective);
			} else {
				System.out.println("Problem not solved");
			}

			cplex.end();

		} catch (IloException ex) {
			Logger.getLogger("LeHieu").log(Level.SEVERE, null, ex);
		}
		// Free data

		ListofListPathOfTarget = null;
		return;
	}

	float getEnergyConsumer(List<Integer> pathYi, int sensor) {
		float result = 0;
		for (int i = 0; i < pathYi.size(); i++) {
			if (i == 0 && pathYi.get(i) == sensor) {
				result += bit * Es;
				if (pathYi.size() == 1) {
					result += bit * TranferEnergy(MinDistanceSink[sensor]);
				} else {
					result += bit * TranferEnergy(Distance[sensor][pathYi.get(i + 1)]);
				}
				return result;
			} else if (pathYi.get(i) == sensor) {
				result += bit * Er;
				if (i == pathYi.size() - 1) {
					result += bit * TranferEnergy(MinDistanceSink[sensor]);
				} else {
					result += bit * TranferEnergy(Distance[sensor][pathYi.get(i + 1)]);
				}
				return result;
			}
		}
		return 0.0f;
	}

	float getEnergyConsumer2(List<Integer> pathYi, int sensor, int pos) {
		float result = 0;

		if (pos == 0) {
			result += bit * Es;
			if (pathYi.size() == 1) {
				result += bit * TranferEnergy(MinDistanceSink[sensor]);
			} else {
				result += bit * TranferEnergy(Distance[sensor][pathYi.get(pos + 1)]);
			}
			return result;
		} else if (pathYi.get(pos) == sensor) {
			result += bit * Er;
			if (pos == pathYi.size() - 1) {
				result += bit * TranferEnergy(MinDistanceSink[sensor]);
			} else {
				result += bit * TranferEnergy(Distance[sensor][pathYi.get(pos + 1)]);
			}
			return result;
		}

		return 0.0f;
	}

	float getMin(float a, float b) {
		if (a < b) {
			return a;
		}
		return b;
	}

	float getMax(float a, float b) {
		if (a > b) {
			return a;
		}
		return b;
	}

	public double ConvertForm2ToForm1(List<CustomPathItem> ListAllPathItem, List<Integer> ListTarget,
			List<List<PathItem>> ListResultX, List<Double> ListTime, boolean isfull) {
		int count = 0;
		double TotalTime = 0;

		System.out.println("Convert ListAllPathItem =" + ListAllPathItem.size());
		do {
			// Sort to calcul nhanh hon
			System.out.println("ConvertForm2ToForm1 do while");
			Collections.sort(ListAllPathItem, new Comparator<CustomPathItem>() {
				@Override
				public int compare(CustomPathItem o1, CustomPathItem o2) {
					double distance1 = o1.getTime();
					double distance2 = o2.getTime();
					return Double.compare(distance2, distance1);
				}
			});

			List<PathItem> listX = new ArrayList<>();
			List<CustomPathItem> listCustomX = new ArrayList<>();
			double Tmin = Double.MAX_VALUE;
			// Reset id
			for (int i = 0; i < ListAllPathItem.size(); i++) {
				CustomPathItem customPathItem = ListAllPathItem.get(i);
				customPathItem.setId(i);
			}
			// Find Ti
			List<CoverageItem> ListT = new ArrayList<>();
			List<HeuristicItem> ListWeight = new ArrayList<>();
			for (int i = 0; i < ListAllPathItem.size(); i++) {
				CustomPathItem customPathItem = ListAllPathItem.get(i);
				int idSensing = customPathItem.getPathItem().getPath().get(0);
				List<Integer> ListCover = new ArrayList<>();
				for (int j = 0; j < ListTarget.size(); j++) {
					if (Distance[idSensing][N + ListTarget.get(j)] <= Rs) {
						ListCover.add(ListTarget.get(j));
					}
				}
				CoverageItem coverageItem = new CoverageItem(i, ListCover);
				ListT.add(coverageItem);
				// Creat list weight
				HeuristicItem heuristicItem = new HeuristicItem(i, 0, customPathItem.getTime());
				ListWeight.add(heuristicItem);
			}

			// Step 1 : Calculate weght with X = rong

			for (int i = 0; i < ListAllPathItem.size(); i++) {
				float wei = ListT.get(i).sizeCoverage();
				ListWeight.get(i).setValue(wei);
			}

			// Step 2: add các path trong P vào tập Xk sao cho tập Xk phủ hết tất cả các
			// target
			List<Integer> listCoverX = new ArrayList<>();
			listCoverX.clear();
			while (!ListWeight.isEmpty() && ListTarget.size() != listCoverX.size()) {

				Collections.sort(ListWeight, new Comparator<HeuristicItem>() {
					@Override
					public int compare(HeuristicItem o1, HeuristicItem o2) {
						float weight1 = o1.getValue();
						float weight2 = o2.getValue();
						if (weight1 != weight2) {
							return Float.compare(weight2, weight1);
						} else {
							return Double.compare(o2.getTime(), o1.getTime());
						}
					}
				});
				// Get gia tri lon nhat
				int id = ListWeight.get(0).getId();

				double time = ListAllPathItem.get(id).getTime();
				listX.add(ListAllPathItem.get(id).getPathItem());
				listCustomX.add(ListAllPathItem.get(id));
				if (time < Tmin) {
					Tmin = time;
				}
				// Remove head of weight
				ListWeight.remove(0);
				// Calculate ListCoverOfX
				List<Integer> listCoverAdd = ListT.get(id).getListCoverage();
				calculateCover(listCoverX, listCoverAdd);
				List<Integer> listNotX = new ArrayList<>();
				boolean check[] = new boolean[ListTarget.size()];
				for (int i = 0; i < listCoverX.size(); i++) {
					for (int j = 0; j < ListTarget.size(); j++) {
						if (Objects.equals(listCoverX.get(i), ListTarget.get(j))) {
							check[j] = true;
							break;
						}
					}
				}
				for (int i = 0; i < check.length; i++) {
					if (!check[i]) {
						listNotX.add(ListTarget.get(i));
					}
				}

				// Update List Weight
				for (int i = 0; i < ListWeight.size(); i++) {
					int idWe = ListWeight.get(i).getId();
					float weight = cal_Weight(ListT.get(idWe).getListCoverage(), listCoverX, listNotX);
					ListWeight.get(i).setValue(weight);
				}

			}
			System.out.println("ConvertForm2ToForm1 do while break");
			if (ListWeight.isEmpty() && ListTarget.size() != listCoverX.size())
				break;

			// Step 3 : update lai time of cac path duwoc add vao Xk theo Tmin

			for (int i = 0; i < listCustomX.size(); i++) {
				CustomPathItem customPathItem = listCustomX.get(i);
				// Reduce time
				double tim = customPathItem.getTime() - Tmin;
				customPathItem.setTime(tim);
				if (tim <= 0) {
					int pos = customPathItem.getId();
					for (int k = 0; k < ListAllPathItem.size(); k++) {
						if (ListAllPathItem.get(k).getId() == pos) {
							ListAllPathItem.remove(k);
							if (!isfull)
								break;
						}
					}
				}
			}

			// Save in result
			ListResultX.add(listX);
			// if (!isfull) {
			// Tmin *= (Math.pow(Anpha, 2) / Math.pow(Anpha + 1, 2));
			// }
			ListTime.add(Tmin);
			TotalTime += Tmin;
			System.out.println("Found X count =" + count + " Size X" + listX.size() + " Time =" + Tmin);
			count++;

		} while (!ListAllPathItem.isEmpty() && checkCoverListTarget(ListAllPathItem, ListTarget));
		System.out.println("Convert TotalTime =" + TotalTime);

		return TotalTime;
	}

	public double ConvertForm2ToForm1_v2(List<CustomPathItem> ListAllPathItem, List<Integer> ListTarget,
			List<List<PathItem>> ListResultX, List<Double> ListTime, boolean isfull) {
		// Creat ListPathTarget
		double TotalTime = 0;
		List<List<CustomPathItem>> ListPathTarget = new ArrayList<>();
		for (int i = 0; i < ListTarget.size(); i++) {
			List<CustomPathItem> listPath = new ArrayList<>();
			ListPathTarget.add(listPath);
		}
		// List Add PositionId of Target in CustomPathItem
		for (int i = 0; i < ListAllPathItem.size(); i++) {
			CustomPathItem customPathItem = ListAllPathItem.get(i);
			List<Integer> listPostionId = customPathItem.getListPositionId();
			listPostionId.clear();
			int idSensing = customPathItem.getPathItem().getPath().get(0);
			for (int j = 0; j < ListTarget.size(); j++) {
				if (Distance[idSensing][N + ListTarget.get(j)] <= Rs) {
					ListPathTarget.get(j).add(customPathItem);
					listPostionId.add(j);
				}
			}

		}
		if (!ListPathTarget.isEmpty()) {
			System.out.println("ListPathTarget 0 =" + ListPathTarget.get(0).size());
		} else {
			return 0;
		}
		// Find X
		List<Integer> listCustomPathTargetId = new ArrayList<>();
		List<Integer> listPositionPathInTarget = new ArrayList<>();
		boolean TargetCover[] = new boolean[ListTarget.size()];
		double MinTime;
		while (checkCorverTargetListExit(ListPathTarget)) {
			List<PathItem> listX = new ArrayList<>();
			listCustomPathTargetId.clear();
			listPositionPathInTarget.clear();

			// reset target cover
			// System.out.println("Reset target cover ");
			for (int i = 0; i < TargetCover.length; i++)
				TargetCover[i] = false;
			MinTime = Double.MAX_VALUE;
			// Find a target not cover
			while (checkNOTCorverTarget(TargetCover)) {
				int Target = 0;
				for (int i = 0; i < TargetCover.length; i++) {
					if (!TargetCover[i]) {
						Target = i;
						break;
					}
				}
				// System.out.println("Target not include X: "+Target);
				// Get a path of Taget
				List<CustomPathItem> listCusPath = ListPathTarget.get(Target);
				int pos = findCustomPathItem(listCusPath, TargetCover);
				CustomPathItem customPathItem = listCusPath.get(pos);
				listX.add(customPathItem.getPathItem());
				if (MinTime > customPathItem.getTime()) {
					MinTime = customPathItem.getTime();
				}
				listCustomPathTargetId.add(Target);
				listPositionPathInTarget.add(pos);
				// Check customPathCover target
				List<Integer> listPosId = customPathItem.getListPositionId();
				for (int j = 0; j < listPosId.size(); j++) {
					TargetCover[listPosId.get(j)] = true;
				}
				// System.out.println("MinTime : "+MinTime);
			}

			// System.out.println("Found a result ");
			// Add result
			ListResultX.add(listX);
			ListTime.add(MinTime);
			TotalTime += MinTime;

			// Update time of path in listX
			for (int i = 0; i < listCustomPathTargetId.size(); i++) {
				int targ = listCustomPathTargetId.get(i);
				int posi = listPositionPathInTarget.get(i);
				CustomPathItem customPathItem = ListPathTarget.get(targ).get(posi);
				double time = customPathItem.getTime() - MinTime;
				customPathItem.setTime(time);
			}
			// Remove all custompath has time =0
			for (int i = 0; i < ListPathTarget.size(); i++) {
				List<CustomPathItem> listPa = ListPathTarget.get(i);
				for (int j = 0; j < listPa.size(); j++) {
					CustomPathItem customPathItem = listPa.get(j);
					if (customPathItem.getTime() == 0) {
						listPa.remove(j);
						break;
					}
				}
			}

		}
		return TotalTime;
	}

	int findCustomPathItem(List<CustomPathItem> listCustomPath, boolean Target[]) {
		int NumberCover = 0;
		int tempCover = 0;
		int NumberNotCover = 0;
		int tempNotCover = 0;
		int pos = 0;
		// Lay phan tu dau tien
		List<Integer> listCoverPos = listCustomPath.get(0).getListPositionId();
		for (int i = 0; i < listCoverPos.size(); i++) {
			if (Target[listCoverPos.get(i)]) {
				NumberCover++;
			} else {
				NumberNotCover++;
			}
		}

		// So sanh voi cac phan tu sau
		for (int i = 1; i < listCustomPath.size(); i++) {
			CustomPathItem customPathItem = listCustomPath.get(i);
			tempCover = 0;
			tempNotCover = 0;
			List<Integer> listCoverPos2 = customPathItem.getListPositionId();

			for (int j = 0; j < listCoverPos2.size(); j++) {
				if (Target[listCoverPos2.get(j)]) {
					tempCover++;
				} else {
					tempNotCover++;
				}
			}

			if (tempCover < NumberCover) {
				NumberCover = tempCover;
				NumberNotCover = tempNotCover;
				pos = i;
			} else if (tempCover == NumberCover && tempNotCover > NumberNotCover) {
				NumberCover = tempCover;
				NumberNotCover = tempNotCover;
				pos = i;
			}

		}

		return pos;

	}

	boolean checkNOTCorverTarget(boolean TargetCover[]) {
		for (int i = 0; i < TargetCover.length; i++) {
			if (!TargetCover[i]) {
				return true;
			}
		}
		return false;
	}

	boolean checkCorverTargetListExit(List<List<CustomPathItem>> ListPathTarget) {
		for (int i = 0; i < ListPathTarget.size(); i++) {
			if (ListPathTarget.get(i).size() == 0) {
				return false;
			}
		}
		return true;
	}

	public double ConvertForm2ToForm1_v3(List<CustomPathItem> ListAllPathItem, List<Integer> ListTarget,
			List<List<PathItem>> ListResultX, List<Double> ListTime, boolean isfull) {
		// Creat ListPathTarget
		double TotalTime = 0;
		List<List<CustomPathItem>> ListPathTarget = new ArrayList<>();
		for (int i = 0; i < ListTarget.size(); i++) {
			List<CustomPathItem> listPath = new ArrayList<>();
			ListPathTarget.add(listPath);
		}
		// List Add PositionId of Target in CustomPathItem
		// Calculate number cover target in a customPathItem
		for (int i = 0; i < ListAllPathItem.size(); i++) {
			CustomPathItem customPathItem = ListAllPathItem.get(i);
			List<Integer> listPostionId = customPathItem.getListPositionId();
			listPostionId.clear();
			int idSensing = customPathItem.getPathItem().getPath().get(0);
			for (int j = 0; j < ListTarget.size(); j++) {
				if (Distance[idSensing][N + ListTarget.get(j)] <= Rs) {
					listPostionId.add(j);
				}
			}

		}

		// Add customPath to ListPathtarget
		for (int i = 0; i < ListAllPathItem.size(); i++) {
			CustomPathItem customPathItem = ListAllPathItem.get(i);
			List<Integer> listPostionId = customPathItem.getListPositionId();
			for (int j = 0; j < listPostionId.size(); j++) {
				int ptarg = listPostionId.get(j);
				CustomPathItem temCustomPath = new CustomPathItem(customPathItem.getId(), customPathItem.getListId(),
						customPathItem.getListPositionId(), customPathItem.getPathItem(), customPathItem.getTime());
				ListPathTarget.get(ptarg).add(temCustomPath);
			}

		}

		// Init temp Energy Using
		float[] tempListEnergyUsing = new float[N];
		for (int i = 0; i < mListSensorNodes.size(); i++) {
			tempListEnergyUsing[i] = 0;
		}

		if (!ListPathTarget.isEmpty()) {
			System.out.println("ListPathTarget 0 =" + ListPathTarget.get(0).size());
		} else {
			return 0;
		}
		// Find X
		List<Integer> listCustomPathTargetId = new ArrayList<>();
		List<Integer> listPositionPathInTarget = new ArrayList<>();
		boolean TargetCover[] = new boolean[ListTarget.size()];
		double MinTime;
		while (checkCorverTargetListExit(ListPathTarget)) {
			List<PathItem> listX = new ArrayList<>();
			listCustomPathTargetId.clear();
			listPositionPathInTarget.clear();
			for (int i = 0; i < ListPathTarget.size(); i++) {
				List<CustomPathItem> tempList = ListPathTarget.get(i);
				Collections.sort(tempList, new Comparator<CustomPathItem>() {
					@Override
					public int compare(CustomPathItem o1, CustomPathItem o2) {
						double distance1 = o1.getTime();
						double distance2 = o2.getTime();
						return Double.compare(distance2, distance1);
					}
				});
			}
			// reset target cover
			// System.out.println("Reset target cover ");
			for (int i = 0; i < TargetCover.length; i++) {
				TargetCover[i] = false;
			}
			MinTime = Double.MAX_VALUE;
			// Find a target not cover
			while (checkNOTCorverTarget(TargetCover)) {
				int Target = 0;
				for (int i = 0; i < TargetCover.length; i++) {
					if (!TargetCover[i]) {
						Target = i;
						break;
					}
				}
				// System.out.println("Target not include X: "+Target);
				// Get a path of Taget
				List<CustomPathItem> listCusPath = ListPathTarget.get(Target);
				int pos = findCustomPathItem(listCusPath, TargetCover);
				CustomPathItem customPathItem = listCusPath.get(pos);
				listX.add(customPathItem.getPathItem());
				if (MinTime > customPathItem.getTime()) {
					MinTime = customPathItem.getTime();
				}
				listCustomPathTargetId.add(Target);
				listPositionPathInTarget.add(pos);
				// Check customPathCover target
				List<Integer> listPosId = customPathItem.getListPositionId();
				for (int j = 0; j < listPosId.size(); j++) {
					TargetCover[listPosId.get(j)] = true;
				}
				// System.out.println("MinTime : "+MinTime);
			}

			// System.out.println("Found a result ");
			// Add result
			ListResultX.add(listX);
			ListTime.add(MinTime);
			TotalTime += MinTime;
			// Calculate Energy using of Sensor
			for (int i = 0; i < listX.size(); i++) {
				List<Integer> tempPath = listX.get(i).getPath();
				for (int j = 0; j < tempPath.size(); j++) {
					int point = tempPath.get(j);
					tempListEnergyUsing[point] += (getEnergyConsumer2(tempPath, point, j) * MinTime);
				}
			}
			// Update time of path in listX
			for (int i = 0; i < listCustomPathTargetId.size(); i++) {
				int targ = listCustomPathTargetId.get(i);
				int posi = listPositionPathInTarget.get(i);
				CustomPathItem customPathItem = ListPathTarget.get(targ).get(posi);
				double time = customPathItem.getTime() - MinTime;
				customPathItem.setTime(time);
			}

			// Remove all custompath has time =0
			if (!isFull) {
				for (int i = 0; i < listCustomPathTargetId.size(); i++) {
					int targ = listCustomPathTargetId.get(i);
					int posi = listPositionPathInTarget.get(i);
					CustomPathItem customPathItem = ListPathTarget.get(targ).get(posi);
					if (customPathItem.getTime() == 0) {
						ListPathTarget.get(targ).remove(posi);
					}
				}
			} else {
				for (int i = 0; i < ListPathTarget.size(); i++) {
					List<CustomPathItem> listPa = ListPathTarget.get(i);
					for (int j = 0; j < listPa.size(); j++) {
						CustomPathItem customPathItem = listPa.get(j);
						if (customPathItem.getTime() == 0) {
							listPa.remove(j);
						}
					}
				}
			}

		}

		float Eij_max = 0;
		for (int m = 0; m < mListSensorNodes.size(); m++) {
			if (tempListEnergyUsing[m] > Eij_max) {
				Eij_max = tempListEnergyUsing[m];
			}
		}
		double ratio = (SensorUtility.mEoValue / Eij_max);
		for (int i = 0; i < ListTime.size(); i++) {
			double time = ListTime.get(i) * ratio;
			ListTime.remove(i);
			ListTime.add(i, time);
		}

		TotalTime *= ratio;
		System.out.println("TotalTime =" + TotalTime + " ratio=" + ratio + " Eijmax =" + Eij_max);
		return TotalTime;
	}

	public double ConvertForm2ToForm1_v4(List<CustomPathItem> ListAllPathItem, List<Integer> ListTarget,
			List<List<PathItem>> ListResultX, List<Double> ListTime, boolean isfull) {
		// Creat ListPathTarget
		double TotalTime = 0;
		List<List<CustomPathItem>> ListPathTarget = new ArrayList<>();
		for (int i = 0; i < ListTarget.size(); i++) {
			List<CustomPathItem> listPath = new ArrayList<>();
			ListPathTarget.add(listPath);
		}
		// List Add PositionId of Target in CustomPathItem
		// Calculate number cover target in a customPathItem
		for (int i = 0; i < ListAllPathItem.size(); i++) {
			CustomPathItem customPathItem = ListAllPathItem.get(i);
			List<Integer> listPostionId = customPathItem.getListPositionId();
			listPostionId.clear();
			int idSensing = customPathItem.getPathItem().getPath().get(0);
			for (int j = 0; j < ListTarget.size(); j++) {
				if (Distance[idSensing][N + ListTarget.get(j)] <= Rs) {
					listPostionId.add(j);
				}
			}

		}

		// Add customPath to ListPathtarget
		for (int i = 0; i < ListAllPathItem.size(); i++) {
			CustomPathItem customPathItem = ListAllPathItem.get(i);
			List<Integer> listPostionId = customPathItem.getListPositionId();
			for (int j = 0; j < listPostionId.size(); j++) {
				int ptarg = listPostionId.get(j);
				CustomPathItem temCustomPath = new CustomPathItem(customPathItem.getId(), customPathItem.getListId(),
						customPathItem.getListPositionId(), customPathItem.getPathItem(), customPathItem.getTime());
				ListPathTarget.get(ptarg).add(temCustomPath);
			}

		}

		// Init temp Energy Using
		float[] tempListEnergyUsing = new float[N];
		for (int i = 0; i < mListSensorNodes.size(); i++) {
			tempListEnergyUsing[i] = 0;
		}

		if (!ListPathTarget.isEmpty()) {
			System.out.println("ListPathTarget 0 =" + ListPathTarget.get(0).size());
		} else {
			return 0;
		}
		// Find X
		List<Integer> listCustomPathTargetId = new ArrayList<>();
		List<Integer> listPositionPathInTarget = new ArrayList<>();
		boolean TargetCover[] = new boolean[ListTarget.size()];
		double MinTime;
		while (checkCorverTargetListExit(ListPathTarget)) {
			List<PathItem> listX = new ArrayList<>();
			listCustomPathTargetId.clear();
			listPositionPathInTarget.clear();
			for (int i = 0; i < ListPathTarget.size(); i++) {
				List<CustomPathItem> tempList = ListPathTarget.get(i);
				Collections.sort(tempList, new Comparator<CustomPathItem>() {
					@Override
					public int compare(CustomPathItem o1, CustomPathItem o2) {
						double distance1 = o1.getTime();
						double distance2 = o2.getTime();
						return Double.compare(distance2, distance1);
					}
				});
			}
			// reset target cover
			// System.out.println("Reset target cover ");
			for (int i = 0; i < TargetCover.length; i++) {
				TargetCover[i] = false;
			}
			MinTime = Double.MAX_VALUE;
			// Find a target not cover

			List<Integer> TargetAndPosi = new ArrayList<>();
			while (checkNOTCorverTarget(TargetCover)) {
				// Intit target and Pos
				TargetAndPosi.clear();
				TargetAndPosi.add(0); // target
				TargetAndPosi.add(0); // postion of path in target
				TargetAndPosi.add(TargetCover.length + 1); // Number cover
				TargetAndPosi.add(0);// Number NOT Cover

				for (int i = 0; i < TargetCover.length; i++) {
					if (!TargetCover[i]) {
						findCustomPathItem2(ListPathTarget, i, TargetCover, TargetAndPosi);
					}
				}
				// System.out.println("Target not include X: "+Target);
				// Get a path of Taget

				int Target = TargetAndPosi.get(0);
				int pos = TargetAndPosi.get(1);
				CustomPathItem customPathItem = ListPathTarget.get(Target).get(pos);
				listX.add(customPathItem.getPathItem());
				if (MinTime > customPathItem.getTime()) {
					MinTime = customPathItem.getTime();
				}
				listCustomPathTargetId.add(Target);
				listPositionPathInTarget.add(pos);
				// Check customPathCover target
				List<Integer> listPosId = customPathItem.getListPositionId();
				for (int j = 0; j < listPosId.size(); j++) {
					TargetCover[listPosId.get(j)] = true;
				}
				// System.out.println("MinTime : "+MinTime);
			}

			// System.out.println("Found a result ");
			// Add result
			ListResultX.add(listX);
			ListTime.add(MinTime);
			TotalTime += MinTime;
			// Calculate Energy using of Sensor
			for (int i = 0; i < listX.size(); i++) {
				List<Integer> tempPath = listX.get(i).getPath();
				for (int j = 0; j < tempPath.size(); j++) {
					int point = tempPath.get(j);
					tempListEnergyUsing[point] += (getEnergyConsumer2(tempPath, point, j) * MinTime);
				}
			}
			// Update time of path in listX
			for (int i = 0; i < listCustomPathTargetId.size(); i++) {
				int targ = listCustomPathTargetId.get(i);
				int posi = listPositionPathInTarget.get(i);
				CustomPathItem customPathItem = ListPathTarget.get(targ).get(posi);
				double time = customPathItem.getTime() - MinTime;
				customPathItem.setTime(time);
			}

			// Remove all custompath has time =0
			if (!isFull) {
				for (int i = 0; i < listCustomPathTargetId.size(); i++) {
					int targ = listCustomPathTargetId.get(i);
					int posi = listPositionPathInTarget.get(i);
					CustomPathItem customPathItem = ListPathTarget.get(targ).get(posi);
					if (customPathItem.getTime() == 0) {
						ListPathTarget.get(targ).remove(posi);
					}
				}
			} else {
				for (int i = 0; i < ListPathTarget.size(); i++) {
					List<CustomPathItem> listPa = ListPathTarget.get(i);
					for (int j = 0; j < listPa.size(); j++) {
						CustomPathItem customPathItem = listPa.get(j);
						if (customPathItem.getTime() == 0) {
							listPa.remove(j);
						}
					}
				}
			}

		}
		// Find Energy max of Sensor
		float Eij_max = 0;
		for (int m = 0; m < mListSensorNodes.size(); m++) {
			if (tempListEnergyUsing[m] > Eij_max) {
				Eij_max = tempListEnergyUsing[m];
			}
		}
		// Calculate ratio and update ListTime
		double ratio = (SensorUtility.mEoValue / Eij_max);
		for (int i = 0; i < ListTime.size(); i++) {
			double time = ListTime.get(i) * ratio;
			ListTime.remove(i);
			ListTime.add(i, time);
		}

		TotalTime *= ratio;
		System.out.println("TotalTime =" + TotalTime + " ratio=" + ratio + " Eijmax =" + Eij_max);
		return TotalTime;
	}

	void findCustomPathItem2(List<List<CustomPathItem>> ListPathTarget, int target, boolean Target[],
			List<Integer> TargetAndPosi) {
		int NumberCover = TargetAndPosi.get(2);
		int tempCover = 0;
		int NumberNotCover = TargetAndPosi.get(3);
		;
		int tempNotCover = 0;

		List<CustomPathItem> listCustomPath = ListPathTarget.get(target);

		// So sanh voi cac phan tu sau
		for (int i = 0; i < listCustomPath.size(); i++) {
			CustomPathItem customPathItem = listCustomPath.get(i);
			tempCover = 0;
			tempNotCover = 0;
			List<Integer> listCoverPos2 = customPathItem.getListPositionId();

			for (int j = 0; j < listCoverPos2.size(); j++) {
				if (Target[listCoverPos2.get(j)]) {
					tempCover++;
				} else {
					tempNotCover++;
				}
			}

			if (tempCover < NumberCover) {
				NumberCover = tempCover;
				NumberNotCover = tempNotCover;

				// Update TargetAndPosi
				TargetAndPosi.remove(0);
				TargetAndPosi.add(0, target); // update target
				TargetAndPosi.remove(1);
				TargetAndPosi.add(1, i); // Update position
				TargetAndPosi.remove(2);
				TargetAndPosi.add(2, NumberCover); // Update Number Cover
				TargetAndPosi.remove(3);
				TargetAndPosi.add(3, NumberNotCover); // Update Number Not Cover
			} else if (tempCover == NumberCover && tempNotCover > NumberNotCover) {
				NumberCover = tempCover;
				NumberNotCover = tempNotCover;
				// Update TargetAndPosi
				TargetAndPosi.remove(0);
				TargetAndPosi.add(0, target); // update target
				TargetAndPosi.remove(1);
				TargetAndPosi.add(1, i); // Update position
				TargetAndPosi.remove(2);
				TargetAndPosi.add(2, NumberCover); // Update Number Cover
				TargetAndPosi.remove(3);
				TargetAndPosi.add(3, NumberNotCover); // Update Number Not Cover
			}

		}

	}

	public double ConvertForm2ToForm1_v5(List<CustomPathItem> ListAllPathItem, List<Integer> ListTarget,
			List<List<PathItem>> ListResultX, List<Double> ListTime, boolean isfull) {
		// Creat ListPathTarget
		System.out.println("ConvertForm2ToForm1_v5 isFull=" + isfull);
		double TotalTime = 0;
		List<List<CustomPathItem>> ListPathTarget = new ArrayList<>();
		for (int i = 0; i < ListTarget.size(); i++) {
			List<CustomPathItem> listPath = new ArrayList<>();
			ListPathTarget.add(listPath);
		}
		// List Add PositionId of Target in CustomPathItem
		// Calculate number cover target in a customPathItem
		for (int i = 0; i < ListAllPathItem.size(); i++) {
			CustomPathItem customPathItem = ListAllPathItem.get(i);
			List<Integer> listPostionId = customPathItem.getListPositionId();
			listPostionId.clear();
			int idSensing = customPathItem.getPathItem().getPath().get(0);
			for (int j = 0; j < ListTarget.size(); j++) {
				if (Distance[idSensing][N + ListTarget.get(j)] <= Rs) {
					listPostionId.add(j);
				}
			}

		}

		// Add customPath to ListPathtarget
		for (int i = 0; i < ListAllPathItem.size(); i++) {
			CustomPathItem customPathItem = ListAllPathItem.get(i);
			List<Integer> listPostionId = customPathItem.getListPositionId();
			for (int j = 0; j < listPostionId.size(); j++) {
				int ptarg = listPostionId.get(j);
				CustomPathItem temCustomPath = new CustomPathItem(customPathItem.getId(), customPathItem.getListId(),
						customPathItem.getListPositionId(), customPathItem.getPathItem(), customPathItem.getTime());
				ListPathTarget.get(ptarg).add(temCustomPath);
			}

		}

		// Init temp Energy Using
		float[] tempListEnergyUsing = new float[N];
		for (int i = 0; i < mListSensorNodes.size(); i++) {
			tempListEnergyUsing[i] = 0;
		}

		if (!ListPathTarget.isEmpty()) {
			System.out.println("ListPathTarget 0 =" + ListPathTarget.get(0).size());
		} else {
			return 0;
		}
		// Find X
		List<Integer> listCustomPathTargetId = new ArrayList<>();
		List<Integer> listPositionPathInTarget = new ArrayList<>();
		boolean TargetCover[] = new boolean[ListTarget.size()];
		double MinTime;
		while (checkCorverTargetListExit(ListPathTarget)) {
			List<PathItem> listX = new ArrayList<>();
			listCustomPathTargetId.clear();
			listPositionPathInTarget.clear();
			for (int i = 0; i < ListPathTarget.size(); i++) {
				List<CustomPathItem> tempList = ListPathTarget.get(i);
				Collections.sort(tempList, new Comparator<CustomPathItem>() {
					@Override
					public int compare(CustomPathItem o1, CustomPathItem o2) {
						double distance1 = o1.getTime();
						double distance2 = o2.getTime();
						return Double.compare(distance2, distance1);
					}
				});
			}
			// reset target cover
			// System.out.println("Reset target cover ");
			for (int i = 0; i < TargetCover.length; i++) {
				TargetCover[i] = false;
			}
			MinTime = Double.MAX_VALUE;
			// Find a target not cover

			List<Integer> TargetAndPosi = new ArrayList<>();
			List<Double> RatioAndTime = new ArrayList<>();
			while (checkNOTCorverTarget(TargetCover)) {
				// Intit target and Pos
				TargetAndPosi.clear();
				RatioAndTime.clear();
				TargetAndPosi.add(0); // target
				TargetAndPosi.add(0); // postion of path in target
				RatioAndTime.add(0.0); // Ratio
				RatioAndTime.add(0.0);// Max Time

				for (int i = 0; i < TargetCover.length; i++) {
					if (!TargetCover[i]) {
						findCustomPathItem3(ListPathTarget, i, TargetCover, TargetAndPosi, RatioAndTime);
					}
				}
				// System.out.println("Target not include X: "+Target);
				// Get a path of Taget

				int Target = TargetAndPosi.get(0);
				int pos = TargetAndPosi.get(1);
				CustomPathItem customPathItem = ListPathTarget.get(Target).get(pos);
				listX.add(customPathItem.getPathItem());
				if (MinTime > customPathItem.getTime()) {
					MinTime = customPathItem.getTime();
				}
				listCustomPathTargetId.add(Target);
				listPositionPathInTarget.add(pos);
				// Check customPathCover target
				List<Integer> listPosId = customPathItem.getListPositionId();
				for (int j = 0; j < listPosId.size(); j++) {
					TargetCover[listPosId.get(j)] = true;
				}
				// System.out.println("MinTime : "+MinTime);
			}

			// System.out.println("Found a result ");
			// Add result
			ListResultX.add(listX);
			ListTime.add(MinTime);
			TotalTime += MinTime;
			// Calculate Energy using of Sensor
			for (int i = 0; i < listX.size(); i++) {
				List<Integer> tempPath = listX.get(i).getPath();
				for (int j = 0; j < tempPath.size(); j++) {
					int point = tempPath.get(j);
					tempListEnergyUsing[point] += (getEnergyConsumer2(tempPath, point, j) * MinTime);
				}
			}
			// Update time of path in listX
			for (int i = 0; i < listCustomPathTargetId.size(); i++) {
				int targ = listCustomPathTargetId.get(i);
				int posi = listPositionPathInTarget.get(i);
				CustomPathItem customPathItem = ListPathTarget.get(targ).get(posi);
				double time = customPathItem.getTime() - MinTime;
				customPathItem.setTime(time);
			}

			// Remove all custompath has time =0
			if (!isFull) {
				for (int i = 0; i < listCustomPathTargetId.size(); i++) {
					int targ = listCustomPathTargetId.get(i);
					int posi = listPositionPathInTarget.get(i);
					CustomPathItem customPathItem = ListPathTarget.get(targ).get(posi);
					if (customPathItem.getTime() == 0) {
						ListPathTarget.get(targ).remove(posi);
					}
				}
			} else {
				for (int i = 0; i < ListPathTarget.size(); i++) {
					List<CustomPathItem> listPa = ListPathTarget.get(i);
					for (int j = 0; j < listPa.size(); j++) {
						CustomPathItem customPathItem = listPa.get(j);
						if (customPathItem.getTime() == 0) {
							listPa.remove(j);
						}
					}
				}
			}

		}
		// Find Energy max of Sensor
		float Eij_max = 0;
		for (int m = 0; m < mListSensorNodes.size(); m++) {
			if (tempListEnergyUsing[m] > Eij_max) {
				Eij_max = tempListEnergyUsing[m];
			}
		}
		// Calculate ratio and update ListTime
		double ratio = (SensorUtility.mEoValue / Eij_max);
		for (int i = 0; i < ListTime.size(); i++) {
			double time = ListTime.get(i) * ratio;
			ListTime.remove(i);
			ListTime.add(i, time);
		}

		TotalTime *= ratio;
		System.out.println("TotalTime =" + TotalTime + " ratio=" + ratio + " Eijmax =" + Eij_max);
		return TotalTime;
	}

	void findCustomPathItem3(List<List<CustomPathItem>> ListPathTarget, int target, boolean Target[],
			List<Integer> TargetAndPosi, List<Double> RatioAndTime) {
		double Ratio = RatioAndTime.get(0);
		int tempCover = 0;
		double MaxTime = RatioAndTime.get(1);
		;
		int tempNotCover = 0;
		double tempTime = 0;

		List<CustomPathItem> listCustomPath = ListPathTarget.get(target);

		// So sanh voi cac phan tu sau
		for (int i = 0; i < listCustomPath.size(); i++) {
			CustomPathItem customPathItem = listCustomPath.get(i);
			tempCover = 0;
			tempNotCover = 0;
			List<Integer> listCoverPos2 = customPathItem.getListPositionId();
			tempTime = customPathItem.getTime();
			for (int j = 0; j < listCoverPos2.size(); j++) {
				if (Target[listCoverPos2.get(j)]) {
					tempCover++;
				} else {
					tempNotCover++;
				}
			}
			double tempRatio = ((double) tempNotCover) / (tempCover + 1);
			if (Ratio < tempRatio) {
				Ratio = tempRatio;
				MaxTime = tempTime;

				// Update TargetAndPosi
				TargetAndPosi.remove(0);
				TargetAndPosi.add(0, target); // update target

				TargetAndPosi.remove(1);
				TargetAndPosi.add(1, i); // Update position

				RatioAndTime.remove(0);
				RatioAndTime.add(1, Ratio); // Update Ratio

				RatioAndTime.remove(1);
				RatioAndTime.add(1, MaxTime); // Update Time
			} else if (Ratio == tempRatio && tempTime > MaxTime) {
				Ratio = tempRatio;
				MaxTime = tempTime;
				// Update TargetAndPosi
				TargetAndPosi.remove(0);
				TargetAndPosi.add(0, target); // update target

				TargetAndPosi.remove(1);
				TargetAndPosi.add(1, i); // Update position

				RatioAndTime.remove(0);
				RatioAndTime.add(1, Ratio); // Update Ratio

				RatioAndTime.remove(1);
				RatioAndTime.add(1, MaxTime); // Update Time
			}

		}

	}

	public void CoppyToListSensor() {
		mListofListPath.clear();
		mListofListPath = resultListY;
		SensorUtility.mListofListPathTime = resultListTi;

		// Create: List All Path and Time
		List<CustomPathItem> ListAllPathItem;
		if (ListCustomPathItem.isEmpty()) {
			ListAllPathItem = mListAllPathItem;
			for (int i = 0; i < ListAllPathItem.size();) {
				CustomPathItem customPathItem = ListAllPathItem.get(i);
				if (customPathItem.getTime() <= 0) {
					ListAllPathItem.remove(i);
				} else {
					i++;
				}
			}
		} else {
			ListAllPathItem = ListCustomPathItem;
		}
		if (isFull) {
			// Reset Energy
			for (int i = 0; i < mListSensorNodes.size(); i++) {
				ListEnergyUsing[i] = 0;
			}
			BlockResultItem blockResultItem = mListBlockResult.get(0);
			List<Double> listTime = blockResultItem.getListTime();
			List<List<PathItem>> ListResultPath = blockResultItem.getListResultX();
			for (int i = 0; i < ListResultPath.size(); i++) {
				List<PathItem> ListPath = ListResultPath.get(i);
				double timeList = listTime.get(i);
				for (int j = 0; j < ListPath.size(); j++) {
					List<Integer> listPoint = ListPath.get(j).getPath();
					for (int k = 0; k < listPoint.size(); k++) {
						int point = listPoint.get(k);
						ListEnergyUsing[point] += (getEnergyConsumer(listPoint, point) * timeList);
					}
				}
			}
			// System.out.println("Nang luong cua cac Sensor :--------------");
			for (int i = 0; i < ListEnergySensor.length; i++) {
				// System.out.print(ListEnergyUsing[i] / 1000000000 + " ");
			}
		}

		// ------Calcular conver to CoverSet

		if (isFull) {
			System.out.println("CoppyToListSensor isFull");
			mListofListNDSTPath = mListBlockResult.get(0).listResultX;
			for (int j = 0; j < mListBlockResult.get(0).listResultX.size(); j++) {
				List<Double> tempListTime = new ArrayList<Double>();
				for (int k = 0; k < mListBlockResult.get(0).listResultX.get(j).size(); k++) {
					tempListTime.add(mListBlockResult.get(0).listTime.get(j));
				}
				mListofListNDSTPathTime.add(tempListTime);
			}

		} else {
			System.out.println("CoppyToListSensor not isFull");
			getListMaxBlock(maxBlockPi, maxBlockPj, mListBlockMaxTime);
			int minBlock = findBlockMinTimefromBlock(mListBlockMaxTime);
			if (minBlock != -1) {
				System.out.println(
						"CoppyToListSensor BlockMinTimefromBlock I=" + mListBlockMaxTime.get(minBlock).getPostionI()
								+ " J=" + mListBlockMaxTime.get(minBlock).getPostionJ() + " TimeBlock="
								+ mListBlockMaxTime.get(minBlock).TotalTime);
				reduceListBlockMaxTime();
				combineTotalXfromBlock(minBlock);
				reduceNDSTPathTime();
			} else {
				System.out.println("CoppyToListSensor can not find minBlock");
			}
		}

		// ---------------------Finish Convert To CoverSet

		converNDSTfollowTarget();
		System.out.println();
	}

	/// -------------------------
	
	void converNDSTfollowTarget() {
		List<List<PathItem>> ListofListResult = new ArrayList<List<PathItem>>(); 
		List<Double> ListTimeResult = new ArrayList<Double>();
		for (int i =0; i < mListofListNDSTPath.size();i++) {
			//
			List<PathItem> ListResultCoverSet = new ArrayList<PathItem>();
			List<PathItem> ListCoverSet = mListofListNDSTPath.get(i);
			for (int j =0; j< T; j++) {
				for (int k =0; k < ListCoverSet.size(); k++) {
					PathItem path = ListCoverSet.get(k);
					if (Distance[path.getPath().get(0)][N +j] <= Rs) {
						ListResultCoverSet.add(path);
						break;
					}
					
				}
				
			}
			ListofListResult.add(ListResultCoverSet);
			ListTimeResult.add(mListofListNDSTPathTime.get(i).get(0));
			System.out.println("converNDSTfollowTarget CoverSet="+i+ " Size="+ListofListResult.get(i).size());
			
		}
		
		//Reurn result
		mListofListNDSTPath = ListofListResult;
		mListofListTime = ListTimeResult;
	}

	void reduceListBlockMaxTime() {
		// Remove ListPath in every Block have TimeLife = 0
		System.out.println("reduceListBlockMaxTime");
		for (int i = 0; i < mListBlockMaxTime.size(); i++) {
			BlockResultItem blockResultItem = mListBlockMaxTime.get(i);
			for (int j = 0; j < blockResultItem.getListResultX().size();) {
				System.out.println("reduceListBlockMaxTime i =" + i + " j =" + j + " TimeListPath="
						+ blockResultItem.getListTime().get(j));
				if (blockResultItem.getListTime().get(j) < 0.001) {
					blockResultItem.getListResultX().remove(j);
					blockResultItem.getListTime().remove(j);
					System.out.println("reduceListBlockMaxTime i =" + i + " j =" + j);
				} else {
					j++;
				}

			}

		}

	}

	
	void reduceNDSTPathTime() {
		// Calculate Sensor Using Enery
		// Reset Energy
		for (int i = 0; i < mListSensorNodes.size(); i++) {
			ListEnergyUsing[i] = 0;
		}
		
		BlockResultItem blockResultItem = mListBlockResult.get(0);
		List<Double> listTime = blockResultItem.getListTime();
		int NumCoverSet = mListofListNDSTPath.size();
		for (int i = 0; i < NumCoverSet ; i++) {
			List<PathItem> ListPath = mListofListNDSTPath.get(i);
			double timeList = mListofListNDSTPathTime.get(i).get(0);
			for (int j = 0; j < ListPath.size(); j++) {
				List<Integer> listPoint = ListPath.get(j).getPath();
				for (int k = 0; k < listPoint.size(); k++) {
					int point = listPoint.get(k);
					ListEnergyUsing[point] += (getEnergyConsumer(listPoint, point) * timeList);
				}
			}
		}
		
		// Find Energy max of Sensor
		float Eij_max = 0;
		for (int m = 0; m < ListEnergyUsing.length; m++) {
			if (ListEnergyUsing[m] > Eij_max) {
				Eij_max = ListEnergyUsing[m];
			}
		}
		// Calculate ratio and update mListofListNDSTPathTime
		double ratio = (SensorUtility.mEoValue / Eij_max);
		for (int i = 0; i < mListofListNDSTPathTime.size(); i++) {
			double time = mListofListNDSTPathTime.get(i).get(0) * ratio;
			List<Double> ListTime = mListofListNDSTPathTime.get(i);
			for (int j =0; j <ListTime.size(); j++) {
			   ListTime.remove(j);
			   ListTime.add(j, time);
			}
		}
		
		//Find total Time of CoverSet after Reduce
		double Total =0;
		for (int i = 0; i < mListofListNDSTPathTime.size(); i++) {
			double time = mListofListNDSTPathTime.get(i).get(0);
			Total +=time;
		}
		System.out.println("ReduceNDSTPathTime  ToatalReduce="+ Total+"ratio=" + ratio + " Eijmax =" + Eij_max);
		
	}
	 

	void combineTotalXfromBlock(int minBlock) {
		int count = 0;
		long TotalTime = 0;
		boolean checkTaget[] = new boolean[T];
		boolean found;
		do {
			count++;
			List<PathItem> listResultPath = new ArrayList<PathItem>();
			List<Double> listResultTime = new ArrayList<Double>();
			double minTime = findMinTimeInPathfromBlock();
			//Reset check target 
			for (int i =0; i< T; i++) {
				checkTaget[i] = false;
			}
			
			for (int i = 0; i < mListBlockMaxTime.size(); i++) {
				List<PathItem> listPath = new ArrayList<PathItem>();
				listPath = mListBlockMaxTime.get(i).listResultX.get(0);
				for (int j = 0; j < listPath.size(); j++) {
					int sensorId = listPath.get(j).getElement(0);
					found = false;
					for (int k =0; k< T; k++) {
						if (Distance[sensorId][N +k] <= Rs) {
							if (!checkTaget[k]) {
								checkTaget[k] = true;
								found = true;
							}
						}
					}
					if (found) {
					        listResultPath.add(listPath.get(j));
				                listResultTime.add(minTime);
					}
				}
			}

			TotalTime += minTime;
			updateTimeInPathfromBlock(minTime);

			mListofListNDSTPath.add(listResultPath);
			mListofListNDSTPathTime.add(listResultTime);
		} while (mListBlockMaxTime.get(minBlock).listResultX.size() > 0);
		System.out.println("combineTotalXfromBlock" + count + " TotalTime:" + TotalTime);
	}

	double findMinTimeInPathfromBlock() {
		double minTime = Double.MAX_VALUE;
		for (int j = 0; j < mListBlockMaxTime.size(); j++) {
			System.out.println("findMinTimeInPathfromBlock j = " + j + " -" + mListBlockMaxTime.get(j).listTime.size());
			double time = mListBlockMaxTime.get(j).listTime.get(0);
			if (time > 0.001 && time < minTime) {
				minTime = time;
			}
		}
		return minTime;
	}

	void updateTimeInPathfromBlock(double minTime) {
		System.out.println("updateTimeInPathfromBlock minTime = " + minTime);
		for (int i = 0; i < mListBlockMaxTime.size(); i++) {
			double time = mListBlockMaxTime.get(i).listTime.get(0);
			double retainTime = time - minTime;
			mListBlockMaxTime.get(i).listTime.remove(0);
			if (retainTime > 0.001) {
				mListBlockMaxTime.get(i).listTime.add(0, retainTime);
			} else {
				// Remove listPath
				mListBlockMaxTime.get(i).listResultX.remove(0);
			}
		}
	}

	int findBlockMinTimefromBlock(List<BlockResultItem> listBlock) {
		int result = -1;
		double timeMin = Double.MAX_VALUE;
		for (int i = 0; i < listBlock.size(); i++) {
			if (listBlock.get(i).TotalTime < timeMin) {
				timeMin = listBlock.get(i).TotalTime;
				result = i;
			}
		}
		return result;
	}

	void getListMaxBlock(int posI, int posJ, List<BlockResultItem> listBlock) {
		double tempx = SensorUtility.numberRow / (2 * R);
		double tempy = SensorUtility.numberColum / (2 * R);
		int Kx = (int) Math.ceil((tempx - posI) / Anpha);
		int Ky = (int) Math.ceil((tempy - posJ) / Anpha);
		listBlock.clear();
		System.out.println("Bat dau chia ");
		for (int u = 0; u <= Kx; u++) {
			for (int v = 0; v <= Ky; v++) {
				int positionI = posI + u * Anpha;
				int positionJ = posJ + v * Anpha;
				System.out.println("Khoi chia I =" + positionI + " J =" + positionJ);
				for (int i = 0; i < mListBlockResult.size(); i++) {
					BlockResultItem blockResultItem = mListBlockResult.get(i);
					if (blockResultItem.getPostionI() == positionI && blockResultItem.getPostionJ() == positionJ
							&& blockResultItem.TotalTime != 0) {
						listBlock.add(blockResultItem);
						System.out.println("Block add list Max =" + i + " - posI=" + positionI + " - posJ=" + positionJ
								+ " Time=" + blockResultItem.getTotalTime() + " blockResultItemSize="
								+ blockResultItem.getListResultX().size() + " " + blockResultItem.getListTime().size());
					}
				}

			}

		}
		System.out.println("Ket thuc chia Size ListMaxBlock " + listBlock.size());

	}

	//// -----------------------------------

	boolean checkCoverListTarget(List<CustomPathItem> listAllPath, List<Integer> ListTarget) {
		boolean check[] = new boolean[ListTarget.size()];
		for (int i = 0; i < listAllPath.size(); i++) {
			CustomPathItem customPathItem = listAllPath.get(i);
			int idSen = customPathItem.getPathItem().getPath().get(0);
			for (int j = 0; j < ListTarget.size(); j++) {
				if (Distance[idSen][N + ListTarget.get(j)] <= Rs) {
					check[j] = true;
				}
			}
		}

		for (int i = 0; i < check.length; i++) {
			if (!check[i])
				return false;
		}
		return true;
	}

	void calculateCover(List<Integer> listCoverX, List<Integer> listAdd) {
		int N1 = listCoverX.size();
		int count;
		for (int i = 0; i < listAdd.size(); i++) {
			count = 0;
			for (int j = 0; j < N1; j++) {
				if (Objects.equals(listAdd.get(i), listCoverX.get(j))) {
					break;
				} else {
					count++;
				}
			}
			if (count == N1) {
				listCoverX.add(listAdd.get(i));
			}
		}
	}

	float cal_Weight(List<Integer> listCoverP, List<Integer> listCoverX, List<Integer> listNotX) {
		int tuso = 0;
		int mauso = 1;
		// Tinh tu so
		for (int i = 0; i < listCoverP.size(); i++) {
			for (int j = 0; j < listNotX.size(); j++) {
				if (Objects.equals(listCoverP.get(i), listNotX.get(j))) {
					tuso++;
					break;
				}
			}
		}

		// Tinh mau so
		for (int i = 0; i < listCoverP.size(); i++) {
			for (int j = 0; j < listCoverX.size(); j++) {
				if (Objects.equals(listCoverP.get(i), listCoverX.get(j))) {
					mauso++;
					break;
				}
			}
		}

		return (float) tuso / mauso;

	}

	public void FindingAllCustomPath(List<Integer> listSensing, List<Integer> listTarget,
			List<List<PathItem>> ListPathY, List<CustomPathItem> listAllPathItem) {
		int count = 0;
		for (int i = 0; i < ListPathY.size(); i++) {
			List<PathItem> PathY = ListPathY.get(i);
			int idSensing = listSensing.get(i);
			// Find list Target duoc phu boi idSening
			List<Integer> listIdTarget = new ArrayList<>();
			for (int j = 0; j < listTarget.size(); j++) {
				if (Distance[idSensing][N + listTarget.get(j)] <= Rs) {
					listIdTarget.add(listTarget.get(j));
				}
			}
			// Add lan luot cac phan tu vao path
			for (int j = 0; j < PathY.size(); j++) {
				PathItem pathItem = PathY.get(j);
				CustomPathItem customPathItem = new CustomPathItem(count, listIdTarget, pathItem, 0);
				for (int k = 0; k < listIdTarget.size(); k++) {
					int idTar = listIdTarget.get(k);
					List<Integer> listPos = mListPosPathOfTarget.get(idTar);
					listPos.add(count);
				}
				listAllPathItem.add(customPathItem);
				count++;
			}
		}

	}

	public void runAlgorithm() {
		System.out.println("NDSTAlgorith12 runAlgorithm");
		mListSensor.clear();
		for (int i = 0; i < mListSensorNodes.size(); i++) {
			mListSensor.add(i);
		}

		mListTarget.clear();
		for (int i = 0; i < mListTargetNodes.size(); i++) {
			mListTarget.add(i);
		}

		mListSink.clear();
		for (int i = 0; i < mListSinkNodes.size(); i++) {
			mListSink.add(i);
		}
		long start = System.currentTimeMillis();
		// Find listSensing of target
		for (int i = 0; i < mListTarget.size(); i++) {
			int idTarget = mListTarget.get(i);
			List<Integer> listSensing = new ArrayList<>();
			for (int j = 0; j < mListSensor.size(); j++) {
				if (Distance[mListSensor.get(j)][N + idTarget] <= Rs) {
					listSensing.add(mListSensor.get(j));
				}

			}
			mListSensingOfTarget.add(listSensing);

			// Create list postion Path of target
			List<Integer> listPos = new ArrayList<>();
			mListPosPathOfTarget.add(listPos);

		}
		// Calculate total Path in network
		Finding_CCP(mListSensor, mListTarget, mListSink, TotalListY);

		// ------Kiem tra ton tai nghiem
		boolean TargetStatus[] = new boolean[T];
		for (int i = 0; i < TotalListY.size(); i++) {
			List<PathItem> ListY = TotalListY.get(i);
			if (ListY.size() == 0)
				continue;
			int SensingId = ListY.get(0).path.get(0);
			// System.out.println("NDSTAlgorith12 "+ListY.size());
			for (int j = 0; j < T; j++) {
				if (Distance[SensingId][N + mListTarget.get(j)] <= Rs) {
					TargetStatus[j] = true;
				}
			}
		}

		for (int j = 0; j < T; j++) {
			if (!TargetStatus[j]) {
				System.out.println("Khong ton tai path tai Target id = " + j);
				return;
			}
		}

		/// ------Ket thuc kiem tra ton tai nghiem--------

		FindingAllCustomPath(mListSensing, mListTarget, TotalListY, mListAllPathItem);

		long end = System.currentTimeMillis();
		AutoNDSTAlgorithm.timeRunFindPath = end - start;
		System.out.println("Time run find path =" + AutoNDSTAlgorithm.timeRunFindPath);

		List<List<List<PathItem>>> tempListOfListY = new ArrayList<>();
		List<List<List<Double>>> tempListOfListTi = new ArrayList<>();

		long start1 = System.currentTimeMillis();
		float MaxSizeBlock = 2 * R * Anpha;
		FloatPointItem tmpUpPoint = new FloatPointItem(0, 0);
		FloatPointItem tmpDownPoint = new FloatPointItem(MaxSizeBlock, MaxSizeBlock);
		List<Integer> tmpListSensor = FindListSensor(tmpUpPoint, tmpDownPoint);
		List<Integer> tmpListTarget = FindListTarget(tmpUpPoint, tmpDownPoint);
		List<Integer> tmpListSink = FindListSink(tmpUpPoint, tmpDownPoint);

		if (tmpListSensor.size() != mListSensor.size() || tmpListTarget.size() != mListTarget.size()
				|| tmpListSink.size() != mListSink.size()) {
			isFull = false;
			List<Thread> mListThread = new ArrayList<>();

			int X = (int) Math.ceil(SensorUtility.numberRow / (2 * R)) + Anpha - 1;
			int Y = (int) Math.ceil(SensorUtility.numberColum / (2 * R)) + Anpha - 1;
			System.out.println("Max postion i :" + X + " - Max postion j :" + Y);
			countBlock = 0;
			for (int i = 1; i <= X; i++) {
				for (int j = 1; j <= Y; j++) {
					// Tao thread
					float x1 = getMax(0, -2 * Anpha * R + 2 * i * R);
					float y1 = getMax(0, -2 * Anpha * R + 2 * j * R);

					float x2 = getMin(2 * i * R, SensorUtility.numberRow);
					float y2 = getMin(2 * j * R, SensorUtility.numberColum);
					if (x2 > x1 && y2 > y1) {
						int positionI = i;
						int positionJ = j;
						FloatPointItem upPoint = new FloatPointItem(x1, y1);
						FloatPointItem downPoint = new FloatPointItem(x2, y2);
						List<Integer> tempListTarget = FindListTarget(upPoint, downPoint);
						System.out.println("Tij  I:" + positionI + "J :" + positionJ + " upPoint=( " + upPoint.getX()
								+ " , " + upPoint.getY() + " )" + "  downPoint=( " + downPoint.getX() + " , "
								+ downPoint.getY() + " )");

						if (!tmpListTarget.isEmpty()) {
							// Kiem tra khoi la full mang
							Thread thread = new Thread(new Runnable() {
								@Override
								public void run() {
									// Find ListSensor in Block

									List<Integer> tempListSensor = FindListSensor(upPoint, downPoint);
									// List<Integer> tempListSink = FindListSink(upPoint, downPoint);
									// Get List
									List<CustomPathItem> ListCustomPath = new ArrayList<>();

									Getting_CCP(mListAllPathItem, tempListTarget, ListCustomPath);

									// Update value in CustomPathItem
									LinearProAlgorithm(ListCustomPath, tempListSensor, tempListTarget,
											SensorUtility.mEoValue, false);

									// Convert Form 2 to Form 1
									List<List<PathItem>> listResultX = new ArrayList<>();
									List<Double> listTime = new ArrayList<>();
									double Totaltime = ConvertForm2ToForm1_v5(ListCustomPath, tempListTarget,
											listResultX, listTime, false);
									BlockResultItem blockResultItem = new BlockResultItem(positionI, positionJ,
											listResultX, listTime, Totaltime);
									mListBlockResult.add(blockResultItem);

									// Add result of Block
									countBlock++;
									System.out.println("Khoi :" + countBlock);
									System.out.println("Toa do : (" + upPoint.getX() + " , " + upPoint.getY() + ") - ("
											+ downPoint.getX() + " , " + downPoint.getY() + ")");

								}

							});
							thread.start();
							mListThread.add(thread);
						}
					}

				}

			}

			// Set main thread wait
			for (int i = 0; i < mListThread.size(); i++) {
				Thread thread = mListThread.get(i);
				try {
					thread.join();
				} catch (InterruptedException ex) {
					Logger.getLogger(NDSTAlgorithm6.class.getName()).log(Level.SEVERE, null, ex);
				}

			}
		} else {
			isFull = true;
			Thread thread = new Thread(new Runnable() {
				@Override
				public void run() {
					List<CustomPathItem> ListCustomPath = new ArrayList<>();

					Getting_CCP(mListAllPathItem, mListTarget, ListCustomPath);

					// Update CustomPathItem
					LinearProAlgorithm(ListCustomPath, mListSensor, mListTarget, SensorUtility.mEoValue, true);

					// Convert Form 2 to Form 1
					List<List<PathItem>> listResultX = new ArrayList<>();
					List<Double> listTime = new ArrayList<>();
					double Totaltime = ConvertForm2ToForm1_v5(ListCustomPath, mListTarget, listResultX, listTime, true);
					BlockResultItem blockResultItem = new BlockResultItem(0, 0, listResultX, listTime, Totaltime);
					mListBlockResult.add(blockResultItem);
				}

			});
			thread.start();
			try {
				thread.join();
			} catch (InterruptedException ex) {
				Logger.getLogger(NDSTAlgorithm8.class.getName()).log(Level.SEVERE, null, ex);
			}
		}
		long end1 = System.currentTimeMillis();
		// AutoNDSTAlgorithm.timeRunCplex = end1-start1;
		System.out.println("Count of Block :" + countBlock);
		System.out.println("Part time Cplex :" + (end1 - start1));

		long start2 = System.currentTimeMillis();
		// Combining_All_Division(mListAllPathItem,resultListY,resultListTi,isFull);
		mTimeLife = Combining_All_Division2(mListBlockResult, isFull);
		long end2 = System.currentTimeMillis();
		// AutoNDSTAlgorithm.timeRunCombine = end2-start2;
		System.out.println("Part time Combine:" + (end2 - start2));

		// Free data

	}

	void reduceListPathYi(List<List<PathItem>> ListPathY, List<List<Double>> ListTimeY) {
		for (int i = 0; i < ListPathY.size(); i++) {
			List<PathItem> pathY = ListPathY.get(i);
			List<Double> timeListY = ListTimeY.get(i);
			// Giam cac TH TY = 0;
			for (int j = 0; j < timeListY.size();) {
				if (timeListY.get(j) < 0.00001d) {
					timeListY.remove(j);
					pathY.remove(j);
				} else {
					j++;
				}

			}

		}

	}

	public List<Integer> FindListSensor(FloatPointItem UpLeftCornerPoint, FloatPointItem DownRightCornerPoint) {
		List<Integer> resultListSensor = new ArrayList<>();
		float Xmax, Xmin, Ymax, Ymin;
		Xmin = UpLeftCornerPoint.getX() - R;
		Xmax = DownRightCornerPoint.getX() + R;
		Ymin = UpLeftCornerPoint.getY() - R;
		Ymax = DownRightCornerPoint.getY() + R;

		for (int i = 0; i < mListSensorNodes.size(); i++) {
			if (mListSensorNodes.get(i).getX() >= Xmin && mListSensorNodes.get(i).getX() < Xmax
					&& mListSensorNodes.get(i).getY() >= Ymin && mListSensorNodes.get(i).getY() < Ymax) {
				resultListSensor.add(i);
			}
		}
		return resultListSensor;
	}

	public List<Integer> FindListTarget(FloatPointItem UpLeftCornerPoint, FloatPointItem DownRightCornerPoint) {
		List<Integer> resultListTarget = new ArrayList<>();
		float Xmax, Xmin, Ymax, Ymin;
		Xmin = UpLeftCornerPoint.getX();
		Xmax = DownRightCornerPoint.getX();
		Ymin = UpLeftCornerPoint.getY();
		Ymax = DownRightCornerPoint.getY();

		for (int i = 0; i < mListTargetNodes.size(); i++) {
			if (mListTargetNodes.get(i).getX() >= Xmin && mListTargetNodes.get(i).getX() < Xmax
					&& mListTargetNodes.get(i).getY() >= Ymin && mListTargetNodes.get(i).getY() < Ymax) {
				resultListTarget.add(i);
			}
		}
		return resultListTarget;
	}

	public List<Integer> FindListSink(FloatPointItem UpLeftCornerPoint, FloatPointItem DownRightCornerPoint) {
		List<Integer> resultListSink = new ArrayList<>();
		float Xmax, Xmin, Ymax, Ymin;
		Xmin = UpLeftCornerPoint.getX() - R - Rc;
		Xmax = DownRightCornerPoint.getX() + R + Rc;
		Ymin = UpLeftCornerPoint.getY() - R - Rc;
		Ymax = DownRightCornerPoint.getY() + R + Rc;

		for (int i = 0; i < mListSinkNodes.size(); i++) {
			if (mListSinkNodes.get(i).getX() >= Xmin && mListSinkNodes.get(i).getX() < Xmax
					&& mListSinkNodes.get(i).getY() >= Ymin && mListSinkNodes.get(i).getY() < Ymax) {
				resultListSink.add(i);
			}
		}
		return resultListSink;
	}

	void unionListY(List<PathItem> inputListY, List<Double> inputTime, List<PathItem> outputListY,
			List<Double> outputTime) {
		if (outputListY.isEmpty()) {
			for (int i = 0; i < inputListY.size(); i++) {
				outputListY.add(inputListY.get(i));
				outputTime.add(inputTime.get(i));
			}

		} else {
			for (int i = 0; i < inputListY.size(); i++) {
				int pos = CheckExitPath(inputListY.get(i), outputListY);
				if (pos == -1) {
					outputListY.add(inputListY.get(i));
					outputTime.add(inputTime.get(i));
				} else {
					double firstTime = outputTime.get(pos);
					double second = inputTime.get(i);
					outputTime.remove(pos);
					outputTime.add(pos, firstTime + second);
				}

			}

		}

	}

	int CheckExitPath(PathItem item, List<PathItem> list) {
		for (int i = 0; i < list.size(); i++) {
			if (PathItem.compareSame(item, list.get(i))) {
				return i;
			}
		}
		return -1;
	}

	// Ghep Xik U Xin....
	void unionXi(List<Integer> listX, List<Integer> resultListX) {
		int n = resultListX.size();
		boolean isExit;
		for (int i = 0; i < listX.size(); i++) {
			isExit = false;
			for (int j = 0; j < n; j++) {
				if (Objects.equals(listX.get(i), resultListX.get(j))) {
					isExit = true;
					break;
				}
			}
			if (!isExit) {
				resultListX.add(listX.get(i));
			}
		}
	}

	// Dua ra vi tri corver segment cua list
	int getPostionOfList(List<Double> listT, double starTime, double endTime) {
		for (int i = 0; i < listT.size(); i++) {
			if (i == 0) {
				if (listT.get(0) >= endTime) {
					return 0;
				}
			} else {
				if (listT.get(i - 1) <= starTime && listT.get(i) >= endTime) {
					return i;
				}
			}
		}
		return 0;
	}

	// Calculate energy using of all block in a division
	void CalculateEnergyUsing(int posI, int posJ, int Kx, int Ky, List<BlockResultItem> ListBlockResult,
			float[] ListEnergyUsing, double minTime) {
		for (int u = 0; u <= Kx; u++) {
			for (int v = 0; v <= Ky; v++) {
				int positionI = posI + u * Anpha;
				int positionJ = posJ + v * Anpha;
				EnergyUsingInBlock(positionI, positionJ, ListBlockResult, ListEnergyUsing, minTime);
			}

		}
	}

	void EnergyUsingInBlock(int positionI, int positionJ, List<BlockResultItem> ListBlockResult,
			float[] ListEnergyUsing, double minTime) {
		for (int i = 0; i < ListBlockResult.size(); i++) {
			BlockResultItem blockResultItem = ListBlockResult.get(i);
			if (blockResultItem.getPostionI() == positionI && blockResultItem.getPostionJ() == positionJ) {
				// Calculate Energy Using
				double totalTimeBlock = blockResultItem.getTotalTime();
				double ratio = minTime / totalTimeBlock;
				List<List<PathItem>> listResultX = blockResultItem.getListResultX();
				List<Double> listTime = blockResultItem.getListTime();
				for (int j = 0; j < listResultX.size(); j++) {
					List<PathItem> listPathX = listResultX.get(j);
					double timePath = listTime.get(j);
					for (int k = 0; k < listPathX.size(); k++) {
						List<Integer> listPoint = listPathX.get(k).getPath();

						for (int m = 0; m < listPoint.size(); m++) {
							int point = listPoint.get(m);
							ListEnergyUsing[point] += (getEnergyConsumer2(listPoint, point, m) * timePath * ratio);
							// ListEnergyUsing[point] += (getEnergyConsumer(listPoint, point) *
							// timePath*ratio);
						}

					}
				}
			}
		}
		// System.out.println("Khong tim thay block");
		return;

	}

	double getMinTimeOfBlock(int posI, int posJ, int Kx, int Ky, List<BlockResultItem> ListBlockResult) {
		double timeMin = Double.MAX_VALUE;
		System.out.println("Bat dau chia ");
		for (int u = 0; u <= Kx; u++) {
			for (int v = 0; v <= Ky; v++) {
				int positionI = posI + u * Anpha;
				int positionJ = posJ + v * Anpha;
				System.out.println("Khoi chia I =" + positionI + " J =" + positionJ);
				double time = findTotalTimeFromListBlock(positionI, positionJ, ListBlockResult);
				if (time != 0 && time < timeMin) {
					timeMin = time;
				}
			}

		}
		System.out.println("Ket thuc chia ");
		return timeMin;
	}

	double findTotalTimeFromListBlock(int positionI, int positionJ, List<BlockResultItem> ListBlockResult) {
		for (int i = 0; i < ListBlockResult.size(); i++) {
			BlockResultItem blockResultItem = ListBlockResult.get(i);
			if (blockResultItem.getPostionI() == positionI && blockResultItem.getPostionJ() == positionJ) {

				return blockResultItem.getTotalTime();
			}
		}
		// System.out.println("Khong tim thay block");
		return 0;
	}

	public double Combining_All_Division2(List<BlockResultItem> ListBlockResult, boolean isFull) {
		double network_timelife = 0;
		if (isFull) {
			// TH mang full
			if (ListBlockResult.isEmpty())
				return network_timelife;
			BlockResultItem blockResultItem = ListBlockResult.get(0);
			List<Double> listTime = blockResultItem.getListTime();
			for (int i = 0; i < listTime.size(); i++) {
				network_timelife += listTime.get(i);
			}

		} else {
			// TH division
			double tempx = SensorUtility.numberRow / (2 * R);
			double tempy = SensorUtility.numberColum / (2 * R);
			int current_lifetime;
			int min_dis;
			int count;
			for (int i = 1; i <= Anpha; i++) {
				for (int j = 1; j <= Anpha; j++) {
					// if (i == 1 || j == 1) {
					current_lifetime = 0;
					count = 0;
					// Reset List Energy using
					/*
					 * for (int m =0; m < mListSensorNodes.size();m++) { ListEnergyUsing[m] = 0; }
					 * /* if (i < j) { min_dis = Anpha - j; } else { min_dis = Anpha - i; }
					 * System.out.println("Bat dau tinh min_dis cach chia. Tij  I:" +i + "J :"+j);
					 * for (int k = 0; k <= min_dis; k++) { int i1 = i + k; int j1 = j + k; int Kx =
					 * (int) Math.ceil((tempx - i1) / Anpha); int Ky = (int) Math.ceil((tempy - j1)
					 * / Anpha); if (Kx > 0 && Ky > 0) { double TimeIJ = getMinTimeOfBlock(i1, j1,
					 * Kx, Ky, ListBlockResult); //Calculation ListEnergyUsing
					 * CalculateEnergyUsing(i1, j1, Kx, Ky, ListBlockResult,ListEnergyUsing,TimeIJ);
					 * 
					 * current_lifetime += TimeIJ; count += 1;
					 * 
					 * }
					 * 
					 * }
					 */

					int Kx = (int) Math.ceil((tempx - i) / Anpha);
					int Ky = (int) Math.ceil((tempy - j) / Anpha);
					if (Kx > 0 && Ky > 0) {
						double TimeIJ = getMinTimeOfBlock(i, j, Kx, Ky, ListBlockResult);

						if (maxBlockTime < TimeIJ) {
							maxBlockTime = TimeIJ;
							maxBlockPi = i;
							maxBlockPj = j;
							System.out.println("Combining_All_Division2 maxBlockTime=" + maxBlockTime + " maxBlockI="
									+ maxBlockPi + " maxBlockJ=" + maxBlockPj);
						}

						// Calculation ListEnergyUsing
						CalculateEnergyUsing(i, j, Kx, Ky, ListBlockResult, ListEnergyUsing, TimeIJ);

						network_timelife += TimeIJ;
						count += 1;

					}
					// Find Eij max in min_dis time
					/*
					 * float Eij_max = 0; for (int m = 0; m < mListSensorNodes.size(); m++) { if
					 * (ListEnergyUsing[m] > Eij_max) { Eij_max = ListEnergyUsing[m]; } }
					 * System.out.println("Tij  I:" +i + "J :"+j + " count= " + count +" E0/E_max: "
					 * + SensorUtility.mEoValue / Eij_max); current_lifetime *=
					 * (SensorUtility.mEoValue / Eij_max); // current_lifetime /= (double)count +
					 * 1.0; if (network_timelife < current_lifetime) { network_timelife =
					 * current_lifetime; }
					 */
					// }
				}

				// network_timelife /= ((double)(Anpha)*(double)(Anpha));
			}
			float Eij_max = 0;
			for (int m = 0; m < mListSensorNodes.size(); m++) {
				if (ListEnergyUsing[m] > Eij_max) {
					Eij_max = ListEnergyUsing[m];
				}
			}
			network_timelife *= (SensorUtility.mEoValue / Eij_max);
		}
		return network_timelife;
	}

	public void Combining_All_Division(List<CustomPathItem> ListAllPath, List<List<PathItem>> returnListY,
			List<List<Double>> returnListTi, boolean isFull) {
		// Gop nghiêm lại chia cho L
		// Reduce năng lượng lớn hơn E0
		// Tính cả TH có chung path (các path bị gộp lại là 1)

		// Reduce path == 0
		List<CustomPathItem> ListCustomPath = new ArrayList<>();
		for (int i = 0; i < ListAllPath.size(); i++) {
			CustomPathItem customPathItem = ListAllPath.get(i);
			if (customPathItem.getTime() > 0) {
				ListCustomPath.add(customPathItem);
			}
		}
		// Divice Time to (mLvalue)
		for (int i = 0; i < ListCustomPath.size(); i++) {
			CustomPathItem customPathItem = ListCustomPath.get(i);

			if (!isFull) {
				double time = customPathItem.getTime();
				time = time / (Anpha * Anpha);
				customPathItem.setTime(time);
			}
		}
		// Finding time for block full network
		if (isFull) {
			for (int i = 0; i < returnListY.size(); i++) {
				List<PathItem> pathY = returnListY.get(i);
				pathY.clear();
				List<Double> time = returnListTi.get(i);
				time.clear();
			}
			// Coppy to ListY and ListTime

			for (int i = 0; i < ListAllPath.size(); i++) {
				CustomPathItem customPathItem = ListAllPath.get(i);
				List<Integer> listTagetId = customPathItem.getListId();
				for (int j = 0; j < listTagetId.size(); j++) {
					int id = listTagetId.get(j);
					PathItem pathItem = customPathItem.getPathItem();

					List<Integer> tempPath = new ArrayList<>();
					for (int k = 0; k < pathItem.getPath().size(); k++) {
						tempPath.add(pathItem.getPath().get(k));
					}
					double time = customPathItem.getTime();
					returnListY.get(id).add(pathItem);
					returnListTi.get(id).add(time);
				}
			}
			ListCustomPath = ListAllPath;
		}

		// Tim list sensor in all Path
		List<Integer> listSensorInAllPath = new ArrayList<>();
		FindListSensorInAllPath(ListCustomPath, listSensorInAllPath);

		if (listSensorInAllPath.isEmpty())
			return;
		// Tao list Energy tuong ung voi cac sensor
		List<EnergyItem> listEnergy = new ArrayList<>();
		for (int i = 0; i < listSensorInAllPath.size(); i++) {
			EnergyItem energyItem = new EnergyItem(listSensorInAllPath.get(i), 0);
			listEnergy.add(energyItem);
		}

		// tao list luu vi tri trung nhau
		List<List<Integer>> ListPostionY = new ArrayList<>();
		for (int i = 0; i < returnListY.size(); i++) {
			List<PathItem> PathY = returnListY.get(i);
			List<Integer> postionY = new ArrayList<>();
			for (int j = 0; j < PathY.size(); j++) {
				postionY.add(-1);
			}
			ListPostionY.add(postionY);
		}

		// Calculate Energy using of Sensor
		for (int i = 0; i < listSensorInAllPath.size(); i++) {
			int sensor = listSensorInAllPath.get(i);
			EnergyItem energyItem = listEnergy.get(i);
			for (int j = 0; j < ListCustomPath.size(); j++) {
				PathItem path = ListCustomPath.get(j).getPathItem();
				Double time = ListCustomPath.get(j).getTime();

				float energyUse = (float) (getEnergyConsumer(path.getPath(), sensor) * time);

				if (energyUse > 0) {
					energyItem.addEnergyUse(energyUse);
					energyItem.addPostion(0, j);
				}

			}

		}

		// Sort cac phan tu Energy giam dan
		Collections.sort(listEnergy, new Comparator<EnergyItem>() {
			@Override
			public int compare(EnergyItem o1, EnergyItem o2) {
				float size1 = o1.getEnergyUse();
				float size2 = o2.getEnergyUse();

				return Float.compare(size2, size1);
			}

		});
		ListCustomPathItem.clear();
		float MaxEnergyInList = listEnergy.get(0).getEnergyUse();
		if (MaxEnergyInList <= SensorUtility.mEoValue + SensorUtility.mOffset) {
			// TH1 : Energy sau khi chia cho L (Anpha) khong lon hon E0
			return;

		} else {
			// TH2 Ton tai Energy cua 1 node lon hon Eo sau khi chia cho Anpha (L)
			// Get list Energy lon hon E0
			List<EnergyItem> listEnergyEo = new ArrayList<>();
			listEnergyEo.clear();
			for (int i = 0; i < listEnergy.size(); i++) {
				EnergyItem energyItem = listEnergy.get(i);
				if (energyItem.getEnergyUse() > SensorUtility.mEoValue + SensorUtility.mOffset) {
					listEnergyEo.add(energyItem);
				} else {
					break; // reason : listEnergy sorted by descending order
				}
			}

			CalculateReduceTime(ListCustomPath, listEnergyEo);
			// Update List returnY and Ti
			// Convert to ListY and ListTime
			// Clear data
			for (int i = 0; i < returnListY.size(); i++) {
				List<PathItem> pathY = returnListY.get(i);
				pathY.clear();
				List<Double> time = returnListTi.get(i);
				time.clear();
			}
			// Coppy to ListY and ListTime
			ListCustomPathItem = ListCustomPath;
			for (int i = 0; i < ListCustomPath.size(); i++) {
				CustomPathItem customPathItem = ListCustomPath.get(i);
				List<Integer> listTagetId = customPathItem.getListId();
				for (int j = 0; j < listTagetId.size(); j++) {
					int id = listTagetId.get(j);
					PathItem pathItem = customPathItem.getPathItem();
					double time = customPathItem.getTime();
					returnListY.get(id).add(pathItem);
					returnListTi.get(id).add(time);
				}
			}

		}
		listEnergy = null;

	}

	boolean CheckEnergyMoreThanEo(List<List<PathItem>> returnListY, List<List<Double>> returnListTi) {
		// Test
		// if (true) return true;
		// Tim list sensor in all Path
		List<Integer> listSensorInAllPath = new ArrayList<>();
		// FindListSensorInAllPath(returnListY,listSensorInAllPath);

		if (listSensorInAllPath.isEmpty())
			return false;
		// Tao list Energy tuong ung voi cac sensor
		List<EnergyItem> listEnergy = new ArrayList<>();
		for (int i = 0; i < listSensorInAllPath.size(); i++) {
			EnergyItem energyItem = new EnergyItem(listSensorInAllPath.get(i), 0);
			listEnergy.add(energyItem);
		}

		// Create: List All Path and Time
		List<CustomPathItem> ListAllPathItem = new ArrayList<>();
		for (int i = 0; i < returnListY.size(); i++) {
			List<PathItem> PathY = returnListY.get(i);
			for (int j = 0; j < PathY.size(); j++) {
				PathItem item = PathY.get(j);
				int postion = checkExitPathItemInList(item, ListAllPathItem);
				if (postion == -1) {
					List<Integer> listId = new ArrayList<>();
					listId.add(i);
					CustomPathItem customPathItem = new CustomPathItem(listId, item);
					customPathItem.setTime(returnListTi.get(i).get(j));
					ListAllPathItem.add(customPathItem);
				} else {
					CustomPathItem customPathItem = ListAllPathItem.get(postion);
					customPathItem.getListId().add(i);
				}
			}
		}

		// Calculate Energy using of Sensor
		for (int i = 0; i < listSensorInAllPath.size(); i++) {
			int sensor = listSensorInAllPath.get(i);
			for (int j = 0; j < ListAllPathItem.size(); j++) {
				PathItem path = ListAllPathItem.get(j).getPathItem();
				Double time = ListAllPathItem.get(j).getTime();
				EnergyItem energyItem = listEnergy.get(i);
				float energyUse = (float) (getEnergyConsumer(path.getPath(), sensor) * time);

				if (energyUse > 0) {
					energyItem.addEnergyUse(energyUse);
				}

			}

		}
		// Check exit Energy > Eo
		for (int i = 0; i < listEnergy.size(); i++) {
			float MaxEnergyInList = listEnergy.get(i).getEnergyUse();
			if (MaxEnergyInList > SensorUtility.mEoValue + SensorUtility.mOffset) {
				return true;
			}
		}
		ListAllPathItem = null;
		return false;
	}

	void CalculateReduceTime(List<CustomPathItem> listAllPathItem, List<EnergyItem> listEnergy) {
		float MaxEnergyInList = listEnergy.get(0).getEnergyUse();

		while (MaxEnergyInList > SensorUtility.mEoValue + SensorUtility.mOffset) {
			float ratio = SensorUtility.mEoValue / MaxEnergyInList;
			// Get list vi tri các Path chứa sensor
			List<Integer> listPosPath = listEnergy.get(0).getPosPathList();
			int sensor = listEnergy.get(0).getId();

			// Update Energy of Sensor in ListEnergy
			for (int i = 0; i < listPosPath.size(); i++) {
				CustomPathItem customPathItem = listAllPathItem.get(listPosPath.get(i));
				List<Integer> list = customPathItem.getPathItem().getPath();
				double timePath = customPathItem.getTime();

				for (int j = 0; j < list.size(); j++) {
					int s = list.get(j);
					// Tính lượng năng lượng cần giảm đi của sensor s
					float energy = (float) (getEnergyConsumer(list, s) * timePath * (1 - ratio));
					updateEnergyOfSensor(listEnergy, s, energy);

				}

				// Change time of Path contain Sensor
				customPathItem.setTime(timePath * ratio);

			}

			// Xoa phan tu có năng lượng sử dụng lớn nhất trong Listenergy
			listEnergy.remove(0);
			if (listEnergy.isEmpty())
				return;

			// Sort lại list energy theo thu tu giam dan
			Collections.sort(listEnergy, new Comparator<EnergyItem>() {
				@Override
				public int compare(EnergyItem o1, EnergyItem o2) {
					float size1 = o1.getEnergyUse();
					float size2 = o2.getEnergyUse();

					return Float.compare(size2, size1);
				}

			});

			// Gan lai gia tri Max cua listEnergy
			MaxEnergyInList = listEnergy.get(0).getEnergyUse();

		}

	}

	void updateEnergyOfSensor(List<EnergyItem> listEnergy, int sensor, float energy) {
		for (int i = 0; i < listEnergy.size(); i++) {
			if (listEnergy.get(i).getId() == sensor) {
				listEnergy.get(i).subEnergyUse(energy);
				break;
			}
		}
	}

	void FindListSensorInAllPath(List<CustomPathItem> listCustomPath, List<Integer> listSenSor) {
		boolean checkExit[];
		checkExit = new boolean[N];
		for (int i = 0; i < listCustomPath.size(); i++) {
			PathItem path = listCustomPath.get(i).getPathItem();
			List<Integer> list = path.getPath();
			for (int k = 0; k < list.size(); k++) {
				checkExit[list.get(k)] = true;
			}
		}

		listSenSor.clear();
		for (int i = 0; i < checkExit.length; i++) {
			if (checkExit[i]) {
				listSenSor.add(i);
			}
		}
	}

	boolean CheckExitListX(List<List<Integer>> listX, List<Integer> setX, int postionX) { // checked
		Collections.sort(setX);
		for (int i = 0; i < listX.size(); i++) {
			List<Integer> tempX = listX.get(i);
			if (tempX.size() != setX.size()) {
				continue;
			} else {
				Collections.sort(tempX);
				int count = 0;
				for (int j = 0; j < tempX.size(); j++) {
					if (!Objects.equals(setX.get(j), tempX.get(j))) {
						break;
					} else {
						count++;
					}
				}
				if (count == setX.size()) {
					postionX = i;
					return true;
				}
			}

		}
		return false;
	}

	int CheckExitListTargetInSaveList(List<Integer> listTarget, List<Integer> listSink) {
		int posTarget = -1;
		for (int i = 0; i < SaveListTarget.size(); i++) {
			List<Integer> list = SaveListTarget.get(i);
			if (list.size() == listTarget.size()) {
				int count = 0;
				for (int j = 0; j < listTarget.size(); j++) {
					if (!Objects.equals(list.get(j), listTarget.get(j))) {
						break;
					} else {
						count++;
					}
				}

				if (list.size() == count) {
					posTarget = i;
				}
			}
		}
		if (posTarget != -1 && posTarget < SaveListSink.size()) {
			List<Integer> listS = SaveListSink.get(posTarget);
			if (listS.size() == listSink.size()) {
				int count1 = 0;
				for (int j = 0; j < listSink.size(); j++) {
					if (!Objects.equals(listS.get(j), listSink.get(j))) {
						break;
					} else {
						count1++;
					}
				}

				if (listS.size() == count1) {
					return posTarget;
				}
			}
		}

		return -1;
	}

}
