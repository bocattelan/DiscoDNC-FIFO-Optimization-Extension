/*
 * This file is part of the Deterministic Network Calculator (DNC).
 *
 * Copyright (C) 2019+ Bruno Oliveira Cattelan
 *
 * http://networkcalculus.org
 *
 *
 * The Deterministic Network Calculator (DNC) is free software;
 * you can redistribute it and/or modify it under the terms of the
 * GNU Lesser General Public License as published by the Free Software Foundation;
 * either version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

package org.networkcalculus.dnc.utils.Converters.CPLEX;

import ilog.concert.*;
import ilog.cplex.IloCplex;
import org.networkcalculus.dnc.Calculator;
import org.networkcalculus.dnc.curves.Curve;
import org.networkcalculus.dnc.curves.ServiceCurve;
import org.networkcalculus.dnc.network.server_graph.Flow;
import org.networkcalculus.dnc.network.server_graph.Server;
import org.networkcalculus.dnc.network.server_graph.ServerGraph;
import org.networkcalculus.dnc.utils.Converters.Converter;
import org.networkcalculus.dnc.utils.SetUtils;
import org.networkcalculus.num.Num;

import java.util.*;

/* Set the solution pool relative gap parameter to obtain solutions
            of objective value within 10% of the optimal */
//cplex.setParam(IloCplex.Param.MIP.Pool.RelGap, 0.1);

public class CplexConverter extends Converter {
    private ServerGraph server_graph;
    private Flow foi;
    private IloCplex cplex;
    private Function F;
    private LinkedList<IloNumVar> times;
    private int timeNum = 1;
    private Map<Server, Set<Integer>> serverOutTimes;
    private Map<Server, Set<Integer>> serverInTimes;
    private HashMap<Integer, HashMap<Server, Integer>> fifoOrder;
    private HashMap<Integer, HashMap<Server, Integer>> scOrder;
    private Server emptyServer;
    private List<TimePair> orderedTimes;
    private LinkedList<Server> relevantServers;
    private Map<TimePair, IloNumVar> binaryVariables;
    private HashMap<Server, Integer> serverLabel;
    private HashMap<Server, HashSet<TimePair>> InequalitiesIn;
    private HashMap<Server, HashSet<TimePair>> InequalitiesOut;
    private HashMap<Server, HashSet<TimePair>> InequalitiesInBin;
    private HashMap<Server, HashSet<TimePair>> InequalitiesOutBin;
    private double timeToSolve = 0;
    private double timeToModel = 0;
    private boundType BOUNDTYPE = boundType.TIGHT;

    private enum constraintType {
        FIFO, SC, MONOIN, MONOOUT, AC
    }

    public boolean test = true;
    private OptimizationProblem op;

    //Lower bound is not implemented
    public enum boundType {
        TIGHT, UPPER, LOWER
    }

    public CplexConverter() {
        //clearObjects(null);
        //exportModel();
    }

    public void createModel(ServerGraph serverGraph, Flow flow_of_interest) {
        createModel(serverGraph, flow_of_interest, boundType.TIGHT);
    }

    public void createModel(ServerGraph serverGraph, Flow flow_of_interest, boundType boundType) {
        this.server_graph = serverGraph;
        this.cplex = CplexObject.getInstance();
        clearModel();

        this.BOUNDTYPE = boundType;

        long start, end;
        start = System.nanoTime();

        this.foi = flow_of_interest;

        orderServers();

        try {
            createTimeVariables();
        } catch (IloException e) {
            e.printStackTrace();
        }

        knownTimeInequalities();

        computeInequalitiesIn(foi.getSink());
        for (Server currentServer : relevantServers) {
            computeInequalitiesOut(currentServer);
            if(currentServer.equals(foi.getSink()))
                continue;
            computeInequalitiesIn(currentServer);
        }
        addTimeConstraints();
        addFifoConstraints();
        addServiceCurveConstraints();
        addArrivalCurveConstraints();

        if (boundType.equals(CplexConverter.boundType.TIGHT)) {
            addMonotonicityConstraints();
            addBinaryMonotonicityConstraints();
            addBinaryArrivalCurveConstraints();
            addBinaryTimeConstraints();
        }

        CplexObject.addRemainingConstraints();
        addObjective();

        end = System.nanoTime();

        timeToModel = (end - start) / 1e9;
    }

    private void orderServers() {
        int label = 1;
        Set<Flow> crossFlows = server_graph.getFlows();
        crossFlows.remove(foi);

        for (Flow flow : crossFlows) {
            for (Server server : flow.getServersOnPath()) {
                serverLabel.put(server, label);
                label++;
            }
        }

        for (Server server : foi.getServersOnPath()) {
            serverLabel.put(server, label);
            label++;
        }
        serverLabel.put(foi.getSource(), 0);
        serverLabel.put(foi.getSink(), label);

    }

    private void checkEqualFs() {
        for (HashMap<Server, HashSet<TimePair>> currentInequalities : Arrays.asList(this.InequalitiesIn, this.InequalitiesInBin, this.InequalitiesOut, this.InequalitiesOutBin)){
            for (Server currentServer : currentInequalities.keySet()) {
                for (Flow flow : this.server_graph.getFlows(currentServer)) {
                    for (TimePair timePair : currentInequalities.get(currentServer)) {
                        if (!F.contains(currentServer.getId(), flow.getId(), timePair.left) || !F.contains(currentServer.getId(), flow.getId(), timePair.right)) {
                            continue;
                        }
                        IloNumVar f1;
                        IloNumVar f2;
                        try {
                            f1 = F.get(currentServer.getId(), flow.getId(), timePair.left);
                            f2 = F.get(currentServer.getId(), flow.getId(), timePair.right);
                            IloConstraint constraint = cplex.ifThen(cplex.eq(times.get(timePair.left - 1), times.get(timePair.right - 1)), cplex.eq(f1, f2));
                            if (test) {
                                String constraintName = "EqualFs" + flow.getAlias() + "_" + timePair;
                                constraint.setName(constraintName);
                                op.addArrivalCurveConstraint(constraint, currentServer);
                            }
                            CplexObject.addConstraint(constraint);
                        } catch (IloException e) {
                            System.out.println("Wrong F");
                            e.printStackTrace();
                        }

                    }
                }

            }
        }
    }

    private Set<TimePair> computeTimeSetBin(Server node, constraintType constraintType) {
        Set<TimePair> timePairs = null;
        switch (constraintType) {
            case AC:
                timePairs = InequalitiesInBin.get(node);
                break;
            case MONOIN:
                timePairs = InequalitiesInBin.get(node);
                break;
            case MONOOUT:
                timePairs = InequalitiesOutBin.get(node);
                break;
        }
        return timePairs;
    }

    private Set<TimePair> computeTimeSet(Server node, constraintType constraintType) {
        Set<TimePair> timePairs = new HashSet<>();
        switch (constraintType) {
            case AC:
                timePairs = InequalitiesIn.get(node);
                break;
            case MONOIN:
                timePairs = InequalitiesIn.get(node);
                break;
            case MONOOUT:
                timePairs = InequalitiesOut.get(node);
                break;
            case FIFO:
                for (int t : serverOutTimes.get(node)) {
                    int tPrime = fifoOrder.get(t).get(node);
                    timePairs.add(new TimePair(t, tPrime));
                }
                break;
            case SC:
                for (int t : serverOutTimes.get(node)) {
                    int tPrime = scOrder.get(t).get(node);
                    timePairs.add(new TimePair(t, tPrime));
                }
                break;
        }
        return timePairs;
    }

    private void addArrivalCurveConstraints() {
        for (Flow flow : server_graph.getFlows()) {

            Server currentServer = flow.getSource();
            Set<TimePair> timePairs = computeTimeSet(currentServer, constraintType.AC);

            for (TimePair timePair : timePairs) {
                List<IloNumExpr> segmentExprs;
                IloNumExpr diffFs;
                try {
                    segmentExprs = curveToExpression(flow.getArrivalCurve(), times.get(timePair.left - 1), times.get(timePair.right - 1));
                    diffFs = cplex.diff(F.get(currentServer.getId(), flow.getId(), timePair.left), F.get(currentServer.getId(), flow.getId(), timePair.right));

                    //For each segment we create a new constraint
                    for (IloNumExpr expr : segmentExprs) {
                        IloConstraint constraint = cplex.le(diffFs, expr);
                        if (test) {
                            String constraintName = "ArrivalCurve_" + flow.getAlias() + "_" + timePair;
                            constraint.setName(constraintName);
                            op.addArrivalCurveConstraint(constraint, currentServer);
                        }
                        CplexObject.addConstraint(constraint);
                    }
                } catch (IloException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    private void addBinaryArrivalCurveConstraints() {
        for (Flow flow : server_graph.getFlows()) {

            Server currentServer = flow.getSource();
            Set<TimePair> timePairs = computeTimeSetBin(currentServer, constraintType.AC);

            for (TimePair timePair : timePairs) {
                List<IloNumExpr> segmentExprs;
                IloNumExpr diffFs;
                try {
                    IloNumVar boolVar = binaryVariables.get(timePair);
                    segmentExprs = curveToExpression(flow.getArrivalCurve(), times.get(timePair.left - 1), times.get(timePair.right - 1));
                    diffFs = cplex.diff(F.get(currentServer.getId(), flow.getId(), timePair.left), F.get(currentServer.getId(), flow.getId(), timePair.right));
                    for (IloNumExpr expr : segmentExprs) {
                        IloConstraint constraint = cplex.ifThen(cplex.eq(boolVar, 1), cplex.le(diffFs, expr));
                        if (test) {
                            String constraintName = "ArrivalCurveBin_" + flow.getAlias() + "_" + boolVar + "_1";
                            constraint.setName(constraintName);
                            op.addArrivalCurveConstraint(constraint, currentServer, boolVar);
                        }
                        CplexObject.addConstraint(constraint);
                    }
                    segmentExprs = curveToExpression(flow.getArrivalCurve(), times.get(timePair.right - 1), times.get(timePair.left - 1));
                    diffFs = cplex.diff(F.get(currentServer.getId(), flow.getId(), timePair.right), F.get(currentServer.getId(), flow.getId(), timePair.left));
                    for (IloNumExpr expr : segmentExprs) {
                        IloConstraint constraint = cplex.ifThen(cplex.le(boolVar, 0), cplex.le(diffFs, expr));
                        if (test) {
                            String constraintName = "ArrivalCurveBin_" + flow.getAlias() + "_" + boolVar + "_0";
                            constraint.setName(constraintName);
                            op.addArrivalCurveConstraint(constraint, currentServer, boolVar);
                        }
                        CplexObject.addConstraint(constraint);
                    }
                } catch (IloException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    private void addServiceCurveConstraints() {
        for (Server currentServer : relevantServers) {
            //We first need to create the list of time pairs
            //For all leftT in the set of out times of the node j, and rightT being the SC(leftT)
            Set<TimePair> timePairs = computeTimeSet(currentServer, constraintType.SC);

            //For each time pair we construct both the left side of the constraint, and the right side
            //Left side is the Fs coming out of the server, and the right side is the Fs coming out,
            //plus the segments of the Service Curve
            for (TimePair timePair : timePairs) {
                LinkedList<IloNumVar> outFs = new LinkedList<>();
                LinkedList<IloNumVar> inFs = new LinkedList<>();
                //We have a F in and a F out for each flow in the server
                for (Flow flow : server_graph.getFlows(currentServer)) {
                    Server succServer = successorServer(currentServer, flow);
                    try {
                        outFs.add(F.get(succServer.getId(), flow.getId(), timePair.left));
                        inFs.add(F.get(currentServer.getId(), flow.getId(), timePair.right));
                    } catch (IloException e) {
                        e.printStackTrace();
                    }
                }
                try {
                    //At this point the left portion is finished, but the right still need the segments of the Service Curve
                    IloNumExpr leftSum = cplex.sum(outFs.toArray(new IloNumExpr[0]));
                    IloNumExpr rightFs = cplex.sum(inFs.toArray(new IloNumExpr[0]));
                    List<IloNumExpr> segmentExprs = curveToExpression(currentServer.getServiceCurve(), times.get(timePair.left - 1), times.get(timePair.right - 1));
                    //For each segment we create a new constraint
                    for (IloNumExpr expr : segmentExprs) {
                        IloNumExpr rightSum = cplex.sum(rightFs, expr);
                        IloConstraint constraint = cplex.ge(leftSum, rightSum);
                        if (test) {
                            String constraintName = "ServiceCurve_" + currentServer.getAlias() + timePair;
                            constraint.setName(constraintName);
                            op.addServiceCurveConstraint(constraint, currentServer);
                        }
                        CplexObject.addConstraint(constraint);
                    }
                } catch (IloException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    private void addFifoConstraints() {
        for (Server currentServer : relevantServers) {
            Set<TimePair> timePairs = computeTimeSet(currentServer, constraintType.FIFO);

            for (TimePair timePair : timePairs) {
                for (Flow flow : server_graph.getFlows(currentServer)) {
                    Server succServer = successorServer(currentServer, flow);
                    try {
                        IloConstraint constraint = cplex.eq(F.get(succServer.getId(), flow.getId(), timePair.left), F.get(currentServer.getId(), flow.getId(), timePair.right));
                        if (test) {
                            String constraintName = "FIFO_" + currentServer.getAlias() + "_" + flow.getAlias();
                            constraint.setName(constraintName);
                            op.addFifoConstraint(constraint, currentServer);
                        }
                        CplexObject.addConstraint(constraint);
                    } catch (IloException e) {
                        e.printStackTrace();
                    }
                }
            }
        }
    }

    private Server successorServer(Server currentServer, Flow flow) {
        Server succServer;
        try {
            succServer = flow.getSucceedingServer(currentServer);
        } catch (Exception e) {
            succServer = emptyServer;
        }
        return succServer;
    }

    private void initializeMaps() {
        for (Server server : server_graph.getServers()) {
            serverInTimes.put(server, new HashSet<>());
            serverOutTimes.put(server, new HashSet<>());
            InequalitiesIn.put(server, new HashSet<>());
            InequalitiesOut.put(server, new HashSet<>());
            InequalitiesInBin.put(server, new HashSet<>());
            InequalitiesOutBin.put(server, new HashSet<>());
        }
        binaryVariables = new HashMap<>();
    }

    /**
     * Exports the model created as an .lp file, which is human readable
     * OBS: it will only export variables that are used in the optimization problem
     */
    public void exportModel(String path) {
        try {
            cplex.exportModel(path);
        } catch (IloException e) {
            e.printStackTrace();
        }
    }

    private int addTimeVariable() throws IloException {
        //TODO add them in a batch too
        IloNumVar t = cplex.numVar(0.0, Double.MAX_VALUE);
        if (test) {
            String variableName = "t" + timeNum;
            t.setName(variableName);
            op.addTimeVariable(t);
        }
        times.addLast(t);

        timeNum++;
        return timeNum - 1;
    }

    private int FIFO(Server node, int t) throws IloException {
        int fifoVarName = addTimeVariable();
        if (fifoOrder.containsKey(t)) {
            fifoOrder.get(t).put(node, fifoVarName);
        } else {
            fifoOrder.put(t, new HashMap<>());
            fifoOrder.get(t).put(node, fifoVarName);
        }
        return fifoVarName;
    }

    private int SC(Server node, int t) throws IloException {
        int scVarName = addTimeVariable();
        if (scOrder.containsKey(t)) {
            scOrder.get(t).put(node, scVarName);
        } else {
            scOrder.put(t, new HashMap<>());
            scOrder.get(t).put(node, scVarName);
        }
        return scVarName;
    }

    private void createTimeVariables() throws IloException {
        Server currentServer = foi.getSink();
        relevantServers.addFirst(currentServer);

        //Add t1
        addTimeVariable();
        serverOutTimes.put(currentServer, new HashSet<>(Collections.singleton(1)));

        serverInTimes.put(currentServer, new HashSet<>());
        serverInTimes.get(currentServer).add(FIFO(currentServer, 1));
        serverInTimes.get(currentServer).add(SC(currentServer, 1));

        createTimeVariables(server_graph.getPredecessors(currentServer));

    }

    private void createTimeVariables(Set<Server> servers) throws IloException {
        LinkedList<Server> orderedServers = new LinkedList<>(servers);
        orderedServers.removeIf(server -> !serverLabel.containsKey(server));

        orderedServers.sort((o1, o2) -> {
            if (serverLabel.get(o1) >= serverLabel.get(o2)) {
                return -1;
            }
            return 1;
        });

        for (Server currentServer : orderedServers) {
            for (Server succ : server_graph.getSuccessors(currentServer)) {
                if (serverOutTimes.containsKey(currentServer)) {
                    serverOutTimes.get(currentServer).addAll(serverInTimes.get(succ));
                } else {
                    serverOutTimes.put(currentServer, new HashSet<>(serverInTimes.get(succ)));
                }

                for (int time : serverInTimes.get(succ)) {
                    if (!(fifoOrder.containsKey(time) && fifoOrder.get(time).containsKey(currentServer))) {
                        serverInTimes.get(currentServer).add(FIFO(currentServer, time));
                        serverInTimes.get(currentServer).add(SC(currentServer, time));
                    }
                }
            }
        }

        for (Server currentServer : orderedServers) {
            relevantServers.addLast(currentServer);
            createTimeVariables(server_graph.getPredecessors(currentServer));
        }
    }

    /**
     * Creates the time variables and add their ordering constraints
     * OBS: fifoOrder and scOrder give the name of the variables, but times uses their name - 1 as position
     */
    private void addTimeConstraints() {
        for (Server currentServer : relevantServers) {
            try {
                for (TimePair timePair : SetUtils.getUnion(InequalitiesOut.get(currentServer), InequalitiesIn.get(currentServer))) {
                    IloConstraint constraint = cplex.ge(times.get(timePair.left - 1), times.get(timePair.right - 1));
                    if (test) {
                        String constraintName = "Time_" + timePair + "_" + currentServer.getAlias();
                        constraint.setName(constraintName);
                        op.addTimeConstraint(constraint, currentServer);
                    }
                    CplexObject.addConstraint(constraint);
                }
            } catch (IloException e) {
                e.printStackTrace();
            }
        }

        for (TimePair timePair : orderedTimes) {
            try {
                cplex.addGe(times.get(timePair.left - 1), times.get(timePair.right - 1));
            } catch (IloException e) {
                e.printStackTrace();
            }
        }
    }

    private void addBinaryTimeConstraints() {
        for (Server currentServer : relevantServers) {
            try {
                for (TimePair timePair : SetUtils.getUnion(InequalitiesOutBin.get(currentServer),InequalitiesInBin.get(currentServer))) {
                    IloNumVar boolVar = binaryVariables.get(timePair);
                    IloConstraint constraint = cplex.ifThen(cplex.eq(boolVar, 1), cplex.ge(times.get(timePair.left - 1), times.get(timePair.right - 1)));
                    if (test) {
                        String constraintName = "TimeBin_" + timePair + "_" + currentServer.getAlias();
                        constraint.setName(constraintName);
                        op.addTimeConstraint(constraint, currentServer, boolVar);
                    }
                    CplexObject.addConstraint(constraint);
                }
            } catch (IloException e) {
                e.printStackTrace();
            }
        }
    }

    private void knownTimeInequalities() {
        for (Server currentServer : relevantServers) {
            for (int t : serverOutTimes.get(currentServer)) {
                //t >= FIFO(t)
                orderedTimes.add(new TimePair(t, fifoOrder.get(t).get(currentServer)));
                //t >= SC(t)
                orderedTimes.add(new TimePair(t, scOrder.get(t).get(currentServer)));
                //FIFO(t) >= SC(t)
                orderedTimes.add(new TimePair(fifoOrder.get(t).get(currentServer), scOrder.get(t).get(currentServer)));
            }
            List<Integer> outTimes = new ArrayList<>(serverOutTimes.get(currentServer));
            for (int i = 0; i < outTimes.size() - 1; i++) {
                for (int j = i + 1; j < outTimes.size(); j++) {
                    int t = outTimes.get(i);
                    int tPrime = outTimes.get(j);
                    if (!orderedTimes.contains(new TimePair(t, tPrime))) {
                        continue;
                    }
                    //orderedTimes.add(new TimePair(t, fifoOrder.get(tPrime).get(currentServer)));
                    //FIFO(t) >= FIFO(tPrime)
                    orderedTimes.add(new TimePair(fifoOrder.get(t).get(currentServer), fifoOrder.get(tPrime).get(currentServer)));
                    //orderedTimes.add(new TimePair(t, scOrder.get(tPrime).get(currentServer)));
                    //SC(t) >= SC(tPrime)
                    orderedTimes.add(new TimePair(scOrder.get(t).get(currentServer), scOrder.get(tPrime).get(currentServer)));
                    //FIFO(t) >= SC(tPrime) -> transitivity
                    //orderedTimes.add(new TimePair(fifoOrder.get(t).get(currentServer), scOrder.get(tPrime).get(currentServer)));
                }
            }
        }
    }

    //Algorithm 1
    private void computeInequalitiesIn(Server currentServer) {
        //Line 2
        for (TimePair timePair : orderedTimes ) {
            if (serverInTimes.get(currentServer).contains(timePair.left) && serverInTimes.get(currentServer).contains(timePair.right)) {
                InequalitiesIn.get(currentServer).add(timePair);
            }
        }
        //Ordered times
        for(TimePair timePair : InequalitiesOut.get(currentServer)){
            TimePair fifo = new TimePair(fifoOrder.get(timePair.left).get(currentServer), fifoOrder.get(timePair.right).get(currentServer));
            TimePair sc = new TimePair(scOrder.get(timePair.left).get(currentServer), scOrder.get(timePair.right).get(currentServer));
            TimePair fifoSc = new TimePair(fifoOrder.get(timePair.left).get(currentServer), scOrder.get(timePair.right).get(currentServer));

            InequalitiesIn.get(currentServer).add(fifo);
            InequalitiesIn.get(currentServer).add(sc);
            InequalitiesIn.get(currentServer).add(fifoSc);
        }

        //Known bs
        for(TimePair timePair : InequalitiesOutBin.get(currentServer)){
            IloNumVar b = binaryVariables.get(timePair);

            TimePair fifo = new TimePair(fifoOrder.get(timePair.left).get(currentServer), fifoOrder.get(timePair.right).get(currentServer));
            TimePair sc = new TimePair(scOrder.get(timePair.left).get(currentServer), scOrder.get(timePair.right).get(currentServer));
            TimePair fifoSc = new TimePair(fifoOrder.get(timePair.left).get(currentServer), scOrder.get(timePair.right).get(currentServer));

            InequalitiesInBin.get(currentServer).add(fifo);
            binaryVariables.put(fifo, b);

            InequalitiesInBin.get(currentServer).add(sc);
            binaryVariables.put(sc, b);

            try {
                IloNumVar bPrime = cplex.boolVar();
                if (test) {
                    String variableName = "b" + fifoSc.left + "_" + fifoSc.right;
                    bPrime.setName(variableName);
                    op.binaries.add(bPrime);
                }
                InequalitiesInBin.get(currentServer).add(fifoSc);
                binaryVariables.put(fifoSc, bPrime);
            } catch (IloException e) {
                e.printStackTrace();
            }
        }

        //Line 8
        for(TimePair timePair : SetUtils.getUnion(InequalitiesOut.get(currentServer), InequalitiesOutBin.get(currentServer))){
            TimePair scFifo = new TimePair(scOrder.get(timePair.left).get(currentServer), fifoOrder.get(timePair.right).get(currentServer));
            try {
                IloNumVar bPrimePrime = cplex.boolVar();
                if (test) {
                    String variableName = "b" + scFifo.left + "_" + scFifo.right;
                    bPrimePrime.setName(variableName);
                    op.binaries.add(bPrimePrime);
                }
                InequalitiesInBin.get(currentServer).add(scFifo);
                binaryVariables.put(scFifo, bPrimePrime);
            } catch (IloException e) {
                e.printStackTrace();
            }
        }
    }

    //Algorithm 2
    private void computeInequalitiesOut(Server currentServer) {
        //Line 2
        for (TimePair timePair : orderedTimes) {
            if (serverOutTimes.get(currentServer).contains(timePair.left) && serverOutTimes.get(currentServer).contains(timePair.right)) {
                InequalitiesOut.get(currentServer).add(timePair);
            }
        }

        Server[] successors = server_graph.getSuccessors(currentServer).toArray(new Server[0]);

        for (Server succ : successors) {
            InequalitiesOut.get(currentServer).addAll(InequalitiesIn.get(succ));
            InequalitiesOutBin.get(currentServer).addAll(InequalitiesInBin.get(succ));
        }
        //Line 3
        for (int l = 0; l < successors.length - 1; l++) {
            for (int m = l + 1; m < successors.length; m++) {
                HashSet<Integer> ts = new HashSet<>();
                for (TimePair timePair : SetUtils.getUnion(InequalitiesIn.get(successors[l]), InequalitiesInBin.get(successors[l]))) {
                    ts.add(timePair.left);
                    ts.add(timePair.right);
                }
                HashSet<Integer> tPrimes = new HashSet<>();
                for (TimePair timePair : SetUtils.getUnion(InequalitiesIn.get(successors[m]), InequalitiesInBin.get(successors[m]))) {
                    tPrimes.add(timePair.left);
                    tPrimes.add(timePair.right);
                }

                for (int t : ts) {
                    for (int tPrime : tPrimes) {
                        if (!orderedTimes.contains(new TimePair(t, tPrime)) && !orderedTimes.contains(new TimePair(tPrime, t))) {
                            try {
                                TimePair timePair = new TimePair(t, tPrime);
                                IloNumVar b = cplex.boolVar();
                                if (test) {
                                    String variableName = "b" + t + "_" + tPrime;
                                    b.setName(variableName);
                                    op.binaries.add(b);
                                }
                                InequalitiesOutBin.get(currentServer).add(timePair);
                                binaryVariables.put(timePair, b);
                                //binaryVariables.put(new TimePair(tPrime, t), b);

                            } catch (IloException e) {
                                e.printStackTrace();
                            }
                        }
                        else{
                            if(orderedTimes.contains(new TimePair(t, tPrime))){
                                InequalitiesOut.get(currentServer).add(new TimePair(t, tPrime));
                            } else{
                                InequalitiesOut.get(currentServer).add(new TimePair(tPrime, t));
                            }
                        }
                    }
                }
            }
        }
    }

    private int findFirstFifoTime(Server currentServer, int lastTime) {
        if (currentServer.equals(emptyServer)) {
            return lastTime;
        }
        Server precedingServer;

        int timeFound = 0;
        try {
            precedingServer = foi.getPrecedingServer(currentServer);
        } catch (Exception e) {
            precedingServer = emptyServer;
        }
        timeFound = findFirstFifoTime(precedingServer, fifoOrder.get(lastTime).get(currentServer));
        if (timeFound != 0) {
            return timeFound;
        }
        return 0;
    }

    /**
     * Computes an expression for each segment of the given Curve
     * OBS: Since this is an internal help method, no check is made regarding the order of the times
     *
     * @param curve either a Service Curve or an Arrival Curve
     * @param gt    is a later time instance
     * @param lt    is a previous time instance
     * @return
     * @throws IloException
     */
    //
    private List<IloNumExpr> curveToExpression(Curve curve, IloNumVar gt, IloNumVar lt) throws IloException {
        LinkedList<IloNumExpr> segmentsExp = new LinkedList<>();
        if (curve instanceof ServiceCurve) {
            for (int i = 0; i < curve.getSegmentCount(); i++) {
                IloNumExpr timeDiff = cplex.diff(gt, lt);
                IloNumExpr latencyDiff = cplex.diff(timeDiff, curve.getSegment(i).getX().doubleValue());
                IloNumExpr prodRate = cplex.prod(latencyDiff, curve.getSegment(i).getGrad().doubleValue());
                segmentsExp.add(prodRate);
            }
        } else {
            for (int i = 0; i < curve.getSegmentCount(); i++) {
                IloNumExpr timeDiff = cplex.diff(gt, lt);
                IloNumExpr shiftDiff = cplex.diff(timeDiff, curve.getSegment(i).getX().doubleValue());
                IloNumExpr prodRate = cplex.prod(shiftDiff, curve.getSegment(i).getGrad().doubleValue());
                IloNumExpr addBurst = cplex.sum(prodRate, curve.getSegment(i).getY().doubleValue());
                segmentsExp.add(addBurst);
            }
            if (curve.getSegmentCount() > 1)
                segmentsExp.removeFirst();
        }

        return segmentsExp;
    }

    private void monotonicityIn() {
        for (Server currentServer : relevantServers) {
            Set<TimePair> timePairs = computeTimeSet(currentServer, constraintType.MONOIN);

            for (TimePair timePair : timePairs) {
                for (Flow flow : server_graph.getFlows(currentServer)) {
                    try {
                        IloConstraint constraint = cplex.ge(F.get(currentServer.getId(), flow.getId(), timePair.left), F.get(currentServer.getId(), flow.getId(), timePair.right));
                        if (test) {
                            String constraintName = "MonotonicityIn_" + currentServer.getAlias();
                            constraint.setName(constraintName);
                            op.addMonoInConstraint(constraint, currentServer);
                        }
                        CplexObject.addConstraint(constraint);
                    } catch (IloException e) {
                        e.printStackTrace();
                    }
                }
            }
        }
    }

    private void monotonicityOut() {
        for (Server currentServer : relevantServers) {
            Set<TimePair> timePairs = computeTimeSet(currentServer, constraintType.MONOOUT);

            for (TimePair timePair : timePairs) {
                for (Flow flow : server_graph.getFlows(currentServer)) {
                    Server succServer = successorServer(currentServer, flow);
                    try {
                        IloConstraint constraint = cplex.ge(F.get(succServer.getId(), flow.getId(), timePair.left), F.get(succServer.getId(), flow.getId(), timePair.right));
                        if (test) {
                            String constraintName = "MonotonicityOut_" + currentServer.getAlias();
                            constraint.setName(constraintName);
                            op.addMonoOutConstraint(constraint, currentServer);
                        }
                        CplexObject.addConstraint(constraint);
                    } catch (IloException e) {
                        e.printStackTrace();
                    }
                }
            }
        }
    }

    private void monotonicityInBin() {
        for (Server currentServer : relevantServers) {
            Set<TimePair> timePairs = computeTimeSetBin(currentServer, constraintType.MONOIN);

            for (TimePair timePair : timePairs) {
                for (Flow flow : server_graph.getFlows(currentServer)) {
                    try {
                        IloNumVar boolVar = binaryVariables.get(timePair);
                        IloConstraint constraint = cplex.ifThen(cplex.eq(boolVar, 1), cplex.ge(F.get(currentServer.getId(), flow.getId(), timePair.left), F.get(currentServer.getId(), flow.getId(), timePair.right)));
                        if (test) {
                            String constraintName = "MonotonicityInBin_" + currentServer.getAlias() + "_1";
                            constraint.setName(constraintName);
                            op.addMonoInConstraint(constraint, currentServer, boolVar);
                        }
                        CplexObject.addConstraint(constraint);

                        constraint = cplex.ifThen(cplex.le(boolVar, 0), cplex.le(F.get(currentServer.getId(), flow.getId(), timePair.left), F.get(currentServer.getId(), flow.getId(), timePair.right)));
                        if (test) {
                            String constraintName = "MonotonicityInBin_" + currentServer.getAlias() + "_0";
                            constraint.setName(constraintName);
                            op.addMonoInConstraint(constraint, currentServer, boolVar);
                        }
                        CplexObject.addConstraint(constraint);
                    } catch (IloException e) {
                        e.printStackTrace();
                    }
                }
            }
        }
    }

    private void monotonicityOutBin() {
        for (Server currentServer : relevantServers) {
            Set<TimePair> timePairs = computeTimeSetBin(currentServer, constraintType.MONOOUT);

            for (TimePair timePair : timePairs) {
                for (Flow flow : server_graph.getFlows(currentServer)) {
                    Server succServer = successorServer(currentServer, flow);
                    try {
                        IloNumVar boolVar = binaryVariables.get(timePair);
                        IloConstraint constraint = cplex.ifThen(cplex.eq(boolVar, 1), cplex.ge(F.get(succServer.getId(), flow.getId(), timePair.left), F.get(succServer.getId(), flow.getId(), timePair.right)));
                        if (test) {
                            String constraintName = "MonotonicityOutBin_" + currentServer.getAlias() + "_1";
                            constraint.setName(constraintName);
                            op.addMonoOutConstraint(constraint, currentServer, boolVar);
                        }
                        CplexObject.addConstraint(constraint);
                        constraint = cplex.ifThen(cplex.le(boolVar, 0), cplex.le(F.get(succServer.getId(), flow.getId(), timePair.left), F.get(succServer.getId(), flow.getId(), timePair.right)));
                        if (test) {
                            String constraintName = "MonotonicityOutBin_" + currentServer.getAlias() + "_0";
                            constraint.setName(constraintName);
                            op.addMonoOutConstraint(constraint, currentServer, boolVar);
                        }
                        CplexObject.addConstraint(constraint);
                    } catch (IloException e) {
                        e.printStackTrace();
                    }
                }
            }
        }
    }

    private void addMonotonicityConstraints() {
        monotonicityIn();
        monotonicityOut();
    }

    private void addBinaryMonotonicityConstraints() {
        monotonicityInBin();
        monotonicityOutBin();
    }

    private void addObjective() {
        try {
            //The objective is a maximization of the delay, which is the interval t1 - FIFO(tn), where tn is the moment
            //in time when the packet of the foi comes out of the first server in its path
            IloNumExpr objective = cplex.diff(times.getFirst(), times.get(findFirstFifoTime(foi.getSink(), 1) - 1));
            cplex.addMaximize(objective);
        } catch (IloException e) {
            e.printStackTrace();
        }
    }

    public boolean solve(boolean printCplexOutput) {
        try {
            if (!printCplexOutput) {
                cplex.setOut(null);
            } else {
                cplex.setOut(System.out);
            }
            long start = System.nanoTime();
            cplex.solve();
            long end = System.nanoTime();
            timeToSolve = (end - start) / 1e9;

        } catch (IloException e) {
            e.printStackTrace();
        }
        return false;
    }

    public boolean solve() {
        return solve(false);
    }

    public Num getDelay() {
        try {
            if (!cplex.getStatus().equals(IloCplex.Status.Optimal)) {
                System.out.println("Warning: " + cplex.getStatus());
            }
            return Num.getFactory(Calculator.getInstance().getNumBackend()).create(cplex.getObjValue());
        } catch (IloException e) {
            return Num.getFactory(Calculator.getInstance().getNumBackend()).createPositiveInfinity();
        }
    }

    @Override
    public void setPath(String path) {
        //unused
    }

    public double getSolveTime() {
        return timeToSolve;
    }

    public double getModelTime() {
        return timeToModel;
    }

    public HashSet<String> getFVarsValues() {
        HashSet<String> values = new HashSet<>();
        for (int t : serverOutTimes.get(foi.getSink())) {
            try {
                values.add("F(" + 1 + ")(" + foi.getId() + ")(" + t + ") = " + cplex.getValue(F.get(1, foi.getId(), t)));
            } catch (IloException e) {
                e.printStackTrace();
            }
        }
        return values;
    }

    public void printSlackVariables(){
        for(IloConstraint constraint : op.getConstraints()){
            if(constraint instanceof IloRange) {
                try {
                    System.out.println(constraint.getName() + ": " + this.cplex.getSlack((IloRange) constraint));
                } catch (IloException e) {
                    e.printStackTrace();
                }
            }
        }
    }
    public void printUnbounded(){
        IloConstraint[] constraints = op.getConstraints().toArray(new IloConstraint[0]);
        try {
            IloLinearNumExprIterator iterator = cplex.getRay().linearIterator();
            System.out.println("Unbounded Variables:");
            while(iterator.hasNext()){
                System.out.println(iterator.nextNumVar());
            }
        } catch (IloException e) {
            System.out.println(e.getMessage());
        }

        try {
            IloCplex.ConflictStatus[] status;
            status = cplex.getConflict(constraints);
            for(IloCplex.ConflictStatus st : status){
                System.out.println(st);
            }
        } catch (IloException e) {
            System.out.println(e.getMessage());
        }

        try {
            double[] doubles = new double[op.getConstraints().size()];
            for(int i = 0; i < doubles.length; i++){
                doubles[i] = 0.0;
            }
            System.out.println(cplex.getStatus());
            cplex.feasOpt(constraints, doubles);
            //get values
            cplex.getInfeasibilities(constraints);
            System.out.println(cplex.getStatus());
            cplex.solve();
            System.out.println(cplex.getObjValue());
        } catch (IloException e) {
            System.out.println(e.getMessage());
        }


    }

    public void clearModel() {
        try {
            //cplex.clearModel();
            cplex.endModel();
            clearObjects();
        } catch (IloException e) {
            e.printStackTrace();
        }
    }

    private void clearObjects() {
        clearObjects(server_graph);
    }

    //Useful to clear some memory
    private void clearUpperObjects() {
        this.serverOutTimes = null;
        this.serverInTimes = null;
        this.fifoOrder = null;
        this.scOrder = null;
        this.orderedTimes = null;
        this.serverLabel = null;
    }

    private void clearObjects(ServerGraph server_graph) {
        this.server_graph = server_graph;
        this.F = new Function();
        this.times = new LinkedList<>();
        this.serverOutTimes = new HashMap<>();
        this.serverInTimes = new HashMap<>();
        this.fifoOrder = new HashMap<>();
        this.scOrder = new HashMap<>();
        this.emptyServer = Server.createExplicitSourceServer();
        this.orderedTimes = new LinkedList<>();
        this.relevantServers = new LinkedList<>();
        this.serverLabel = new HashMap<>();
        this.timeToSolve = 0;
        this.timeToModel = 0;
        this.timeNum = 1;
        this.InequalitiesOut = new HashMap<>();
        this.InequalitiesOutBin = new HashMap<>();
        this.InequalitiesIn = new HashMap<>();
        this.InequalitiesInBin = new HashMap<>();
        this.op = new OptimizationProblem();
        initializeMaps();
        this.cplex = CplexObject.getInstance();
    }

    public IloCplex.Status getStatus() {
        try {
            return cplex.getStatus();
        } catch (IloException e) {
            e.printStackTrace();
        }
        return IloCplex.Status.Error;
    }

    public void setTimeLimit(double timeLimit) {
        try {
            cplex.setParam(IloCplex.Param.TimeLimit, timeLimit);
        } catch (IloException e) {
            e.printStackTrace();
        }
    }

    public void printVariableValues() {
        try {
            if (test) {
                System.out.println("Debug Mode On");
            }
            System.out.println("Binary variables:");
            for (IloNumVar bin : op.binaries) {
                System.out.println(bin.getName() + ": " + cplex.getValue(bin));

            }
            System.out.println("F variables:");
            for (Server server : foi.getPath().getServers()) {
                for (Flow flow : server_graph.getFlows()) {
                    for (int t = 0; t <= times.size(); t++) {
                        if (F.contains(server.getId(), flow.getId(), t)) {
                            IloNumVar f = F.get(server.getId(), flow.getId(), t);
                            System.out.println(f.getName() + ": " + cplex.getValue(f));
                        }
                    }
                }
            }
            Server server = emptyServer;
            for (Flow flow : server_graph.getFlows()) {
                for (int t = 0; t <= times.size(); t++) {
                    if (F.contains(server.getId(), flow.getId(), t)) {
                        IloNumVar f = F.get(server.getId(), flow.getId(), t);
                        System.out.println(f.getName() + ": " + cplex.getValue(f));
                    }
                }
            }

            System.out.println("Time variables:");
            for (int t = 0; t < times.size(); t++) {
                IloNumVar tVar = times.get(t);
                System.out.println(tVar.getName() + ": " + cplex.getValue(tVar));
            }
        } catch (IloException e) {
            e.printStackTrace();
        }
    }

    public Map<Server, Num> computeThetas(){
        Num factory = Num.getFactory(Calculator.getInstance().getNumBackend());
        HashMap<Server, Num> thetas = new HashMap<>();
        for(Server server : relevantServers){
            Num theta = factory.createZero();
            for(int t : serverInTimes.get(server)){
                for(Flow flow : server_graph.getFlows(server)){
                    try {
                        theta = theta.max(factory.create(cplex.getValue(F.get(server.getId(), flow.getId(), t))), theta);
                    } catch (IloException e) {
                        e.printStackTrace();
                    }
                }
            }
            theta = theta.div(theta, server.getServiceCurve().getUltAffineRate());
            if(thetas.containsKey(server)){
                thetas.put(server, theta.max(thetas.get(server), factory.add(theta, server.getServiceCurve().getLatency())));
            }else {
                thetas.put(server, factory.add(theta, server.getServiceCurve().getLatency()));
            }

        }
        return thetas;
    }

    public void sensitivityAnalysis(){
        try {
            cplex.exportModel("sens.lp");
        } catch (IloException e) {
            e.printStackTrace();
        }
    }

    public Map<Server, Set<Integer>> getServerOutTimes(){
        return serverOutTimes;
    }

    public Map<Server, Set<Integer>> getServerInTimes(){
        return serverInTimes;
    }

    private class TimePair {
        int left;
        int right;

        TimePair(int left, int right) {
            this.left = left;
            this.right = right;
        }

        public String toString() {
            return "(" + left + "," + right + ")";
        }

        @Override
        public boolean equals(Object obj) {
            if (obj instanceof TimePair) {
                return ((TimePair) obj).left == this.left && ((TimePair) obj).right == this.right;
            }
            return false;
        }

        @Override
        public int hashCode() {
            return this.left + this.right;
        }
    }

    /**
     * The idea of this class is to abstract an unknown function as a series of variables.
     * Instead of using a matrix, we use a sequence of maps - a matrix would be wasteful in this context
     */
    private class Function {
        private Map<Integer, Map<Integer, Map<Integer, IloNumVar>>> value = new HashMap<>();

        public boolean contains(int node, int flow, int t) {
            if (value.containsKey(node)) {
                if (value.get(node).containsKey(flow)) {
                    return value.get(node).get(flow).containsKey(t);
                }
            }
            return false;
        }

        public void put(int node, int flow, int t, IloNumVar f) {
            initialize(node, flow, t);
            value.get(node).get(flow).put(t, f);
        }

        public IloNumVar get(int node, int flow, int t) throws IloException {
            initialize(node, flow, t);
            if (value.get(node).get(flow).get(t) == null) {
                IloNumVar f = cplex.numVar(0.0, Double.MAX_VALUE, IloNumVarType.Float);
                if (test) {
                    String variableName;
                    if (node == -1) {
                        variableName = "F(e)(" + flow + ")(" + t + ")";
                    } else {
                        variableName = "F(" + node + ")(" + flow + ")(" + t + ")";
                    }
                    f.setName(variableName);
                    op.addFVariable(f);
                }
                value.get(node).get(flow).put(t, f);
            }
            return value.get(node).get(flow).get(t);
        }

        private void initialize(int node, int flow, int t) {
            boolean hasNode = false;
            boolean hasFlow = false;
            boolean hasT = false;
            if (value.containsKey(node)) {
                if (value.get(node).containsKey(flow)) {
                    if (value.get(node).get(flow).containsKey(t)) {
                        hasT = true;
                    }
                    hasFlow = true;
                }
                hasNode = true;
            }

            if (!hasNode)
                value.put(node, new HashMap<>());
            if (!hasFlow)
                value.get(node).put(flow, new HashMap<>());
            if (!hasT)
                value.get(node).get(flow).put(t, null);
        }

        private void clear() {
            value = new HashMap<>();
        }
    }

    public Map<Integer, Map<Integer, Map<Integer, Double>>> getFs() {
        Map<Integer, Map<Integer, Map<Integer, Double>>> values = new HashMap<>();
        for(int node : F.value.keySet()){
            values.put(node, new HashMap<>());
            for(int flow : F.value.get(node).keySet()){
                values.get(node).put(flow, new HashMap<>());
                for(int t : F.value.get(node).get(flow).keySet()){
                    try {
                        values.get(node).get(flow).put(t, cplex.getValue(F.get(node, flow, t)));
                    } catch (IloException e) {
                        e.printStackTrace();
                    }
                }
            }
        }
        return values;
    }
}
