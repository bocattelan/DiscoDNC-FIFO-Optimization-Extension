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

import ilog.concert.IloConstraint;
import ilog.concert.IloNumVar;
import ilog.concert.IloObjective;
import org.networkcalculus.dnc.network.server_graph.Server;

import java.util.*;

public class OptimizationProblem {
    //Constraints
    Map<Server, List<IloConstraint>> ArrivalCurve;
    Map<Server, List<IloConstraint>> ArrivalCurveBin;
    Map<Server, List<IloConstraint>> ServiceCurve;
    Map<Server, List<IloConstraint>> MonoIn;
    Map<Server, List<IloConstraint>> MonoInBin;
    Map<Server, List<IloConstraint>> MonoOut;
    Map<Server, List<IloConstraint>> MonoOutBin;
    Map<Server, List<IloConstraint>> Fifo;
    Map<Server, List<IloConstraint>> Time;
    Map<Server, List<IloConstraint>> TimeBin;

    //Variables
    HashSet<IloNumVar> times;
    HashSet<IloNumVar> Fs;
    HashSet<IloNumVar> binaries;

    IloObjective objective;
    
    public OptimizationProblem(){
        ArrivalCurve = new HashMap<>();
        ArrivalCurveBin = new HashMap<>();
        ServiceCurve = new HashMap<>();
        MonoIn = new HashMap<>();
        MonoInBin = new HashMap<>();
        MonoOut = new HashMap<>();
        MonoOutBin = new HashMap<>();
        Fifo = new HashMap<>();
        Time = new HashMap<>();
        TimeBin = new HashMap<>();
        
        times = new HashSet<>();
        Fs = new HashSet<>();
        binaries = new HashSet<>();
    }
    
    public void addArrivalCurveConstraint(IloConstraint constraint, Server server){
        addElement(ArrivalCurve, constraint, server);
    }

    public void addArrivalCurveConstraint(IloConstraint constraint, Server server, IloNumVar binary){
        addElement(ArrivalCurveBin, constraint, server);
        binaries.add(binary);
    }

    public void addServiceCurveConstraint(IloConstraint constraint, Server server){
        addElement(ServiceCurve, constraint, server);
    }

    public void addMonoInConstraint(IloConstraint constraint, Server server){
        addElement(MonoIn, constraint, server);
    }

    public void addMonoInConstraint(IloConstraint constraint, Server server, IloNumVar binary){
        addElement(MonoInBin, constraint, server);
        binaries.add(binary);
    }

    public void addMonoOutConstraint(IloConstraint constraint, Server server){
        addElement(MonoOut, constraint, server);
    }

    public void addMonoOutConstraint(IloConstraint constraint, Server server, IloNumVar binary){
        addElement(MonoOutBin, constraint, server);
        binaries.add(binary);
    }

    public void addTimeConstraint(IloConstraint constraint, Server server){
        addElement(Time, constraint, server);
    }

    public void addTimeConstraint(IloConstraint constraint, Server server, IloNumVar binary){
        addElement(TimeBin, constraint, server);
        binaries.add(binary);
    }

    public void addFifoConstraint(IloConstraint constraint, Server server){
        addElement(Fifo, constraint, server);
    }

    public void addTimeVariable(IloNumVar t){
        times.add(t);
    }

    public void addFVariable(IloNumVar F){
        Fs.add(F);
    }

    public void addObjective(IloObjective objective){
        this.objective = objective;
    }

    private void addElement(Map<Server, List<IloConstraint>> map, IloConstraint constraint, Server server){
        if(map.containsKey(server)){
            map.get(server).add(constraint);
        }
        else{
            map.put(server, new LinkedList<>());
            map.get(server).add(constraint);
        }
    }

    public LinkedList<IloConstraint> getConstraints(Server server){
        LinkedList<IloConstraint> constraints = new LinkedList<>();

        if(Time.containsKey(server))
            constraints.addAll(Time.get(server));
        if(TimeBin.containsKey(server))
            constraints.addAll(TimeBin.get(server));
        if(ServiceCurve.containsKey(server))
            constraints.addAll(ServiceCurve.get(server));
        if(Fifo.containsKey(server))
            constraints.addAll(Fifo.get(server));
        if(MonoIn.containsKey(server))
            constraints.addAll(MonoIn.get(server));
        if(MonoInBin.containsKey(server))
            constraints.addAll(MonoInBin.get(server));
        if(MonoOut.containsKey(server))
            constraints.addAll(MonoOut.get(server));
        if(MonoOutBin.containsKey(server))
            constraints.addAll(MonoOutBin.get(server));
        if(ArrivalCurve.containsKey(server))
            constraints.addAll(ArrivalCurve.get(server));
        if(ArrivalCurveBin.containsKey(server))
            constraints.addAll(ArrivalCurveBin.get(server));

        return constraints;
    }

    public LinkedList<IloConstraint> getConstraints(){
        LinkedList<IloConstraint> constraints = new LinkedList<>();

        for(Server server : Time.keySet()){
            constraints.addAll(getConstraints(server));
        }

        return constraints;
    }
}
