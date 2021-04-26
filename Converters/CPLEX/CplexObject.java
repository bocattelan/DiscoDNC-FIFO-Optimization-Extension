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
import ilog.concert.IloException;
import ilog.cplex.IloCplex;

import java.util.Arrays;

public enum CplexObject {
    INSTANCE;
    private static IloCplex cplex;
    private static Integer bufferSize;
    private static IloConstraint[] constraintsBuffer;

    private static Integer index;

    static {
        try {
            cplex = new IloCplex();
            bufferSize = 1000;
            constraintsBuffer = new IloConstraint[bufferSize];
            index = 0;
            //Give focus on numerical precision, but can take longer to solve
            cplex.setParam(IloCplex.BooleanParam.NumericalEmphasis, true);

            //This means that we only accept optimal solutions
            //cplex.setParam(IloCplex.DoubleParam.EpGap, 0);

            //Activates data checking with modeling assistance
            //cplex.setParam(IloCplex.Param.Read.DataCheck, 2);

            //Print log at each node
            cplex.setParam(IloCplex.Param.MIP.Interval, 1);

            //Improve memory usage at runtime of solver
            //cplex.setParam(IloCplex.BooleanParam.MemoryEmphasis, true);
        } catch (IloException e) {
            e.printStackTrace();
        }
    }

    public static IloCplex getInstance(){
        return cplex;
    }

    public static void addConstraint(IloConstraint constraint){
        if(index < bufferSize){
            constraintsBuffer[index] = constraint;
            index++;
        } else {
            try {
                cplex.add(constraintsBuffer);
            } catch (IloException e) {
                e.printStackTrace();
            }
            constraintsBuffer[0] = constraint;
            index = 1;
        }
    }

    public static void addRemainingConstraints(){
        IloConstraint[] remainingConstraints = Arrays.copyOf(constraintsBuffer, index);
        try {
            cplex.add(remainingConstraints);
        } catch (IloException e) {
            e.printStackTrace();
        }
        index = 0;
    }


}
