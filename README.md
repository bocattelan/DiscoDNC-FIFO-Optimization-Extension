# [DiscoDNC](https://github.com/NetCal) FIFO Optimization Extension

This is an extension to the DiscoDNC Tool, which allows researchers to compute tight delay bounds using the ideas from [FIFO](https://ieeexplore.ieee.org/document/6861458). It makes use of the CPLEX java API for solving the problem.

## Installation

This extension should be located inside the DiscoDNC Tool, in "/dnc/src/main/java/org/networkcalculus/dnc/utils/Converters".

It requires the presence and configuration of CPLEX version 12.9.
I.e., the linkage of the jar file in "/opt/ibm/ILOG/CPLEX_Studio129/cplex/lib/cplex.jar" and the following command line argument for the java virtual machine: "-Djava.library.path=/opt/ibm/ILOG/CPLEX_Studio129/cplex/bin/x86-64_linux/". These are example paths, and can vary from installation to installation.

## Usage

```java
CplexConverter cplex = new CplexConverter () ;
cplex.createModel(serverGraph ,flowOfInterest);
cplex.solve () ;
System.out.println("CPLEX:" + cplex.getDelay()) ;
```
