#﻿This is a brief description of the *CPANT* project for *OMNeT++ 4.2.1*
## You can import the project to your workspace and run it as usual. Be aware to adjust your packet settings regarding on which directory is placed in.

## **CPANT** (*Colored Pheromones Ants*) is an algorithm based on **Ant Colony Optimization** by *M. Dorigo* and his *AntHocNet* routing protocol for *Ad hoc* networks.
File *'routing.cc'* is the most important one, about the routing protocols.
In file *'routing .ned'* you can select the routing protocol, by the parameter called **'sort'** as **1: static, 2: antHocNet, 3: cpant**  


1. Demonstrates *static shortest-path* routing. Routing tables are set up at the
beginning of the simulation using the *cTopology* class. The model is
intentionally kept simple to facilitate understanding.  


The network topology is the one widely known as the *"NTT backbone"*, and it was
contributed by *Mudassar Farooq*.  


Every node queries the topology of the network independently, using a *cTopology*
object. Then it computes shortest paths to every other node, and stores the
first nodes of the paths in a next-hop table. (Actually the table contains
the port number to the next-hop node not the node address itself -- the table
thus provides dest-address -> next-hop-address mapping). All the above takes
place once, at the beginning of this simulation. The topology is static during
the simulation, and so there's no need for the nodes to do anything to keep
the tables up-to-date. There's no routing protocol in the model.  


Once the routing tables are set up, nodes start sending packets at random
intervals. Every node gets a list of destination addresses in a parameter,
and for every packet it randomly chooses a destination from the list.  


2. Simple view of *AntHocNet* routing protocol,  similar to previous one but with extended *Routing Table*, added probability rate field as double.
Then initialize *Probability Routing Table* to *1/NumNodes* (probably, will suppress it)
Method *'calculateUnweightedSingleShortestPathsTo'* applies *Dijkstra* algorithm, but only save the shortest path, so a future modification is to save more paths in order to use stochastic selection (right now is not activated) 
  

3. *Cpant* is not available at the moment

---------------------------
  
  
There are two apps provided: *App* generates packets with exponential interarrival times, while *BurstyApp* alternates between active and idle periods. BurstyApp's implementation demonstrates the use of the FSMs (Finite State Machine). You can choose them in the *'omnet.ini'* file 
as **.appType= "App" or "BurstyApp"

Project is only tested in *'Mesh'* network
See files *'License'* and *'gpl'* for licesne details
#### 2012 *L.Jacob Mariscal Fernández* based on Routing sample  Copyright (C) 1992-2008 *Andras Varga*