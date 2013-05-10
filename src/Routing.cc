//
// This file is part of an OMNeT++/OMNEST simulation example.
//
// 2012 L.Jacob Mariscal Fernández based on Copyright (C) 1992-2008 Andras Varga
//
// This file is distributed WITHOUT ANY WARRANTY. See the file
// `license' for details on this and other legal matters.
//

#ifdef _MSC_VER
#pragma warning(disable:4786)
#endif

#include <map>
#include <omnetpp.h>
#include "Packet_m.h"
#include <iostream>
#include <fstream>


/**
 * Demonstrates static routing, utilizing the cTopology class.
 * Also introduces AntHocNet and Cpant routing protocols
 */
class Routing : public cSimpleModule
{
  private:
    int myAddress;
    cPar *mySort; // MySort(defined by sort in omnet.ini) specifies sort of routing as 1 : Static , 2 : AntHocNet
    cPar *coefPh; // Pheromon learning coefficient (defined in omnet.ini) should be on  0 < coefPh  < 1
    cPar *flooding; // specify if use flooding technique (boolean defined in omnet.ini)
    cPar *metrics;  // select either 1 :TimeToEndDelay 2: Hops 3: Lineal combination of previous metrics (defined in omnet.ini)
    int maxHops;
    int redundant;
    int baCounter; // counter of BA packets
    int faCounter; // counter of FA packets
    int nbCounter; // counter of Hello msg (detect neighbors issue)
    int visitor; // FA packets visitor counter to handle Visiting table
    // Tables
    typedef std::map<int,int> RoutingTable; // destaddr -> gateindex
    RoutingTable rtable;
    RoutingTable ntable; // Neighbors table; gateindex -> neighbor address
    typedef std::map<int,double> ProbTable; // gateindex -> probability
    typedef std::map<int,ProbTable> RPtable; // destaddr -> probability table of gateOutIndex
    typedef std::map<int,long int> VisitingTable; // position visitor -> FA id visitor
    RPtable ptable;
    VisitingTable vtable;

    // Signals
    simsignal_t dropSignal;
    simsignal_t outputIfSignal;

    // Local functions
    bool locateNeighbor(int dest , int & outgate);
    void updateRPTable (int dest, unsigned int outgate, double cost,int hops );
    int symmetricGate(int g);
    bool wasHereBefore(long int id);

  protected:
    virtual void initialize();
    virtual void handleMessage(cMessage *msg);
    virtual void finish();
};


Define_Module(Routing);


void Routing::initialize()
{
    myAddress = getParentModule()->par("address");
    mySort =  &par("sort") ;
    flooding = &par("flooding");
    metrics = &par("metrics");
    dropSignal = registerSignal("drop");
    outputIfSignal = registerSignal("outputIf");
    //baCounter=0;
    //nbCounter=0;
    coefPh= &par("coefPh"); // Pheromone update coefficient
    //
    // Brute force approach -- every node does topology discovery on its own,
    // and finds routes to all other nodes independently, at the beginning
    // of the simulation. This could be improved: (1) central routing database,
    // (2) on-demand route calculation
    //
    cTopology *topo = new cTopology("topo");
    std::vector<std::string> nedTypes;
    nedTypes.push_back(getParentModule()->getNedTypeName());
    topo->extractByNedTypeName(nedTypes);
    EV << "cTopology found " << topo->getNumNodes() << " nodes\n";
    maxHops = topo->getNumNodes() - 1; // other option:  maxHopsRate(~1.75) * NumNodes
    cTopology::Node *thisNode = topo->getNodeFor(getParentModule());
    if (  mySort->longValue() == 1 ) // static routing
    {
        // find and store next hops
        for (int i=0; i<topo->getNumNodes(); i++)
        {
            if (topo->getNode(i)==thisNode) continue; // skip ourselves
            topo->calculateUnweightedSingleShortestPathsTo(topo->getNode(i));

            if (thisNode->getNumPaths()==0)
                {
                EV << "Not connected , node: " << i << endl;
                continue; // not connected
                }

            cGate *parentModuleGate = thisNode->getPath(0)->getLocalGate();
            int gateIndex = parentModuleGate->getIndex();
            int address = topo->getNode(i)->getModule()->par("address");
            rtable[address] = gateIndex;
            EV << "  towards address " << address << " gateIndex is " << gateIndex << endl;
        }
    }
    else if (mySort->longValue() == 2) // AntHocNet routing
    {
        EV << "AntHocNet routing chosen, but still is in progress" << endl;
        // Locate Neighbors

        for (int j=0; j<thisNode->getNumOutLinks();j++)
        {
            cTopology::LinkOut *nlink = thisNode->getLinkOut(j);
            if (nlink->isEnabled()) {
                //enable = true;
                int neighbourIndex = nlink->getLocalGate()->getIndex();
                //EV << "Node: " << i << " OutGate :" << gateIndex <<  << endl;
                char pkname[40];
                sprintf(pkname,"pk-locate-neihgbor-node %d-from gate %d-#%d", myAddress,neighbourIndex,nbCounter++ );
                EV << "generating packet " << pkname << endl;
                Packet *pk = new Packet(pkname);
                //cPar *packetLengthBytes = &par("packetLength");
                //pk->setByteLength(packetLengthBytes->longValue());
                pk->setSrcAddr(myAddress);
                pk->setKind(1); // 1: hello msg
                pk->setDestAddr(myAddress);
                pk->setTransientNodesArraySize(1); // initialize to safe value
                send(pk,"out",neighbourIndex);
                                    }
         }

    // initialize Probability Routing Table to 1/NumNodes -> not needed

            EV << "Node : " << myAddress <<  " Size : " << size() << endl;
            bool enable, enable_dest = false;
            for (int i=0; i<topo->getNumNodes(); i++)
            {
                if (topo->getNode(i)==thisNode) continue; // skip ourselves
                //topo->calculateUnweightedSingleShortestPathsTo(topo->getNode(i));
                //if  (thisNode->isEnabled() ) { //(thisNode->getNumPaths()==0) {
                  //  EV << "Not connected , node: " << i << endl;
                   // continue; // not connected
                    //}
               for (int j=0; j<topo->getNode(i)->getNumOutLinks();j++) {
                   cTopology::LinkOut *link_dest = topo->getNode(i)->getLinkOut(j);
                   if (link_dest->isEnabled()) {
                       enable_dest = true;
                   }
               }
               if (!enable_dest) {
                  EV << "Not connected to destination node: " << i << endl;
                                      continue; // not connected
               }
                for (int j=0; j<thisNode->getNumOutLinks();j++)
                {
                    //int address = topo->getNode(i)->getModule()->par("address");
                    //probtable[address][j] = 1.0 / (topo->getNumNodes());
                    cTopology::LinkOut *link = thisNode->getLinkOut(j);
                    //int r = link->getRemoteGateId();
                    if (link->isEnabled()) {
                        enable = true;
                        int gateIndex = link->getLocalGate()->getIndex();
                        int address = topo->getNode(i)->getModule()->par("address");
                        ptable[address] [gateIndex]= 1.0 / (thisNode->getNumOutLinks());
                        EV << "Node: " << i << " OutGate :" << gateIndex << " Prob: " << ptable[address][gateIndex]  << " OutLinks: " << thisNode->getNumOutLinks() << endl;
                    }
                }
                if (!enable) {
                    EV << "Not connected outside, source node: " << i << endl;
                    continue; // not connected
                }
            }
            //int temp=3;
            //double test = ptable[myAddress-1] [temp] +  (double) 1/temp;
            //EV << "Double test: " <<  test << " temp⁻1: " << (double) 1/ temp << " Valor a anterior nodo en tercer puerto: "<< ptable[myAddress-1] [temp]<< endl;
    }
    else // Cpant routing
    {
        EV << "Cpant routing chosen, but still is not available" << endl;
        EV << "Node : " << thisNode->getModuleId() << " Address : " << myAddress <<  " Size : " << size() << endl;

    }
    delete topo;
}

int Routing:: symmetricGate(int g )
{
    int sym= g+1;
    if ((g== 0) || (g==2) || (g==4)) {
        EV << "sym: " << sym << endl;

    }else {
        sym = g-1;
        EV << "sym: " << sym << endl;
    }
    return sym;
}
bool Routing::locateNeighbor (int dest, int & outgate)
{
    for (unsigned int i=0; i < (ntable.size()); i++) {
         if ((dest == ntable[i]) && (ptable[dest][i] > 0)){
             EV << "Checking if destination address " << dest << " in table with size " << ntable.size() << " is neighbor "<< ntable[i] << " on gate : " << i << endl;
             //outgate = gate(i)->getIndex();
             outgate = i;
             return true;
          }
          EV << "Checking if destination address " << dest << " in table with size " << ntable.size() << " is neighbor "<< ntable[i] << " on gate : " << i << endl;
    }
    return false;
}
bool Routing::wasHereBefore(long int id)
{
    for (int i=0; i< visitor; i++){
        if (vtable[i] == id) return true;
    }
    return false;
}
void Routing::updateRPTable (int dest,unsigned int outgate, double cost,int hops )
{
    //double c = cost.dbl(); // use with simtime_t parameter
    // evaporation in other values
    for (unsigned int j=0; j< ntable.size();j++){
        if (j != outgate) {
            ptable [dest] [j] = ptable [dest] [j] * coefPh->doubleValue();
        }
    }

    if (metrics->longValue() ==1) // Travel time
    ptable [dest] [outgate] = ptable [dest] [outgate] * coefPh->doubleValue() + ((1/(cost * 10000) )* (1 - coefPh->doubleValue())); // use cost as Travel time; added 10⁻^3 to normalize (unitary)
    else if (metrics->longValue() == 2) // Hops
        if (hops > 1 )  ptable [dest] [outgate] = ptable [dest] [outgate] * coefPh->doubleValue() + (double) 1/hops; // use cost as Hops, makes hops more accuracy for paths
        else ptable [dest] [outgate] = ptable [dest] [outgate] * coefPh->doubleValue() + ((1/(hops) )* (1 - coefPh->doubleValue())); // use cost as Hops;
    else  // Linear combination of Travel time and hops
        ptable [dest] [outgate] = ptable [dest] [outgate] * coefPh->doubleValue() + 0.5*( (((1/(hops) )* (1 - coefPh->doubleValue())) + ((1/(cost * 10000) )* (1 - coefPh->doubleValue())))); // use lineal combination ; added 10⁻^3 to normalize (unitary)
    EV << "Pheromone Table Updated to value: "<< ptable [dest] [outgate] << " Dest: " << dest << " Gate "<< outgate << "Hops "<< hops << endl;
}

void Routing::handleMessage(cMessage *msg)
{
    Packet *pk = check_and_cast<Packet *>(msg);
    if (pk->getKind() == 1) { // hello msg -> neighbor purpose
        if (pk->getDestAddr() == pk->getSrcAddr() ) {
            // hello msg, locate neighbor
            pk->setSrcAddr(myAddress);
            pk->setHopCount(pk->getHopCount()+1);
            int originGateId = pk->getArrivalGateId();
            int origin = gate(originGateId)->getIndex();
            ntable[origin] = pk->getSrcAddr(); // update with the origin neighbor acknowledge
            EV << "forwarding seeking-neighbor packet " << pk->getName() << " on gate index " << origin << " (coming home) " << endl;
            emit(outputIfSignal, origin);
            send(pk, "out", origin);
            return;
        } else {
            // update neighbor table
            int originGateId = pk->getArrivalGateId();
            int origin = gate(originGateId)->getIndex();
            //int symGateId = pk->getSenderGateId();
            if (ntable[origin] ==  pk->getSrcAddr()) { // neighbor table already updated
                EV << "Neighbor table is already up to date, discarding packet:  "<< pk->getName() << endl;
                EV << "Redundant packets : "<< redundant++ << endl;
                emit(dropSignal, (long)pk->getByteLength());
                delete pk;
                return;
            }
            ntable [origin] = pk->getSrcAddr();
            EV <<"Neighbor " << ntable[origin] << " on node "<< myAddress << "on gate " << origin << " #" << originGateId << endl;
            EV << "local delivery of packet " << pk->getName() << endl;
            pk->setTravelTime(pk->getArrivalTime() - pk->getCreationTime());
            send(pk, "localOut");
            emit(outputIfSignal, -1); // -1: local
            return;
        }
    }
    int destAddr = pk->getDestAddr();
    if (destAddr == myAddress)   // packet arrived desired destination
    {
        EV << "local delivery of packet " << pk->getName() << endl;
        pk->setTravelTime(pk->getArrivalTime() - pk->getCreationTime());
        EV << "In (time calculated in Routing) : " << pk->getTravelTime() << endl;
        //EV << "Inside arrival branch, before send LocalOut" << endl;
        unsigned int originGateId = pk->getArrivalGateId();
        unsigned int senderGateId = pk->getSenderGateId();
        EV << "originGateIds : " << originGateId << " senderGateId: " << senderGateId << endl;
        send(pk, "localOut");
        emit(outputIfSignal, -1); // -1: local
        if (pk->getSrcAddr() == destAddr) return; // same destination and source, no need to check AntHoc features
        else{
        if (mySort->longValue() == 2) // AntHocNet
        {

            if (pk->getKind() == 2) { // FA arrived -> BA created & launched back
                //EV << "Inside AntHocNet-FA arrived branch" << endl;
                //unsigned int originGateId = pk->getArrivalGateId();
                //unsigned int senderGateId = pk->getSenderGateId();
                //EV << "originGateIds : " << originGateId << senderGateId << endl;
                //unsigned int sender = gate(senderGateId)->getIndex();
                //EV << "sender : " << sender << endl;
                //cGate * originGate = pk->getArrivalGate();
                //int origin =0;
                //if (originGate->isConnectedOutside() ) {
                int origin = gate(originGateId)->getIndex();
                int symmetric = symmetricGate(origin);

                //int symmetric = gate(senderGateId);
                EV << "origin (1st): " << origin << " symmetric: " << symmetric << " Id. pk: "<< pk->getId() << endl;
                //}else {
                //        origin = pk->getTransientNodes(pk->getHopCount()-1);
                //        EV << "origin (good branch): " << origin << endl;
                //        int dest = origin;
                //        if  (locateNeighbor(dest,origin)) EV << "origin(gate) : " << origin << endl;
                //       }

                char pkname[40];
                sprintf(pkname,"BA-%d-to-%d-#%d", myAddress, pk->getSrcAddr(), baCounter++);
                EV << "generating Backward Ant " << pkname << endl;
                Packet *ba = new Packet(pkname);
                //ba->setByteLength(packetLengthBytes->longValue());
                ba->setSrcAddr(myAddress);
                ba->setKind(3); // BA packet
                ba->setDestAddr(pk->getSrcAddr()); // fatal error, its src address!

                int k = pk->getHopCount();
                ba->setTransientNodesArraySize(k); // initialize to safe value, store hops count
                ba->setHopCount(0);
                ba->setTravelTime(pk->getTravelTime());

                for (int i=0;i<k;i++) {
                    ba->setTransientNodes(i,pk->getTransientNodes(i));
                }
                EV << "forwarding Backward Ant packet " << ba->getName() << " on gate index " << origin << " (coming back) " << endl;
                // BA needs to update Routing table!! 2 times, bidirectional
                //updateRPTable(destAddr,origin,pk->getTravelTime().dbl(),ba->getTransientNodesArraySize()); // same node, dest, not neccessary
                updateRPTable(ba->getDestAddr(),origin,pk->getTravelTime().dbl(),ba->getTransientNodesArraySize());
                //delete pk;
                EV << "Control packets, neighbors: " << nbCounter << " FA: " << faCounter++ << " BA: "<< baCounter << endl; // increase FA when received
                emit(outputIfSignal, origin);
                send(ba,"out",origin);
            } else if (pk->getKind() == 3) { // BA arrived
                EV << "Inside AntHocNet-BA arrived branch" << endl;
                //int originGateId = pk->getArrivalGateId();
                //EV << "originGateId : " << originGateId << endl;
                unsigned int origin = gate(originGateId)->getIndex();
                int symmetric = symmetricGate(origin);
                EV << "origin : " << origin << " symmetric: " << symmetric << endl;
                // TOCheck update routing table ,symmetric
                //updateRPTable(destAddr,origin,simTime().dbl() - pk->getCreationTime().dbl(),pk->getTransientNodesArraySize()); // same dest and actual node, not neccessary
                updateRPTable(pk->getSrcAddr(),origin,simTime().dbl() - pk->getCreationTime().dbl(),pk->getTransientNodesArraySize());
                pk->setHopCount(pk->getHopCount()+1);
                emit(dropSignal, (long)pk->getByteLength());
                EV << "BA mission completed"<< endl;
                // delete pk;
            }
        }
        return;
    }   }
    if (mySort->longValue() == 2) // AntHocNet
    {
        if (pk->getKind() == 3) { // BA arrives
            int baGateIndex =0;
            pk->setHopCount(pk->getHopCount()+1);
            destAddr = pk->getTransientNodes(pk->getTransientNodesArraySize()-1 -pk->getHopCount());
            if (locateNeighbor(destAddr, baGateIndex)) { // &&(ptable2[outGateIndex] > 0)) { // check if destination is a neighbor node and possible way
                EV << "Destination detected in neighbor table, gate: " << baGateIndex << " destination: " << destAddr << endl;
                // TOCHECK update routing table
                int originGateId = pk->getArrivalGateId();
                //int senderGateId = pk->getSenderGateId();
                unsigned int origin = gate(originGateId)->getIndex();
                unsigned int symmetric = symmetricGate(origin);
                EV << "Origin gate id: "<< originGateId << " origin: "<< origin << " symmetric: " << symmetric << endl;
                updateRPTable(destAddr,baGateIndex,pk->getTravelTime().dbl(),1);
                updateRPTable(pk->getSrcAddr(),origin,(simTime()-pk->getCreationTime()).dbl(),pk->getHopCount());
                EV << "forwarding BA packet " << pk->getName() << " on gate index " << baGateIndex << " steps left: " << pk->getTransientNodesArraySize()-pk->getHopCount() << endl;
                //pk->setHopCount(pk->getHopCount()-1);
                emit(outputIfSignal, baGateIndex);
                send(pk, "out", baGateIndex);
                return;
            } else {
                EV << "Error detected when routing BA: " << pk->getName() << " when routing to " << destAddr << endl;
                emit(dropSignal, (long)pk->getByteLength());
                delete pk;
                return;
            }
        }
        RPtable::iterator it = ptable.find(destAddr);
            if (it==ptable.end())
            {
                EV << "address " << destAddr << " unreachable, discarding packet " << pk->getName() << endl;
                emit(dropSignal, (long)pk->getByteLength());
                delete pk;
                return;
            }

        if (pk->getHopCount() == maxHops)
        {
            EV << "Max hops = " << maxHops << " reached, discarding route & packet " << pk->getName() << endl;
            emit(dropSignal, (long)pk->getByteLength());
            if (pk->getKind() == 2) { // FA branch, counting
            EV << "Control packets, neighbors: " << nbCounter << " FA: " << faCounter++ << " BA: "<< baCounter << endl; // increase FA
            }
            delete pk;
            return;
        }
        int k = pk->getHopCount();
        if (k > 2){
            EV << "Last 3 transient nodes: " << pk->getTransientNodes(k-3)  << pk->getTransientNodes(k-2) << pk->getTransientNodes(k-1) << " Total : " << k << endl;
        }
        int originGateId = pk->getArrivalGateId();
        EV << "originGateId : " << originGateId << " Id pk: " << pk->getTreeId() << endl;
        if ((pk->getKind() == 2) && (flooding->boolValue())){ // FA arrives and active flooding
            //VisitingTable::iterator itv = vtable.find(pk->getTreeId());
                if ((visitor == 0) || (! wasHereBefore(pk->getTreeId())))
                {
                    vtable[visitor] = pk->getTreeId();
                    EV << "Updated  visiting table, position: "<< visitor << " FA id: " << vtable[visitor] << endl;
                                    visitor++;
                } else {
                    EV << "FA id: " <<  pk->getTreeId() << " already checked here by best route, discarding packet " << pk->getName() << endl;
                    emit(dropSignal, (long)pk->getByteLength());
                    EV << "Control packets, neighbors: " << nbCounter << " FA: " << faCounter++ << " BA: "<< baCounter << endl; // increase FA before deleted
                    delete pk;
                    return;
                }
                                 }
        unsigned int origin = gate(originGateId)->getIndex();
        EV << "origin : " << origin << endl;
        for (int i=0;i<k;i++) {
            if (pk->getTransientNodes(i) == myAddress) {
                EV << "Loop detected, in node: " << myAddress << endl;
                pk->setHopCount(k-2);
                // pk->setTransientNodesArraySize(k-2); // delete last transient
                EV << "forwarding packet " << pk->getName() << " on gate index " << origin << " (coming back) " << endl;
                emit(outputIfSignal, origin);
                send(pk, "out", origin);
                return;
            }
        }
        ProbTable ptable2 = (*it).second;
        // stochastic selection win = rand(0,1); for ants r
        // double win = uniform(0,1); // probability of the winner gate

        //EV << "Random : " << win << endl;

        double win = 0;
        int  outGateIndex = 0;
        int choices = 0;
        bool curl = false;
        if ((locateNeighbor(destAddr, outGateIndex)) && (ptable2[outGateIndex] > 0)) { // check if destination is a neighbor node and possible way
            EV << "Destination detected in neighbor table, gate: " << outGateIndex << endl;
        } else {
        RoutingTable aux; // choice -> gateIndex
        for (unsigned int j=0; j< ntable.size();j++)
        {
          if (((pk->getHopCount() == 0)) || (j != origin)) { // skipping origin gate
            if (win < ptable2[j] )
            {
                for (int i=0;i<k;i++) {
                            if (pk->getTransientNodes(i) == ntable[j]) {
                                curl = true; i = k; // loop detected
                                EV << "Curl skipped, node :  " << ntable[j] <<endl;
                            }
                }
                if (curl) {
                } else {
                win = ptable2[j];
                outGateIndex = j;
                aux[choices]= j;
                if (pk->getKind() == 2) { // FA , not only seek on best route way
                    choices++;
                            }
                else choices=1; // Data packets takes only the best route
                }
            }else if ((ptable2[j] > 0) && (win == ptable2[j])){
                for (int i=0;i<k;i++) {
                    if (pk->getTransientNodes(i) == ntable[j]) {
                    curl = true; i = k; // loop detected
                    EV << "Curl skipped, node :  " << ntable[j] <<endl;
                    }
                }
                if (curl) {
                } else {
                aux[choices]=j;
                choices++;      }
            }else if ((ptable2[j] > 0) && (pk->getKind() == 2)) { // FA include all no null possibilities
                for (int i=0;i<k;i++) {
                                    if (pk->getTransientNodes(i) == ntable[j]) {
                                    curl = true; i = k; // loop detected
                                    EV << "Curl skipped, node :  " << ntable[j] <<endl;
                                    }
                                }
                                if (curl) {
                                } else {
                                aux[choices]=j;
                                choices++;      }
            }
          }
          curl = false;
            EV << "Node: "<< myAddress << " Destination :" << destAddr  << " Origin : " << origin <<  " Choices: " << choices << " Gate: " << j << " Prob:" << ptable2[j] << " win: "<< win << " neighbour:"<< ntable[j] << endl;
        }
        if (choices > 1) {
            if ((pk->getKind() == 2) && (flooding->boolValue())) { // FA , implement flooding option
                pk->setTransientNodes(pk->getHopCount() ,myAddress);
                pk->setHopCount(pk->getHopCount()+1);
                for (int i=0; i < choices; i++) {
                    outGateIndex= aux[i];
                    win = ptable2[aux[i]];
                    EV << "Flooding! forwarding packet " << pk->getName() << " on gate index " << outGateIndex << " with prob: " << win << endl;
                    emit(outputIfSignal, outGateIndex);
                    if (i == 0)  send(pk,"out",outGateIndex);
                    else send(pk->dup(), "out", outGateIndex);
                }
                aux.clear();
                return;
            }
            choices--;
            int b = intuniform(0,choices); // in case of draw, select randomly the outgate
            EV << "Random chosen : " << b  << " Aux : " << aux[0] << aux[1] << aux[2] << aux[3] << endl;
            outGateIndex = aux[b];
            win = ptable2[aux[b]];
        } else if (choices == 0) {
            EV << "No choices to route, so discarding route & packet " << pk->getName() << endl;
            emit(dropSignal, (long)pk->getByteLength());
            if (pk->getKind() == 2) {
            EV << "Control packets, neighbors: " << nbCounter << " FA: " << faCounter++ << " BA: "<< baCounter << endl; // increase FA before deleting
            }
            delete pk;
            return;
        }
        aux.clear();
        }

        pk->setTransientNodes(pk->getHopCount() ,myAddress);
        EV << "forwarding packet " << pk->getName() << " on gate index " << outGateIndex << " with prob: " << win << endl;
        pk->setHopCount(pk->getHopCount()+1);
        emit(outputIfSignal, outGateIndex);
        send(pk, "out", outGateIndex);
        return;
    }
    RoutingTable::iterator it = rtable.find(destAddr);
    if (it==rtable.end())
    {
        EV << "address " << destAddr << " unreachable, discarding packet " << pk->getName() << endl;
        emit(dropSignal, (long)pk->getByteLength());
        if (pk->getKind() == 2) {
            EV << "Control packets, neighbors: " << nbCounter << " FA: " << faCounter++ << " BA: "<< baCounter << endl; // increase FA before deleting it
        }
        delete pk;
        return;
    }

    int outGateIndex = (*it).second;

    EV << "forwarding packet " << pk->getName() << " on gate index " << outGateIndex << endl;
    pk->setHopCount(pk->getHopCount()+1);
    emit(outputIfSignal, outGateIndex);

    send(pk, "out", outGateIndex);
}

void Routing::finish()
{
    //EV << "Simulation time: " << simTime()<< endl;
    //recordScalar("Simulation time: ", simTime());
    //EV << "Control packets: (nb, fa, ba) : " << nbCounter << faCounter << baCounter << endl;
    struct controlPkcounter{
        int nb;
        int fa, ba, sum;
    };
    controlPkcounter count;
    //cObject * punt = &count;
    count.nb = nbCounter; count.fa = faCounter; count.ba = baCounter; count.sum = count.nb + count.ba +count.fa;
    //snapshot(this,"Control packets ");
//using namespace std;

//int main () {
  std::ofstream myfile("control-packet.txt", myfile.app );
  if (myfile.is_open())
  {

    myfile << count.nb << "," << count.fa << "," << count.ba << "," << count.sum <<  "\n";
    myfile.close();
  }
  else { EV << "Unable to open file" << endl; }

}
