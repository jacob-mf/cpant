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


/**
 * Demonstrates static routing, utilizing the cTopology class.
 * Also introduces AntHocNet and Cpant routing protocols
 */
class Routing : public cSimpleModule
{
  private:
    int myAddress;
    cPar *mySort; // MySort(defined by sort in omnet.ini) specifies sort of routing as 1 : Static , 2 : AntHocNet
    int maxHops;
    int redundant;

    // Tables
    typedef std::map<int,int> RoutingTable; // destaddr -> gateindex
    RoutingTable rtable;
    RoutingTable ntable; // Neighbors table; gateindex -> neighbor address
    typedef std::map<int,double> ProbTable; // gateindex -> probability
    typedef std::map<int,ProbTable> RPtable; // destaddr -> probability table of gateOutIndex
    RPtable ptable;

    // Signals
    simsignal_t dropSignal;
    simsignal_t outputIfSignal;

    // Local functions
    bool locateNeighbor(int dest , int & outgate);


  protected:
    virtual void initialize();
    virtual void handleMessage(cMessage *msg);
};

Define_Module(Routing);


void Routing::initialize()
{
    myAddress = getParentModule()->par("address");
    mySort =  &par("sort") ;
    dropSignal = registerSignal("drop");
    outputIfSignal = registerSignal("outputIf");

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
                sprintf(pkname,"pk-locate-neihgbor-node %d-from gate %d-#", myAddress,neighbourIndex );
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
    }
    else // Cpant routing
    {
        EV << "Cpant routing chosen, but still is not available" << endl;
        EV << "Node : " << thisNode->getModuleId() << " Address : " << myAddress <<  " Size : " << size() << endl;

    }
    delete topo;
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
        send(pk, "localOut");
        emit(outputIfSignal, -1); // -1: local
        if (mySort->longValue() == 2) // AntHocNet
        {

        }
        return;
    }
    if (mySort->longValue() == 2) // AntHocNet
    {

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
            delete pk;
            return;
        }
        int k = pk->getHopCount();
        if (k > 2){
            EV << "Last 3 transient nodes: " << pk->getTransientNodes(k-3)  << pk->getTransientNodes(k-2) << pk->getTransientNodes(k-1) << " Total : " << k << endl;
        }
        int originGateId = pk->getArrivalGateId();
        unsigned int origin = gate(originGateId)->getIndex();

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
          if (j != origin) { // skipping origin gate
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
                choices++;
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
            }
          }
          curl = false;
            EV << "Node: "<< myAddress << " Destination :" << destAddr  << " Origin : " << origin <<  " Choices: " << choices << " Gate: " << j << " Prob:" << ptable2[j] << ntable[j] << endl;
        }
        if (choices > 1) {
            choices--;
            int b = intuniform(0,choices); // in case of draw, select randomly the outgate
            EV << "Random chosen : " << b  << " Aux : " << aux[0] << aux[1] << aux[2] << aux[3] << endl;
            outGateIndex = aux[b];
            win = ptable2[aux[b]];
        } else if (choices == 0) {
            EV << "No choices to route, so discarding route & packet " << pk->getName() << endl;
            emit(dropSignal, (long)pk->getByteLength());
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
        delete pk;
        return;
    }

    int outGateIndex = (*it).second;

    EV << "forwarding packet " << pk->getName() << " on gate index " << outGateIndex << endl;
    pk->setHopCount(pk->getHopCount()+1);
    emit(outputIfSignal, outGateIndex);

    send(pk, "out", outGateIndex);
}
