//
// This file is part of an OMNeT++/OMNEST simulation example.
//
// 2012 L.Jacob Mariscal Fernández based on Copyright (C) 1992-2008 Andras Varga
//
// This file is distributed WITHOUT ANY WARRANTY. See the file
// `license' for details on this and other legal matters.
//


//
// Represents a packet in the network.
//
packet Packet
{
    int srcAddr;
    int destAddr; // also contain neighbour in Hello msgs
    int hopCount; // also give number of transient nodes 
    simtime_t travelTime;
    int transientNodes[]; //std::vector<int> transientNodes;
}
// kind = 0 Data, 1 Hello_Msg, 2 Forward_Ant, 3 Backward_Ant
