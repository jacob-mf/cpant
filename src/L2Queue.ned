//
// This file is part of an OMNeT++/OMNEST simulation example.
//
// 2012 L.Jacob Mariscal Fernández based on Copyright (C) 1992-2008 Andras Varga
//
// This file is distributed WITHOUT ANY WARRANTY. 
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 
package cpant;

//
// Represents the network interface for a point-to-point connection.
//
// Packets arriving from the upper layer ("in" gate) are sent out on
// the "line" gate, which is expected to be connected to a link with
// nonzero data rate. Packets that arrive while a previous packet is
// being transmitted are queued up. The maximum queue size in packets
// can be specified in a parameter. Excess frames are simply discarded
// and recorded as statistics.
//
// The model can be easily extended in several ways: to make it possible to
// query the queue length from another module via a direct method call
// interface, to implement priority queueing, or to collect link statistics
// (utilization, etc.)
//
simple L2Queue
{
    parameters:
        int frameCapacity = default(0); // max number of packets; 0 means no limit
        @display("i=block/queue;q=queue");
        @signal[qlen](type="int");
        @signal[busy](type="bool");
        @signal[queueingTime](type="simtime_t");
        @signal[drop](type="int");
        @signal[txBytes](type="long");
        @signal[rxBytes](type="long");
        @statistic[qlen](title="queue length";record=vector?,timeavg,max;interpolationmode=sample-hold);
        @statistic[busy](title="server busy state";record=vector?,timeavg;interpolationmode=sample-hold);
        @statistic[queueingTime](title="queueing time at dequeue";unit=s;interpolationmode=none);
        @statistic[drop](title="dropped packet byte length";unit=bytes;record=vector?,count,sum;interpolationmode=none);
        @statistic[txBytes](title="transmitting packet byte length";unit=bytes;record=vector?,count,sum,histogram;interpolationmode=none);
        @statistic[rxBytes](title="received packet byte length";unit=bytes;record=vector?,count,sum,histogram;interpolationmode=none);
    gates:
        input in;
        output out;
        inout line;
}


