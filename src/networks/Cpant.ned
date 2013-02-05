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
//
//
// 2012 L.Jacob Mariscal Fernández based on Copyright (C) 1992-2008 Andras Varga
//
// This file is distributed WITHOUT ANY WARRANTY. See the file
// `license' for details on this and other legal matters.
//

package cpant.networks;

import cpant.Node;

network Cpant
{
    parameters:
        double height @prompt("Number of rows") = default(3);
        double width @prompt("Number of columns") = default(4);
    types:
        channel C1 extends ned.DatarateChannel
        {
            parameters:
                //double jitter @unit(s) = default(uniform(1ms));
                @display("ls=green,2"); 
                delay = default(1ms);
                datarate = default(20Mbps);
        }
        channel C2 extends ned.DatarateChannel
        {
            parameters:
                //double jitter @unit(s) = default(uniform(8ms)); 
                @display("ls=yellow,2");
                delay = default(40ms);
                datarate = default(20Mbps);
        }
        channel C3 extends ned.DatarateChannel
        {
            parameters:
                //double jitter @unit(s) = default(uniform(16ms)); 
                @display("ls=orange,2");
                delay = default(1ms);
                datarate = default(8Mbps);
        }
        channel C4 extends ned.DatarateChannel
        {
            parameters:
                //double jitter @unit(s) = default(uniform(32ms)); 
                @display("ls=red,2");
                delay = default(80ms);
                datarate = default(4Mbps);
        }
    submodules:
        node[13]: Node { // (height*width)+1
            parameters:
                address = index+1;
                @display("p=40,40,m,4,40,40"); //@display("p=,,m,$width,40,40");
            gates:
                port[6];
        }
    connections allowunconnected:
    node[0].port[0] <-->  C1 <--> node[5].port[1];
    node[4].port[2] <-->  C3 <--> node[5].port[3];
    node[8].port[1] <-->  C3 <--> node[5].port[4];   
    node[2].port[4] <-->  C2 <--> node[5].port[5];
    node[5].port[2] <-->  C1 <--> node[6].port[3];
    node[5].port[0] <-->  C3 <--> node[9].port[1];
    node[9].port[2] <-->  C3 <--> node[10].port[3];
    node[9].port[5] <-->  C4 <--> node[6].port[4];
    
    node[6].port[5] <-->  C3 <--> node[3].port[4];
    node[2].port[2] <-->  C2 <--> node[3].port[3];
    node[1].port[3] <-->  C4 <--> node[2].port[3];
    node[2].port[0] <-->  C4 <--> node[7].port[1];
    node[7].port[5] <-->  C1 <--> node[3].port[0];
    node[6].port[2] <-->  C1 <--> node[7].port[3];
    node[6].port[0] <-->  C1 <--> node[10].port[1];
    node[10].port[2] <-->  C4 <--> node[11].port[3];
    node[11].port[1] <-->  C4 <--> node[7].port[0];    
}      