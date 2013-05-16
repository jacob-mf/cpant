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
// This file is part of an OMNeT++/OMNEST simulation example.
//
// 2012 L.Jacob Mariscal Fernández based on Copyright (C) 1992-2008 Andras Varga
//
// This file is distributed WITHOUT ANY WARRANTY. See the file
// `license' for details on this and other legal matters.
//

package cpant.networks;

import cpant.Node;

network Anthoc
{
    parameters:
        double height @prompt("Number of rows") = default(9);
        double width @prompt("Number of columns") = default(7);
    types:
        channel C extends ned.DatarateChannel
        {
            parameters:
                delay = default(0.1ms);
                datarate = default(1Gbps);
        }
    submodules:
        node[height*width]: Node {
            parameters:
                address = index;
                @display("p=,,m,$width,40,40");
            gates:
                port[4];
        }
    connections allowunconnected:
        for i=1..height, for j=1..width-1 {
            node[j+((i-1)*width)].port[0] <--> C <--> node[(width+j)+((i-1)*width)].port[1] if i!=height;
            node[((i-1)*width)+j].port[2] <--> C <--> node[((i-1)*width)+j+1].port[3] if j!=width-1;    
        }
        for i=1..height {
            node[width*i].port[0] <--> C <--> node[(width*(i+1))].port[1] if i<height-1;
            node[width*i].port[2] <--> C <--> node[(width*i)+1].port[3] if i<height-1;
        }
        node[width*(height-1)].port[2] <--> C <--> node[1+(width*(height-1))].port[3]; // not need when more 
}


