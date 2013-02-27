//
// This file is part of an OMNeT++/OMNEST simulation example.
//
// Julio 2012 L.Jacob Mariscal Fernández based on Andras Varga's file
//
// This file is distributed WITHOUT ANY WARRANTY. 
//
// This file is free software: you can redistribute it and/or modify
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
// Models a router.
//
simple Routing
{
    parameters:
        int sort; // Sort of routing 1: Static, 2: AntHocNet, 3: CPANT (Colored Pheromones Ant)
        double coefPh; // Update Pheromone Coefficient 
        @display("i=block/switch");
        @signal[drop](type="long");
        @signal[outputIf](type="int");
        @statistic[drop](title="dropped packet byte length";unit=bytes;record=vector?,count,sum;interpolationmode=none);
        @statistic[outputIf](title="output interface for each routed packet";record=histogram;interpolationmode=none);
    gates:
        input in[];
        output out[];
        input localIn;
        output localOut;
}


