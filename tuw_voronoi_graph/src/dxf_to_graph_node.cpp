/*
 * Copyright (c) 2017, <copyright holder> <email>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY <copyright holder> <email> ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <copyright holder> <email> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <tuw_voronoi_graph/dxf_to_graph_node.h>

int main(int argc, char** argv)
{

    ros::init(argc, argv, "dxf_graph_node");   
    ros::NodeHandle n;
    
    tuw_graph::DxfToGraphNode dxf2graph (n);
    
    std::cout << "hello1" << std::endl;
    dxf2graph.writeGraph();
    std::cout << "hello3" << std::endl;
    
    return 0;
}


namespace tuw_graph
{
    DxfToGraphNode::DxfToGraphNode(ros::NodeHandle &n) :
        n_(n),
        n_param_("~"),
        DxfToGraph()
    {
        dxfPath_ = "segments.dxf";
        n_param_.param("dxf_path", dxfPath_, dxfPath_);

        segmentLength_ = 1.0;   //meter
        n_param_.param("segment_length", segmentLength_, segmentLength_);

        graphPath_ = "graph";
        n_param_.param("dxf_path", graphPath_, graphPath_);
        
    }

    void DxfToGraphNode::writeGraph()
    {
        std::cout << "hello2" << std::endl;
        parseGraph(dxfPath_, segmentLength_);
        serializeGraph(graphPath_);
    }
}