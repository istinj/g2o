// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/stuff/sampler.h"
#include "g2o/stuff/command_args.h"
#include "g2o/core/factory.h"

using namespace std;
using namespace g2o;

int main (int argc, char** argv) {
  // command line parsing
  int nodes_in;
  int nodes_out;
  double radius_in;
  double radius_out;
  string filename;
  CommandArgs arg;
  arg.param("o", filename, "-", "output filename");
  arg.param("nodes_in", nodes_in, 20, "how many nodes in the inner radius");
  arg.param("nodes_out", nodes_out, 50, "how many nodes in the outer radius");
  arg.param("radius_in", radius_in, 5., "inner radius ");
  arg.param("radius_out", radius_out, 20., "outer_radius ");
  arg.parseArgs(argc, argv);

  double alpha_step_in=2*M_PI/nodes_in;
  double alpha_step_out=2*M_PI/(nodes_out*nodes_in); // we move along a spiral
  double alpha_in=0;
  double alpha_out=0;
  vector<VertexSE3*> vertices;
  vector<EdgeSE3*> edges;
  int id = 0;
  Eigen::Isometry3d robot;
  robot.setIdentity();

  // construct trajectory and odometry
  Eigen::Matrix<double, 6, 6> information=Eigen::Matrix<double, 6, 6>::Identity();
  VertexSE3* v_previous=0;
  Eigen::Isometry3d T_base=Eigen::Isometry3d::Identity();
  T_base.linear()=Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY()).toRotationMatrix();
  for (int n_out = 0; n_out < nodes_out; ++n_out){
    for (int n_in = 0; n_in < nodes_in; ++n_in, ++id) {
      Eigen::Isometry3d T_out;
      T_out.linear()=Eigen::AngleAxisd(alpha_out, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      T_out.translation().setZero();
      T_out.translation()=T_out*Eigen::Vector3d(0, radius_out, 0);

      Eigen::Isometry3d T_in;
      T_in.linear()=Eigen::AngleAxisd(alpha_in, Eigen::Vector3d::UnitX()).toRotationMatrix();
      T_in.translation().setZero();
      T_in.translation()=T_in*Eigen::Vector3d(0,radius_in,0);

      Eigen::Isometry3d T_total=T_out*T_in*T_base;

      VertexSE3* v = new VertexSE3;
      v->setEstimate(T_total);
      v->setId(id);
      vertices.push_back(v);
      if (v_previous) {
        EdgeSE3* e=new EdgeSE3;
        e->setVertex(0,v_previous);
        e->setVertex(1,v);
	e->setMeasurement(v_previous->estimate().inverse()*v->estimate());
        e->setInformation(information);
        edges.push_back(e);
      }
      alpha_in+=alpha_step_in;
      alpha_out+=alpha_step_out;
      v_previous=v;
    }
  }
  cerr << "v.size: " << vertices.size() << endl;
  // connect vertices between spiral rings
  int ivs=vertices.size();
  for (int i=0; i<(int) vertices.size(); ++i){
    int past_idx=(i-nodes_in+ivs)%ivs;
    VertexSE3* v = vertices[i];
    VertexSE3* v_past=vertices[past_idx];

    EdgeSE3* e=new EdgeSE3;
    e->setVertex(0,v_past);
    e->setVertex(1,v);
    e->setMeasurement(v_past->estimate().inverse()*v->estimate());
    e->setInformation(information);
    edges.push_back(e);
  }

  // write output
  ofstream stream;
  if (filename != "-") {
    cerr << "Writing into [" << filename << "]" << endl;
    stream.open(filename.c_str());
  } else {
    cerr << "writing to stdout" << endl;
  }

  string vertexTag = Factory::instance()->tag(vertices[0]);
  string edgeTag = Factory::instance()->tag(edges[0]);

  ostream& fout = filename != "-" ? stream : cout;
  for (size_t i = 0; i < vertices.size(); ++i) {
    VertexSE3* v = vertices[i];
    fout << vertexTag << " " << v->id() << " ";
    v->write(fout);
    fout << endl;
  }

  for (size_t i = 0; i < edges.size(); ++i) {
    EdgeSE3* e = edges[i];
    VertexSE3* from = static_cast<VertexSE3*>(e->vertex(0));
    VertexSE3* to = static_cast<VertexSE3*>(e->vertex(1));
    fout << edgeTag << " " << from->id() << " " << to->id() << " ";
    e->write(fout);
    fout << endl;
  }

  return 0;
}
