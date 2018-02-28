#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include "g2o/core/eigen_types.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/stuff/sampler.h"
#include "g2o/stuff/command_args.h"
#include "g2o/core/factory.h"

using namespace std;
using namespace g2o;

int main (int argc, char** argv) {
  // // command line parsing
  int nodes_x;
  int nodes_y;
  int nodes_z;
  string filename;
  CommandArgs arg;
  arg.param("o", filename, "-", "output filename");
  arg.param("nodes_x", nodes_x, 5, "how many nodes on the x-axis - width");
  arg.param("nodes_y", nodes_y, 5, "how many nodes on the y-axis - height");
  arg.param("nodes_z", nodes_z, 5, "how many nodes on the z-axis - depth");
  arg.parseArgs(argc, argv);

  Isometry3 robot = Isometry3::Identity();
  robot.setIdentity();

  // construct trajectory and odometry
  const int step(1);
  double angle = 0;
  uint64_t vertex_id = 0;
  Vector3 t = Vector3::Zero();
  Matrix3 R = Matrix3::Identity();
  Isometry3 T = Isometry3::Identity();
  std::vector<VertexSE3*> vertices;
  VertexSE3* prev_v = nullptr;

  Matrix6 information = Matrix6::Identity();
  Matrix3 base_R = Eigen::AngleAxisd(-M_PI/2, Vector3::UnitY()).toRotationMatrix();
  Matrix3 y_turn_R = Matrix3::Identity();
  Matrix3 x_turn_R = Matrix3::Identity();
  for (int x = 0; x < nodes_x; x+=step) {
    for (int y = 0; y < nodes_y; y+=step) {
      for (int z = 0; z < nodes_z; z+=step) {
        t = Vector3(x,y,z);
        R = x_turn_R * y_turn_R * base_R;

        T.translation() = t;
        T.linear() = R;

        VertexSE3* curr_v = new VertexSE3();
        curr_v->setEstimate(T);
        curr_v->setId(vertex_id);
        vertices.push_back(curr_v);
        y_turn_R.setIdentity();
        x_turn_R.setIdentity();
        ++vertex_id;
      }
      y_turn_R = Eigen::AngleAxisd(-M_PI/2, Vector3::UnitX()).toRotationMatrix();
    }
    x_turn_R = Eigen::AngleAxisd(M_PI/2, Vector3::UnitZ()).toRotationMatrix();
  }

  // write output
  ofstream stream;
  if (filename != "-") {
    cerr << "Writing into [" << filename << "]" << endl;
    stream.open(filename.c_str());
  } else {
    cerr << "writing to stdout" << endl;
  }

  string vertex_tag = Factory::instance()->tag(vertices[0]);
  // string edgeTag = Factory::instance()->tag(edges[0]);

  ostream& fout = filename != "-" ? stream : cout;
  for (size_t i = 0; i < vertices.size(); ++i) {
    VertexSE3* v = vertices[i];
    fout << vertex_tag << " " << v->id() << " ";
    v->write(fout);
    fout << endl;
  }

  // for (size_t i = 0; i < edges.size(); ++i) {
  //   EdgeSE3* e = edges[i];
  //   VertexSE3* from = static_cast<VertexSE3*>(e->vertex(0));
  //   VertexSE3* to = static_cast<VertexSE3*>(e->vertex(1));
  //   fout << edgeTag << " " << from->id() << " " << to->id() << " ";
  //   e->write(fout);
  //   fout << endl;
  // }

  stream.close();

  return 0;
}
