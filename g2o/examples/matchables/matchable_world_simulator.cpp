#include "matchable_world_simulator.h"
#include "g2o/stuff/timeutil.h"

namespace g2o {
  namespace matchables {
    WorldSimulator::WorldSimulator() {
      _vertex_id = 0;
    }

    WorldSimulator::~WorldSimulator() {
    }

    void WorldSimulator::init() {
      if (!_vertices) {
        throw std::runtime_error("please set the vertices");
      }

      if (!_edges) {
        throw std::runtime_error("please set the edges");
      }

      if (!_world) {
        throw std::runtime_error("please set the world");
      }

      if (!_world->isValid()) {
        throw std::runtime_error("world is not valid");
      }        

      if (!_params.factors_types.checkFactors()) {
        throw std::runtime_error("no factors selected");
      }

      if (_params.sense_radius < 0.0)
        throw std::runtime_error("invalid sense radius");

      if (_params.num_poses < 1)
        std::cerr << " warning: invalid simulation steps" << std::endl;
      
      _vertices->clear();
      _edges->clear();
    }

    void WorldSimulator::compute() {
      _params.simulator_stats.setZero();
      uint64_t sim_steps = 0;
      
      bool continue_=true;
      Vector6 minimal_estimate = Vector6::Zero();
      // minimal_estimate.head(3) = Vector3((_world->width()/2-1) * _world->resolution(),
      //                                    (_world->height()/2-1) * _world->resolution(),
      //                                    0);


      VertexSE3Chord* prev_vertex = new VertexSE3Chord();
      prev_vertex->setId(_vertex_id);
      prev_vertex->setFixed(true);
      prev_vertex->setEstimate(internal::fromVectorET(minimal_estimate));
      _vertices->insert(std::make_pair(_vertex_id++, prev_vertex));

      std::cerr << "simulating robot motion in the world" << std::endl;
      while(continue_){        
        //ia attempt to move
        VertexSE3Chord* new_vertex = _moveRobot(prev_vertex);
        if (!new_vertex)
          continue;
        
        //sense
        std::cerr << "s"; 
        _senseMatchables(prev_vertex);

        // ia generate odom - actual movement
        std::cerr << "m";
        EdgeSE3Chord* e = new EdgeSE3Chord();
        e->vertices()[0] = prev_vertex;
        e->vertices()[1] = new_vertex;
        e->information().setIdentity();
        e->setMeasurementFromState();
        _edges->insert(e);
        
        // end
        prev_vertex = new_vertex;
        if(sim_steps > _params.num_poses)
          continue_=false;

        ++sim_steps;
        // std::cin.get();
      }
      std::cerr << std::endl;

      std::cerr << "removed " << _world->numRemovedWalls() << " walls during motion" << std::endl;
      std::cerr << "\nfinal graph has " << std::endl;
      _params.simulator_stats.print();
    }
    

    void WorldSimulator::_senseMatchables(g2o::VertexSE3Chord* v_) {

      const Eigen::Isometry3d& robot_pose = v_->estimate();
      const Eigen::Isometry3d& robot_pose_inverse = robot_pose.inverse();

      for(Matchable* mptr : _world->landmarks()) {
        if (mptr->applyTransform(robot_pose_inverse).point().norm() >= _params.sense_radius)
          continue;

        VertexMatchable* v_m = new VertexMatchable();
        v_m->setId(_vertex_id);
        v_m->setEstimate(*mptr);
        _vertices->insert(std::make_pair(_vertex_id++, v_m));

        switch (mptr->type()) {
        case Matchable::Type::Point:
          {
            if (_params.factors_types.point_factors) {
              HyperGraph::Edge* e = _computePointEdge(v_, v_m);
              if (e) {
                _edges->insert(e);
                ++_params.simulator_stats.point_point;
              }
            }
            break;
          }
        case Matchable::Type::Line:
          {
            if (_params.factors_types.line_factors) {
              HyperGraph::Edge* e = _computeLineEdge(v_, v_m);
              if (e){
                _edges->insert(e);
                ++_params.simulator_stats.line_line;
              }
            }
            if (_params.factors_types.line_point_factors) {
              HyperGraph::Edge* e = _computeLinePointEdge(v_, v_m);
              if (e){
                _edges->insert(e);
                ++_params.simulator_stats.line_point;
              }
            }
            break;
          }
        case Matchable::Type::Plane:
          {
            if (_params.factors_types.plane_factors) {
              HyperGraph::Edge* e = _computePlaneEdge(v_, v_m);
              if (e){
                _edges->insert(e);
                ++_params.simulator_stats.plane_plane;
              }
            }
            if (_params.factors_types.plane_line_factors) {
              // HyperGraph::Edge* e = _computePlaneLineEdge(v_, v_m);
              // if (e){
              //   _edges->insert(e);
              //   ++_params.simulator_stats.plane_line;
              // }
              EdgeVector e_vec = _computePlaneLineEdge(v_, v_m);
              if (e_vec.size()) {
                for (HyperGraph::Edge* e : e_vec) {
                  std::cerr << "\nedge between " << e->vertices()[0]->id()
                            << "->" << e->vertices()[1]->id() << std::endl;
                  _edges->insert(e);
                  ++_params.simulator_stats.plane_line;
                }
              }
            }
            if (_params.factors_types.plane_point_factors) {
              HyperGraph::Edge* e = _computePlanePointEdge(v_, v_m);
              if (e){
                _edges->insert(e);
                ++_params.simulator_stats.plane_point;
              }
            }
            break;
          }
        default:
          throw std::runtime_error("unexepected matchable type");
        }
      }
    }


    VertexSE3Chord* WorldSimulator::_moveRobot(VertexSE3Chord* from_vertex_) {
      std::random_device rd;
      std::mt19937 generator(rd()); //ia not required until you need a lot of numbers
      std::uniform_real_distribution<> uniform_distribution(0, 1);

      const Vector6 current_estimate = internal::toVectorET(from_vertex_->estimate());
      Vector6 next_estimate = Vector6::Zero();

      //ia sample new motion direction
      const number_t current_theta = current_estimate[5];
      number_t next_theta = 0;
      number_t x = 0, y = 0;
      number_t dir_selector = uniform_distribution(generator);

      if (dir_selector < 0.5) {
        x = round(cos(current_theta));
        y = round(sin(current_theta));
      } else if (dir_selector < 0.75 && 0.5 < dir_selector) {
        x = round(-sin(current_theta));
        y = round(cos(current_theta));
        next_theta = M_PI/2.0f;
      } else {
        x = round(sin(current_theta));
        y = round(-cos(current_theta));
        next_theta = -M_PI/2.0f;
      }

      next_estimate.head(2) = current_estimate.head(2) + Vector2(x,y);
      next_estimate[5] = current_estimate[5] + next_theta;

      const Vector2I& current_cell = current_estimate.head(2).cast<int>();
      const Vector2I& next_cell = next_estimate.head(2).cast<int>();

      //ia check if it is inside
      if (next_cell.x() < 0 || _world->params().width < (size_t)next_cell.x() ||
          next_cell.y() < 0 || _world->params().height < (size_t)next_cell.y()) {
        return 0;
      }

      //ia TODO check for walls and remove them
      CellPair cell_motion(current_cell, next_cell);
      if (_world->removeWall(cell_motion)) {
        // std::cerr << "hit a wall -> removed" << std::endl;
      } 

      //ia create the new vertex
      VertexSE3Chord* to_vertex = new VertexSE3Chord();
      to_vertex->setId(_vertex_id);
      to_vertex->setEstimate(internal::fromVectorET(next_estimate));
      _vertices->insert(std::make_pair(_vertex_id++, to_vertex));
      
      return to_vertex;      
    }
    
    //ia private things
    HyperGraph::Edge* WorldSimulator::_computePointEdge(VertexSE3Chord* vfrom_,
                                                        VertexMatchable* vto_){
      const Isometry3& pose = vfrom_->estimate();
      const Matchable& matchable = vto_->estimate();

      Matchable measurement = matchable.applyTransform(pose.inverse());
      Matrix7 omega = Matrix7::Zero();
      omega.block<3,3>(0,0) = matchable.omega();

      //TODO some consistency checks could be done here and return NULL if something is wrong
      // if (measurement_is_not_valid) return 0

      EdgeSE3Matchable* e = new g2o::matchables::EdgeSE3Matchable();
      e->vertices()[0] = vfrom_;
      e->vertices()[1] = vto_;
      e->setInformation(omega);
      e->setMeasurement(measurement);
      return e;
    }

    HyperGraph::Edge* WorldSimulator::_computeLineEdge(VertexSE3Chord* vfrom_,
                                                       VertexMatchable* vto_){
      const Isometry3& pose = vfrom_->estimate();
      const Matchable& matchable = vto_->estimate();

      Matchable measurement = matchable.applyTransform(pose.inverse());
      Matrix7 omega = Matrix7::Zero();
      omega.block<3,3>(0,0) = matchable.omega();
      omega.block<3,3>(3,3) = Eigen::Matrix3d::Identity();

      //TODO some consistency checks could be done here and return NULL if something is wrong
      // if (measurement_is_not_valid) return 0

      EdgeSE3Matchable* e = new EdgeSE3Matchable();
      e->vertices()[0] = vfrom_;
      e->vertices()[1] = vto_;
      e->setInformation(omega);
      e->setMeasurement(measurement);
      return e;
    }

    HyperGraph::Edge* WorldSimulator::_computePlaneEdge(VertexSE3Chord* vfrom_,
                                                        VertexMatchable* vto_){
      const Isometry3& pose = vfrom_->estimate();
      const Matchable& matchable = vto_->estimate();

      Matchable measurement = matchable.applyTransform(pose.inverse());

      Matrix7 omega = Matrix7::Zero();
      omega.block<3,3>(0,0) = matchable.omega();
      omega.block<3,3>(3,3) = Matrix3::Identity();

      //TODO some consistency checks could be done here and return NULL if something is wrong
      // if (measurement_is_not_valid) return 0

      EdgeSE3Matchable* e = new EdgeSE3Matchable();
      e->vertices()[0] = vfrom_;
      e->vertices()[1] = vto_;
      e->setInformation(omega);
      e->setMeasurement(measurement);
      return e;
    }

    
    HyperGraph::Edge* WorldSimulator::_computeLinePointEdge(VertexSE3Chord* vfrom_,
                                                            VertexMatchable* vto_) {
      const Isometry3& inv_pose = vfrom_->estimate().inverse();
      const Matchable& matchable = vto_->estimate();
      
      Matchable trans_match = matchable.applyTransform(inv_pose);
      const Vector3& pl = trans_match.point();
      const Vector3& nl = trans_match.rotation().col(0);

      // fn orthogonality check
      number_t x_d = Vector3::UnitZ().dot(nl);
      if(x_d<1e-3){
        return nullptr;
      }

      Matchable measurement(Matchable::Type::Point, pl);

      Matrix7 omega = Matrix7::Zero();
      omega.block<3,3>(0,0) = matchable.omega();

      EdgeSE3Matchable* e = new EdgeSE3Matchable();
      e->vertices()[0] = vfrom_;
      e->vertices()[1] = vto_;
      e->setInformation(omega);
      e->setMeasurement(measurement);
      return e;
    }


    HyperGraph::Edge* WorldSimulator::_computePlanePointEdge(VertexSE3Chord* vfrom_,
                                                             VertexMatchable* vto_) {

      const Matchable& matchable = vto_->estimate();
      const Isometry3& inv_pose = vfrom_->estimate().inverse();
      Matchable trans_match = matchable.applyTransform(inv_pose);

      Matchable measurement(Matchable::Type::Point, trans_match.point());
      Matrix7 omega = Matrix7::Zero();
      omega.block<3,3>(0,0) = matchable.omega();

      EdgeSE3Matchable* e = new EdgeSE3Matchable();
      e->vertices()[0] = vfrom_;
      e->vertices()[1] = vto_;
      e->setInformation(omega);
      e->setMeasurement(measurement);
      
      return e;
    }

    //ia this is the cause of the evil
    WorldSimulator::EdgeVector WorldSimulator::_computePlaneLineEdge(VertexSE3Chord* vfrom_,
                                                                     VertexMatchable* vto_) {
      EdgeVector e_container;

      const Isometry3& inv_pose = vfrom_->estimate().inverse();
      const Matchable& matchable = vto_->estimate();

      Matchable trans_match = matchable.applyTransform(inv_pose);
      const Vector3& pl = trans_match.point();
      const Vector3& nl = trans_match.rotation().col(0);
      
      Vector3 nz = Vector3::UnitZ().cross(nl);

      //orthogonality check
      if (nz.norm() < 1e-3)
        return e_container;
        // return nullptr;
        
      nz.normalize();

      //ia we rotate the normal of the landmark in order to get another measurement
      size_t num_intervals = 1;
      Matrix3 R = Matrix3::Identity();
      for (size_t i = 0; i < num_intervals; ++i) {
        // std::cerr << "\nangle: " << i*M_PI/num_intervals << std::endl;
        R = AngleAxis(i*M_PI/num_intervals, nz);
        // std::cerr << "R:\n" << R << std::endl;

        const Vector3 nz_rotated = R * nz;

        Matchable measurement_rotated(Matchable::Type::Line,pl);
        measurement_rotated.computeRotationMatrixZXY(nz_rotated);
        Matrix7 omega_rotated = Matrix7::Zero();
        omega_rotated.block<3,3>(0,0) = matchable.omega();
        omega_rotated(6,6) = 1;

        EdgeSE3Matchable* e_rotated = new EdgeSE3Matchable();
        e_rotated->vertices()[0] = vfrom_;
        e_rotated->vertices()[1] = vto_;
        e_rotated->setInformation(omega_rotated);
        e_rotated->setMeasurement(measurement_rotated);

        e_container.push_back(e_rotated);
      }
      // std::cin.get();

      // //ia no rotation
      // Matchable measurement(Matchable::Type::Line,pl);
      // measurement.computeRotationMatrixZXY(nz);
      // Matrix7 omega = Matrix7::Zero();
      // omega.block<3,3>(0,0) = matchable.omega();
      // omega(6,6) = 1;

      // EdgeSE3Matchable* e = new EdgeSE3Matchable();
      // e->vertices()[0] = vfrom_;
      // e->vertices()[1] = vto_;
      // e->setInformation(omega);
      // e->setMeasurement(measurement);
      
      // //ia pi/3
      // Matrix3 R0 = Matrix3::Identity();
      // R0 = AngleAxis(M_PI/3, nl);
      
      // const Vector3 nz_rotated_0 = R0 * nz;
      
      // Matchable measurement_rotated_0(Matchable::Type::Line,pl);
      // measurement_rotated_0.computeRotationMatrixZXY(nz_rotated_0);
      // Matrix7 omega_rotated_0 = Matrix7::Zero();
      // omega_rotated_0.block<3,3>(0,0) = matchable.omega();
      // omega_rotated_0(6,6) = 1;

      // EdgeSE3Matchable* e_rotated_0 = new EdgeSE3Matchable();
      // e_rotated_0->vertices()[0] = vfrom_;
      // e_rotated_0->vertices()[1] = vto_;
      // e_rotated_0->setInformation(omega_rotated_0);
      // e_rotated_0->setMeasurement(measurement_rotated_0);

      // //ia 2/3 pi
      // Matrix3 R1 = Matrix3::Identity();
      // R1 = AngleAxis(2*M_PI/3, nl);
      
      // const Vector3 nz_rotated_1 = R1 * nz;
      // Matchable measurement_rotated_1(Matchable::Type::Line,pl);
      // measurement_rotated_1.computeRotationMatrixZXY(nz_rotated_1);
      // Matrix7 omega_rotated_1 = Matrix7::Zero();
      // omega_rotated_1.block<3,3>(0,0) = matchable.omega();
      // omega_rotated_1(6,6) = 1;

      // EdgeSE3Matchable* e_rotated_1 = new EdgeSE3Matchable();
      // e_rotated_1->vertices()[0] = vfrom_;
      // e_rotated_1->vertices()[1] = vto_;
      // e_rotated_1->setInformation(omega_rotated_1);
      // e_rotated_1->setMeasurement(measurement_rotated_1);

      // e_container.push_back(e);
      // e_container.push_back(e_rotated_0);
      // e_container.push_back(e_rotated_1);

      return e_container;
    }
  } //ia end namespace matchable
} //ia end namespace g2o
