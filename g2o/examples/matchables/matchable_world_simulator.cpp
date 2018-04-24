#include "matchable_world_simulator.h"

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
      
      _vertices->clear();
      _edges->clear();
    }

    void WorldSimulator::senseMatchables(g2o::VertexSE3Chord* v_){

      const Eigen::Isometry3d& robot_pose = v_->estimate();
      const Eigen::Isometry3d& robot_pose_inverse = robot_pose.inverse();

      for(MatchablePtr mptr : _world->landmarks()){

        const Vector3& m_position = mptr->point();
        const Vector3& robot_position = robot_pose.translation();
        

        if (mptr->applyTransformation(robot_pose_inverse).point().norm() < _sense_radius) {

          VertexMatchable* v_m = new VertexMatchable();
          v_m->setId(_vertex_id);
          v_m->setEstimate(*mptr);
          _vertices->insert(std::make_pair(_vertex_id++, v_m));

          switch (mptr->type()) {
          case Matchable::Type::Point:
            {
              if (_factors_types.point_factors) {
               HyperGraph::Edge* e = _computePointEdge(v_, v_m);
               if (e)
                 _edges->insert(e);
              }
              //ia here you can add other constraints if you want
              break;
            }
          case Matchable::Type::Line:
            {
              if (_factors_types.line_factors) {
                HyperGraph::Edge* e = _computeLineEdge(v_, v_m);
                if (e)
                  _edges->insert(e);
              } else if (_factors_types.line_point_factor) {
                e = _computeLinePointEdge(v_from, v_to);
                if (e)
                  _edges.insert(e);
              }
              //ia here you can add other constraints if you want
              break;
            }
          case Matchable::Type::Plane:
            {
              if (_factors_types.plane_factors) {
                HyperGraph::Edge* e = _computePlaneEdge(v_, v_m);
                if (e)
                  _edges->insert(e);
              } else if (_factors_types.plane_line_factor) {
                HyperGraph::Edge* e = _computePlaneLineEdge(v_, v_m);
                if (e)
                  _edges->insert(e);                
              } else if (_factors_types.plane_point_factor) {
                HyperGraph::Edge* e = _computePlanePointEdge(v_, v_m);
                if (e)
                  _edges->insert(e);                
              }              
              //ia here you can add other constraints if you want
              break;
            }
          default:
            throw std::runtime_error("unexepected matchable type");
          }
        }
      }
    }

    void WorldSimulator::compute(){

      int count=0;
      bool continue_=true;
      std::random_device rd;
      std::mt19937 gen(rd());
      std::uniform_real_distribution<> dis(0, 1);
      Vector3 new_position = Vector3::Zero();

      Vector3 cell_pos(_world->width()/2-1, _world->height()/2-1, 0);
      Vector3 position((_world->width()/2-1) * _world->resolution(),
                       (_world->height()/2-1) * _world->resolution(),
                       0.0f);

      Vector6 minimal_estimate = Vector6::Zero();
      minimal_estimate.head(3) = Vector3(position.x(), position.y(), 0);
      minimal_estimate.tail(3) = Vector3(0,0,position.z());

      VertexSE3Chord* prev_vertex = new VertexSE3Chord();      
      prev_vertex->setId(_vertex_id);
      prev_vertex->setFixed(true);
      prev_vertex->setEstimate(internal::fromVectorET(minimal_estimate));
      _vertices->insert(std::make_pair(_vertex_id++, prev_vertex));

      while(continue_){

        //sample new position
        Vector3 increment = Vector3::Zero();
        float n = dis(gen);

        //go forward
        if(n < 0.5f){
          increment.x() = round(cos(position.z()));
          increment.y() = round(sin(position.z()));
        }
        //go left
        if(n >= 0.5f && n < 0.75f){
          increment.x() = round(-sin(position.z()));
          increment.y() = round(cos(position.z()));
          increment.z() = M_PI/2.0f;
        }
        //go right
        if(n >= 0.75f){
          increment.x() = round(sin(position.z()));
          increment.y() = round(-cos(position.z()));
          increment.z() = -M_PI/2.0f;
        }

        Vector3 new_cell_pos = cell_pos+increment;

        //check if new position is out of grid
        if(new_cell_pos.x() < 0.0f || new_cell_pos.x() > (float)(_world->width()-1) ||
           new_cell_pos.y() < 0.0f || new_cell_pos.y() > (float)(_world->height()-1)){
          continue;
        }

        new_position.head(2) = new_cell_pos.head(2)*_world->resolution();
        new_position.z() = new_cell_pos.z();
        
        //sense
        senseMatchables(prev_vertex);

        //move
        //new position is valid
        position = new_position;
        cell_pos = new_cell_pos;

        //generate pose vertex
        minimal_estimate.head(3) = Vector3(position.x(), position.y(), 0);
        minimal_estimate.tail(3) = Vector3(0,0,position.z());
        VertexSE3Chord* vertex = new VertexSE3Chord();
        vertex->setId(_vertex_id);
        vertex->setEstimate(internal::fromVectorET(minimal_estimate));
        _vertices->insert(std::make_pair(_vertex_id++, vertex));

        // ia generate odom
        EdgeSE3Chord* e = new EdgeSE3Chord();
        e->vertices()[0] = prev_vertex;
        e->vertices()[1] = vertex;
        e->information().setIdentity();
        e->setMeasurementFromState();
        _edges->insert(e);

        // end
        prev_vertex = vertex;
        if(count > _num_poses)
          continue_=false;

        count++;
      }

    }

    //ia private things
    HyperGraph::Edge* WorldSimulator::_computePointEdge(VertexSE3Chord* vfrom_,
                                                        VertexMatchable* vto_){
      const Isometry3& pose = vfrom_->estimate();
      const Matchable& matchable = vto_->estimate();

      Matchable measurement = matchable.applyTransformation(pose.inverse());
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

      Matchable measurement = matchable.applyTransformation(pose.inverse());
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
      //      std::cerr << "created line edge " << vfrom_->id() << "->" << vto_->id() << std::endl;
      return e;
    }

    HyperGraph::Edge* WorldSimulator::_computePlaneEdge(VertexSE3Chord* vfrom_,
                                                        VertexMatchable* vto_){
      const Isometry3& pose = vfrom_->estimate();
      const Matchable& matchable = vto_->estimate();

      Matchable measurement = matchable.applyTransformation(pose.inverse());
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
      //      std::cerr << "created plane edge " << vfrom_->id() << "->" << vto_->id() << std::endl;
      return e;
    }

    HyperGraph::Edge* WorldSimulator::_computePlaneLineEdge(VertexSE3Chord* vfrom_,
                                                            VertexMatchable* vto_) {
      const Isometry3& pose = vfrom_->estimate();
      const Matchable& matchable = vto_->estimate();

      Matchable measurement = matchable.applyTransformation(pose.inverse());
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

  } //ia end namespace matchable
} //ia end namespace g2o
