#include <iostream>
#include <g2o/stuff/color_macros.h>
#include "matchable_generator.h"


namespace g2o {
  namespace matchables {

    Matrix3 d2R(const Vector3& d) {
      Matrix3 R;
      float d_norm = std::sqrt(d(0)*d(0) + d(1)*d(1));
      if(d_norm > 1e-4f)
        R << d(0),d(1)/d_norm,d(0)*d(2)/d_norm,
          d(1),-d(0)/d_norm,d(1)*d(2)/d_norm,
          d(2),0,-d_norm;
      else
        R.setIdentity();

      return R;
    }
    
    MatchableGenerator::MatchableGenerator() {
      _id = 0;
      _is_setup = false;
      _is_init = false;
    }

    MatchableGenerator::~MatchableGenerator() {
      //ia placeholder
    }

    void MatchableGenerator::setup() {
      if (!_params.num_poses)
        throw std::runtime_error("no poses");

      if (!_params.num_points && !_params.num_lines && !_params.num_planes)
        throw std::runtime_error("no landmarks");

      if (!_params.has_point_factors && !_params.has_line_factors && !_params.has_plane_factors)
        throw std::runtime_error("no factors");

      if (!(_params.world_size > 0))
        throw std::runtime_error("invalid world size");
      
      _is_setup = true;
    }

    void MatchableGenerator::init() {
      if (!_vertices)
        throw std::runtime_error("you must set the vertices");
      if (!_edges)
        throw std::runtime_error("you must set the edges");

      _vertices->clear();
      _edges->clear();
      _factory = Factory::instance();

      _is_init = true;
    }

    void MatchableGenerator::compute() {
      if (!_is_setup)
        throw std::runtime_error("please setup the MatchableGenerator");
      if (!_is_init)
        throw std::runtime_error("please initialize the MatchableGenerator");
      
      std::cerr << "generate poses" << std::endl;
      _computePoses();

      std::cerr << "generate matchables" << std::endl;
      if (_params.num_points) {
        _computePoints();
      }

      if (_params.num_lines) {
        _computeLines();
      }

      if (_params.num_planes) {
        _computePlanes();
      }

      std::cerr << "generate edges" << std::endl;
      _computeEdges();

      std::cerr << CL_GREEN("done!") << std::endl << std::endl;
      std::cerr << "num vertices: " << _vertices->size() << std::endl;
      std::cerr << "num edges   : " << _edges->size() << std::endl;
      std::cerr << std::endl;
    }
    
    void MatchableGenerator::_computePoses() {
      Matrix6 rand_scale = Matrix6::Identity();
      rand_scale.block<3,3>(0,0) *= (0.5*_params.world_size);
      rand_scale.block<3,3>(3,3) *= M_PI;

      //ia the first pose vertex is always the gauge. stop.
      VertexSE3Chord* gauge = new VertexSE3Chord();
      gauge->setEstimate(Isometry3::Identity());
      gauge->setId(_id);
      gauge->setFixed(true);
      _vertices->insert(std::make_pair(_id++, gauge));

      for (int i=1; i<_params.num_poses; ++i, ++_id) {
        Vector6 minimal_estimate = Vector6::Random()-0.5*Vector6::Ones();
        VertexSE3Chord* v = new VertexSE3Chord();
        v->setEstimate(internal::fromVectorET(minimal_estimate));
        v->setId(_id);
        _vertices->insert(std::make_pair(_id, v));
      }
    }

    void MatchableGenerator::_computePoints() {
      for (int i = 0; i < _params.num_points; ++i, ++_id) {
        Vector3 p = (Vector3::Random()-0.5*Vector3::Ones())*_params.world_size;
        Matrix3 R = Matrix3::Identity();
        Matchable m(Matchable::Type::Point,p,R);

        VertexMatchable* v = new VertexMatchable();
        v->setId(_id);
        v->setEstimate(Matchable(Matchable::Type::Point,p,R));
        
        //TODO some check could be done here
        _vertices->insert(std::make_pair(_id, v));
      }
    }

    void MatchableGenerator::_computeLines() {
      for (int i = 0; i < _params.num_lines; ++i, ++_id) {
        Vector3 p = (Vector3::Random()-0.5*Vector3::Ones())*_params.world_size;
        Vector3 d = (Vector3::Random()-0.5*Vector3::Ones())*_params.world_size;
        d.normalize();
        Matrix3 R = d2R(d);

        VertexMatchable* v = new VertexMatchable();
        v->setId(_id);
        v->setEstimate(Matchable(Matchable::Type::Line,p,R));
        
        //TODO some check could be done here
        _vertices->insert(std::make_pair(_id, v));
      }
    }

    void MatchableGenerator::_computePlanes() {
      for (int i = 0; i < _params.num_planes; ++i, ++_id) {
        Vector3 p = (Vector3::Random()-0.5*Vector3::Ones())*_params.world_size;
        Vector3 d = (Vector3::Random()-0.5*Vector3::Ones())*_params.world_size;
        d.normalize();
        Matrix3 R = d2R(d);

        VertexMatchable* v = new VertexMatchable();
        v->setId(_id);
        v->setEstimate(Matchable(Matchable::Type::Plane,p,R));

        //TODO some check could be done here
        _vertices->insert(std::make_pair(_id, v));
      }
    }

    //ia edges are generated supposing that each pose sees all landmarks
    void MatchableGenerator::_computeEdges() {
      //ia for each vertex
      VertexSE3Chord* v_from = 0;
      VertexMatchable* v_to = 0;
      for (HyperGraph::VertexIDMap::const_iterator v_from_it = _vertices->begin();
           v_from_it != _vertices->end(); ++v_from_it) {

        //ia if the first vertex is not a pose, continue
        if (_factory->tag(v_from_it->second) == "VERTEX_SE3:CHORD") {
          v_from = static_cast<VertexSE3Chord*>(v_from_it->second);
        } else {
          v_from = 0;
          continue;
        }

        //ia take all the the other matchable vertexes
        for (HyperGraph::VertexIDMap::const_iterator v_to_it = _vertices->begin();
             v_to_it != _vertices->end(); ++v_to_it) {
          
          if (_factory->tag(v_to_it->second) == "VERTEX_MATCHABLE") {
            v_to = static_cast<VertexMatchable*>(v_to_it->second);
          } else {
            v_to = 0;
            continue;
          }

          switch (v_to->estimate().type()) {
          case Matchable::Type::Point:
            {
              if (_params.has_point_factors) {
                HyperGraph::Edge* e_point = _computePointEdge(v_from, v_to);
                if (e_point) {
                  _edges->insert(e_point);
                } else {
                  std::cerr << "skip invalid point edge" << std::endl;
                }
              }

              //ia here you can add other constraints if you want
              
              break;
            }
          case Matchable::Type::Line:
            {
              if (_params.has_line_factors) {
                HyperGraph::Edge* e_point = _computeLineEdge(v_from, v_to);
                if (e_point) {
                  _edges->insert(e_point);
                } else {
                  std::cerr << "skip invalid line edge" << std::endl;
                }
              }

              //ia here you can add other constraints if you want
              
              break;
            }
          case Matchable::Type::Plane:
            {
              if (_params.has_plane_factors) {
                HyperGraph::Edge* e_point = _computePlaneEdge(v_from, v_to);
                if (e_point) {
                  _edges->insert(e_point);
                } else {
                  std::cerr << "skip invalid plane edge" << std::endl;
                }
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

    HyperGraph::Edge* MatchableGenerator::_computePointEdge(VertexSE3Chord* vfrom_,
                                                            VertexMatchable* vto_) {
      const Isometry3& pose = vfrom_->estimate();
      const Matchable& matchable = vto_->estimate();
      
      Matchable measurement = matchable.transform(pose.inverse());
      Matrix7 omega = Matrix7::Zero();
      omega.block<3,3>(0,0) = matchable.omega();

      //TODO some consistency checks could be done here and return NULL if something is wrong
      // if (measurement_is_not_valid) return 0

      EdgeSE3Matchable* e = new EdgeSE3Matchable();
      e->vertices()[0] = vfrom_;
      e->vertices()[1] = vto_;
      e->setInformation(omega);
      e->setMeasurement(measurement);
      return e;
    }

    HyperGraph::Edge* MatchableGenerator::_computeLineEdge(VertexSE3Chord* vfrom_,
                                                           VertexMatchable* vto_) {
      const Isometry3& pose = vfrom_->estimate();
      const Matchable& matchable = vto_->estimate();
      
      Matchable measurement = matchable.transform(pose.inverse());
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

    HyperGraph::Edge* MatchableGenerator::_computePlaneEdge(VertexSE3Chord* vfrom_,
                                                            VertexMatchable* vto_) {
      const Isometry3& pose = vfrom_->estimate();
      const Matchable& matchable = vto_->estimate();
      
      Matchable measurement = matchable.transform(pose.inverse());
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
    
  } //ia end namespace matchables
} //ia end namespace g2o
