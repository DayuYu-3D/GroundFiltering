#ifndef BVHINTERSECTOR_H
#define BVHINTERSECTOR_H

#include <map>

#include <FastBVH.h> //path: Fast-BVH\include

#include "triangle.h"
#include "vertex.h"

//! \brief Converts a face into a bounding box.
//! Used by @ref FaceBVH::BVH during the build process.
template<typename Float>
class FaceBoxConverter final {
  /// The minimum X values of the mesh faces.
  std::vector<Float> min_x;

  /// The minimum Y values of the mesh faces.
  std::vector<Float> min_y;

  /// The minimum Z values of the mesh faces.
  std::vector<Float> min_z;

  /// The maximum X values of the mesh faces.
  std::vector<Float> max_x;

  /// The maximum Y values of the mesh faces.
  std::vector<Float> max_y;

  /// The maximum Z values of the mesh faces.
  std::vector<Float> max_z;

 public:
  //! Constructs a new instance of the box converter.
  //! \param attrib Used to resolve the face indices
  //! into points that can be used to determine the
  //! bounding box of a face.
  //! \param faces The faces of the mesh.
  FaceBoxConverter(const std::map<size_t, std::vector<Vertex>>& mvVertex,
                   const std::vector<Triangle>& faces)
  {
    min_x.resize(faces.size());
    min_y.resize(faces.size());
    min_z.resize(faces.size());

    max_x.resize(faces.size());
    max_y.resize(faces.size());
    max_z.resize(faces.size());

    std::size_t nFaces = faces.size();
    for (std::size_t i = 0; i < nFaces; ++i) {
      const auto& face = faces[i];

      FastBVH::Vector3<Float> pos_a = {static_cast<Float>(mvVertex.at(face._geomIndex).at(face._vertexIndexs[0])._coor[0]),
                                       static_cast<Float>(mvVertex.at(face._geomIndex).at(face._vertexIndexs[0])._coor[1]),
                                       static_cast<Float>(mvVertex.at(face._geomIndex).at(face._vertexIndexs[0])._coor[2])};

      FastBVH::Vector3<Float> pos_b = {static_cast<Float>(mvVertex.at(face._geomIndex).at(face._vertexIndexs[1])._coor[0]),
                                       static_cast<Float>(mvVertex.at(face._geomIndex).at(face._vertexIndexs[1])._coor[1]),
                                       static_cast<Float>(mvVertex.at(face._geomIndex).at(face._vertexIndexs[1])._coor[2])};

      FastBVH::Vector3<Float> pos_c = {static_cast<Float>(mvVertex.at(face._geomIndex).at(face._vertexIndexs[2])._coor[0]),
                                       static_cast<Float>(mvVertex.at(face._geomIndex).at(face._vertexIndexs[2])._coor[1]),
                                       static_cast<Float>(mvVertex.at(face._geomIndex).at(face._vertexIndexs[2])._coor[2])};

      auto min = FastBVH::min(pos_a, pos_b);
      auto max = FastBVH::max(pos_a, pos_b);

      min = FastBVH::min(min, pos_c);
      max = FastBVH::max(max, pos_c);

      min_x[i] = min.x;
      min_y[i] = min.y;
      min_z[i] = min.z;

      max_x[i] = max.x;
      max_y[i] = max.y;
      max_z[i] = max.z;
    }
  }

  //! Converts a face to a bounding box.
  //! \param face_index The index of the face to get the bounding box of.
  //! \return The bounding box that fits the specified face.
  FastBVH::BBox<Float> operator()(std::size_t face_index) const noexcept {
    FastBVH::Vector3<Float> min{
        min_x[face_index],
        min_y[face_index],
        min_z[face_index],
    };

    FastBVH::Vector3<Float> max{
        max_x[face_index],
        max_y[face_index],
        max_z[face_index],
    };

    return FastBVH::BBox<Float>(min, max);
  }
};

//! \brief Checks for intersection between a ray and a face.
//! Used by @ref FaceBVH::Traverse during BVH traversal.
template<typename Float>
class FaceIntersector final {
  //! Contains the object file vertices;
  const std::map<size_t, std::vector<Vertex>>& _mvVertex;

  //! The faces.
  const std::vector<Triangle>& _faces;

 public:
  //! \brief Constructs a new instance of a face intersector.
  //! Normally, at this point, we'd pre-compute some more acceleration data,
  //! but we're going to keep this example short and sweet.
  FaceIntersector(const std::map<size_t, std::vector<Vertex>>& vvVertex, const std::vector<Triangle>& faces)
      : _mvVertex(vvVertex), _faces(faces) {}

  FastBVH::Intersection<Float, uint32_t> operator()(uint32_t face_index, const FastBVH::Ray<Float>& ray) const
      noexcept {
    const auto& face = _faces[face_index];

    FastBVH::Vector3<Float> v0 = {static_cast<Float>(_mvVertex.at(face._geomIndex).at(face._vertexIndexs[0])._coor[0]),
                                  static_cast<Float>(_mvVertex.at(face._geomIndex).at(face._vertexIndexs[0])._coor[1]),
                                  static_cast<Float>(_mvVertex.at(face._geomIndex).at(face._vertexIndexs[0])._coor[2])};

    FastBVH::Vector3<Float> v1 = {static_cast<Float>(_mvVertex.at(face._geomIndex).at(face._vertexIndexs[1])._coor[0]),
                                  static_cast<Float>(_mvVertex.at(face._geomIndex).at(face._vertexIndexs[1])._coor[1]),
                                  static_cast<Float>(_mvVertex.at(face._geomIndex).at(face._vertexIndexs[1])._coor[2])};

    FastBVH::Vector3<Float> v2 = {static_cast<Float>(_mvVertex.at(face._geomIndex).at(face._vertexIndexs[2])._coor[0]),
                                  static_cast<Float>(_mvVertex.at(face._geomIndex).at(face._vertexIndexs[2])._coor[1]),
                                  static_cast<Float>(_mvVertex.at(face._geomIndex).at(face._vertexIndexs[2])._coor[2])};

    // Basic Möller and Trumbore algorithm

    auto v0v1 = v1 - v0;
    auto v0v2 = v2 - v0;

    auto pvec = cross(ray.d, v0v2);

    auto det = dot(v0v1, pvec);

    if (std::fabs(det) < std::numeric_limits<Float>::epsilon()) {
       // This ray is parallel to this triangle.
      return FastBVH::Intersection<Float, uint32_t>{};
    }

    auto inv_det = 1.0f / det;

    auto tvec = ray.o - v0;

    auto u = dot(tvec, pvec) * inv_det;

    if ((u < 0) || (u > 1)) {
      return FastBVH::Intersection<Float, uint32_t>{};
    }

    auto qvec = cross(tvec, v0v1);

    auto v = dot(ray.d, qvec) * inv_det;

    if ((v < 0) || (u + v) > 1) {
      return FastBVH::Intersection<Float, uint32_t>{};
    }

    // At this stage we can compute t to find out where the intersection point is on the line.
    auto t = dot(v0v2, qvec) * inv_det;
    if (t < std::numeric_limits<Float>::epsilon()) {
      return FastBVH::Intersection<Float, uint32_t>{};
    }

    // At this point, we know we have a hit.

    return FastBVH::Intersection<Float, uint32_t>{t, &face_index};// ray intersection
  }
};

#endif // BVHINTERSECTOR_H
