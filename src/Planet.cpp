#include "Planet.h"

namespace Terrific {

Planet::Planet(Object3D& object,
               DrawableGroup3D* group,
               std::vector<Vector3d> const &points,
               bool debugMode
               ) : Geometry::SphericalVoronoi(points, debugMode), GL::SphereDrawable{object, group} {
}
}
