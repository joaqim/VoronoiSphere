
#include <Terrific/Math/Types.h>
#include <Terrific/Math/Math.h>

namespace Terrific {
namespace  Math {

std::tuple<Vector3d, CubeFaceBitSet> Point::cubeCoord() const {
  Vector3d absPos = Magnum::Math::abs(position);
  Vector3d result;
  CubeFaceBitSet faceSet;
  double ratio;
  if (absPos.z() >= fmax(absPos.x(), absPos.y()))
  {
    ratio = 1.0 / absPos.z();
    faceSet |= (position.z() > 0 ? CF_POSZ_BITMASK : CF_NEGZ_BITMASK);
    if (absPos.z() == absPos.x())
    {
      faceSet |= (position.x() > 0 ? CF_POSX_BITMASK : CF_NEGX_BITMASK);
    }
    if (absPos.z() == absPos.y())
    {
      faceSet |= (position.y() > 0 ? CF_POSY_BITMASK : CF_NEGY_BITMASK);
    }
  }
  else if (absPos.y() >= fmax(absPos.x(), absPos.z()))
  {
    ratio = 1.0 / absPos.y();
    faceSet |= (position.y() > 0 ? CF_POSY_BITMASK : CF_NEGY_BITMASK);
    if (absPos.y() == absPos.x())
    {
      faceSet |= (position.x() > 0 ? CF_POSX_BITMASK : CF_NEGX_BITMASK);
    }
    if (absPos.y() == absPos.z())
    {
      faceSet |= (position.z() > 0 ? CF_POSZ_BITMASK : CF_NEGZ_BITMASK);
    }
  }
  else {
    ratio = 1.0 / absPos.x();
    faceSet |= (position.x() > 0 ? CF_POSX_BITMASK : CF_NEGX_BITMASK);
    if (absPos.x() == absPos.y())
    {
      faceSet |= (position.y() > 0 ? CF_POSY_BITMASK : CF_NEGY_BITMASK);
    }
    if (absPos.x() == absPos.z())
    {
      faceSet |= (position.z() > 0 ? CF_POSZ_BITMASK : CF_NEGZ_BITMASK);
    }
  }
  result = position * ratio;
  return std::tie(result, faceSet);
}

Vector3d Point::tangent() const {
  CubeFaceBitSet bitSet;

  Vector3d binormal(1, 0, 0);

  if (bitSet.test(CF_POSX))
  {
    binormal = Vector3d(0, 1, 0);
  }
  else if (bitSet.test(CF_NEGX))
  {
    binormal = Vector3d(0, 1, 0);
  }
  else if (bitSet.test(CF_POSY))
  {
    binormal = Vector3d(1, 0, 0);
  }
  else if (bitSet.test(CF_NEGY))
  {
    binormal = Vector3d(-1, 0, 0);
  }
  else if (bitSet.test(CF_POSZ))
  {
    binormal = Vector3d(0, 1, 0);
  }
  else if (bitSet.test(CF_NEGZ))
  {
    binormal = Vector3d(0, 1, 0);
  }
  else
  {
    assert(false);
  }

  Vector3d normal = position;
  Vector3d result = normalize(cross(binormal, normal));
  return result;
}

Vector3d Point::binormal() const {
  Vector3d t = tangent();
  Vector3d normal = position;
  Vector3d result = normalize(cross(normal, t));
  return result;
}

// http://www.cgafaq.info/wiki/Intersection_of_three_planes
bool threePlanesIntersection(const Plane& planeA, const Plane& planeB, const Plane& planeC, Vector3d& result) {
  Vector3d bcCross = cross(planeB.normal(), planeC.normal());
  double denom = dot(planeA.normal(), bcCross);

  if (denom == 0) {
    result = Vector3d(0);
    return false;
  }
  else {
    result = (-planeA.distance() * bcCross
              - planeB.distance() * cross(planeC.normal(), planeA.normal())
              - planeC.distance() * cross(planeA.normal(), planeB.normal())) / denom;
    return true;
  }
}

// http://tavianator.com/2011/05/fast-branchless-raybounding-box-intersections/
bool rayAabbIntersection(const Ray& ray, const AABB& aabb) {
  Vector3d n_inv = Vector3d(1.0) / ray.direction();

  double tx1 = (aabb.min().x() - ray.origin().x())*n_inv.x();
  double tx2 = (aabb.max().x() - ray.origin().x())*n_inv.x();

  double tmin = fmin(tx1, tx2);
  double tmax = fmax(tx1, tx2);

  double ty1 = (aabb.min().y() - ray.origin().y())*n_inv.y();
  double ty2 = (aabb.max().y() - ray.origin().y())*n_inv.y();

  tmin = fmax(tmin, fmin(ty1, ty2));
  tmax = fmin(tmax, fmax(ty1, ty2));

  double tz1 = (aabb.min().z() - ray.origin().z())*n_inv.z();
  double tz2 = (aabb.max().z() - ray.origin().z())*n_inv.z();

  tmin = fmax(tmin, fmin(tz1, tz2));
  tmax = fmin(tmax, fmax(tz1, tz2));

  return tmax >= fmax(tmin, 0.0);
}

std::vector<Vector3d> splitSphericalLineSegment(const Point& start, const Point& end, double deltaAngle) {
  std::vector<Vector3d> result;

  assert(start.position != -end.position);

  auto direction = normalize(cross(start.position, end.position));
  double distance = acos(dot(start.position, end.position));

  result.push_back(start.position);

  for (double angle = deltaAngle; angle<distance; angle+=deltaAngle)
  {
    //Mat4 rotation = rotate(Mat4(1.0), angle, direction);
    //Matrix4 rotation = Matrix4::rotation(Rad(static_cast<float>(angle)), direction);
    Matrix4d rotation = Matrix4d::rotation(Rad{angle}, direction);
    //Vector3d pos = normalize(Vector3d(rotation * Vector4(start.position, 1.0)));
    Vector3d pos = (rotation * Vector4d(start.position, 1.f)).xyz().normalized(); //NOTE: .xyz() converts Vec4 to Vec3 as per glm semantics.

    result.push_back(pos);
  }

  result.push_back(end.position);

  return result;
}

double lagrangeInterpolate(double x, const std::vector<double>& xArray, const std::vector<double>& yArray) {
#ifdef TERRIFIC_DEBUG
  assert(xArray.size() == yArray.size());
#endif

  double sum = 0.0;
  for (unsigned int i = 0; i < xArray.size(); ++i)
  {
    double Xi, Yi;
    Xi = xArray[i];
    Yi = yArray[i];
    double factor = 1.0;
    for (unsigned int j = 0; j < xArray.size(); ++j)
    {
      if (i != j)
      {
        double Xj = xArray[j];
        factor *= (x - Xj) / (Xi - Xj);
      }
    }
    sum += factor * Yi;
  }
  return sum;
}

double interpolateSphericalSamples(const Point& p0, const std::vector<Point>& points, const std::vector<double>& values) {
  double totalSqrDistance = std::accumulate(points.begin(), points.end(), 0.0, [p0](double res, const Point& p) {
                                                                                 double d = p.sphericalDistance(p0);
                                                                                 return res + d * d;
                                                                              });

  double sum = 0.0;
  double weight = 0.0;

  for (size_t i = 0; i < points.size(); ++i)
  {
    const Point& p = points[i];
    double d = p.sphericalDistance(p0);
    double w = (totalSqrDistance - d*d) / totalSqrDistance;
    sum += w * values[i];
    weight += w;
  }
  return sum / weight;
}

  double computeTriangleArea(const Vector3d& p0, const Vector3d& p1, const Vector3d& p2) {
    Vector3d v12 = p2 - p1;
    Vector3d v02 = p2 - p0;
    Vector3d v12n = normalize(v12);
    double t = dot(v02, v12n);
    Vector3d c = p2 - v12n * t;
    double d = distance(p0, c);
    double l12 = length(v12);
    return l12 * d * 0.5;
  }

void faceAxisDirection(ECubeFace face, Vector3d& s_dir, Vector3d& t_dir, Vector3d& p_dir) {
  switch (face)
  {
    case CF_POSX:
      p_dir = Vector3d(1, 0, 0);
        s_dir = Vector3d(0, 0, -1);
        t_dir = Vector3d(0, 1, 0);
        break;
      case CF_NEGX:
        p_dir = Vector3d(-1, 0, 0);
        s_dir = Vector3d(0, 0, 1);
        t_dir = Vector3d(0, 1, 0);
        break;
      case CF_POSY:
        p_dir = Vector3d(0, 1, 0);
        s_dir = Vector3d(0, 0, 1);
        t_dir = Vector3d(1, 0, 0);
        break;
      case CF_NEGY:
        p_dir = Vector3d(0, -1, 0);
        s_dir = Vector3d(0, 0, 1);
        t_dir = Vector3d(-1, 0, 0);
        break;
      case CF_POSZ:
        p_dir = Vector3d(0, 0, 1);
        s_dir = Vector3d(1, 0, 0);
        t_dir = Vector3d(0, 1, 0);
        break;
      case CF_NEGZ:
        p_dir = Vector3d(0, 0, -1);
        s_dir = Vector3d(-1, 0, 0);
        t_dir = Vector3d(0, 1, 0);
        break;
      default:
        assert(0);
        p_dir = Vector3d(1, 0, 0);
        s_dir = Vector3d(0, 0, -1);
        t_dir = Vector3d(0, 1, 0);
    }
  }
}
}




