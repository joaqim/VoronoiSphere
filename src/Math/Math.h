
#include <cassert>
#include <ostream>
#include <sstream>
#include <vector>
#include <tuple>
#include <bitset>
#include <numeric> // for std::accumulate

#include <limits>

#define FLT_MAX std::numeric_limits<double>::max()
#define FLT_MIN std::numeric_limits<double>::min()

#include "xs_Float.h"  // http://stereopsis.com/sree/fpu2006.html

#define FLOOR_TO_INT(val)      xs_FloorToInt(val)
#define ROUND_TO_INT(val)      xs_RoundToInt(val)
#define CEIL_TO_INT(val)       xs_CeilToInt(val)

#ifndef _MATH_DEFINES_DEFINED
#define M_PI 3.14159265358979323846
#endif

#include <Magnum/Math/Vector2.h>
#include <Magnum/Math/Vector3.h>
#include <Magnum/Math/Vector4.h>

#include <Magnum/Math/Matrix4.h>
#include <Magnum/Math/Functions.h>

#include "Types.h"

#include <math.h>


namespace Terrific {
  namespace Math {

    inline std::string stringFromVector3d(const Vector3d &r3) {
      std::stringstream ss;
      ss << "<" << r3.x() << "," << r3.y() << "," << r3.z() << ">";
      return ss.str();
    }

#if 0
  inline auto atan(float const x, float const y) {
    //return static_cast<float>(Magnum::Math::atan(y / x));
    return ext::atan2(y, x);
  }
  inline float acos(float const &value) {
    //return static_cast<float>(Magnum::Math::acos(value));
    return ext::acos(value);
  }

  inline float asin(float const &value) {
    //return static_cast<float>(Magnum::Math::asin(value));
    return ext::asin(value);
  }

  inline float sin(float const &value) {
    return std::sin(value);
  }

  inline float cos(float const &value) {
    return std::cos(value);
  }

  template<class T>
      inline const T& absT(const T& value) {
    return Magnum::Math::abs(value);
  }

  auto const abs = absT<float>;

  template<class T>
      inline const T& maxT(const T& value, const T &maximum) {
    return Magnum::Math::max(value, maximum);
  }

  template<class T>
      inline const T& minT(const T& value, const T &minimum) {
    return Magnum::Math::min(value, minimum);
  }
  auto const max = maxT<float>;
  auto const min = minT<float>;
#endif

  template<class T> inline
      const T clamp(const T& value, const T& min, const T& max) {
    return Magnum::Math::clamp<T>(value, min, max);
    //return ext::fmin(max, ext::fmax(value, min));
  }

  inline auto length(Vector3d const val) {
    return val.length();
  }

  inline auto normalize(Vector3d const val ) {
    return val.normalized();
  }

  inline double distance(Vector3d const &lhs, Vector3d const &rhs) {

    double x1 = lhs.x();
    double y1 = lhs.y();
    double x2 = rhs.x();
    double y2 = rhs.y();

    auto val1 = sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
#ifdef TERRIFIC_DEBUG
    auto val2 = abs(lhs.length() - rhs.length());
    assert(val1 == val2);
#endif

    //auto val1 = 1.f;
    return val1;
  }

  class Ray;
  class AABB;
  class Plane;
  class Point;

  std::vector<Vector3d> splitSphericalLineSegment(const Point& start, const Point& end, double deltaAngle);
  double lagrangeInterpolate(double x, const std::vector<double>& xArray, const std::vector<double>& yArray);
  double interpolateSphericalSamples(const Point& p0, const std::vector<Point>& points, const std::vector<double>& values);
  double computeTriangleArea(const Vector3d& p0, const Vector3d& p1, const Vector3d& p2);

  inline Vector3d getTransformPosition(const Matrix4d& transform) {
    return Vector3d(transform[3][0], transform[3][1], transform[3][2]);
  }

  enum ECubeFace
  {
    CF_POSX = 0,
    CF_FIRST = 0,
    CF_NEGX,
    CF_POSY,
    CF_NEGY,
    CF_POSZ,
    CF_NEGZ,

    CF_MAX,

    CF_POSX_BITMASK = (1 << CF_POSX),
    CF_NEGX_BITMASK = (1 << CF_NEGX),
    CF_POSY_BITMASK = (1 << CF_POSY),
    CF_NEGY_BITMASK = (1 << CF_NEGY),
    CF_POSZ_BITMASK = (1 << CF_POSZ),
    CF_NEGZ_BITMASK = (1 << CF_NEGZ),

    CF_INVALID = 0xFF
  };

  typedef std::bitset<CF_MAX> CubeFaceBitSet;

  void faceAxisDirection(ECubeFace face, Vector3d& s_dir, Vector3d& t_dir, Vector3d& p_dir);

  class Point
  {
 public:
 Point() : theta(0), phi(0) { computePosition(); }

 Point(double theta_, double phi_)
     : theta(theta_), phi(phi_)
    {
      computePosition();
    }

    Point(const Vector3d& direction)
    {
      assignDirection(direction);
    }

    Point(double x, double y, double z)
    {
      assignDirection(Vector3d(x, y, z));
    }


    void calculateCubeSet() {

     }


     //NOTE: Redefinition when using double instead of double
     /*
       Point(const F3& pos)
       : Point(pos.x(), pos.y(), pos.z)
       {
       }
     */

     double theta;
     double phi;
     Vector3d position;

     std::tuple<Vector3d, CubeFaceBitSet> cubeCoord() const;

     Vector3d tangent() const;
     Vector3d binormal() const;

     void assignDirection(const Vector3d& direction)
     {
       double r = length(direction);
       //double r = direction.length();
       assert(r > 0);
       theta = acos(Magnum::Math::clamp<double>(direction.z() / r, -1.0, 1.0));
#ifdef TERRIFIC_DEBUG
       auto theta2 = acos(clamp<double>(direction.z() / r, -1.0, 1.0));
       assert(theta == theta2);
#endif

       phi = atan2(direction.y(), direction.x());
       position = direction / r;
     }

     double sphericalDistance(const Point& p2) const
     {
       double dot = Magnum::Math::dot(position, p2.position);
       double result = acos(clamp<double>(dot, -1.0, 1.0));
       return result;
     }

     bool equalWithEps(const Point& p2, double eps) const
     {
       return std::abs(position.x() - p2.position.x()) < eps &&
           std::abs(position.y() - p2.position.y()) < eps &&
           std::abs(position.z() - p2.position.z()) < eps;
     }

     bool operator < (const Point& p2) const
     {
       return (theta < p2.theta) || (theta == p2.theta && phi < p2.phi);
     }

     friend std::ostream& operator << (std::ostream& stream, const Point& p)
      {
        return stream << p.theta << "," << p.phi;
      }

      template <class Archive>
          void serialize(Archive& ar)
      {
        ar(theta, phi, position.x(), position.y(), position.z());
      }

   private:
      void computePosition()
      {
        position = Vector3d(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
      }
    };

    namespace Util
    {
      extern std::vector<Vector3d> splitSphericalLineSegment(const Point& start, const Point& end, double deltaAngle = M_PI / 180.0);
      extern double lagrangeInterpolate(double x, const std::vector<double>& xArray, const std::vector<double>& yArray);
      extern double interpolateSphericalSamples(const Point& p0, const std::vector<Point>& points, const std::vector<double>& values);
      extern double computeTriangleArea(const Vector3d& p0, const Vector3d& p1, const Vector3d& p2);

      extern void faceAxisDirection(ECubeFace face, Vector3d& s_dir, Vector3d& t_dir, Vector3d& p_dir);

      inline double sqrDistance(Vector2d a, Vector2d b)
      {
        auto p = a - b;
        return dot(p, p);
      }
    }

    class SphericalLine
    {
   public:
   SphericalLine() : direction(Vector3d(0, 0, 1)), xi(0) {}
   SphericalLine(const Vector3d& direction_, double xi_) : direction(normalize(direction_)), xi(xi_) {}

      Vector3d direction;
      double xi;
    };

    class AABB
    {
   public:
      AABB();
      AABB(const Vector3d& p);
      AABB(const Vector3d& min, const Vector3d& max);

      void reset();
      bool isValid() const;
      bool isEmpty() const;

      const Vector3d& min() const { assert(isValid()); return m_min; }
      const Vector3d& max() const { assert(isValid()); return m_max; }

      Vector3d center() const { assert(isValid()); return (m_min + m_max) * 0.5f; }
      Vector3d size() const { assert(isValid()); return m_max - m_min; }
      Vector3d extent() const { return size() * 0.5f; }

      void getMajorVertices(const Vector3d& direction, Vector3d& P, Vector3d& N) const;

      void unionWith(const Vector3d& p);
      void unionWith(const AABB& aabb);
      bool contains(const Vector3d& p) const;
      bool contains(const AABB& aabb) const;

      bool operator == (const AABB& aabb) const;
   private:
      Vector3d m_min, m_max;
    };

    class Ray
    {
   public:
      Ray();
      Ray(const Vector3d& origin, const Vector3d& direction);

      inline const Vector3d& origin() const { return m_origin; }
      inline void setOrigin(const Vector3d& origin) { m_origin = origin; }
      inline const Vector3d& direction() const { return m_direction; }
      inline void setDirection(const Vector3d& direction) { m_direction = normalize(direction); }
      inline void setNormalizedDirection(const Vector3d& direction) { m_direction = direction; }

   private:
      Vector3d m_origin;
      Vector3d m_direction;
    };

    class Plane
    {
   public:
      Plane();
      Plane(const Plane& other);
      Plane(const Vector3d& normal, double distance);
      Plane(const Vector4d& vec);
      Plane(const Vector3d& a, const Vector3d& b, const Vector3d& c);

      const Vector3d& normal() const { return m_normal; }
      void setNormal(const Vector3d& normal) { m_normal = normal; }
      double distance() const { return m_distance; }
      void setDistance(double distance) { m_distance = distance; }

      Plane normalize() const;
      Plane transform(const Matrix4d transform) const;
      double distance(const Vector3d& point) const;
      bool pointOnSide(const Vector3d& point) const;
      bool lineIntersection(const Vector3d& ptA, const Vector3d& ptB, Vector3d& resultDestination) const;

   private:
      Vector3d m_normal;
      double m_distance;
    };

    bool threePlanesIntersection(const Plane& planeA, const Plane& planeB, const Plane& planeC, Vector3d& result);

    bool rayAabbIntersection(const Ray& ray, const AABB& aabb);

    template <typename T>
        class PositionT
    {
   public:
      //using Vec3 = tvec3<T>;
      using Vector3d = Magnum::Math::Vector3<T>;
      PositionT();
      PositionT(ECubeFace face, T s, T t, T p);
      PositionT(ECubeFace face, const Vector3d& stp);

      ECubeFace face() const { return m_face; }
      const Vector3d& surfacePoint() const { return m_surfacePoint; }
      Vector3d stpCoords() const;
      const Vector3d& spacePosition() const { return m_spacePosition; }

   private:
      ECubeFace m_face;
      T m_height;
      Vector3d m_surfacePoint;
      Vector3d m_spacePosition;
    };

    typedef PositionT<float> PositionF;
    typedef PositionT<double> Position;
    typedef PositionT<double> PositionD;
  }
}


#include "Math.hpp"
