#pragma once

#include <Magnum/Math/Vector3.h>
#include "Math/Types.h"

#include <memory> // shared_ptr
#include <vector> // vector
#include <set>    // set
#include <ostream>// ostream
#include <algorithm> // find
#include <functional> // function

#include "Math/Math.h"

namespace Terrific {
  namespace Geometry {


    class HalfEdge;
    class Cell;
    class Vertex;
    class BeachArc;
    class SiteEvent;
    class CircleEvent;

    typedef std::shared_ptr<Vertex> Vertex_ptr;
    typedef std::shared_ptr<HalfEdge> HalfEdge_ptr;
    typedef std::shared_ptr<Cell> Cell_ptr;
    typedef std::shared_ptr<BeachArc> BeachArc_ptr;
    typedef std::vector<BeachArc_ptr> beach_type;
    typedef std::shared_ptr<CircleEvent> CircleEvent_ptr;


    typedef Math::Point Point;

    class Cell {
   public:
      Cell (uint32_t i, Point const &p);
      uint32_t index;
      Point point;

      std::vector<HalfEdge_ptr> halfEdges;

      void reset();
      //friend std::ostream& operator<< (std::ostream stream, Cell const& c);

    };

    class Vertex {
   public:
      Vertex(Point const &p, Cell_ptr c0, Cell_ptr c1);
      Vertex(Point const &p, Cell_ptr c0, Cell_ptr c1, Cell_ptr c2);

      uint32_t index;
      Point point;
      std::vector<HalfEdge_ptr> halfEdges;
      std::set<Cell_ptr> cells;

      void reset();

      //friend std::ostream& operator<< (std::ostream& stream, Vertex const &v);
      friend std::ostream& operator<< (std::ostream& stream, Vertex const &v) {
        stream << "point(" << v.point << ")";
        stream << " Cells<";
        for (auto c : v.cells)
        {
          stream << c->index << ",";
        }
        stream << ">";
        return stream;
      }

    };


    class HalfEdge {
   public:
      HalfEdge(std::shared_ptr<Vertex> s, std::shared_ptr<Vertex> e);
      uint32_t index;
      Cell_ptr pCell;
      Cell_ptr otherCell;
      Vertex_ptr start;
      Vertex_ptr end;
      HalfEdge_ptr prev;
      HalfEdge_ptr next;
      HalfEdge_ptr twin;

      void reset();

      friend std::ostream& operator<< (std::ostream& stream, const HalfEdge& e) {
        stream << "s: " << *e.start << "e: " << *e.end;
        return stream;
      }
    };

    class BeachArc {
   public:
      BeachArc(std::shared_ptr<Cell> Cell_);

      std::shared_ptr<Cell> pCell;

      std::shared_ptr<CircleEvent> circleEvent;      // the related circle event

      std::shared_ptr<Vertex> startVertex;

      bool operator< (const BeachArc& ba) const;

      friend std::ostream& operator<< (std::ostream& stream, const BeachArc& arc) {
        stream << "Cell " << arc.pCell;
        if (arc.startVertex) {
          stream << "startVertex " << *arc.startVertex;
        }
        else {
          stream << "startVertex NONE";
        }
        return stream;
      }
      //friend std::ostream& operator<< (std::ostream& stream, const BeachArc& arc);
    };


    class SiteEvent {
   public:
      SiteEvent(std::shared_ptr<Cell> Cell_);

      std::shared_ptr<Cell> pCell;
      double theta;
      double phi;

      bool operator< (const SiteEvent& right) const;
      bool operator>=(const SiteEvent& right) const;

      //friend std::ostream& operator<< (std::ostream& stream, const SiteEvent& e);
      friend std::ostream& operator<< (std::ostream& stream, const SiteEvent& e) {
        return stream << e.pCell;
      }
    };

    class CircleEvent {
   public:
      CircleEvent(const BeachArc_ptr& arc_i_, const BeachArc_ptr& arc_j_, const BeachArc_ptr& arc_k_);

      BeachArc_ptr arc_i;
      BeachArc_ptr arc_j;
      BeachArc_ptr arc_k;

      Cell_ptr cell_i() const { return arc_i->pCell; }
      Cell_ptr cell_j() const { return arc_j->pCell; }
      Cell_ptr cell_k() const { return arc_k->pCell; }

      Point circle_center;
      double circle_radius;

      double theta;        // the lowest point on circle

      bool operator< (const CircleEvent& ce) const;

      friend std::ostream& operator<< (std::ostream& stream, const CircleEvent& e) {
        stream << "[" << e.cell_i()->index << "," << e.cell_j()->index << "," << e.cell_k()->index << "] " << "theta " << e.theta;
        return stream;
      }
    };

    struct compare_CircleEvent_priority {
      bool operator()(const std::shared_ptr<CircleEvent>& left, const std::shared_ptr<CircleEvent>& right) const;
    };

    class SphericalVoronoi {
   public:
      SphericalVoronoi(std::vector<Vector3d> const &directions, bool const debugMode_=false);
      SphericalVoronoi(bool const debugMode_=false);
      void initialize(std::vector<Vector3d> const &directions);

      static std::vector<Vector3d> generatePoints(std::size_t const count);
      void setDebugMode(bool debugMode);

      bool isFinished() const;
      bool isInitialized() const;
      void step(double maxDeltaXi);
      void solve(std::function<void(int)> cb = nullptr);       // step until finished

      const std::vector<HalfEdge_ptr>& getHalfEdges() const { return halfEdges; }
      const std::vector<Vertex_ptr>& getVertices() const { return vertices; }
      const std::vector<Cell_ptr>& getCells() const { return cells; }

      const std::vector<HalfEdge_ptr>& cloneHalfEdges() const;
      const std::vector<Vertex_ptr>& cloneVertices() const;
      const std::vector<Cell_ptr>& cloneCells() const;

      friend std::ostream& operator<< (std::ostream stream, Cell const& c) {
        return stream << "<" << c.index << "> " << "(" << c.point << ")";
      }

   protected:

      void dumpBeachState(std::ostream& stream);

      void finializeGraph();
      void cleanupMiddleVertices();
      void duplicateHalfEdges();
      void bindHalfEdgesToCells();

      beach_type::const_iterator getPrevArcOnBeach(beach_type::const_iterator it) const;

      beach_type::const_iterator getNextArcOnBeach(beach_type::const_iterator it) const;

      bool intersectWithNextArc(beach_type::const_iterator itArc, double xi, Point& oPoint) const;
      bool intersectWithPrevArc(beach_type::const_iterator itArc, double xi, Point& oPoint) const;
      void handleSiteEvent(SiteEvent& event);
      void handleCircleEvent(const CircleEvent_ptr& event);

      static Point thetaToPoint(double theta, bool positive, double xi, double theta1, double phi1);
      static Point phiToPoint(double phi, double xi, double theta1, double phi1);
      static bool arcsIntersection(const BeachArc& arc1, const BeachArc& arc2, double xi, Point& oPoint);
      static bool comparePhi(double const lhs, double const rhs);

      Math::SphericalLine scanLine;
      int nbSteps;
      bool debugMode;
      bool initialized{false};

      constexpr static double eps = 1e-5;

      std::vector<HalfEdge_ptr> halfEdges;
      std::vector<Vertex_ptr> vertices;
      std::vector<Cell_ptr> cells;

      beach_type beach;

      bool isArcOnBeach(const BeachArc_ptr& arc) const;

      std::vector<SiteEvent> siteEventQueue;
      std::vector<CircleEvent_ptr> circleEventQueue;
      std::set<std::weak_ptr<CircleEvent>, std::owner_less<std::weak_ptr<CircleEvent>>> circleEventDeletedEvents;

      void addNewSiteEvent(const SiteEvent& event);

      void addNewCircleEvent(const std::shared_ptr<CircleEvent>& event);

      void removeCircleEvent(const std::shared_ptr<CircleEvent>& event);

      std::shared_ptr<CircleEvent> getCircleEvent();

      void popCircleEvent();
      size_t getCircleEventQueueSize() const;
    };
  }
}
