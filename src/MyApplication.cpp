#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/Platform/Sdl2Application.h>

#include "MyApplication.h"
#include <Magnum/MeshTools/RemoveDuplicates.h>
#include <Magnum/MeshTools/Duplicate.h>
#include <Magnum/MeshTools/CombineIndexedArrays.h>

#include <Magnum/Primitives/Cube.h>

#include <Corrade/Utility/Debug.h>
#include <Corrade/Utility/Assert.h>

#define SEEDED

using namespace Magnum;

 void MyApplication::CreateColors(std::size_t const count) {
    srand (time(NULL));
    for (size_t i = 0; i < count; i++) {
      _colors.emplace_back(rand() % 200 + 50, rand() % 200 + 55, rand() % 200 + 50);}
    //std::cout << rand() % 200 + 50 << ", " << rand() % 200 + 55 << ", " << rand() % 200 + 50 << "\n";
    //std::cout << rand() % 200 + 50 << ", " << rand() % 200 + 55 << ", " << rand() % 200 + 50 << "\n";
  }

  template<class T>
  Vector3f CalculateSurfaceNormal (Magnum::Math::Vector3<T> const &p1, Magnum::Math::Vector3<T> const &p2, Magnum::Math::Vector3<T> const &p3) {
    Vector3f const u{p2 - p1};
    Vector3f const v{p3 - p1};

    Vector3f n;
    n.x() = u.y() * v.z() - u.z() * v.y();
    n.y() = u.z() * v.x() - u.x() * v.z();
    n.z() = u.x() * v.y() - u.y() * v.x();
    return n;
  }

  using Terrific::Geometry::HalfEdge_ptr;
  using Terrific::Geometry::Cell_ptr;
  using Terrific::Geometry::Vertex_ptr;
  using Terrific::Geometry::Point;

  struct compare_edges {// : public std::binary_function<sv::half_edge_ptr, sv::half_edge_ptr, Vector3d> {
    Vector3d operator() (Vector3d &result, HalfEdge_ptr const &edge) {
      //return result + edge->end->point.position + edge->twin->end->point.position;
      return result + edge->end->point.position;
    }
  };



inline void addVerticesAndNormals(Vector3f const &p1, Vector3f const &p2, Vector3f const &p3, std::vector<Vector3f> &vertices, std::vector<Vector3f> &normals ) {
    vertices.emplace_back(p1);
    vertices.emplace_back(p2);
    vertices.emplace_back(p3);

    std::fill_n(std::back_inserter(normals), 3, CalculateSurfaceNormal(p1, p2, p3));
  }


  template <class T>
  inline Vector2f getUVCoords(Magnum::Math::Vector3<T> const &p) {
    auto const phi = Magnum::Math::atan(p.x() / p.y());
    auto const theta = Magnum::Math::acos(p.normalized().z());

    auto const u = Magnum::Math::sin(theta)*Magnum::Math::cos(phi);
    auto const v = Magnum::Math::sin(theta)*Magnum::Math::sin(phi);
#if 1
    //#ifdef DEBUG
    CORRADE_ASSERT(-1 <= u <= 1, "Error: !(-1 <= u <= 1)", Vector2{});
    CORRADE_ASSERT(-1 <= v <= 1, "Error: !(-1 <= v <= 1)", Vector2{});
    CORRADE_ASSERT(Magnum::Math::pow<2>(u)+Magnum::Math::pow<2>(v) <= 1, "Error: !(u^2 + v^2 <= 1)", Vector2{});
#endif
    return Vector2f{static_cast<float>(u), static_cast<float>(v)};
  }

  inline void addVerticesAndNormals(Vector3d const &p1, Vector3d const &p2, Vector3d const &p3, std::vector<Vector3f> &vertices, std::vector<Vector3f> &normals, std::vector<Vector2> &uvs) {
    vertices.emplace_back(p1);
    vertices.emplace_back(p2);
    vertices.emplace_back(p3);

    std::fill_n(std::back_inserter(normals), 3, CalculateSurfaceNormal(p1, p2, p3));

    uvs.emplace_back(getUVCoords(p1));
    uvs.emplace_back(getUVCoords(p2));
    uvs.emplace_back(getUVCoords(p3));
  }

  void fillVectors(
                   std::vector<HalfEdge_ptr> const &edges,
                   std::vector<Vector3f> &meshVertices,
                   std::vector<Vector3f> &meshNormals,
                   std::vector<UnsignedInt> &meshIndices,
                   std::vector<Color3> &meshColors,
                   std::vector<Vector2> &meshUVs,
                   std::vector<Color3> &colors)
  {
    auto count = UnsignedInt{0};
    for(auto const &e : edges) {

      addVerticesAndNormals(e->pCell->point.position, e->start->point.position, e->end->point.position, meshVertices, meshNormals, meshUVs);

      auto te = e->twin;
#define USE_INDICES
      meshIndices.push_back(count++);
      meshIndices.push_back(count++);
      meshIndices.push_back(count++);

      std::fill_n(std::back_inserter(meshColors),4, colors[e->end->index%255]);

      meshVertices.push_back(static_cast<Vector3f>(te->pCell->point.position));
      meshNormals.push_back(CalculateSurfaceNormal(te->pCell->point.position, te->start->point.position, te->end->point.position));

      meshIndices.push_back(count++);
      meshIndices.push_back(count - 2);
      meshIndices.push_back(count - 3);
    }
    //break;
#define REMOVE_DUPS
#ifdef REMOVE_DUPS
    {
      //#define TEST_CUBE
#ifdef TEST_CUBE
      {
        const Trade::MeshData3D cube = Primitives::cubeSolid();
        meshIndices = cube.indices();
        meshNormals = cube.normals(0);
        meshVertices = cube.positions(0);
        meshColors.clear();
        UnsignedInt i{0};
        for(auto const &v : meshVertices) {
          meshColors.push_back(colors[i++]);
        }
        CORRADE_ASSERT(meshIndices.size() == 36,"TEST_CUBE failed",);
        CORRADE_ASSERT(meshNormals.size() == 24,"TEST_CUBE failed",);
        CORRADE_ASSERT(meshVertices.size() == 24,"TEST_CUBE failed",);
        CORRADE_ASSERT(meshColors.size() == 24,"TEST_CUBE failed",);
      }
#endif
#if 1
      //TERRIFIC_INFO("\t Indices: {0}", meshIndices.size());
      //TERRIFIC_INFO("\t Normals: {0}", meshNormals.size());
      //TERRIFIC_INFO("\t Vertices: {0}", meshVertices.size());
      //TERRIFIC_INFO("\t Colors: {0}" , meshColors.size());
#endif

#if 1

      //      auto meshVerticesIndices = MeshTools::removeDuplicates(meshVertices);
      //auto meshVerticesIndices = MeshTools::duplicate( meshIndices, MeshTools::removeDuplicates(meshVertices));
      meshVertices = MeshTools::duplicate(meshIndices, meshVertices);
      meshNormals = MeshTools::duplicate(meshIndices, meshNormals);
      meshColors = MeshTools::duplicate(meshIndices, meshColors);

      auto meshVerticesIndices = MeshTools::removeDuplicates(meshVertices, .1f);
      auto meshNormalsIndices = MeshTools::removeDuplicates(meshNormals);
      auto meshColorsIndices = MeshTools::removeDuplicates(meshColors);

#if 0
      TERRIFIC_CORE("\tVerticesIndices Min: {0}", meshVerticesIndices.size());
      TERRIFIC_CORE("\tNormalIndices Min: {0}", meshNormalsIndices.size());
      TERRIFIC_CORE("\tColorIndices Min: {0}", meshColors.size());
#endif

#ifdef TEST_CUBE
      CORRADE_ASSERT(meshVerticesIndices.size() == 36,"TEST_CUBE failed",);
      CORRADE_ASSERT(meshNormalsIndices.size() == 36,"TEST_CUBE failed",);
#endif


      //std::vector<UnsignedInt>
      //      meshIndices.clear();
      meshIndices =
        //MeshTools::duplicate(meshIndices,
        MeshTools::combineIndexedArrays(
                                        std::make_pair(std::cref(meshVerticesIndices), std::ref(meshVertices)),
                                        std::make_pair(std::cref(meshNormalsIndices), std::ref(meshNormals)),
                                        std::make_pair(std::cref(meshColorsIndices), std::ref(meshColors))
                                        //)
                                        );

#if 0
      TERRIFIC_CORE("\t Indices: {0}", meshIndices.size());
      TERRIFIC_CORE("\t Normals: {0}", meshNormals.size());
      TERRIFIC_CORE("\t Vertices: {0}", meshVertices.size());
      TERRIFIC_CORE("\t Colors: {0}", meshColors.size());
      TERRIFIC_CORE("\t Colors Indices: {0}", meshColorsIndices.size());
#endif

      //meshIndices = Magnum::MeshTools::removeDuplicates(meshVertices);
      //std::cout << meshIndices.size() << std::endl;
#endif
    }
#endif

  }

MyApplication::MyApplication(const Arguments& arguments): Platform::Application{arguments} {
	using namespace Terrific::Geometry;
	auto points = SphericalVoronoi::generatePoints(180);
	auto sphere = SphericalVoronoi{points, true};
    CreateColors(256);
	sphere.solve();
    auto cells = sphere.getCells();
    auto vertices = sphere.getVertices();
    auto edges = sphere.getHalfEdges();



      using namespace Math::Literals;
  GL::Renderer::enable(GL::Renderer::Feature::DepthTest);
  GL::Renderer::enable(GL::Renderer::Feature::FaceCulling);
  //GL::Renderer::setFrontFace(GL::Renderer::FrontFace::ClockWise);
  {
    _pCameraObject = new Object3D{&_scene};
    _pCameraObject->translate(Vector3f::zAxis(5.0f));
    _pCamera = new SceneGraph::Camera3D(*_pCameraObject);

    _viewMatrix = (Matrix4::lookAt({0.f, 0.f, 5.f}, {0.f, 0.f, 0.f}, Vector3f::zAxis(1.f)));

    _pCamera->setProjectionMatrix(
        Matrix4::perspectiveProjection(
            60.0_degf,
            Vector2{GL::defaultFramebuffer.viewport().size()}.aspectRatio(),
            0.001f,
            100.0f))
        .setAspectRatioPolicy(SceneGraph::AspectRatioPolicy::Extend)
        .setViewport(GL::defaultFramebuffer.viewport().size());

    _pManipulator = new Object3D{&_scene};

    (*_pManipulator)
        .scale(Vector3f{2.0f, 2.0f,2.0f});
#if 0
    (*_pManipulator)
        .translate(Vector3f::yAxis(-0.3f))
        .rotateX(Math::Rad<float>(30.f));
    //.rotateX(30.0_degf);
#endif

    std::vector<Vector3f> meshVertices;
    std::vector<UnsignedInt> meshIndices;
    std::vector<Vector3ui> meshFaces;

    std::vector<Vector3f> meshNormals;
    std::vector<Color3> meshColors;
    std::vector<Vector2> meshUVs;

    fillVectors(
                edges,
                meshVertices,
                meshNormals,
                meshIndices,
                meshColors,
                meshUVs,
                _colors);



    _pSphere = new Terrific::SphereDrawable{*_pManipulator, &_drawables,
                                            meshVertices,
                                            meshNormals,
                                            meshIndices,
                                            meshColors};
   
  }

}

void MyApplication::drawEvent() {
  GL::defaultFramebuffer.clear(GL::FramebufferClear::Color | GL::FramebufferClear::Depth);

    /* TODO: Add your drawing code here */
    _pCamera->draw(_drawables);
    swapBuffers();
    redraw();
}

Vector3f MyApplication::positionOnSphere(const Vector2i& position) const {
  const Vector2 positionNormalized = Vector2{position*2.f}/Vector2{_pCamera->viewport()} - Vector2{1.f};
  //const Vector2 positionNormalized = Vector2{position}/Vector2{_pCamera->viewport()} - Vector2{0.5f};
  const Float length = positionNormalized.length();
  Vector3f const result(length > 1.0f
                        ? Vector3f(positionNormalized, 0.0f)
                        : Vector3f(positionNormalized, 1.0f - length));
  //result.y() *= -1.0f;
  return (result*Vector3f::yScale(-1.0f)).normalized();
  //result = Vector3f(result * Vector3f::yScale(-1.0f));
  //return result.normalized();
}

#ifndef CORRADE_TARGET_ANDROID
void MyApplication::viewportEvent(ViewportEvent &event) {
  GL::defaultFramebuffer.setViewport({{}, event.windowSize()});
  _pCamera->setViewport(event.windowSize());
}

void MyApplication::mouseScrollEvent(MouseScrollEvent& event) {
  if(!event.offset().y()) return;

  /* Distance to origin */
  Float distance = _pCameraObject->transformation().translation().z();

  /* Move 15% of the distance back or forward */
  distance *= 1 - (event.offset().y() > 0 ? 1/0.85f : 0.85f);
  _pCameraObject->translate(Vector3f::zAxis(distance));

  redraw();
}

void MyApplication::keyPressEvent(KeyEvent &event) {
  if(event.key() == KeyEvent::Key::Q)
    exit();
}
#endif

void MyApplication::mousePressEvent(MouseEvent& event) {
  if(event.button() == MouseEvent::Button::Left)
    _previousPosition = positionOnSphere(event.position());  
  event.setAccepted();
}

void MyApplication::mouseReleaseEvent(MouseEvent& event) {
  if(event.button() == MouseEvent::Button::Left)
    _previousPosition = Vector3f{0};
  event.setAccepted();
}
void MyApplication::mouseMoveEvent(MouseMoveEvent& event) {
  if(!(event.buttons() & MouseMoveEvent::Button::Left)) return;
  if (_previousPosition.isZero()) return;

  const Vector3f currentPosition = positionOnSphere(event.position());
  const Vector3f axis = Math::cross(_previousPosition, currentPosition);

  if(_previousPosition.length() < 0.001f || axis.length() < 0.001f) return;

  _pManipulator->rotate(Math::angle(_previousPosition, currentPosition), axis.normalized());
  _previousPosition = currentPosition;

  event.setAccepted();
  redraw();
}

MAGNUM_APPLICATION_MAIN(MyApplication)
