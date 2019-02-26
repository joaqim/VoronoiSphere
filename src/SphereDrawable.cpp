#include "SphereDrawable.h"
#include <iostream>
namespace Terrific {
using namespace Magnum;

SphereDrawable::SphereDrawable(Object3D& object,
                               SceneGraph::DrawableGroup3D* group,
                               std::vector<Vector3> const &vertices,
                               std::vector<Vector3> const &normals,
                               std::vector<UnsignedInt> const &indices,
                               std::vector<Color3> const &colors
                               ): SceneGraph::Drawable3D{object, group}
{
  using namespace Magnum::Math::Literals;
#if 0
  _phongShader
      .setAmbientColor(0x111111_rgbf)
      .setSpecularColor(0xffffff_rgbf)
      .setShininess(80.0f);
  _textureShader
      .setAmbientColor(0x111111_rgbf)
      .setSpecularColor(0x111111_rgbf)
      .setShininess(80.0f);
#endif

  _vertexBuffer.setData(vertices, GL::BufferUsage::StaticDraw);
  _normalsBuffer.setData(normals, GL::BufferUsage::StaticDraw);
  _indexBuffer.setData(indices, GL::BufferUsage::StaticDraw);

  _mesh.setIndexBuffer(_indexBuffer, 0, MeshIndexType::UnsignedInt, 0, _indexBuffer.size());

  auto verticesCount = indices.size();
  {
    Containers::Array<char> indexData;
    MeshIndexType indexType;
    UnsignedInt indexStart, indexEnd;
    if(indices.size() > 0) {
      std::tie(indexData, indexType, indexStart, indexEnd) = MeshTools::compressIndices(indices);
      std::cout << "Index size:  \t" << indices.size() << std::endl;
      std::cout << "Index Data size: "<< indexData.size() << std::endl;
      _indexBuffer.setData(indexData, GL::BufferUsage::StaticDraw);
      _mesh.setIndexBuffer(_indexBuffer, 0, indexType, indexStart, indexEnd);
    }
#if defined(CORRADE_TARGET_EMSCRIPTEN)
    // Use gl_PointSize builtin vertex shader variable in OpenGL ES and WebGL instead.
#else
    GL::Renderer::setPointSize(5.f);
#endif

    _mesh
      .setPrimitive( MeshPrimitive::Triangles )
       //.setPrimitive( MeshPrimitive::Lines )
        .addVertexBuffer(_normalsBuffer, 0,  Shaders::Phong::Normal{})
        .addVertexBuffer(_vertexBuffer, 0, Shaders::Phong::Position{});


    _mesh.setCount(verticesCount);
  }
}

void SphereDrawable::draw(const Matrix4& transformationMatrix, SceneGraph::Camera3D &camera) {
  using namespace Magnum::Math::Literals;
#if 1
  _phongShader
      .setDiffuseColor(0xa5c9ea_rgbf)
      .setLightPosition(camera.cameraMatrix().transformPoint({-17.0f, 17.0f, 17.0f}))
      .setTransformationMatrix(transformationMatrix)
      .setNormalMatrix(transformationMatrix.rotationScaling())
      .setProjectionMatrix(camera.projectionMatrix());

  _mesh.draw(_phongShader);
#else
  _textureShader
      .bindDiffuseTexture(_texture)
      .setLightPosition(camera.cameraMatrix().transformPoint({-17.0f, 17.0f, 17.0f}))
      .setTransformationMatrix(transformationMatrix)
      .setNormalMatrix(transformationMatrix.rotationScaling())
      .setProjectionMatrix(camera.projectionMatrix());
#endif
}

}


