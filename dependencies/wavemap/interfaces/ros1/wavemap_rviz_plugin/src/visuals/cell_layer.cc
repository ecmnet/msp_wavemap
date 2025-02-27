#include "wavemap_rviz_plugin/visuals/cell_layer.h"

#include <algorithm>
#include <memory>
#include <sstream>
#include <vector>

#include <OgreCamera.h>
#include <OgreMaterialManager.h>
#include <OgreRoot.h>
#include <OgreTechnique.h>
#include <OgreTexture.h>
#include <glog/logging.h>
#include <qglobal.h>
#include <ros/console.h>
#include <rviz/ogre_helpers/compatibility.h>
#include <rviz/ogre_helpers/custom_parameter_indices.h>

namespace wavemap {
static constexpr float kPointVertices[3] = {0.0f, 0.0f, 0.0f};

static constexpr float kBoxVertices[6 * 6 * 3] = {
    // clang-format off
    // front
    -0.5f, 0.5f, -0.5f, -0.5f, -0.5f, -0.5f, 0.5f, 0.5f, -0.5f, 0.5f, 0.5f, -0.5f, -0.5f, -0.5f, -0.5f,
    0.5f, -0.5f, -0.5f,

    // back
    -0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, -0.5f, -0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, -0.5f, 0.5f, -0.5f, -0.5f, 0.5f,

    // right
    0.5, 0.5, 0.5, 0.5, 0.5, -0.5, 0.5, -0.5, 0.5, 0.5, 0.5, -0.5, 0.5, -0.5, -0.5, 0.5, -0.5, 0.5,

    // left
    -0.5, 0.5, 0.5, -0.5, -0.5, 0.5, -0.5, 0.5, -0.5, -0.5, 0.5, -0.5, -0.5, -0.5, 0.5, -0.5, -0.5, -0.5,

    // top
    -0.5, 0.5, -0.5, 0.5, 0.5, -0.5, -0.5, 0.5, 0.5, 0.5, 0.5, -0.5, 0.5, 0.5, 0.5, -0.5, 0.5, 0.5,

    // bottom
    -0.5, -0.5, -0.5, -0.5, -0.5, 0.5, 0.5, -0.5, -0.5, 0.5, -0.5, -0.5, -0.5, -0.5, 0.5, 0.5, -0.5, 0.5,
};  // clang-format on

const Ogre::String CellLayer::movable_type_name_ = "CellLayer";

CellLayer::CellLayer(const Ogre::MaterialPtr& cell_material)
    : cell_material_(cell_material) {
  // Detect geometry shader support
  current_mode_supports_geometry_shader_ = false;
  if (Ogre::Technique* best = cell_material_->getBestTechnique(); best) {
    if (cell_material_->getBestTechnique()->getName() == "gp") {
      current_mode_supports_geometry_shader_ = true;
    }
  } else {
    ROS_ERROR_STREAM("No techniques available for material "
                     << cell_material_->getName());
  }
}

void CellLayer::clear() {
  bounding_box_.setNull();
  bounding_radius_ = 0.0f;

  if (getParentSceneNode()) {
    for (const auto& renderable : renderables_) {
      getParentSceneNode()->detachObject(renderable.get());
    }
    getParentSceneNode()->needUpdate();
  }

  renderables_.clear();
}

void CellLayer::setCellDimensions(float width, float height, float depth) {
  width_ = width;
  height_ = height;
  depth_ = depth;

  const Ogre::Vector4 size(width_, height_, depth_, 0.0f);
  for (const auto& renderable : renderables_) {
    renderable->setCustomParameter(SIZE_PARAMETER, size);
  }
}

void CellLayer::setAlpha(float alpha) {
  alpha_ = alpha;

  const Ogre::Vector4 alpha4(alpha_, alpha_, alpha_, alpha_);
  for (const auto& renderable : renderables_) {
    renderable->setCustomParameter(ALPHA_PARAMETER, alpha4);
  }
}

void CellLayer::setCells(const std::vector<Cell>& cells) {
  if (!renderables_.empty()) {
    clear();
  }
  if (cells.empty()) {
    return;
  }

  Ogre::Root* root = Ogre::Root::getSingletonPtr();

  uint32_t vpp = getVerticesPerCell();
  Ogre::RenderOperation::OperationType op_type;
  if (current_mode_supports_geometry_shader_) {
    op_type = Ogre::RenderOperation::OT_POINT_LIST;
  } else {
    op_type = Ogre::RenderOperation::OT_TRIANGLE_LIST;
  }

  const float* vertices = nullptr;
  if (current_mode_supports_geometry_shader_) {
    vertices = kPointVertices;
  } else {
    vertices = kBoxVertices;
  }

  CellLayerRenderablePtr renderable;
  Ogre::HardwareVertexBufferSharedPtr vbuf;
  void* vdata = nullptr;
  Ogre::RenderOperation* op = nullptr;
  float* fptr = nullptr;

  Ogre::AxisAlignedBox aabb;
  aabb.setNull();
  uint32_t current_vertex_count = 0;
  uint32_t buffer_size = 0;
  bounding_box_.setNull();
  for (size_t current_cell = 0; current_cell < cells.size(); ++current_cell) {
    // if we didn't create a renderable yet,
    // or we've reached the vertex limit for the current renderable,
    // create a new one.
    while (!renderable || current_vertex_count >= buffer_size) {
      if (renderable) {
        CHECK_EQ(current_vertex_count, buffer_size);

        op->vertexData->vertexCount =
            renderable->getBuffer()->getNumVertices() -
            op->vertexData->vertexStart;
        CHECK_LE(op->vertexData->vertexCount + op->vertexData->vertexStart,
                 renderable->getBuffer()->getNumVertices());
        vbuf->unlock();
        renderable->setBoundingBox(aabb);
        renderable->setBoundingRadius(
            Ogre::Math::Sqrt(std::max(aabb.getMinimum().squaredLength(),
                                      aabb.getMaximum().squaredLength())));
        bounding_box_.merge(aabb);
      }

      constexpr size_t kVectorBufferCapacity = 36 * 1024 * 10;
      buffer_size =
          std::min(kVectorBufferCapacity, (cells.size() - current_cell) * vpp);

      renderable = createRenderable(buffer_size);
      vbuf = renderable->getBuffer();
      vdata = vbuf->lock(Ogre::HardwareBuffer::HBL_NO_OVERWRITE);

      op = renderable->getRenderOperation();
      op->operationType = op_type;
      current_vertex_count = 0;

      fptr = reinterpret_cast<float*>(vdata);

      aabb.setNull();
    }

    const Cell& p = cells[current_cell];
    uint32_t color;
    root->convertColourValue(p.color, &color);

    aabb.merge(p.center);

    float x = p.center.x;
    float y = p.center.y;
    float z = p.center.z;

    for (uint32_t j = 0; j < vpp; ++j, ++current_vertex_count) {
      *fptr++ = x;
      *fptr++ = y;
      *fptr++ = z;

      if (!current_mode_supports_geometry_shader_) {
        *fptr++ = vertices[(j * 3)];
        *fptr++ = vertices[(j * 3) + 1];
        *fptr++ = vertices[(j * 3) + 2];
      }
      std::memcpy(fptr++, &color, sizeof(float));
    }
  }

  op->vertexData->vertexCount =
      current_vertex_count - op->vertexData->vertexStart;
  renderable->setBoundingBox(aabb);
  bounding_box_.merge(aabb);
  bounding_radius_ =
      Ogre::Math::Sqrt(std::max(bounding_box_.getMinimum().squaredLength(),
                                bounding_box_.getMaximum().squaredLength()));
  CHECK_LE(op->vertexData->vertexCount + op->vertexData->vertexStart,
           renderable->getBuffer()->getNumVertices());

  vbuf->unlock();

  shrinkRenderables();

  if (getParentSceneNode()) {
    getParentSceneNode()->needUpdate();
  }
}

void CellLayer::shrinkRenderables() {
  while (!renderables_.empty()) {
    CellLayerRenderable& renderable = *renderables_.back();
    Ogre::RenderOperation* op = renderable.getRenderOperation();
    if (op->vertexData->vertexCount == 0) {
      renderables_.pop_back();
    } else {
      break;
    }
  }
}

void CellLayer::_updateRenderQueue(Ogre::RenderQueue* queue) {
  for (const auto& renderable : renderables_) {
    queue->addRenderable(renderable.get());
  }
}

uint32_t CellLayer::getVerticesPerCell() const {
  if (current_mode_supports_geometry_shader_) {
    return 1;
  } else {
    return 36;
  }
}

CellLayerRenderablePtr CellLayer::createRenderable(size_t num_cells) {
  CellLayerRenderablePtr renderable = std::make_shared<CellLayerRenderable>(
      this, num_cells, !current_mode_supports_geometry_shader_);
  rviz::setMaterial(*renderable, cell_material_);
  const Ogre::Vector4 size(width_, height_, depth_, 0.0f);
  const Ogre::Vector4 alpha(alpha_, 0.0f, 0.0f, 0.0f);
  renderable->setCustomParameter(SIZE_PARAMETER, size);
  renderable->setCustomParameter(ALPHA_PARAMETER, alpha);
  renderable->setCustomParameter(NORMAL_PARAMETER,
                                 Ogre::Vector4(Ogre::Vector3::NEGATIVE_UNIT_Z));
  renderable->setCustomParameter(UP_PARAMETER,
                                 Ogre::Vector4(Ogre::Vector3::UNIT_Y));
  if (getParentSceneNode()) {
    getParentSceneNode()->attachObject(renderable.get());
  }
  renderables_.emplace_back(renderable);

  return renderable;
}

CellLayerRenderable::CellLayerRenderable(CellLayer* parent, size_t num_cells,
                                         bool use_tex_coords)
    : parent_(parent) {
  // Initialize render operation
  mRenderOp.operationType = Ogre::RenderOperation::OT_POINT_LIST;
  mRenderOp.useIndexes = false;
  mRenderOp.vertexData = new Ogre::VertexData;
  mRenderOp.vertexData->vertexStart = 0;
  mRenderOp.vertexData->vertexCount = 0;

  Ogre::VertexDeclaration* decl = mRenderOp.vertexData->vertexDeclaration;
  size_t offset = 0;

  decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
  offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

  if (use_tex_coords) {
    decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_TEXTURE_COORDINATES,
                     0);
    offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
  }

  decl->addElement(0, offset, Ogre::VET_COLOUR, Ogre::VES_DIFFUSE);

  Ogre::HardwareVertexBufferSharedPtr vbuf =
      Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
          mRenderOp.vertexData->vertexDeclaration->getVertexSize(0), num_cells,
          Ogre::HardwareBuffer::HBU_DYNAMIC);

  // Bind buffer
  mRenderOp.vertexData->vertexBufferBinding->setBinding(0, vbuf);
}

CellLayerRenderable::~CellLayerRenderable() {
  delete mRenderOp.vertexData;
  delete mRenderOp.indexData;
}

Ogre::HardwareVertexBufferSharedPtr CellLayerRenderable::getBuffer() {
  return mRenderOp.vertexData->vertexBufferBinding->getBuffer(0);
}

Ogre::Real CellLayerRenderable::getSquaredViewDepth(
    const Ogre::Camera* cam) const {
  const Ogre::Vector3& min = mBox.getMinimum();
  const Ogre::Vector3& max = mBox.getMaximum();
  const Ogre::Vector3 mid = ((max - min) * 0.5f) + min;
  const Ogre::Vector3 offset = cam->getDerivedPosition() - mid;

  return offset.squaredLength();
}
}  // namespace wavemap
