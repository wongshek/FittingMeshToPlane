#pragma once

// Basic //
#include <fstream>
#include <iostream>
// Basic //

// Eigen //
#include <Eigen/Dense>
using Vec2d = Eigen::Vector2d;
using Vec3d = Eigen::Vector3d;
using Vec4d = Eigen::Vector4d;
using Mat3d = Eigen::Matrix3d;
// Eigen //

// Open Mesh //
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Utils/Property.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>

struct MyTraits : public OpenMesh::DefaultTraits
{
	typedef OpenMesh::Vec3d Point;
	typedef OpenMesh::Vec3d Normal;
};

using MeshT = OpenMesh::TriMesh_ArrayKernelT<MyTraits>;
using MeshP = OpenMesh::PolyMesh_ArrayKernelT<MyTraits>;

using Face = OpenMesh::SmartFaceHandle;
using Vertex = OpenMesh::SmartVertexHandle;

using FaceHandles = std::vector<Face>;
using VertexHandles = std::vector<Vertex>;
// Open Mesh //
