#include "Basic.h"

void ReadMesh(const std::string& path, MeshT* mesh)
{
	if (!OpenMesh::IO::read_mesh(*mesh, path))
	{
		std::cout << "Fail to read mesh!" << std::endl;
	}

	mesh->request_face_normals();
	mesh->request_vertex_normals();
	mesh->update_normals();
}

Vec3d GetCenter(const MeshT& mesh)
{
	double sum_x = 0;
	double sum_y = 0;
	double sum_z = 0;

	for (const auto& v_h : mesh.vertices())
	{
		sum_x += mesh.point(v_h)[0];
		sum_y += mesh.point(v_h)[1];
		sum_z += mesh.point(v_h)[2];
	}

	return Vec3d(sum_x / mesh.n_vertices(), sum_y / mesh.n_vertices(), sum_z / mesh.n_vertices());
}

Vec3d GetMidNormal(const MeshT& facade)
{
	std::vector<double> norm_set_x;
	std::vector<double> norm_set_y;
	std::vector<double> norm_set_z;

	norm_set_x.reserve(facade.n_faces());
	norm_set_y.reserve(facade.n_faces());
	norm_set_z.reserve(facade.n_faces());

	for (const auto& f_h : facade.faces())
	{
		norm_set_x.emplace_back(facade.normal(f_h)[0]);
		norm_set_y.emplace_back(facade.normal(f_h)[1]);
		norm_set_z.emplace_back(facade.normal(f_h)[2]);
	}

	double median_norm_x, median_norm_y, median_norm_z;
	std::nth_element(norm_set_x.begin(), norm_set_x.begin() + norm_set_x.size() / 2, norm_set_x.end());
	std::nth_element(norm_set_y.begin(), norm_set_y.begin() + norm_set_y.size() / 2, norm_set_y.end());
	std::nth_element(norm_set_z.begin(), norm_set_z.begin() + norm_set_z.size() / 2, norm_set_z.end());

	median_norm_x = norm_set_x[norm_set_x.size() / 2];
	median_norm_y = norm_set_y[norm_set_y.size() / 2];
	median_norm_z = norm_set_z[norm_set_z.size() / 2];

	return Vec3d(median_norm_x, median_norm_y, median_norm_z);
}

inline double CalNormalAngle(const Vec3d& n1, const Vec3d& n2)
{
	auto nn1 = n1.normalized();
	auto nn2 = n2.normalized();
	double dot = nn1.dot(nn2);
	dot = std::min(1.0, std::max(-1.0, dot));

	return acos(dot) * 180 / M_PI;
}

template <typename T>
inline bool CalNormalAngle(const Vec3d& n1, const Vec3d& n2, const T ang_thres)
{
	return CalNormalAngle(n1, n2) < ang_thres;

}

// Assume ground normal is (0 0 1) and facade is ortho to ground
void FittingFacade(const MeshT& facade, Vec4d* plane)
{
	Eigen::MatrixXd A;
	A.resize(facade.n_vertices(), 2);

	const Vec3d facade_center = GetCenter(facade);

	int i = 0;
	for (const auto& v_h : facade.vertices())
	{
		const auto& pt = facade.point(v_h);
		A.row(i) = Vec2d(pt[0] - facade_center[0], pt[1] - facade_center[1]);
		i++;
	}

	Eigen::Matrix2d covar_matrix = A.transpose() * A;
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver;
	eigensolver.computeDirect(covar_matrix);

	auto& eigenvalues = eigensolver.eigenvalues();
	auto& eigenvectors = eigensolver.eigenvectors();

	Vec3d plane_normal(eigenvectors(0, 0), eigenvectors(0, 1), 0);

	const Vec3d facade_normal = GetMidNormal(facade);

	// Check if plane reverse
	if (!CalNormalAngle(facade_normal, plane_normal, 90))
	{
		plane_normal = -plane_normal;
	}

	double d = -plane_normal.transpose() * facade_center;
	*plane = Vec4d(plane_normal[0], plane_normal[1], plane_normal[2], d);
}

void FittingPlane(const MeshT& facade, Vec4d* plane)
{
	Eigen::MatrixXd A;
	A.resize(facade.n_vertices(), 3);

	const Vec3d facade_center = GetCenter(facade);

	int i = 0;
	for (const auto& v_h : facade.vertices())
	{
		const auto& pt = facade.point(v_h);
		A.row(i) = Vec3d(pt[0] - facade_center[0], pt[1] - facade_center[1], pt[2] - facade_center[2]);
		i++;
	}

	Eigen::Matrix3d covar_matrix = A.transpose() * A;
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver;
	eigensolver.computeDirect(covar_matrix);

	auto& eigenvalues = eigensolver.eigenvalues();
	auto& eigenvectors = eigensolver.eigenvectors();

	Vec3d plane_normal(eigenvectors(0, 0), eigenvectors(0, 1), eigenvectors(0, 2));

	const Vec3d facade_normal = GetMidNormal(facade);

	// Check if plane reverse
	if (!CalNormalAngle(facade_normal, plane_normal, 90))
	{
		plane_normal = -plane_normal;
	}

	double d = -plane_normal.transpose() * facade_center;
	*plane = Vec4d(plane_normal[0], plane_normal[1], plane_normal[2], d);
}

void ExportPlane(const std::string& out_name, const Vec4d& plane)
{
	std::ofstream outFile(out_name);

	outFile << plane[0] << plane[1] << plane[2] << plane[3] << std::endl;

	outFile.close();
}

int main(int argc, char** argv)
{
	std::string mesh_name = argv[1];
	std::string out_name = argv[2];
	int isFacade = std::stoi(argv[3]);

	MeshT mesh;
	ReadMesh(mesh_name, &mesh);

	Vec4d plane;
	if (isFacade)
	{
		FittingFacade(mesh, &plane);
	}
	else
	{
		FittingPlane(mesh, &plane);
	}

	ExportPlane(out_name, plane);
}