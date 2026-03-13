#include "backend/pnp_g2o.h" 

#include <g2o/core/base_vertex.h>
#include <sophus/se3.hpp>
#include <g2o/core/base_unary_edge.h>

#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/block_solver.h>
#include <memory>

namespace{
    // 1. define vertices for pose and edges for landmarks observed
    constexpr int kPoseDim = 6;
    
    class VertexPose: public g2o::BaseVertex<kPoseDim, Sophus::SE3d>{
        public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        void setToOriginImpl() override {
            _estimate = Sophus::SE3d();
        }

        void oplusImpl(const double* update) override {
            Eigen::Map<const Eigen::Matrix<double, kPoseDim, 1>> update_eigen(update);
            _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
        }

        bool read(std::istream& in) override {return true;}
        bool write(std::ostream& out) const override {return true;}
    };

    constexpr int kPixelDims = 2;
    class EdgeProjection: public g2o::BaseUnaryEdge<kPixelDims, Eigen::Vector2d, VertexPose>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        EdgeProjection(const Eigen::Vector3d& pos3d, const calib::PinholeCameraIntrinsics& intrinsics): pos3d_(pos3d), intrinsics_(intrinsics){};

        void computeError() override{
            const VertexPose* v = static_cast<const VertexPose*> (_vertices[0]);
            const Sophus::SE3d& T = v->estimate();
            Eigen::Vector3d pos_pixel = intrinsics_.K * (T * pos3d_);
            pos_pixel /= pos_pixel[2];
            _error = _measurement - pos_pixel.head<2>();
        }

        void linearizeOplus() override{
            const VertexPose* v = static_cast<const VertexPose*> (_vertices[0]);
            const Sophus::SE3d& T = v->estimate();
            const Eigen::Vector3d p_cam = (T * pos3d_);

            const double fx = intrinsics_.fx();
            const double fy = intrinsics_.fy();

            const double X = p_cam(0);
            const double Y = p_cam(1);
            const double Z = p_cam(2);

            _jacobianOplusXi << 
            -fx/Z, 0.0, fx * X / (Z*Z), fx * X * Y / (Z*Z), -fx - fx * (X*X) / (Z*Z), fx * Y / Z,
            0.0, -fy/Z, fy*Y / (Z*Z), fy + fy * (Y*Y) / (Z*Z), -fy * X * Y / (Z * Z), -fy * X / Z;
            
        }

        bool read(std::istream& in) override {return true;}
        bool write(std::ostream& out) const override {return true;}

        
    private:
        Eigen::Vector3d pos3d_;
        const calib::PinholeCameraIntrinsics& intrinsics_;
    };

    constexpr int kLandmarkDim = 3;
    
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<kPoseDim, kLandmarkDim>> BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;
}

namespace backend{
void G2OPnPSolver::solve(
    const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& points3d,
    const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>& points2d_img2,
    const calib::PinholeCameraIntrinsics& intrinsics2,
    Sophus::SE3d& c2_T_c1) const {

    auto solver = new g2o::OptimizationAlgorithmGaussNewton(
        std::make_unique<BlockSolverType>(
            std::make_unique<LinearSolverType>()
        )
    );

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    int id = 0;
    VertexPose* v_pose = new VertexPose();
    v_pose->setEstimate(Sophus::SE3d());
    v_pose->setId(id++);
    optimizer.addVertex(v_pose);

    // add all edges
    for (size_t i = 0; i < std::min(points3d.size(), points2d_img2.size()); ++i){
        EdgeProjection* edge = new EdgeProjection(points3d[i], intrinsics2);
        edge->setId(id++);
        edge->setVertex(0, v_pose);
        edge->setMeasurement(points2d_img2[i]);
        edge->setInformation(Eigen::Matrix2d::Identity());
        optimizer.addEdge(edge);
    }

    optimizer.initializeOptimization();
    constexpr int kIters = 10;
    optimizer.optimize(kIters);

    c2_T_c1 = v_pose->estimate();
}
} // namespace backend
