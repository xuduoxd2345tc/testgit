// Created by starry on 12/21/16.

#include "circle_fit.h"

namespace hitcrt {
  CircleFit::CircleFit(int _maxIterations) {
    maxIterations = _maxIterations;
    verbose = false;
  }
  CircleFit::~CircleFit() {


}

  void CircleFit::get_center(std::vector<cv::Point2f>& data,cv::Point2d inital, cv::Point2d &center) {
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<Eigen::Dynamic, Eigen::Dynamic> >  MyBlockSolver;
    typedef g2o::LinearSolverCSparse<MyBlockSolver::PoseMatrixType> MyLinearSolver;

    g2o::SparseOptimizer optimizer;
    MyLinearSolver* linearSolver = new MyLinearSolver();
    MyBlockSolver* solver_ptr = new MyBlockSolver(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    //g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);

    optimizer.setAlgorithm(solver);
    VertexCircle* circle = new VertexCircle();
    circle->setId(0);
    circle->setEstimate(Eigen::Vector2d(inital.x,inital.y)); // some initial value for the circle
    optimizer.addVertex(circle);

    for (size_t i = 0; i < data.size(); ++i) {
      Eigen::Vector2d point(data[i].x,data[i].y);
      EdgePointOnCircle* e = new EdgePointOnCircle;
      e->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
      e->setVertex(0, circle);
      e->setMeasurement(point);
      optimizer.addEdge(e);
    }
    optimizer.initializeOptimization();
    optimizer.setVerbose(verbose);
    optimizer.optimize(maxIterations);
    if (verbose)
      std::cout << std::endl;

    center.x = circle->estimate()[0];
    center.y = circle->estimate()[1];
    std::cout << "center of the circle " << circle->estimate().head<2>().transpose() << std::endl;
    std::cout << "error " << errorOfSolution(data.size(),data, circle->estimate()) << std::endl;
    std::cout << std::endl;
  }

  double CircleFit::errorOfSolution(int numPoints,
                                    std::vector<cv::Point2f>& data,
                                    const Eigen::Vector2d& circle)
  {
    Eigen::Vector2d center = circle.head<2>();
    double radius =0.1; // radius of cylinders
    double error = 0;
    for (size_t i = 0; i < data.size(); ++i) {
      Eigen::Vector2d estimate(data[i].x,data[i].y);
      double d = ( estimate - center).norm() - radius;
      error += d*d;
    }
    return error;
  }


}

