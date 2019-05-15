- Eigen

  ```c++
  #include <iostream>
  #include <cmath>
  using namespace std;

  #include <Eigen/Core>
  // Eigen 几何模块
  #include <Eigen/Geometry>

  /****************************
  * 本程序演示了 Eigen 几何模块的使用方法
  ****************************/

  int main ( int argc, char** argv )
  {
      // Eigen/Geometry 模块提供了各种旋转和平移的表示
      // 3D 旋转矩阵直接使用 Matrix3d 或 Matrix3f
      Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
      
    	// 旋转向量使用 AngleAxis, 它底层不直接是Matrix，但运算可以当作矩阵（因为重载了运算符）
      Eigen::AngleAxisd rotation_vector ( M_PI/4, Eigen::Vector3d ( 0,0,1 ) ); //沿 Z 轴旋转 45 度
      cout .precision(3);
      cout<<"rotation matrix =\n"<<rotation_vector.matrix() <<endl;// 用matrix()转换成矩阵
      
    	// 也可以直接赋值
      rotation_matrix = rotation_vector.toRotationMatrix();
      // 用 AngleAxis 可以进行坐标变换
      Eigen::Vector3d v ( 1,0,0 );
      Eigen::Vector3d v_rotated = rotation_vector * v;
      cout<<"(1,0,0) after rotation = "<<v_rotated.transpose()<<endl;
      // 或者用旋转矩阵
      v_rotated = rotation_matrix * v;
      cout<<"(1,0,0) after rotation = "<<v_rotated.transpose()<<endl;

      // 欧拉角: 可以将旋转矩阵直接转换成欧拉角
      Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles ( 2,1,0 ); // ZYX顺序，即roll pitch yaw顺序
      cout<<"yaw pitch roll = "<<euler_angles.transpose()<<endl;

      // 欧氏变换矩阵使用 Eigen::Isometry
      Eigen::Isometry3d T=Eigen::Isometry3d::Identity();// 虽然称为3d，实质上是4＊4的矩阵
      T.rotate ( rotation_vector ); // 按照rotation_vector进行旋转
      T.pretranslate ( Eigen::Vector3d ( 1,3,4 ) );// 把平移向量设成(1,3,4)
      cout << "Transform matrix = \n" << T.matrix() <<endl;

      // 用变换矩阵进行坐标变换
      Eigen::Vector3d v_transformed = T*v;                              // 相当于R*v+t
      cout<<"v tranformed = "<<v_transformed.transpose()<<endl;

      // 对于仿射和射影变换，使用 Eigen::Affine3d 和 Eigen::Projective3d 即可，略

      // 四元数
      // 可以直接把AngleAxis赋值给四元数，反之亦然
      Eigen::Quaterniond q = Eigen::Quaterniond ( rotation_vector );
      cout<<"quaternion = \n"<<q.coeffs() <<endl;   // 请注意coeffs的顺序是(x,y,z,w),w为实部，前三者为虚部
      // 也可以把旋转矩阵赋给它
      q = Eigen::Quaterniond ( rotation_matrix );
      cout<<"quaternion = \n"<<q.coeffs() <<endl;
      // 使用四元数旋转一个向量，使用重载的乘法即可
      v_rotated = q*v; // 注意数学上是qvq^{-1}
      cout<<"(1,0,0) after rotation = "<<v_rotated.transpose()<<endl;

      return 0;
  }
  ```

- Sorphus

  ```c++
  #include <iostream>
  #include <cmath>
  using namespace std; 

  #include <Eigen/Core>
  #include <Eigen/Geometry>

  #include "sophus/so3.h"
  #include "sophus/se3.h"

  int main( int argc, char** argv )
  {
      // 沿Z轴转90度的旋转矩阵
      Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0,0,1)).toRotationMatrix();
      
      Sophus::SO3 SO3_R(R);               // Sophus::SO(3)可以直接从旋转矩阵构造
      Sophus::SO3 SO3_v( 0, 0, M_PI/2 );  // 亦可从旋转向量构造
      Eigen::Quaterniond q(R);            // 或者四元数
      Sophus::SO3 SO3_q( q );
      // 上述表达方式都是等价的
      // 输出SO(3)时，以so(3)形式输出
      cout<<"SO(3) from matrix: "<<SO3_R<<endl;
      cout<<"SO(3) from vector: "<<SO3_v<<endl;
      cout<<"SO(3) from quaternion :"<<SO3_q<<endl;
      
      // 使用对数映射获得它的李代数
      Eigen::Vector3d so3 = SO3_R.log();
      cout<<"so3 = "<<so3.transpose()<<endl;
      // hat 为向量到反对称矩阵
      cout<<"so3 hat=\n"<<Sophus::SO3::hat(so3)<<endl;
      // 相对的，vee为反对称到向量
      cout<<"so3 hat vee= "<<Sophus::SO3::vee( Sophus::SO3::hat(so3) ).transpose()<<endl; // transpose纯粹是为了输出美观一些
      
      // 增量扰动模型的更新
      Eigen::Vector3d update_so3(1e-4, 0, 0); //假设更新量为这么多
      Sophus::SO3 SO3_updated = Sophus::SO3::exp(update_so3)*SO3_R;
      cout<<"SO3 updated = "<<SO3_updated<<endl;
      
      /********************萌萌的分割线*****************************/
      cout<<"************我是分割线*************"<<endl;
      // 对SE(3)操作大同小异
      Eigen::Vector3d t(1,0,0);           // 沿X轴平移1
      Sophus::SE3 SE3_Rt(R, t);           // 从R,t构造SE(3)
      Sophus::SE3 SE3_qt(q,t);            // 从q,t构造SE(3)
      cout<<"SE3 from R,t= "<<endl<<SE3_Rt<<endl;
      cout<<"SE3 from q,t= "<<endl<<SE3_qt<<endl;
      // 李代数se(3) 是一个六维向量，方便起见先typedef一下
      typedef Eigen::Matrix<double,6,1> Vector6d;
      Vector6d se3 = SE3_Rt.log();
      cout<<"se3 = "<<se3.transpose()<<endl;
      // 观察输出，会发现在Sophus中，se(3)的平移在前，旋转在后.
      // 同样的，有hat和vee两个算符
      cout<<"se3 hat = "<<endl<<Sophus::SE3::hat(se3)<<endl;
      cout<<"se3 hat vee = "<<Sophus::SE3::vee( Sophus::SE3::hat(se3) ).transpose()<<endl;
      
      // 最后，演示一下更新
      Vector6d update_se3; //更新量
      update_se3.setZero();
      update_se3(0,0) = 1e-4d;
      Sophus::SE3 SE3_updated = Sophus::SE3::exp(update_se3)*SE3_Rt;
      cout<<"SE3 updated = "<<endl<<SE3_updated.matrix()<<endl;
      
      return 0;
  }
  ```

- Cerces

  ```c++
  #include <iostream>
  #include <opencv2/core/core.hpp>
  #include <ceres/ceres.h>
  #include <chrono>

  using namespace std;

  // 代价函数的计算模型
  struct CURVE_FITTING_COST
  {
      CURVE_FITTING_COST ( double x, double y ) : _x ( x ), _y ( y ) {}
      // 残差的计算
      template <typename T>
      bool operator() (
          const T* const abc,     // 模型参数，有3维
          T* residual ) const     // 残差
      {
          residual[0] = T ( _y ) - ceres::exp ( abc[0]*T ( _x ) *T ( _x ) + abc[1]*T ( _x ) + abc[2] ); // y-exp(ax^2+bx+c)
          return true;
      }
      const double _x, _y;    // x,y数据
  };

  int main ( int argc, char** argv )
  {
      double a=1.0, b=2.0, c=1.0;         // 真实参数值
      int N=100;                          // 数据点
      double w_sigma=1.0;                 // 噪声Sigma值
      cv::RNG rng;                        // OpenCV随机数产生器
      double abc[3] = {0,0,0};            // abc参数的估计值

      vector<double> x_data, y_data;      // 数据

      cout<<"generating data: "<<endl;
      for ( int i=0; i<N; i++ )
      {
          double x = i/100.0;
          x_data.push_back ( x );
          y_data.push_back (
              exp ( a*x*x + b*x + c ) + rng.gaussian ( w_sigma )
          );
          cout<<x_data[i]<<" "<<y_data[i]<<endl;
      }

      // 构建最小二乘问题
      ceres::Problem problem;
      for ( int i=0; i<N; i++ )
      {
          problem.AddResidualBlock (     // 向问题中添加误差项
          // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
              new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3> ( 
                  new CURVE_FITTING_COST ( x_data[i], y_data[i] )
              ),
              nullptr,            // 核函数，这里不使用，为空
              abc                 // 待估计参数
          );
      }

      // 配置求解器
      ceres::Solver::Options options;     // 这里有很多配置项可以填
      options.linear_solver_type = ceres::DENSE_QR;  // 增量方程如何求解
      options.minimizer_progress_to_stdout = true;   // 输出到cout

      ceres::Solver::Summary summary;                // 优化信息
      chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
      ceres::Solve ( options, &problem, &summary );  // 开始优化
      chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
      chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
      cout<<"solve time cost = "<<time_used.count()<<" seconds. "<<endl;

      // 输出结果
      cout<<summary.BriefReport() <<endl;
      cout<<"estimated a,b,c = ";
      for ( auto a:abc ) cout<<a<<" ";
      cout<<endl;

      return 0;
  }
  ```

- g2o

  > 图优化
  >
  > 1. 定义顶点和边的类型
  > 2. 构建图
  > 3. 选择优化算法
  > 4. 调用 g2o 进行优化，返回结果

  ```
  #include <iostream>
  #include <g2o/core/base_vertex.h>
  #include <g2o/core/base_unary_edge.h>
  #include <g2o/core/block_solver.h>
  #include <g2o/core/optimization_algorithm_levenberg.h>
  #include <g2o/core/optimization_algorithm_gauss_newton.h>
  #include <g2o/core/optimization_algorithm_dogleg.h>
  #include <g2o/solvers/dense/linear_solver_dense.h>
  #include <Eigen/Core>
  #include <opencv2/core/core.hpp>
  #include <cmath>
  #include <chrono>
  using namespace std; 

  // 曲线模型的顶点，模板参数：优化变量维度和数据类型
  class CurveFittingVertex: public g2o::BaseVertex<3, Eigen::Vector3d>
  {
  public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      virtual void setToOriginImpl() // 重置
      {
          _estimate << 0,0,0;
      }
      
      virtual void oplusImpl( const double* update ) // 更新
      {
          _estimate += Eigen::Vector3d(update);
      }
      // 存盘和读盘：留空
      virtual bool read( istream& in ) {}
      virtual bool write( ostream& out ) const {}
  };

  // 误差模型 模板参数：观测值维度，类型，连接顶点类型
  class CurveFittingEdge: public g2o::BaseUnaryEdge<1,double,CurveFittingVertex>
  {
  public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      CurveFittingEdge( double x ): BaseUnaryEdge(), _x(x) {}
      // 计算曲线模型误差
      void computeError()
      {
          const CurveFittingVertex* v = static_cast<const CurveFittingVertex*> (_vertices[0]);
          const Eigen::Vector3d abc = v->estimate();
          _error(0,0) = _measurement - std::exp( abc(0,0)*_x*_x + abc(1,0)*_x + abc(2,0) ) ;
      }
      virtual bool read( istream& in ) {}
      virtual bool write( ostream& out ) const {}
  public:
      double _x;  // x 值， y 值为 _measurement
  };

  int main( int argc, char** argv )
  {
      double a=1.0, b=2.0, c=1.0;         // 真实参数值
      int N=100;                          // 数据点
      double w_sigma=1.0;                 // 噪声Sigma值
      cv::RNG rng;                        // OpenCV随机数产生器
      double abc[3] = {0,0,0};            // abc参数的估计值

      vector<double> x_data, y_data;      // 数据
      
      cout<<"generating data: "<<endl;
      for ( int i=0; i<N; i++ )
      {
          double x = i/100.0;
          x_data.push_back ( x );
          y_data.push_back (
              exp ( a*x*x + b*x + c ) + rng.gaussian ( w_sigma )
          );
          cout<<x_data[i]<<" "<<y_data[i]<<endl;
      }
      
      // 构建图优化，先设定g2o
      typedef g2o::BlockSolver< g2o::BlockSolverTraits<3,1> > Block;  // 每个误差项优化变量维度为3，误差值维度为1
      Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>(); // 线性方程求解器
      Block* solver_ptr = new Block( linearSolver );      // 矩阵块求解器
      // 梯度下降方法，从GN, LM, DogLeg 中选
      g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( solver_ptr );
      // g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr );
      // g2o::OptimizationAlgorithmDogleg* solver = new g2o::OptimizationAlgorithmDogleg( solver_ptr );
      g2o::SparseOptimizer optimizer;     // 图模型
      optimizer.setAlgorithm( solver );   // 设置求解器
      optimizer.setVerbose( true );       // 打开调试输出
      
      // 往图中增加顶点
      CurveFittingVertex* v = new CurveFittingVertex();
      v->setEstimate( Eigen::Vector3d(0,0,0) );
      v->setId(0);
      optimizer.addVertex( v );
      
      // 往图中增加边
      for ( int i=0; i<N; i++ )
      {
          CurveFittingEdge* edge = new CurveFittingEdge( x_data[i] );
          edge->setId(i);
          edge->setVertex( 0, v );                // 设置连接的顶点
          edge->setMeasurement( y_data[i] );      // 观测数值
          edge->setInformation( Eigen::Matrix<double,1,1>::Identity()*1/(w_sigma*w_sigma) ); // 信息矩阵：协方差矩阵之逆
          optimizer.addEdge( edge );
      }
      
      // 执行优化
      cout<<"start optimization"<<endl;
      chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
      optimizer.initializeOptimization();
      optimizer.optimize(100);
      chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
      chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
      cout<<"solve time cost = "<<time_used.count()<<" seconds. "<<endl;
      
      // 输出优化值
      Eigen::Vector3d abc_estimate = v->estimate();
      cout<<"estimated model: "<<abc_estimate.transpose()<<endl;
      
      return 0;
  }
  ```

  ​

  ​

- 点云拼接

  ```c++
  #include <iostream>
  #include <fstream>
  using namespace std;
  #include <opencv2/core/core.hpp>
  #include <opencv2/highgui/highgui.hpp>
  #include <Eigen/Geometry> 
  #include <boost/format.hpp>
  #include <pcl/point_types.h> 
  #include <pcl/io/pcd_io.h> 
  #include <pcl/visualization/pcl_visualizer.h>

  int main( int argc, char** argv )
  {
      vector<cv::Mat> colorImgs, depthImgs;    // 彩色图和深度图
      vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;       
    
      // 相机位姿
      // 打开 pose.txt 文件
      ifstream fin("./pose.txt");
      if (!fin)
      {
          cerr<<"请在有pose.txt的目录下运行此程序"<<endl;

          return 1;
      }
      
      for ( int i=0; i<5; i++ )
      {
          boost::format fmt( "./%s/%d.%s" ); 
          // 图像文件格式
          colorImgs.push_back( cv::imread( (fmt%"color"%(i+1)%"png").str() ));
          
        	// 使用-1读取原始图像
          depthImgs.push_back( cv::imread( (fmt%"depth"%(i+1)%"pgm").str(), -1 )); 
          
  	    // 位姿每一行有7个数，前三个是平移向量，后4个是4元数
          double data[7] = {0};
          for ( auto& d:data )
              fin>>d;
  	    
          // 四元数
          Eigen::Quaterniond q( data[6], data[3], data[4], data[5] );
          
          // 欧式变换矩阵
  	    Eigen::Isometry3d T(q);
          
          // 设置平移向量
  	    T.pretranslate( Eigen::Vector3d( data[0], data[1], data[2] ));
          poses.push_back( T );
      }
      
      // 计算点云并拼接
      
    	// 相机内参 
      double cx = 325.5;
      double cy = 253.5;
      double fx = 518.0;
      double fy = 519.0;
      double depthScale = 1000.0;
      
      cout<<"正在将图像转换为点云..."<<endl;
      
      // 定义点云使用的格式：这里用的是XYZRGB
      typedef pcl::PointXYZRGB PointT; 
      typedef pcl::PointCloud<PointT> PointCloud;
      
      // 新建一个点云
      PointCloud::Ptr pointCloud( new PointCloud ); 
      for ( int i=0; i<5; i++ )
      {
          cout<<"转换图像中: "<<i+1<<endl; 
          cv::Mat color = colorImgs[i]; 
          cv::Mat depth = depthImgs[i];
          Eigen::Isometry3d T = poses[i];
          for ( int v=0; v<color.rows; v++ )
              for ( int u=0; u<color.cols; u++ )
              {
                  unsigned int d = depth.ptr<unsigned short> ( v )[u]; // 深度值
                  if ( d==0 ) continue; // 为0表示没有测量到
                  
                  // 从图像根据内参，从图像转换到相机坐标
                  Eigen::Vector3d point; 
                  point[2] = double(d)/depthScale; 
                  point[0] = (u-cx)*point[2]/fx;
                  point[1] = (v-cy)*point[2]/fy; 
                  
  				// 从相机坐标转换到世界坐标
  				Eigen::Vector3d pointWorld = T*point;
                  
  		 		// 转换位点云中的点
                  PointT p ;
                  p.x = pointWorld[0];
                  p.y = pointWorld[1];
                  p.z = pointWorld[2];
                  p.b = color.data[ v*color.step+u*color.channels() ];
                  p.g = color.data[ v*color.step+u*color.channels()+1 ];
                  p.r = color.data[ v*color.step+u*color.channels()+2 ];
                  pointCloud->points.push_back( p );
              }
      }
      
      pointCloud->is_dense = false;
      cout<<"点云共有"<<pointCloud->size()<<"个点."<<endl;
      pcl::io::savePCDFileBinary("map.pcd", *pointCloud );
      return 0;
  }
  ```