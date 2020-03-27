#include <cmath>
#include <cstdio>
#include <iostream>

#include "ceres/ceres.h"
#include "ceres/rotation.h"

#include "glog/logging.h"
#include "gflags/gflags.h"

class BALProblem {
 public:
  ~BALProblem() {//释放内存空间
    delete[] point_index_;
    delete[] camera_index_;
    delete[] observations_;
    delete[] parameters_;
  }

  int num_observations()       const { return num_observations_;               }//返回观测值个数
  const double* observations() const { return observations_;                   }//返回观测值指针
  double* mutable_cameras()          { return parameters_;                     }//返回存储相机参数指针
  double* mutable_points()           { return parameters_  + 9 * num_cameras_; }//返回存储点的指针
  int num_cameras()                  { return num_cameras_;                    }//返回相机个数
  int num_parameters()               { return num_parameters_;                 }//返回参数个数

  double* mutable_camera_for_observation(int i) {        //返回第i个观测值中的相机外参指针
    return mutable_cameras() + camera_index_[i] * 9;
  }
  double* camera_inner_for_i_camera(int i) {             //返回第i个观测值中的相机内参指针
    return ((mutable_cameras() + camera_index_[i] * 9)+6);
  }
  double* mutable_point_for_observation(int i) {         //返回第i个观测值中的点的参数指针
    return mutable_points() + point_index_[i] * 3;
  }

  bool LoadFile(const char* filename) {                  //加载文件
    FILE* fptr = fopen(filename, "r");                   //只读方式打开文件，指针为fptr
    if (fptr == NULL) {
      return false;
    };

    FscanfOrDie(fptr, "%d", &num_cameras_);              //读取相机数，十进制
    FscanfOrDie(fptr, "%d", &num_points_);               //读取观测点数
    FscanfOrDie(fptr, "%d", &num_observations_);         //读取观测值数

    point_index_ = new int[num_observations_];           //申请观测点的内存空间
    camera_index_ = new int[num_observations_];          //申请相机的内存空间
    observations_ = new double[2 * num_observations_];   //申请坐标位置内存空间，因为是二维，所以是两倍

    num_parameters_ = 9 * num_cameras_ + 3 * num_points_;//参数个数，相机9个参数，观测点3个参数
    parameters_ = new double[num_parameters_];           //申请参数的内存空间

    for (int i = 0; i < num_observations_; ++i) {        //读取观测值
      FscanfOrDie(fptr, "%d", camera_index_ + i);        //读取相机索引
      FscanfOrDie(fptr, "%d", point_index_ + i);         //读取点索引
      for (int j = 0; j < 2; ++j) {
        FscanfOrDie(fptr, "%lf", observations_ + 2*i + j);
      }
    }

    for (int i = 0; i < num_parameters_; ++i) {//读取参数列表
      FscanfOrDie(fptr, "%lf", parameters_ + i);
    }//读取参列表
    return true;
  }

 private:
  template<typename T>
  void FscanfOrDie(FILE *fptr, const char *format, T *value) {
    int num_scanned = fscanf(fptr, format, value);//从fptr中读取格式为format的值进入value中，fscanf返回读取值的个数
    if (num_scanned != 1) {
      LOG(FATAL) << "Invalid UW data file.";
    }
  }

  int num_cameras_;       //相机数
  int num_points_;        //点数
  int num_observations_;  //观测值个数
  int num_parameters_;    //参数个数

  int* point_index_;      //指针
  int* camera_index_;     //相机指针
  double* observations_;  //观测值指针
  double* parameters_;    //参数指针
};//class BALProblem

struct SnavelyReprojectionError {//计算重投影误差
  SnavelyReprojectionError(double observed_x, double observed_y)//构造函数
      : observed_x(observed_x), observed_y(observed_y) {}//初始化参数

  template <typename T>
  bool operator()(const T* const camera_outer,
                  const T* const camera_inner,
                  const T* const point,
                  T* residuals) const {

    T p[3];
    ceres::AngleAxisRotatePoint(camera_outer, point, p);//求解角轴变换矩阵

    p[0] += camera_outer[3];
    p[1] += camera_outer[4];
    p[2] += camera_outer[5];//在角轴旋转基础上加上了平移矢量
    
    T xp = - p[0] / p[2];
    T yp = - p[1] / p[2];

    const T& l1 = camera_inner[1];
    const T& l2 = camera_inner[2];
    T r2 = xp*xp + yp*yp;
    T distortion = 1.0 + r2  * (l1 + l2  * r2);

    const T& focal = camera_inner[0];
    T predicted_x = focal * distortion * xp;
    T predicted_y = focal * distortion * yp;

    residuals[0] = predicted_x - observed_x;//x方向残差
    residuals[1] = predicted_y - observed_y;//y方向残差

    return true;
  }

  static ceres::CostFunction* Create(const double observed_x,
                                     const double observed_y) {
    return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 6, 3, 3>(
                new SnavelyReprojectionError(observed_x, observed_y)));
  }

  double observed_x;
  double observed_y;
};//重投影误差

int main(int argc, char** argv) {//主函数
  google::InitGoogleLogging(argv[0]);//这个函数什么事情也没有做
  if (argc != 2) {//参数不等于2个，输出错误
    std::cerr << "usage: bundle_adjuster_genet_reduce <bal_problem>\n";
    return 1;
  }

  BALProblem bal_problem;//建立BA问题的对象
  if (!bal_problem.LoadFile(argv[1])) {//加载文件是否成功
    std::cerr << "ERROR: unable to open file " << argv[1] << "\n";//输出错误
    return 1;
  }

  const double* observations = bal_problem.observations();//定义常指针，指向观测值位置

  ceres::Problem problem;//建立Ceres的Problem对象
  for (int i = 0; i < bal_problem.num_observations(); ++i) {
    ceres::CostFunction* cost_function =
        SnavelyReprojectionError::Create(observations[2 * i + 0],
                                         observations[2 * i + 1]);//建立一个当前观测点的成本函数（投影误差）
    problem.AddResidualBlock(cost_function,//
                             NULL /* squared loss */,//LossFunction可选
                             bal_problem.mutable_camera_for_observation(i),
                             bal_problem.camera_inner_for_i_camera(i),
                             bal_problem.mutable_point_for_observation(i));//将当前点的成本函数添加到残差中，
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;//线性求解器类型
  options.minimizer_progress_to_stdout = true;

  for(int i=0; i < bal_problem.num_cameras(); ++i){
    problem.SetParameterBlockConstant((bal_problem.mutable_cameras()+9*i+6));
  }

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);//调用求解器，输入选项问题和总结
  std::cout << summary.FullReport() << "\n";

  //for(int i=0; i<bal_problem.num_parameters(); ++i){
  //  std::cout<<*(bal_problem.mutable_cameras()+i)<<"\n";
  //}
  return 0;
}
