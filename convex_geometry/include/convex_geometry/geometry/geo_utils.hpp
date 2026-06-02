/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#ifndef GEO_UTILS_HPP
#define GEO_UTILS_HPP

#include "convex_geometry/geometry/quickhull.hpp"
#include "convex_geometry/geometry/sdlp.hpp"

#include <Eigen/Eigen>

#include <cfloat>
#include <cstdint>
#include <set>
#include <chrono>

// 几何工具命名空间，提供多面体几何运算的实用函数
namespace geo_utils
{

    // 查找凸多面体的内部点
    // 每一行的hPoly定义为 h0, h1, h2, h3，表示半空间约束：
    // h0*x + h1*y + h2*z + h3 <= 0
    // 参数:
    //   hPoly: 半空间表示的多面体，每行表示一个半平面约束
    //   interior: 输出参数，找到的内部点
    // 返回: 如果找到有效的内部点返回true，否则返回false
    inline bool findInterior(const Eigen::MatrixX4d &hPoly,
                             Eigen::Vector3d &interior)
    {
        const int m = hPoly.rows();  // 半空间约束的数量

        // 构造线性规划问题以找到最大化到所有边界的最小距离的点
        Eigen::MatrixX4d A(m, 4);
        Eigen::VectorXd b(m);
        Eigen::Vector4d c, x;
        const Eigen::ArrayXd hNorm = hPoly.leftCols<3>().rowwise().norm();  // 每个法向量的范数
        A.leftCols<3>() = hPoly.leftCols<3>().array().colwise() / hNorm;   // 归一化法向量
        A.rightCols<1>().setConstant(1.0);  // 最后一列用于表示到边界的距离
        b = -hPoly.rightCols<1>().array() / hNorm;  // 归一化的偏移量
        c.setZero();
        c(3) = -1.0;  // 目标函数：最大化最小距离

        // 使用单纯形线性规划求解
        const double minmaxsd = sdlp::linprog<4>(c, A, b, x);
        interior = x.head<3>();  // 提取前3个分量作为内部点坐标

        // 如果最小距离为负且有限，说明找到了有效的内部点
        return minmaxsd < 0.0 && !std::isinf(minmaxsd);
    }

    // 检查两个凸多面体是否重叠
    // 参数:
    //   hPoly0: 第一个多面体的半空间表示
    //   hPoly1: 第二个多面体的半空间表示
    //   eps: 重叠判断的容差阈值
    // 返回: 如果两个多面体有重叠返回true，否则返回false
    inline bool overlap(const Eigen::MatrixX4d &hPoly0,
                        const Eigen::MatrixX4d &hPoly1,
                        const double eps = 1.0e-6)

    {
        const int m = hPoly0.rows();  // 第一个多面体的约束数量
        const int n = hPoly1.rows();  // 第二个多面体的约束数量

        // 合并两个多面体的约束，构造联合的线性规划问题
        Eigen::MatrixX4d A(m + n, 4);
        Eigen::Vector4d c, x;
        Eigen::VectorXd b(m + n);
        A.leftCols<3>().topRows(m) = hPoly0.leftCols<3>();      // 第一个多面体的法向量
        A.leftCols<3>().bottomRows(n) = hPoly1.leftCols<3>();   // 第二个多面体的法向量
        A.rightCols<1>().setConstant(1.0);  // 距离变量
        b.topRows(m) = -hPoly0.rightCols<1>();     // 第一个多面体的偏移量
        b.bottomRows(n) = -hPoly1.rightCols<1>();  // 第二个多面体的偏移量
        c.setZero();
        c(3) = -1.0;  // 目标：最大化最小距离

        // 求解线性规划，如果存在同时满足两个多面体约束的点，则重叠
        const double minmaxsd = sdlp::linprog<4>(c, A, b, x);

        // 如果最小距离小于-eps且有限，说明两个多面体有重叠区域
        return minmaxsd < -eps && !std::isinf(minmaxsd);
    }

    // 三维向量的字典序比较器，用于std::set去重
    // 按照x, y, z的顺序进行字典序比较
    struct filterLess
    {
        inline bool operator()(const Eigen::Vector3d &l,
                               const Eigen::Vector3d &r) const
        {
            // 字典序比较：先比较x，再比较y，最后比较z
            return l(0) < r(0) ||
                   (l(0) == r(0) &&
                    (l(1) < r(1) ||
                     (l(1) == r(1) &&
                      l(2) < r(2))));
        }
    };

    // 过滤顶点集合，去除数值上相近的重复顶点
    // 参数:
    //   rV: 原始顶点矩阵，每列表示一个3D顶点
    //   epsilon: 判断两个顶点是否相同的容差
    //   fV: 输出参数，过滤后的顶点矩阵
    inline void filterVs(const Eigen::Matrix3Xd &rV,
                         const double &epsilon,
                         Eigen::Matrix3Xd &fV)
    {
        // 计算顶点坐标的量级
        const double mag = std::max(fabs(rV.maxCoeff()), fabs(rV.minCoeff()));
        // 确定量化分辨率，考虑数值精度
        const double res = mag * std::max(fabs(epsilon) / mag, DBL_EPSILON);

        // 使用set进行去重，基于量化后的坐标
        std::set<Eigen::Vector3d, filterLess> filter;
        fV = rV;
        int offset = 0;  // 过滤后的顶点索引
        Eigen::Vector3d quanti;  // 量化后的坐标

        for (int i = 0; i < rV.cols(); i++)
        {
            // 将顶点坐标量化到离散网格上
            quanti = (rV.col(i) / res).array().round();

            // 如果该量化坐标未出现过，保留此顶点
            if (filter.find(quanti) == filter.end())
            {
                filter.insert(quanti);
                fV.col(offset) = rV.col(i);
                offset++;
            }
        }
        // 截取有效的顶点列
        fV = fV.leftCols(offset).eval();
        return;
    }

    // 枚举凸多面体的所有顶点（H-representation到V-representation的转换）
    // 每行的hPoly定义为 h0, h1, h2, h3，表示半空间约束：
    // h0*x + h1*y + h2*z + h3 <= 0
    // 参数:
    //   hPoly: 半空间表示的凸多面体
    //   inner: 多面体的一个已知内部点
    //   vPoly: 输出参数，多面体的顶点表示，每列为一个顶点
    //   epsilon: 数值容差，建议值为1.0e-6
    inline void enumerateVs(const Eigen::MatrixX4d &hPoly,
                            const Eigen::Vector3d &inner,
                            Eigen::Matrix3Xd &vPoly,
                            const double epsilon = 1.0e-6)
    {
        // 将半空间约束转换到相对于内部点的坐标系
        // b表示每个半平面到内部点的有符号距离
        const Eigen::VectorXd b = -hPoly.rightCols<1>() - hPoly.leftCols<3>() * inner;

        // 构造对偶空间的点：每个半平面对应对偶空间的一个点
        // A的每一列表示对偶空间中的一个点
        const Eigen::Matrix<double, 3, -1, Eigen::ColMajor> A =
            (hPoly.leftCols<3>().array().colwise() / b.array()).transpose();

        // 使用QuickHull算法计算对偶空间点集的凸包
        quickhull::QuickHull<double> qh;
        const double qhullEps = std::min(epsilon, quickhull::defaultEps<double>());
        // CCW设为false，因为quickhull中法向量指向内部
        const auto cvxHull = qh.getConvexHull(A.data(), A.cols(), false, true, qhullEps);

        // 获取凸包的三角面片索引
        const auto &idBuffer = cvxHull.getIndexBuffer();
        const int hNum = idBuffer.size() / 3;  // 三角面片数量

        // 将对偶空间的面转换回原始空间的顶点
        Eigen::Matrix3Xd rV(3, hNum);
        Eigen::Vector3d normal, point, edge0, edge1;
        for (int i = 0; i < hNum; i++)
        {
            // 获取三角面片的三个顶点
            point = A.col(idBuffer[3 * i + 1]);
            edge0 = point - A.col(idBuffer[3 * i]);
            edge1 = A.col(idBuffer[3 * i + 2]) - point;

            // 计算面的外法向量（顺时针叉乘得到外法向）
            normal = edge0.cross(edge1);

            // 对偶变换：将对偶空间的面转换为原始空间的顶点
            rV.col(i) = normal / normal.dot(point);
        }

        // 过滤重复的顶点
        filterVs(rV, epsilon, vPoly);

        // 将顶点从相对于内部点的坐标系转换回全局坐标系
        vPoly = (vPoly.array().colwise() + inner.array()).eval();
        return;
    }

    // 枚举凸多面体的所有顶点（自动查找内部点版本）
    // 每行的hPoly定义为 h0, h1, h2, h3，表示半空间约束：
    // h0*x + h1*y + h2*z + h3 <= 0
    // 参数:
    //   hPoly: 半空间表示的凸多面体
    //   vPoly: 输出参数，多面体的顶点表示，每列为一个顶点
    //   epsilon: 数值容差，建议值为1.0e-6
    // 返回: 成功枚举顶点返回true，失败返回false
    inline bool enumerateVs(const Eigen::MatrixX4d &hPoly,
                            Eigen::Matrix3Xd &vPoly,
                            const double epsilon = 1.0e-6)
    {
        Eigen::Vector3d inner;
        // 首先尝试找到多面体的内部点
        if (findInterior(hPoly, inner))
        {
            // 使用找到的内部点进行顶点枚举
            enumerateVs(hPoly, inner, vPoly, epsilon);
            return true;
        }
        else
        {
            // 如果找不到内部点，说明多面体可能退化或为空
            return false;
        }
    }

} // namespace geo_utils

#endif
