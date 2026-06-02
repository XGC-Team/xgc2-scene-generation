/*
 * 版权所有 (c) 1990 Michael E. Hohmeyer,
 *       hohmeyer@icemcfd.com
 * 允许以任何方式修改和重新分发此代码，只要保留此声明。
 * 所有标准免责声明均适用。
 *
 * R. Seidel的线性规划（LP）求解算法
 *
 * Copyright (c) 1990 Michael E. Hohmeyer,
 *       hohmeyer@icemcfd.com
 * Permission is granted to modify and re-distribute this code in any manner
 * as long as this notice is preserved.  All standard disclaimers apply.
 *
 * R. Seidel's algorithm for solving LPs (linear programs.)
 */

/*
 * 版权所有 (c) 2021 Zhepei Wang,
 *       wangzhepei@live.com
 * 修改内容：
 * 1. 修复"move_to_front"函数的bug：原代码中"prev[m]"被非法访问，
 *    因为"prev"原本只有m个整数。通过分配m+1个整数的"prev"数组修复。
 * 2. 添加Eigen库接口。
 * 3. 递归模板化。
 * 允许以任何方式修改和重新分发此代码，只要保留此声明。
 * 所有标准免责声明均适用。
 *
 * 参考文献: Seidel, R. (1991), "Small-dimensional linear programming and convex
 *          hulls made easy", Discrete & Computational Geometry 6 (1): 423–434,
 *          doi:10.1007/BF02574699
 *
 * Copyright (c) 2021 Zhepei Wang,
 *       wangzhepei@live.com
 * 1. Bug fix in "move_to_front" function that "prev[m]" is illegally accessed
 *    while "prev" originally has only m ints. It is fixed by allocating a
 *    "prev" with m + 1 ints.
 * 2. Add Eigen interface.
 * 3. Resursive template.
 * Permission is granted to modify and re-distribute this code in any manner
 * as long as this notice is preserved.  All standard disclaimers apply.
 *
 * Ref: Seidel, R. (1991), "Small-dimensional linear programming and convex
 *      hulls made easy", Discrete & Computational Geometry 6 (1): 423–434,
 *      doi:10.1007/BF02574699
 */

#ifndef SDLP_HPP
#define SDLP_HPP

#include <Eigen/Eigen>
#include <cmath>
#include <random>

// SDLP命名空间：Small-Dimensional Linear Programming（小维度线性规划）
namespace sdlp
{
    // 数值容差，用于浮点数比较
    constexpr double eps = 1.0e-12;

    // 线性规划求解器的状态枚举
    enum
    {
        /* 找到最小值 */
        MINIMUM = 0,
        /* 不可行区域（无解） */
        INFEASIBLE,
        /* 无界解（解趋于无穷） */
        UNBOUNDED,
        /* 模糊解（仅在解集中存在一个顶点） */
        AMBIGUOUS,
    };

    // 计算二维向量的点积
    // 参数: a - 第一个二维向量
    //       b - 第二个二维向量
    // 返回: 两向量的点积 a·b
    inline double dot2(const double a[2],
                       const double b[2])
    {
        return a[0] * b[0] + a[1] * b[1];
    }

    // 计算二维向量的叉积（标量形式）
    // 参数: a - 第一个二维向量
    //       b - 第二个二维向量
    // 返回: 叉积的z分量 (a×b)_z = a_x*b_y - a_y*b_x
    inline double cross2(const double a[2],
                         const double b[2])
    {
        return a[0] * b[1] - a[1] * b[0];
    }

    // 将二维向量单位化（归一化）
    // 参数: a - 输入的二维向量
    //       b - 输出的单位向量
    // 返回: 如果向量长度过小（接近零向量）返回true，否则返回false
    inline bool unit2(const double a[2],
                      double b[2])
    {
        // 计算向量的模长
        const double mag = std::sqrt(a[0] * a[0] +
                                     a[1] * a[1]);
        // 如果模长太小，认为是零向量
        if (mag < 2.0 * eps)
        {
            return true;
        }
        // 归一化向量
        b[0] = a[0] / mag;
        b[1] = a[1] / mag;
        return false;
    }

    /* 将d+1维点单位化（归一化） */
    // 模板参数: d - 维度参数，实际向量维度为d+1
    // 参数: a - 输入输出的(d+1)维向量，归一化后直接修改原数组
    // 返回: 如果向量长度过小（接近零向量）返回true，否则返回false
    template <int d>
    inline bool unit(double *a)
    {
        // 计算向量的模长平方
        double mag = 0.0;
        for (int i = 0; i <= d; i++)
        {
            mag += a[i] * a[i];
        }
        // 如果模长平方太小，认为是零向量
        if (mag < (d + 1) * eps * eps)
        {
            return true;
        }
        // 计算归一化系数（模长的倒数）
        mag = 1.0 / std::sqrt(mag);
        // 归一化向量的每个分量
        for (int i = 0; i <= d; i++)
        {
            a[i] *= mag;
        }
        return false;
    }

    /* 优化无约束目标函数 */
    // 优化线性分式函数 dot(x, n_vec) / dot(x, d_vec) 在无约束条件下的最小值
    // 模板参数: d - 维度参数
    // 参数: n_vec - 分子向量（d+1维）
    //       d_vec - 分母向量（d+1维）
    //       opt   - 输出的最优点（d+1维）
    // 返回: 优化状态（MINIMUM或AMBIGUOUS）
    template <int d>
    inline int lp_no_con(const double *n_vec,
                         const double *d_vec,
                         double *opt)
    {
        // 计算分子向量和分母向量的点积
        double n_dot_d = 0.0;
        // 计算分母向量的模长平方
        double d_dot_d = 0.0;
        for (int i = 0; i <= d; i++)
        {
            n_dot_d += n_vec[i] * d_vec[i];
            d_dot_d += d_vec[i] * d_vec[i];
        }
        // 如果分母向量接近零向量，设置默认值
        if (d_dot_d < eps * eps)
        {
            n_dot_d = 0.0;
            d_dot_d = 1.0;
        }
        // 计算最优点：将n_vec投影到垂直于d_vec的超平面上，并取反方向
        // opt = -n_vec + d_vec * (n_vec·d_vec) / (d_vec·d_vec)
        for (int i = 0; i <= d; i++)
        {
            opt[i] = -n_vec[i] +
                     d_vec[i] * n_dot_d / d_dot_d;
        }
        /* 归一化最优点 */
        if (unit<d>(opt))
        {
            // 如果最优点是零向量，设置为单位向量
            opt[d] = 1.0;
            return AMBIGUOUS;
        }
        else
        {
            return MINIMUM;
        }
    }

    /* 将索引i移动到链表前端，返回原来在i位置的平面索引 */
    // 使用双向链表实现，将指定元素移动到链表头部（仅次于头节点0）
    // 参数: i    - 要移动的平面索引
    //       next - 双向链表的next指针数组
    //       prev - 双向链表的prev指针数组
    // 返回: 移动前在i位置的前一个元素索引
    inline int move_to_front(const int i,
                             int *next,
                             int *prev)
    {
        // 如果i已经在最前面或就是第一个元素，无需移动
        if (i == 0 || i == next[0])
        {
            return i;
        }
        // 保存i的前驱节点
        const int previ = prev[i];
        /* 将i从当前位置移除 */
        next[prev[i]] = next[i];
        prev[next[i]] = prev[i];
        /* 将i放到链表前端（0号节点之后） */
        next[i] = next[0];
        prev[i] = 0;
        prev[next[i]] = i;
        next[0] = i;
        return previ;
    }

    // 最小化线性分式函数在圆弧区域上的最小值
    // 在单位圆上的楔形（wedge）区域内，找到线性分式函数 dot(x,n_vec)/dot(x,d_vec) 的最小值
    // 参数: degen   - 是否为退化情况（楔形区域退化为一条射线）
    //       cw_vec  - 顺时针方向的边界向量
    //       ccw_vec - 逆时针方向的边界向量
    //       n_vec   - 目标函数分子向量
    //       d_vec   - 目标函数分母向量
    //       opt     - 输出的最优点
    inline void lp_min_lin_rat(const bool degen,
                               const double cw_vec[2],
                               const double ccw_vec[2],
                               const double n_vec[2],
                               const double d_vec[2],
                               double opt[2])
    {
        /* 线性分式函数情况 */
        // 计算边界向量与分母向量的点积
        const double d_cw = dot2(cw_vec, d_vec);
        const double d_ccw = dot2(ccw_vec, d_vec);
        // 计算边界向量与分子向量的点积
        const double n_cw = dot2(cw_vec, n_vec);
        const double n_ccw = dot2(ccw_vec, n_vec);
        if (degen)
        {
            /* 退化情况：直接比较边界点的函数值 */
            if (n_cw / d_cw < n_ccw / d_ccw)
            {
                opt[0] = cw_vec[0];
                opt[1] = cw_vec[1];
            }
            else
            {
                opt[0] = ccw_vec[0];
                opt[1] = ccw_vec[1];
            }
            /* 检查顺时针/逆时针边界是否接近极点 */
        }
        else if (std::fabs(d_cw) > 2.0 * eps &&
                 std::fabs(d_ccw) > 2.0 * eps)
        {
            /* 可行域不包含极点（分母不为零的点） */
            if (d_cw * d_ccw > 0.0)
            {
                /* 两个边界点的分母同号，比较哪个端点有最小值 */
                if (n_cw / d_cw < n_ccw / d_ccw)
                {
                    opt[0] = cw_vec[0];
                    opt[1] = cw_vec[1];
                }
                else
                {
                    opt[0] = ccw_vec[0];
                    opt[1] = ccw_vec[1];
                }
            }
            else
            {
                /* 可行域包含极点（分母为零的点，函数值趋于负无穷） */
                if (d_cw > 0.0)
                {
                    // 极点在d_vec的垂直方向（逆时针90度）
                    opt[0] = -d_vec[1];
                    opt[1] = d_vec[0];
                }
                else
                {
                    // 极点在d_vec的垂直方向（顺时针90度）
                    opt[0] = d_vec[1];
                    opt[1] = -d_vec[0];
                }
            }
        }
        else if (std::fabs(d_cw) > 2.0 * eps)
        {
            /* 逆时针边界接近极点 */
            if (n_ccw * d_cw > 0.0)
            {
                /* 逆时针边界是正极点（函数值趋于正无穷） */
                opt[0] = cw_vec[0];
                opt[1] = cw_vec[1];
            }
            else
            {
                /* 逆时针边界是负极点（函数值趋于负无穷） */
                opt[0] = ccw_vec[0];
                opt[1] = ccw_vec[1];
            }
        }
        else if (std::fabs(d_ccw) > 2.0 * eps)
        {
            /* 顺时针边界接近极点 */
            if (n_cw * d_ccw > 2.0 * eps)
            {
                /* 顺时针边界是正极点 */
                opt[0] = ccw_vec[0];
                opt[1] = ccw_vec[1];
            }
            else
            {
                /* 顺时针边界是负极点 */
                opt[0] = cw_vec[0];
                opt[1] = cw_vec[1];
            }
        }
        else
        {
            /* 两个边界都接近极点 */
            // 通过叉积判断选择哪个边界
            if (cross2(d_vec, n_vec) > 0.0)
            {
                opt[0] = cw_vec[0];
                opt[1] = cw_vec[1];
            }
            else
            {
                opt[0] = ccw_vec[0];
                opt[1] = ccw_vec[1];
            }
        }
    }

    // 计算单位圆上由半平面约束形成的楔形（wedge）可行域
    // 参数: halves  - 半平面数组，每个半平面用2D法向量表示
    //       m       - 终止标记，表示链表结束
    //       next    - 双向链表的next指针数组
    //       prev    - 双向链表的prev指针数组
    //       cw_vec  - 输出的顺时针边界向量
    //       ccw_vec - 输出的逆时针边界向量
    //       degen   - 输出是否为退化情况（可行域退化为一条射线）
    // 返回: 可行域状态（MINIMUM/UNBOUNDED/INFEASIBLE）
    inline int wedge(const double (*halves)[2],
                     const int m,
                     int *next,
                     int *prev,
                     double cw_vec[2],
                     double ccw_vec[2],
                     bool *degen)
    {
        int i;
        double d_cw, d_ccw;
        bool offensive;

        // 初始化为非退化
        *degen = false;
        // 找到第一个非零半平面，初始化楔形区域
        for (i = 0; i != m; i = next[i])
        {
            if (!unit2(halves[i], ccw_vec))
            {
                /* 顺时针边界：将ccw_vec旋转-90度 */
                cw_vec[0] = ccw_vec[1];
                cw_vec[1] = -ccw_vec[0];
                /* 逆时针边界：cw_vec的反向 */
                ccw_vec[0] = -cw_vec[0];
                ccw_vec[1] = -cw_vec[1];
                break;
            }
        }
        // 如果所有半平面都是零向量，问题无约束
        if (i == m)
        {
            return UNBOUNDED;
        }
        i = 0;
        // 遍历所有半平面，逐步收紧楔形区域
        while (i != m)
        {
            offensive = false;
            // 计算当前楔形边界与半平面法向量的点积
            d_cw = dot2(cw_vec, halves[i]);
            d_ccw = dot2(ccw_vec, halves[i]);
            // 情况1: 半平面违反顺时针边界
            if (d_ccw >= 2.0 * eps)
            {
                if (d_cw <= -2.0 * eps)
                {
                    // 更新顺时针边界为半平面的切线方向（法向量旋转90度）
                    cw_vec[0] = halves[i][1];
                    cw_vec[1] = -halves[i][0];
                    unit2(cw_vec, cw_vec);
                    offensive = true;
                }
            }
            // 情况2: 半平面违反逆时针边界
            else if (d_cw >= 2.0 * eps)
            {
                if (d_ccw <= -2.0 * eps)
                {
                    // 更新逆时针边界为半平面的切线方向（法向量旋转-90度）
                    ccw_vec[0] = -halves[i][1];
                    ccw_vec[1] = halves[i][0];
                    unit2(ccw_vec, ccw_vec);
                    offensive = true;
                }
            }
            // 情况3: 半平面同时违反两个边界，不可行
            else if (d_ccw <= -2.0 * eps &&
                     d_cw <= -2.0 * eps)
            {
                return INFEASIBLE;
            }
            // 情况4: 退化情况（可行域退化为一条射线）
            else if (d_cw <= -2.0 * eps ||
                     d_ccw <= -2.0 * eps ||
                     cross2(cw_vec, halves[i]) < 0.0)
            {
                /* 退化 */
                if (d_cw <= -2.0 * eps)
                {
                    // 顺时针边界违反，两边界重合在逆时针边界
                    unit2(ccw_vec, cw_vec);
                }
                else if (d_ccw <= -2.0 * eps)
                {
                    // 逆时针边界违反，两边界重合在顺时针边界
                    unit2(cw_vec, ccw_vec);
                }
                *degen = true;
                offensive = true;
            }
            /* 将这个违反约束的平面移到链表前端 */
            if (offensive)
            {
                i = move_to_front(i, next, prev);
            }
            i = next[i];
            // 如果已经退化，跳出循环
            if (*degen)
            {
                break;
            }
        }
        // 退化情况下，继续检查剩余的半平面约束
        if (*degen)
        {
            while (i != m)
            {
                d_cw = dot2(cw_vec, halves[i]);
                d_ccw = dot2(ccw_vec, halves[i]);
                if (d_cw < -2.0 * eps)
                {
                    if (d_ccw < -2.0 * eps)
                    {
                        // 两个边界都违反约束，不可行
                        return INFEASIBLE;
                    }
                    else
                    {
                        // 顺时针边界违反，收敛到逆时针边界
                        cw_vec[0] = ccw_vec[0];
                        cw_vec[1] = ccw_vec[1];
                    }
                }
                else if (d_ccw < -2.0 * eps)
                {
                    // 逆时针边界违反，收敛到顺时针边界
                    ccw_vec[0] = cw_vec[0];
                    ccw_vec[1] = cw_vec[1];
                }
                i = next[i];
            }
        }
        return MINIMUM;
    }

    /* 在投影线（单位圆）上返回最小值 - 线性规划的基础情况（1维） */
    // 这是递归算法的基础情况，处理1维线性规划问题
    // 参数: halves --- 半平面数组（法向量）
    //       m      --- 终止标记
    //       n_vec  --- 分子函数向量
    //       d_vec  --- 分母函数向量
    //       opt    --- 输出的最优解
    //       next, prev  --- 双向链表的索引数组
    // 返回: 优化状态
    inline int lp_base_case(const double (*halves)[2], /* halves --- half lines */
                            const int m,               /* m      --- terminal marker */
                            const double n_vec[2],     /* n_vec  --- numerator funciton */
                            const double d_vec[2],     /* d_vec  --- denominator function */
                            double opt[2],             /* opt    --- optimum  */
                            int *next,                 /* next, prev  --- double linked list of indices */
                            int *prev)
    {
        double cw_vec[2], ccw_vec[2];
        bool degen;
        int status;

        /* 找到直线（单位圆）上的可行域 */
        status = wedge(halves, m, next, prev, cw_vec, ccw_vec, &degen);

        if (status == INFEASIBLE)
        {
            return status;
        }
        /* 平面上没有非平凡约束：返回无约束最优解 */
        if (status == UNBOUNDED)
        {
            return lp_no_con<1>(n_vec, d_vec, opt);
        }

        // 检查分子和分母向量是否线性相关（叉积接近零）
        if (std::fabs(cross2(n_vec, d_vec)) < 2.0 * eps * eps)
        {
            if (dot2(n_vec, n_vec) < 2.0 * eps * eps ||
                dot2(d_vec, d_vec) > 2.0 * eps * eps)
            {
                /* 分子为零或分子分母线性相关 */
                // 函数值在可行域内为常数，任意可行点都是最优解
                opt[0] = cw_vec[0];
                opt[1] = cw_vec[1];
                status = AMBIGUOUS;
            }
            else
            {
                /* 分子非零且分母为零，在圆上最小化线性函数 */
                if (!degen &&
                    cross2(cw_vec, n_vec) <= 0.0 &&
                    cross2(n_vec, ccw_vec) <= 0.0)
                {
                    /* 最优点在可行域内部 */
                    // 最优方向是-n_vec方向
                    opt[0] = -n_vec[0];
                    opt[1] = -n_vec[1];
                }
                else if (dot2(n_vec, cw_vec) > dot2(n_vec, ccw_vec))
                {
                    /* 最优点在逆时针边界 */
                    opt[0] = ccw_vec[0];
                    opt[1] = ccw_vec[1];
                }
                else
                {
                    /* 最优点在顺时针边界 */
                    opt[0] = cw_vec[0];
                    opt[1] = cw_vec[1];
                }
                status = MINIMUM;
            }
        }
        else
        {
            /* 分子和分母都非零 */
            // 在楔形区域上最小化线性分式函数
            lp_min_lin_rat(degen, cw_vec, ccw_vec, n_vec, d_vec, opt);
            status = MINIMUM;
        }
        return status;
    }

    /* 找到平面方程中绝对值最大的系数 */
    // 用于选择消元变量，选择系数最大的变量可以提高数值稳定性
    // 模板参数: d - 维度参数
    // 参数: pln  - 平面方程系数数组（d+1维）
    //       imax - 输出最大系数的索引
    template <int d>
    inline void findimax(const double *pln,
                         int *imax)
    {
        *imax = 0;
        double rmax = std::fabs(pln[0]);
        // 遍历所有系数，找到绝对值最大的
        for (int i = 1; i <= d; i++)
        {
            const double ab = std::fabs(pln[i]);
            if (ab > rmax)
            {
                *imax = i;
                rmax = ab;
            }
        }
    }

    // 向量升维操作：从d维向量恢复到d+1维向量（回代过程）
    // 在消元后，通过约束方程恢复被消去的变量
    // 模板参数: d - 目标维度参数（实际维度为d+1）
    // 参数: equation   - 约束平面方程（用于消元的方程）
    //       ivar       - 被消去的变量索引
    //       low_vector - 低维向量（d维）
    //       vector     - 输出的高维向量（d+1维）
    template <int d>
    inline void vector_up(const double *equation,
                          const int ivar,
                          const double *low_vector,
                          double *vector)
    {
        vector[ivar] = 0.0;
        // 复制低维向量的分量，并计算被消去变量的值
        for (int i = 0; i <= d; i++)
        {
            if (i != ivar)
            {
                // 计算低维向量中对应的索引
                const int j = i < ivar ? i : i - 1;
                vector[i] = low_vector[j];
                // 累加用于求解第ivar个变量的值
                vector[ivar] -= equation[i] * low_vector[j];
            }
        }
        // 通过约束方程求解第ivar个变量
        vector[ivar] /= equation[ivar];
    }

    // 向量降维操作：将d+1维向量投影到d维空间（消元过程）
    // 通过消去第ivar个变量，将向量投影到消元方程定义的超平面上
    // 模板参数: d - 原始维度参数（实际维度为d+1）
    // 参数: elim_eqn - 消元方程（定义要投影到的超平面）
    //       ivar     - 要消去的变量索引
    //       old_vec  - 原始高维向量（d+1维）
    //       new_vec  - 输出的低维向量（d维）
    template <int d>
    inline void vector_down(const double *elim_eqn,
                            const int ivar,
                            const double *old_vec,
                            double *new_vec)
    {
        // 计算old_vec与消元方程的点积
        double ve = 0.0;
        // 计算消元方程的模长平方
        double ee = 0.0;
        for (int i = 0; i <= d; i++)
        {
            ve += old_vec[i] * elim_eqn[i];
            ee += elim_eqn[i] * elim_eqn[i];
        }
        // 投影系数
        const double fac = ve / ee;
        // 投影：old_vec减去其在elim_eqn方向的分量，然后去掉第ivar维
        for (int i = 0; i <= d; i++)
        {
            if (i != ivar)
            {
                new_vec[i < ivar ? i : i - 1] =
                    old_vec[i] - elim_eqn[i] * fac;
            }
        }
    }

    // 平面降维操作：通过高斯消元将平面方程降到低一维空间
    // 消去第ivar个变量，将平面方程投影到d维空间
    // 模板参数: d - 原始维度参数（实际维度为d+1）
    // 参数: elim_eqn  - 消元方程
    //       ivar      - 要消去的变量索引
    //       old_plane - 原始平面方程（d+1维）
    //       new_plane - 输出的低维平面方程（d维）
    template <int d>
    inline void plane_down(const double *elim_eqn,
                           const int ivar,
                           const double *old_plane,
                           double *new_plane)
    {
        // 计算消元系数，使得新平面方程的第ivar项为0
        const double crit = old_plane[ivar] / elim_eqn[ivar];
        // 用消元方程消去old_plane的第ivar项
        for (int i = 0; i <= d; i++)
        {
            if (i != ivar)
            {
                new_plane[i < ivar ? i : i - 1] =
                    old_plane[i] - elim_eqn[i] * crit;
            }
        }
    }

    // 线性分式规划求解器 - Seidel递归算法的核心函数
    // 求解: min dot(x, n_vec) / dot(x, d_vec)
    //       s.t. halves[i]·x >= 0, for all i
    // 模板参数: d - 维度参数（实际空间维度为d+1）
    template <int d>
    inline int linfracprog(const double *halves, /* halves  --- 半空间约束数组 */
                           const int max_size,   /* max_size --- halves数组的大小 */
                           const int m,          /* m       --- 终止标记 */
                           const double *n_vec,  /* n_vec   --- 目标函数分子向量 */
                           const double *d_vec,  /* d_vec   --- 目标函数分母向量 */
                           double *opt,          /* opt     --- 最优解 */
                           double *work,         /* work    --- 工作空间（见下文） */
                           int *next,            /* next    --- halves索引数组 */
                           int *prev)            /* prev    --- halves索引数组 */
    /*
    **
    ** 半空间约束形式：
    ** halves[i][0]*x[0] + halves[i][1]*x[1] +
    ** ... + halves[i][d-1]*x[d-1] + halves[i][d]*x[d] >= 0
    **
    ** 系数应该被归一化
    ** 半空间应该以随机顺序排列
    ** 半空间的顺序为: 0, next[0], next[next[0]], ...
    ** 并且满足 prev[next[i]] = i
    **
    ** halves: (max_size) x (d+1) 矩阵
    **
    ** 最优解已经为以下半空间计算完成：
    ** 0, next[0], next[next[0]], ... , prev[0]
    ** 下一个需要测试的平面是 0
    **
    ** m 是第一个不在链表中的平面索引
    ** 即 m 是链表的终止标记
    **
    ** 目标函数是 dot(x, n_vec) / dot(x, d_vec)
    ** 如果要求解标准的d维线性规划问题，则设置：
    ** n_vec = (x0, x1, x2, ..., xd-1, 0)
    ** d_vec = ( 0,  0,  0, ...,    0, 1)
    ** halves[0] = (0, 0, ... , 1)
    **
    ** work 指向 (max_size+3)*(d+2)*(d-1)/2 个double的工作空间
    */
    {
        int status, imax;
        double *new_opt, *new_n_vec, *new_d_vec, *new_halves, *new_work;
        const double *plane_i;

        // 检查分母向量是否为零向量
        double val = 0.0;
        for (int j = 0; j <= d; j++)
        {
            val += d_vec[j] * d_vec[j];
        }
        const bool d_vec_zero = (val < (d + 1) * eps * eps);

        /* 首先求解无约束最小值 */
        status = lp_no_con<d>(n_vec, d_vec, opt);
        if (m <= 0)
        {
            // 没有约束，直接返回无约束解
            return status;
        }

        /* 为下一层递归分配内存 */
        // 从work空间中依次分配各个数据结构所需的空间
        new_opt = work;
        new_n_vec = new_opt + d;
        new_d_vec = new_n_vec + d;
        new_halves = new_d_vec + d;
        new_work = new_halves + max_size * d;
        // 增量式算法：逐个检查约束，如果违反则递归求解
        for (int i = 0; i != m; i = next[i])
        {
            /* 如果最优解不在半空间i内，则将问题投影到该平面上 */
            plane_i = halves + i * (d + 1);
            /* 判断最优解是否在plane_i的正确一侧 */
            val = 0.0;
            for (int j = 0; j <= d; j++)
            {
                val += opt[j] * plane_i[j];
            }
            // 如果违反约束（点积为负），需要将问题投影到该平面上
            if (val < -(d + 1) * eps)
            {
                /* 找到系数中绝对值最大的，用于消元 */
                findimax<d>(plane_i, &imax);
                /* 消去该变量 */
                if (i != 0)
                {
                    // 将之前的所有约束投影到plane_i定义的超平面上
                    const double fac = 1.0 / plane_i[imax];
                    for (int j = 0; j != i; j = next[j])
                    {
                        const double *old_plane = halves + j * (d + 1);
                        const double crit = old_plane[imax] * fac;
                        double *new_plane = new_halves + j * d;
                        for (int k = 0; k <= d; k++)
                        {
                            const int l = k < imax ? k : k - 1;
                            new_plane[l] = k != imax ? old_plane[k] - plane_i[k] * crit : new_plane[l];
                        }
                    }
                }
                /* 将目标函数投影到低维空间 */
                if (d_vec_zero)
                {
                    // 分母为零的情况：只投影分子
                    vector_down<d>(plane_i, imax, n_vec, new_n_vec);
                    for (int j = 0; j < d; j++)
                    {
                        new_d_vec[j] = 0.0;
                    }
                }
                else
                {
                    // 一般情况：同时投影分子和分母
                    plane_down<d>(plane_i, imax, n_vec, new_n_vec);
                    plane_down<d>(plane_i, imax, d_vec, new_d_vec);
                }
                /* 递归求解低维子问题 */
                status = linfracprog<d - 1>(new_halves, max_size, i, new_n_vec,
                                            new_d_vec, new_opt, new_work, next, prev);
                /* 回代：将低维解升维到原空间 */
                if (status != INFEASIBLE)
                {
                    vector_up<d>(plane_i, imax, new_opt, opt);

                    /* 归一化最优点（内联代码） */
                    double mag = 0.0;
                    for (int j = 0; j <= d; j++)
                    {
                        mag += opt[j] * opt[j];
                    }
                    mag = 1.0 / sqrt(mag);
                    for (int j = 0; j <= d; j++)
                    {
                        opt[j] *= mag;
                    }
                }
                else
                {
                    return status;
                }
                /* 将这个违反约束的平面移到链表前端 */
                i = move_to_front(i, next, prev);
            }
        }
        return status;
    }

    // 模板特化：1维情况的线性分式规划（递归终止条件）
    // 当维度降到1时，调用基础情况求解器
    template <>
    inline int linfracprog<1>(const double *halves,
                              const int max_size,
                              const int m,
                              const double *n_vec,
                              const double *d_vec,
                              double *opt,
                              double *work,
                              int *next,
                              int *prev)
    {
        if (m > 0)
        {
            // 有约束：调用1维基础情况求解器
            return lp_base_case((const double(*)[2])halves, m,
                                n_vec, d_vec, opt, next, prev);
        }
        else
        {
            // 无约束：直接返回无约束最优解
            return lp_no_con<1>(n_vec, d_vec, opt);
        }
    }

    // 生成随机排列：使用Fisher-Yates洗牌算法
    // 将数组[0, 1, 2, ..., n-1]随机打乱
    // 参数: n - 数组长度
    //       p - 输出的排列数组
    inline void rand_permutation(const int n,
                                 int *p)
    {
        typedef std::uniform_int_distribution<int> rand_int;
        typedef rand_int::param_type rand_range;
        static std::mt19937_64 gen;  // Mersenne Twister 64位随机数生成器
        static rand_int rdi(0, 1);
        int j, k;
        // 初始化为顺序排列
        for (int i = 0; i < n; i++)
        {
            p[i] = i;
        }
        // Fisher-Yates洗牌：从前向后，每次与后面的随机位置交换
        for (int i = 0; i < n; i++)
        {
            rdi.param(rand_range(0, n - i - 1));
            j = rdi(gen) + i;  // 在[i, n-1]范围内随机选择
            k = p[j];
            p[j] = p[i];
            p[i] = k;
        }
    }

    // 标准线性规划求解器（Eigen接口）
    // 求解: min c^T * x
    //       s.t. A * x <= b
    // 这是一个高层接口函数，将标准LP问题转换为线性分式规划问题
    // 模板参数: d - 决策变量的维度
    // 参数: c - 目标函数系数向量（d维）
    //       A - 约束矩阵（m×d，m为约束数量）
    //       b - 约束右端向量（m维）
    //       x - 输出的最优解（d维）
    // 返回: 最优目标函数值，如果无界返回-INFINITY
    // 注意: dim(x) << dim(b) 表示决策变量维度远小于约束数量（小维度LP）
    template <int d>
    inline double linprog(const Eigen::Matrix<double, d, 1> &c,
                          const Eigen::Matrix<double, -1, d> &A,
                          const Eigen::Matrix<double, -1, 1> &b,
                          Eigen::Matrix<double, d, 1> &x)
    /*
    **  min c^T*x, s.t. A*x <= b
    **  dim(x) << dim(b)
    */
    {
        // m为半空间总数（约束数+1个额外约束用于标准化）
        int m = b.size() + 1;
        x.setZero();
        if (m <= 1)
        {
            // 无约束情况：如果c非零则无界，否则最优值为0
            return c.cwiseAbs().maxCoeff() > 0.0 ? -INFINITY : 0.0;
        }

        // 分配数据结构
        Eigen::VectorXi perm(m - 1);  // 随机排列
        Eigen::VectorXi next(m);      // 双向链表next指针
        /* 原始分配大小为m，这里改为m+1以便合法访问尾部 */
        Eigen::VectorXi prev(m + 1);  // 双向链表prev指针
        Eigen::Matrix<double, d + 1, 1> n_vec;  // 分子向量（齐次坐标）
        Eigen::Matrix<double, d + 1, 1> d_vec;  // 分母向量（齐次坐标）
        Eigen::Matrix<double, d + 1, 1> opt;    // 最优解（齐次坐标）
        Eigen::Matrix<double, d + 1, -1, Eigen::ColMajor> halves(d + 1, m);  // 半空间数组
        Eigen::VectorXd work((m + 3) * (d + 2) * (d - 1) / 2);  // 工作空间

        // 构造半空间约束
        // 第0列：x_d >= 1（标准化约束，用于齐次坐标）
        halves.col(0).setZero();
        halves(d, 0) = 1.0;
        // 其余列：-A^T * x + b >= 0，即 A*x <= b
        halves.topRightCorner(d, m - 1) = -A.transpose();
        halves.bottomRightCorner(1, m - 1) = b.transpose();
        /* 归一化所有半空间，这是linfracprog的要求 */
        halves.colwise().normalize();
        // 构造目标函数（线性规划转为线性分式规划）
        // 最小化 c^T*x 等价于最小化 (c^T*x) / 1
        n_vec.head(d) = c;   // 分子：c^T*x
        n_vec(d) = 0.0;
        d_vec.setZero();     // 分母：常数1
        d_vec(d) = 1.0;

        /* 随机化输入平面的顺序 */
        // Seidel算法的期望复杂度依赖于随机顺序
        rand_permutation(m - 1, perm.data());
        /* 0的前驱实际上从未使用 */
        prev(0) = 0;
        /* 在开头链接0号位置 */
        next(0) = perm(0) + 1;
        prev(perm(0) + 1) = 0;
        /* 链接其他平面，构建双向链表 */
        for (int i = 0; i < m - 2; i++)
        {
            next(perm(i) + 1) = perm(i + 1) + 1;
            prev(perm(i + 1) + 1) = perm(i) + 1;
        }
        /* 标记最后一个平面 */
        next(perm(m - 2) + 1) = m;

        // 调用核心线性分式规划求解器
        int status = sdlp::linfracprog<d>(halves.data(), m, m,
                                          n_vec.data(), d_vec.data(),
                                          opt.data(), work.data(),
                                          next.data(), prev.data());

        /* 处理linprog的状态，其定义与linfracprog不同 */
        double minimum = INFINITY;
        if (status != sdlp::INFEASIBLE)
        {
            // 有界解：从齐次坐标转换为笛卡尔坐标
            if (opt(d) != 0.0 && status != sdlp::UNBOUNDED)
            {
                x = opt.head(d) / opt(d);  // 齐次坐标归一化
                minimum = c.dot(x);
            }

            // 无界解：opt(d) == 0表示解在无穷远处
            if (opt(d) == 0.0 || status == sdlp::UNBOUNDED)
            {
                x = opt.head(d);  // 无界方向
                minimum = -INFINITY;
            }
        }
        // status == INFEASIBLE时，minimum保持为INFINITY

        return minimum;
    }

} // namespace sdlp

#endif
