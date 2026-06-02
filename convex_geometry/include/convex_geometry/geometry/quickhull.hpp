// Public domain code from Antti Kuukka
// https://github.com/akuukka/quickhull
// Many thanks to him.
// 公共领域代码，来自 Antti Kuukka
// QuickHull算法实现：用于计算3D点云的凸包

#ifndef QUICKHULL_HPP
#define QUICKHULL_HPP

#include <iostream>
#include <cmath>
#include <limits>
#include <memory>
#include <algorithm>
#include <array>
#include <vector>
#include <deque>
#include <unordered_map>
#include <fstream>
#include <cassert>

namespace quickhull
{

    // ----------------------- BasicData Part -----------------------
    // ----------------------- 基础数据结构部分 -----------------------

    /**
     * @brief 三维向量类
     * @tparam T 坐标类型（通常为float或double）
     */
    template <typename T>
    class Vector3
    {
    public:
        Vector3() = default;

        Vector3(const T &x, const T &y, const T &z) : x(x), y(y), z(z) {}

        T x, y, z; // 三维坐标分量

        /**
         * @brief 计算与另一个向量的点积
         */
        inline T dotProduct(const Vector3 &other) const
        {
            return x * other.x + y * other.y + z * other.z;
        }

        /**
         * @brief 向量减法运算符
         */
        inline Vector3 operator-(const Vector3 &other) const
        {
            return Vector3(x - other.x, y - other.y, z - other.z);
        }

        /**
         * @brief 向量加法运算符
         */
        inline Vector3 operator+(const Vector3 &other) const
        {
            return Vector3(x + other.x, y + other.y, z + other.z);
        }

        /**
         * @brief 向量加法赋值运算符
         */
        inline Vector3 &operator+=(const Vector3 &other)
        {
            x += other.x;
            y += other.y;
            z += other.z;
            return *this;
        }

        /**
         * @brief 向量减法赋值运算符
         */
        inline Vector3 &operator-=(const Vector3 &other)
        {
            x -= other.x;
            y -= other.y;
            z -= other.z;
            return *this;
        }

        /**
         * @brief 标量乘法赋值运算符
         */
        inline Vector3 &operator*=(const T &c)
        {
            x *= c;
            y *= c;
            z *= c;
            return *this;
        }

        /**
         * @brief 标量除法赋值运算符
         */
        inline Vector3 &operator/=(const T &c)
        {
            x /= c;
            y /= c;
            z /= c;
            return *this;
        }

        /**
         * @brief 向量取负运算符
         */
        inline Vector3 operator-() const
        {
            return Vector3(-x, -y, -z);
        }

        /**
         * @brief 标量乘法运算符
         */
        template <typename S>
        inline Vector3 operator*(const S &c) const
        {
            return Vector3(x * c, y * c, z * c);
        }

        /**
         * @brief 标量除法运算符
         */
        template <typename S>
        inline Vector3 operator/(const S &c) const
        {
            return Vector3(x / c, y / c, z / c);
        }

        /**
         * @brief 获取向量长度的平方
         */
        inline T getLengthSquared() const
        {
            return x * x + y * y + z * z;
        }

        /**
         * @brief 不等于运算符
         */
        inline bool operator!=(const Vector3 &o) const
        {
            return x != o.x || y != o.y || z != o.z;
        }

        /**
         * @brief 计算到另一个点的距离平方
         */
        inline T getSquaredDistanceTo(const Vector3 &other) const
        {
            const T dx = x - other.x;
            const T dy = y - other.y;
            const T dz = z - other.z;
            return dx * dx + dy * dy + dz * dz;
        }
    };

    /**
     * @brief 重载输出流运算符，用于调试打印
     */
    template <typename T>
    inline std::ostream &operator<<(std::ostream &os, const Vector3<T> &vec)
    {
        os << "(" << vec.x << "," << vec.y << "," << vec.z << ")";
        return os;
    }

    /**
     * @brief 标量与向量的乘法运算符（标量在左侧）
     */
    template <typename T>
    inline Vector3<T> operator*(T c, const Vector3<T> &v)
    {
        return Vector3<T>(v.x * c, v.y * c, v.z * c);
    }

    /**
     * @brief 平面类
     * 表示3D空间中的平面，使用法向量和距离原点的距离表示
     */
    template <typename T>
    class Plane
    {
    public:
        Vector3<T> m_N; // 平面法向量

        // 平面到原点的有符号距离（当法向量长度为1时）
        T m_D;

        // 法向量长度的平方
        T m_sqrNLength;

        /**
         * @brief 判断点是否在平面的正侧
         * @param Q 待判断的点
         * @return 如果点在平面正侧返回true
         */
        inline bool isPointOnPositiveSide(const Vector3<T> &Q) const
        {
            T d = m_N.dotProduct(Q) + m_D;
            if (d >= 0)
                return true;
            return false;
        }

        Plane() = default;

        /**
         * @brief 使用法向量和平面上的一点构造平面
         * @param N 平面法向量
         * @param P 平面上的任意一点
         */
        Plane(const Vector3<T> &N, const Vector3<T> &P)
            : m_N(N), m_D(-N.dotProduct(P)),
              m_sqrNLength(m_N.x * m_N.x + m_N.y * m_N.y + m_N.z * m_N.z) {}
    };

    /**
     * @brief 射线结构
     * 由起点和方向向量定义的射线
     */
    template <typename T>
    struct Ray
    {
        const Vector3<T> m_S;            // 射线起点
        const Vector3<T> m_V;            // 射线方向向量
        const T m_VInvLengthSquared;     // 方向向量长度平方的倒数

        Ray(const Vector3<T> &S, const Vector3<T> &V)
            : m_S(S), m_V(V), m_VInvLengthSquared(T{1} / m_V.getLengthSquared()) {}
    };

    /**
     * @brief 对象池类
     * 用于重用对象，减少内存分配和释放的开销
     */
    template <typename S>
    class Pool
    {
        std::vector<std::unique_ptr<S>> m_data; // 对象池存储

    public:
        /**
         * @brief 清空对象池
         */
        inline void clear()
        {
            m_data.clear();
        }

        /**
         * @brief 回收对象到池中
         */
        inline void reclaim(std::unique_ptr<S> &ptr)
        {
            m_data.push_back(std::move(ptr));
        }

        /**
         * @brief 从池中获取对象
         * @return 如果池为空则创建新对象，否则返回池中的对象
         */
        inline std::unique_ptr<S> get()
        {
            if (m_data.empty())
            {
                return std::unique_ptr<S>(new S());
            }
            auto it = m_data.end() - 1;
            std::unique_ptr<S> r = std::move(*it);
            m_data.erase(it);
            return r;
        }
    };

    /**
     * @brief 顶点数据源类
     * 对顶点数据的轻量级包装，避免数据拷贝
     */
    template <typename T>
    class VertexDataSource
    {
        const Vector3<T> *m_ptr;  // 指向顶点数据的指针
        size_t m_count;           // 顶点数量

    public:
        VertexDataSource(const Vector3<T> *ptr, size_t count)
            : m_ptr(ptr), m_count(count) {}

        VertexDataSource(const std::vector<Vector3<T>> &vec)
            : m_ptr(&vec[0]), m_count(vec.size()) {}

        VertexDataSource()
            : m_ptr(nullptr), m_count(0) {}

        inline VertexDataSource &operator=(const VertexDataSource &other) = default;

        /**
         * @brief 获取顶点数量
         */
        inline size_t size() const
        {
            return m_count;
        }

        /**
         * @brief 通过索引访问顶点
         */
        inline const Vector3<T> &operator[](size_t index) const
        {
            return m_ptr[index];
        }

        /**
         * @brief 获取起始迭代器
         */
        inline const Vector3<T> *begin() const
        {
            return m_ptr;
        }

        /**
         * @brief 获取结束迭代器
         */
        inline const Vector3<T> *end() const
        {
            return m_ptr + m_count;
        }
    };

    // ----------------------- MeshBuilder Part -----------------------
    // ----------------------- 网格构建器部分 -----------------------

    /**
     * @brief 网格构建器类
     * 使用半边数据结构构建凸包网格
     */
    template <typename T>
    class MeshBuilder
    {
    public:
        /**
         * @brief 半边结构
         * 半边数据结构的核心元素，每条边由两条相对的半边组成
         */
        struct HalfEdge
        {
            size_t m_endVertex;  // 半边的终点顶点索引
            size_t m_opp;        // 相对半边的索引
            size_t m_face;       // 半边所属面的索引
            size_t m_next;       // 面上的下一条半边索引

            /**
             * @brief 禁用半边
             */
            inline void disable()
            {
                m_endVertex = std::numeric_limits<size_t>::max();
            }

            /**
             * @brief 检查半边是否被禁用
             */
            inline bool isDisabled() const
            {
                return m_endVertex == std::numeric_limits<size_t>::max();
            }
        };

        /**
         * @brief 面结构
         * 表示凸包网格中的一个三角形面
         */
        struct Face
        {
            size_t m_he;                         // 面上的一条半边索引
            Plane<T> m_P;                        // 面所在的平面
            T m_mostDistantPointDist;            // 面正侧最远点的距离
            size_t m_mostDistantPoint;           // 面正侧最远点的索引
            size_t m_visibilityCheckedOnIteration; // 可见性检查的迭代计数
            std::uint8_t m_isVisibleFaceOnCurrentIteration : 1; // 当前迭代中面是否可见
            std::uint8_t m_inFaceStack : 1;      // 面是否在面栈中

            // 每条半边对应一个位，0或1表示该边是否属于地平线边
            std::uint8_t m_horizonEdgesOnCurrentIteration : 3;
            std::unique_ptr<std::vector<size_t>> m_pointsOnPositiveSide; // 面正侧的点集

            Face() : m_he(std::numeric_limits<size_t>::max()),
                     m_P(),
                     m_mostDistantPointDist(0),
                     m_mostDistantPoint(0),
                     m_visibilityCheckedOnIteration(0),
                     m_isVisibleFaceOnCurrentIteration(0),
                     m_inFaceStack(0),
                     m_horizonEdgesOnCurrentIteration(0) {}

            /**
             * @brief 禁用面
             */
            inline void disable()
            {
                m_he = std::numeric_limits<size_t>::max();
            }

            /**
             * @brief 检查面是否被禁用
             */
            inline bool isDisabled() const
            {
                return m_he == std::numeric_limits<size_t>::max();
            }
        };

        // 网格数据
        std::vector<Face> m_faces;          // 面集合
        std::vector<HalfEdge> m_halfEdges;  // 半边集合

        // 当网格被修改且面和半边从中移除时，我们不会真正从容器中删除它们
        // 而是将它们标记为禁用，这意味着当需要向网格添加新面和半边时
        // 可以重用这些索引。我们将空闲索引存储在以下向量中。
        std::vector<size_t> m_disabledFaces, m_disabledHalfEdges;

        /**
         * @brief 添加新面
         * @return 新面的索引，如果有可重用的禁用面则重用，否则创建新面
         */
        inline size_t addFace()
        {
            if (m_disabledFaces.size())
            {
                size_t index = m_disabledFaces.back();
                auto &f = m_faces[index];
                assert(f.isDisabled());
                assert(!f.m_pointsOnPositiveSide);
                f.m_mostDistantPointDist = 0;
                m_disabledFaces.pop_back();
                return index;
            }
            m_faces.emplace_back();
            return m_faces.size() - 1;
        }

        /**
         * @brief 添加新半边
         * @return 新半边的索引，如果有可重用的禁用半边则重用，否则创建新半边
         */
        inline size_t addHalfEdge()
        {
            if (m_disabledHalfEdges.size())
            {
                const size_t index = m_disabledHalfEdges.back();
                m_disabledHalfEdges.pop_back();
                return index;
            }
            m_halfEdges.emplace_back();
            return m_halfEdges.size() - 1;
        }

        /**
         * @brief 禁用面并返回面正侧的点集
         * @param faceIndex 要禁用的面索引
         * @return 面正侧的点集指针
         */
        inline std::unique_ptr<std::vector<size_t>> disableFace(size_t faceIndex)
        {
            auto &f = m_faces[faceIndex];
            f.disable();
            m_disabledFaces.push_back(faceIndex);
            return std::move(f.m_pointsOnPositiveSide);
        }

        /**
         * @brief 禁用半边
         * @param heIndex 要禁用的半边索引
         */
        inline void disableHalfEdge(size_t heIndex)
        {
            auto &he = m_halfEdges[heIndex];
            he.disable();
            m_disabledHalfEdges.push_back(heIndex);
        }

        MeshBuilder() = default;

        /**
         * @brief 创建初始四面体网格 ABCD
         * AB 与三角形 ABC 的法向量的点积应该为负
         * @param a, b, c, d 四个顶点的索引
         */
        inline void setup(size_t a, size_t b, size_t c, size_t d)
        {
            m_faces.clear();
            m_halfEdges.clear();
            m_disabledFaces.clear();
            m_disabledHalfEdges.clear();

            m_faces.reserve(4);
            m_halfEdges.reserve(12);

            // 创建半边
            // 四面体有4个面，每个面3条边，共12条半边
            HalfEdge AB;
            AB.m_endVertex = b;
            AB.m_opp = 6;
            AB.m_face = 0;
            AB.m_next = 1;
            m_halfEdges.push_back(AB);

            HalfEdge BC;
            BC.m_endVertex = c;
            BC.m_opp = 9;
            BC.m_face = 0;
            BC.m_next = 2;
            m_halfEdges.push_back(BC);

            HalfEdge CA;
            CA.m_endVertex = a;
            CA.m_opp = 3;
            CA.m_face = 0;
            CA.m_next = 0;
            m_halfEdges.push_back(CA);

            HalfEdge AC;
            AC.m_endVertex = c;
            AC.m_opp = 2;
            AC.m_face = 1;
            AC.m_next = 4;
            m_halfEdges.push_back(AC);

            HalfEdge CD;
            CD.m_endVertex = d;
            CD.m_opp = 11;
            CD.m_face = 1;
            CD.m_next = 5;
            m_halfEdges.push_back(CD);

            HalfEdge DA;
            DA.m_endVertex = a;
            DA.m_opp = 7;
            DA.m_face = 1;
            DA.m_next = 3;
            m_halfEdges.push_back(DA);

            HalfEdge BA;
            BA.m_endVertex = a;
            BA.m_opp = 0;
            BA.m_face = 2;
            BA.m_next = 7;
            m_halfEdges.push_back(BA);

            HalfEdge AD;
            AD.m_endVertex = d;
            AD.m_opp = 5;
            AD.m_face = 2;
            AD.m_next = 8;
            m_halfEdges.push_back(AD);

            HalfEdge DB;
            DB.m_endVertex = b;
            DB.m_opp = 10;
            DB.m_face = 2;
            DB.m_next = 6;
            m_halfEdges.push_back(DB);

            HalfEdge CB;
            CB.m_endVertex = b;
            CB.m_opp = 1;
            CB.m_face = 3;
            CB.m_next = 10;
            m_halfEdges.push_back(CB);

            HalfEdge BD;
            BD.m_endVertex = d;
            BD.m_opp = 8;
            BD.m_face = 3;
            BD.m_next = 11;
            m_halfEdges.push_back(BD);

            HalfEdge DC;
            DC.m_endVertex = c;
            DC.m_opp = 4;
            DC.m_face = 3;
            DC.m_next = 9;
            m_halfEdges.push_back(DC);

            // 创建四个三角形面
            Face ABC;
            ABC.m_he = 0;
            m_faces.push_back(std::move(ABC));

            Face ACD;
            ACD.m_he = 3;
            m_faces.push_back(std::move(ACD));

            Face BAD;
            BAD.m_he = 6;
            m_faces.push_back(std::move(BAD));

            Face CBD;
            CBD.m_he = 9;
            m_faces.push_back(std::move(CBD));
        }

        /**
         * @brief 获取面的三个顶点索引
         * @param f 面
         * @return 三个顶点索引数组
         */
        inline std::array<size_t, 3> getVertexIndicesOfFace(const Face &f) const
        {
            std::array<size_t, 3> v;
            const HalfEdge *he = &m_halfEdges[f.m_he];
            v[0] = he->m_endVertex;
            he = &m_halfEdges[he->m_next];
            v[1] = he->m_endVertex;
            he = &m_halfEdges[he->m_next];
            v[2] = he->m_endVertex;
            return v;
        }

        /**
         * @brief 获取半边的两个端点顶点索引
         * @param he 半边
         * @return 两个端点顶点索引数组[起点, 终点]
         */
        inline std::array<size_t, 2> getVertexIndicesOfHalfEdge(const HalfEdge &he) const
        {
            return {m_halfEdges[he.m_opp].m_endVertex, he.m_endVertex};
        }

        /**
         * @brief 获取面的三条半边索引
         * @param f 面
         * @return 三条半边索引数组
         */
        inline std::array<size_t, 3> getHalfEdgeIndicesOfFace(const Face &f) const
        {
            return {f.m_he, m_halfEdges[f.m_he].m_next, m_halfEdges[m_halfEdges[f.m_he].m_next].m_next};
        }
    };

    // ----------------------- MathUtils Part -----------------------
    // ----------------------- 数学工具部分 -----------------------

    namespace mathutils
    {

        /**
         * @brief 计算点到射线的距离平方
         * @param p 点
         * @param r 射线
         * @return 点到射线的距离平方
         */
        template <typename T>
        inline T getSquaredDistanceBetweenPointAndRay(const Vector3<T> &p, const Ray<T> &r)
        {
            const Vector3<T> s = p - r.m_S;
            T t = s.dotProduct(r.m_V);
            return s.getLengthSquared() - t * t * r.m_VInvLengthSquared;
        }

        /**
         * @brief 计算点到平面的有符号距离
         * 注意：返回的距离单位是相对于平面法向量长度的
         * （如果需要"真实"距离，需要除以法向量的长度）
         * @param v 点
         * @param p 平面
         * @return 有符号距离（正表示在平面正侧）
         */
        template <typename T>
        inline T getSignedDistanceToPlane(const Vector3<T> &v, const Plane<T> &p)
        {
            return p.m_N.dotProduct(v) + p.m_D;
        }

        /**
         * @brief 计算三角形的法向量
         * 使用叉积 (a-c) × (b-c) 计算法向量
         * @param a, b, c 三角形的三个顶点
         * @return 三角形法向量（未归一化）
         */
        template <typename T>
        inline Vector3<T> getTriangleNormal(const Vector3<T> &a,
                                            const Vector3<T> &b,
                                            const Vector3<T> &c)
        {
            // 计算 (a-c).crossProduct(b-c)
            // 避免构造临时向量
            T x = a.x - c.x;
            T y = a.y - c.y;
            T z = a.z - c.z;
            T rhsx = b.x - c.x;
            T rhsy = b.y - c.y;
            T rhsz = b.z - c.z;
            T px = y * rhsz - z * rhsy;
            T py = z * rhsx - x * rhsz;
            T pz = x * rhsy - y * rhsx;
            return Vector3<T>(px, py, pz);
        }

    } // namespace mathutils

    // ----------------------- HalfEdgeMesh Part -----------------------
    // ----------------------- 半边网格部分 -----------------------

    /**
     * @brief 半边网格类
     * 最终的凸包网格表示，使用半边数据结构
     */
    template <typename FloatType, typename IndexType>
    class HalfEdgeMesh
    {
    public:
        /**
         * @brief 半边结构（最终版本）
         */
        struct HalfEdge
        {
            IndexType m_endVertex;  // 终点顶点索引
            IndexType m_opp;        // 相对半边索引
            IndexType m_face;       // 所属面索引
            IndexType m_next;       // 下一条半边索引
        };

        /**
         * @brief 面结构（最终版本）
         */
        struct Face
        {
            // 面上的一条半边索引
            IndexType m_halfEdgeIndex;
        };

        std::vector<Vector3<FloatType>> m_vertices; // 顶点集合
        std::vector<Face> m_faces;                  // 面集合
        std::vector<HalfEdge> m_halfEdges;          // 半边集合

        /**
         * @brief 从MeshBuilder构造半边网格
         * 将构建过程中的临时网格转换为最终的紧凑网格表示
         * @param builderObject 网格构建器对象
         * @param vertexData 顶点数据源
         */
        HalfEdgeMesh(const MeshBuilder<FloatType> &builderObject,
                     const VertexDataSource<FloatType> &vertexData)
        {
            // 创建索引映射，从构建器的索引映射到最终网格的索引
            std::unordered_map<IndexType, IndexType> faceMapping;
            std::unordered_map<IndexType, IndexType> halfEdgeMapping;
            std::unordered_map<IndexType, IndexType> vertexMapping;

            size_t i = 0;
            // 复制所有未禁用的面
            for (const auto &face : builderObject.m_faces)
            {
                if (!face.isDisabled())
                {
                    m_faces.push_back({static_cast<IndexType>(face.m_he)});
                    faceMapping[i] = m_faces.size() - 1;

                    // 收集面上的所有顶点
                    const auto heIndices = builderObject.getHalfEdgeIndicesOfFace(face);
                    for (const auto heIndex : heIndices)
                    {
                        const IndexType vertexIndex = builderObject.m_halfEdges[heIndex].m_endVertex;
                        if (vertexMapping.count(vertexIndex) == 0)
                        {
                            m_vertices.push_back(vertexData[vertexIndex]);
                            vertexMapping[vertexIndex] = m_vertices.size() - 1;
                        }
                    }
                }
                i++;
            }

            i = 0;
            // 复制所有未禁用的半边
            for (const auto &halfEdge : builderObject.m_halfEdges)
            {
                if (!halfEdge.isDisabled())
                {
                    m_halfEdges.push_back({static_cast<IndexType>(halfEdge.m_endVertex),
                                           static_cast<IndexType>(halfEdge.m_opp),
                                           static_cast<IndexType>(halfEdge.m_face),
                                           static_cast<IndexType>(halfEdge.m_next)});
                    halfEdgeMapping[i] = m_halfEdges.size() - 1;
                }
                i++;
            }

            // 更新面的半边索引引用
            for (auto &face : m_faces)
            {
                assert(halfEdgeMapping.count(face.m_halfEdgeIndex) == 1);
                face.m_halfEdgeIndex = halfEdgeMapping[face.m_halfEdgeIndex];
            }

            // 更新半边的所有索引引用
            for (auto &he : m_halfEdges)
            {
                he.m_face = faceMapping[he.m_face];
                he.m_opp = halfEdgeMapping[he.m_opp];
                he.m_next = halfEdgeMapping[he.m_next];
                he.m_endVertex = vertexMapping[he.m_endVertex];
            }
        }
    };

    // ----------------------- Convex Hull Part -----------------------
    // ----------------------- 凸包部分 -----------------------

    /**
     * @brief 凸包类
     * 表示计算得到的凸包，包含顶点和索引缓冲
     */
    template <typename T>
    class ConvexHull
    {
        std::unique_ptr<std::vector<Vector3<T>>> m_optimizedVertexBuffer; // 优化的顶点缓冲
        VertexDataSource<T> m_vertices;  // 顶点数据源
        std::vector<size_t> m_indices;   // 三角形索引缓冲（每3个索引构成一个三角形）

    public:
        ConvexHull() {}

        /**
         * @brief 拷贝构造函数
         */
        ConvexHull(const ConvexHull &o)
        {
            m_indices = o.m_indices;
            if (o.m_optimizedVertexBuffer)
            {
                m_optimizedVertexBuffer.reset(new std::vector<Vector3<T>>(*o.m_optimizedVertexBuffer));
                m_vertices = VertexDataSource<T>(*m_optimizedVertexBuffer);
            }
            else
            {
                m_vertices = o.m_vertices;
            }
        }

        /**
         * @brief 拷贝赋值运算符
         */
        inline ConvexHull &operator=(const ConvexHull &o)
        {
            if (&o == this)
            {
                return *this;
            }
            m_indices = o.m_indices;
            if (o.m_optimizedVertexBuffer)
            {
                m_optimizedVertexBuffer.reset(new std::vector<Vector3<T>>(*o.m_optimizedVertexBuffer));
                m_vertices = VertexDataSource<T>(*m_optimizedVertexBuffer);
            }
            else
            {
                m_vertices = o.m_vertices;
            }
            return *this;
        }

        /**
         * @brief 移动构造函数
         */
        ConvexHull(ConvexHull &&o)
        {
            m_indices = std::move(o.m_indices);
            if (o.m_optimizedVertexBuffer)
            {
                m_optimizedVertexBuffer = std::move(o.m_optimizedVertexBuffer);
                o.m_vertices = VertexDataSource<T>();
                m_vertices = VertexDataSource<T>(*m_optimizedVertexBuffer);
            }
            else
            {
                m_vertices = o.m_vertices;
            }
        }

        /**
         * @brief 移动赋值运算符
         */
        inline ConvexHull &operator=(ConvexHull &&o)
        {
            if (&o == this)
            {
                return *this;
            }
            m_indices = std::move(o.m_indices);
            if (o.m_optimizedVertexBuffer)
            {
                m_optimizedVertexBuffer = std::move(o.m_optimizedVertexBuffer);
                o.m_vertices = VertexDataSource<T>();
                m_vertices = VertexDataSource<T>(*m_optimizedVertexBuffer);
            }
            else
            {
                m_vertices = o.m_vertices;
            }
            return *this;
        }

        /**
         * @brief 从半边网格和点云构造顶点和索引缓冲
         * @param mesh 网格构建器对象
         * @param pointCloud 原始点云
         * @param CCW 是否使用逆时针方向
         * @param useOriginalIndices 是否使用原始索引
         */
        ConvexHull(const MeshBuilder<T> &mesh,
                   const VertexDataSource<T> &pointCloud,
                   bool CCW,
                   bool useOriginalIndices)
        {
            if (!useOriginalIndices)
            {
                m_optimizedVertexBuffer.reset(new std::vector<Vector3<T>>());
            }

            std::vector<std::uint8_t> faceProcessed(mesh.m_faces.size(), 0);
            std::vector<size_t> faceStack;
            // 将原始点云的顶点索引映射到新网格的顶点索引
            std::unordered_map<size_t, size_t> vertexIndexMapping;
            // 找到第一个未禁用的面作为起点
            for (size_t i = 0; i < mesh.m_faces.size(); i++)
            {
                if (!mesh.m_faces[i].isDisabled())
                {
                    faceStack.push_back(i);
                    break;
                }
            }
            if (faceStack.size() == 0)
            {
                return;
            }

            const size_t iCCW = CCW ? 1 : 0; // 根据CCW标志确定顶点顺序
            const size_t finalMeshFaceCount = mesh.m_faces.size() - mesh.m_disabledFaces.size();
            m_indices.reserve(finalMeshFaceCount * 3); // 每个面3个顶点索引

            // 使用深度优先搜索遍历所有面
            while (faceStack.size())
            {
                auto it = faceStack.end() - 1;
                size_t top = *it;
                assert(!mesh.m_faces[top].isDisabled());
                faceStack.erase(it);
                if (faceProcessed[top])
                {
                    continue;
                }
                else
                {
                    faceProcessed[top] = 1;
                    auto halfEdges = mesh.getHalfEdgeIndicesOfFace(mesh.m_faces[top]);
                    // 获取三个相邻面
                    size_t adjacent[] = {mesh.m_halfEdges[mesh.m_halfEdges[halfEdges[0]].m_opp].m_face,
                                         mesh.m_halfEdges[mesh.m_halfEdges[halfEdges[1]].m_opp].m_face,
                                         mesh.m_halfEdges[mesh.m_halfEdges[halfEdges[2]].m_opp].m_face};
                    // 将未处理的相邻面加入栈
                    for (auto a : adjacent)
                    {
                        if (!faceProcessed[a] && !mesh.m_faces[a].isDisabled())
                        {
                            faceStack.push_back(a);
                        }
                    }
                    auto vertices = mesh.getVertexIndicesOfFace(mesh.m_faces[top]);
                    if (!useOriginalIndices)
                    {
                        // 为顶点创建新索引
                        for (auto &v : vertices)
                        {
                            auto itV = vertexIndexMapping.find(v);
                            if (itV == vertexIndexMapping.end())
                            {
                                m_optimizedVertexBuffer->push_back(pointCloud[v]);
                                vertexIndexMapping[v] = m_optimizedVertexBuffer->size() - 1;
                                v = m_optimizedVertexBuffer->size() - 1;
                            }
                            else
                            {
                                v = itV->second;
                            }
                        }
                    }
                    // 按照CCW或CW顺序添加索引
                    m_indices.push_back(vertices[0]);
                    m_indices.push_back(vertices[1 + iCCW]);
                    m_indices.push_back(vertices[2 - iCCW]);
                }
            }

            if (!useOriginalIndices)
            {
                m_vertices = VertexDataSource<T>(*m_optimizedVertexBuffer);
            }
            else
            {
                m_vertices = pointCloud;
            }
        }

        /**
         * @brief 获取索引缓冲
         */
        inline std::vector<size_t> &getIndexBuffer()
        {
            return m_indices;
        }

        /**
         * @brief 获取索引缓冲（const版本）
         */
        inline const std::vector<size_t> &getIndexBuffer() const
        {
            return m_indices;
        }

        /**
         * @brief 获取顶点缓冲
         */
        inline VertexDataSource<T> &getVertexBuffer()
        {
            return m_vertices;
        }

        /**
         * @brief 获取顶点缓冲（const版本）
         */
        inline const VertexDataSource<T> &getVertexBuffer() const
        {
            return m_vertices;
        }

        /**
         * @brief 将网格导出为Wavefront OBJ文件
         * @param filename 输出文件名
         * @param objectName 对象名称
         */
        inline void writeWaveformOBJ(const std::string &filename,
                                     const std::string &objectName = "quickhull") const
        {
            std::ofstream objFile;
            objFile.open(filename);
            objFile << "o " << objectName << "\n";
            for (const auto &v : getVertexBuffer())
            {
                objFile << "v " << v.x << " " << v.y << " " << v.z << "\n";
            }
            const auto &indBuf = getIndexBuffer();
            size_t triangleCount = indBuf.size() / 3;
            for (size_t i = 0; i < triangleCount; i++)
            {
                objFile << "f " << indBuf[i * 3] + 1 << " "
                        << indBuf[i * 3 + 1] + 1 << " "
                        << indBuf[i * 3 + 2] + 1 << "\n";
            }
            objFile.close();
        }
    };

    // ----------------------- Quick Hull Part -----------------------
    // ----------------------- QuickHull算法部分 -----------------------

    /**
     * 3D QuickHull算法实现，作者：Antti Kuukka
     *
     * 无版权限制，100%公共领域代码
     *
     * 输入：3D空间中的点集合（例如，3D网格的顶点）
     *
     * 输出：ConvexHull对象，提供生成的凸包的顶点和索引缓冲
     *       （以三角形网格形式表示）
     *
     * 线程安全性：如果每个线程使用自己的QuickHull对象，则实现是线程安全的
     *
     * 算法概述：
     *   - 使用极值点创建初始单纯形（四面体）
     *     现在有四个面，它们构成凸网格M
     *   - 对于每个点，将其分配给第一个使其位于正侧的面
     *     （每个点最多分配给一个面）。初始四面体内的点
     *     现在被留下，不再影响后续计算
     *   - 将所有分配了点的面添加到面栈中
     *   - 迭代直到面栈为空：
     *       - 从栈中弹出最上面的面F
     *       - 从分配给F的点中，选择距离F定义的平面最远的点P
     *       - 找到M的所有使P在其正侧的面，称为"可见面"
     *       - 由于M的构造方式，这些面是连通的
     *         求解它们的地平线边环
     *       - "向P拉伸"：通过将P与地平线边上的点连接来创建新面
     *         将新面添加到M，并从M中删除可见面
     *       - 分配给可见面的每个点现在最多分配给一个新创建的面
     *       - 将那些分配了点的新面添加到面栈的顶部
     *   - M现在就是凸包
     *
     * 待办事项：
     *   - 实现适当的2D QuickHull来解决退化的2D情况
     *     （当所有点都位于3D空间的同一平面上时）
     */

    /**
     * @brief 诊断数据结构
     */
    struct DiagnosticsData
    {
        // QuickHull无法解决地平线边的次数。失败会导致退化的凸包。
        size_t m_failedHorizonEdges;

        DiagnosticsData() : m_failedHorizonEdges(0) {}
    };

    /**
     * @brief 默认epsilon值
     * 用于数值比较的容差
     */
    template <typename T>
    inline T defaultEps()
    {
        return T{1} / T{10000000};
    }

    /**
     * @brief QuickHull算法主类
     * 计算3D点云的凸包
     */
    template <typename T>
    class QuickHull
    {
        using vec3 = Vector3<T>;

        T m_epsilon, m_epsilonSquared, m_scale;  // 数值容差和点云缩放因子
        bool m_planar;                            // 是否为平面点云
        std::vector<vec3> m_planarPointCloudTemp; // 平面情况下的临时点云
        VertexDataSource<T> m_vertexData;         // 顶点数据源
        MeshBuilder<T> m_mesh;                    // 网格构建器
        std::array<size_t, 6> m_extremeValues;    // 极值点索引（x最大/最小，y最大/最小，z最大/最小）
        DiagnosticsData m_diagnostics;            // 诊断数据

        // 迭代过程中使用的临时变量
        std::vector<size_t> m_newFaceIndices;          // 新创建的面索引
        std::vector<size_t> m_newHalfEdgeIndices;      // 新创建的半边索引
        std::vector<std::unique_ptr<std::vector<size_t>>> m_disabledFacePointVectors; // 禁用面的点集
        std::vector<size_t> m_visibleFaces;            // 可见面索引
        std::vector<size_t> m_horizonEdges;            // 地平线边索引

        /**
         * @brief 面数据结构
         * 用于可见性检查过程
         */
        struct FaceData
        {
            size_t m_faceIndex;            // 面索引
            // 如果面不可见，则此半边将被标记为地平线边
            size_t m_enteredFromHalfEdge;  // 进入面的半边索引
            FaceData(size_t fi, size_t he) : m_faceIndex(fi), m_enteredFromHalfEdge(he) {}
        };
        std::vector<FaceData> m_possiblyVisibleFaces;  // 可能可见的面
        std::deque<size_t> m_faceList;                 // 面列表（待处理的面队列）

        /**
         * @brief 创建表示基础四面体的半边网格
         * 这是QuickHull迭代的起点
         * 调用此函数时，m_extremeValues必须已正确设置
         */
        inline void setupInitialTetrahedron()
        {
            const size_t vertexCount = m_vertexData.size();

            // 如果最多只有4个点，返回退化的四面体
            if (vertexCount <= 4)
            {
                size_t v[4] = {0,
                               std::min((size_t)1, vertexCount - 1),
                               std::min((size_t)2, vertexCount - 1),
                               std::min((size_t)3, vertexCount - 1)};
                const Vector3<T> N = mathutils::getTriangleNormal(m_vertexData[v[0]],
                                                                  m_vertexData[v[1]],
                                                                  m_vertexData[v[2]]);
                const Plane<T> trianglePlane(N, m_vertexData[v[0]]);
                // 确保顶点顺序正确（逆时针方向）
                if (trianglePlane.isPointOnPositiveSide(m_vertexData[v[3]]))
                {
                    std::swap(v[0], v[1]);
                }
                return m_mesh.setup(v[0], v[1], v[2], v[3]);
            }

            // 找到两个最远的极值点
            T maxD = m_epsilonSquared;
            std::pair<size_t, size_t> selectedPoints;
            for (size_t i = 0; i < 6; i++)
            {
                for (size_t j = i + 1; j < 6; j++)
                {
                    const T d = m_vertexData[m_extremeValues[i]].getSquaredDistanceTo(m_vertexData[m_extremeValues[j]]);
                    if (d > maxD)
                    {
                        maxD = d;
                        selectedPoints = {m_extremeValues[i], m_extremeValues[j]};
                    }
                }
            }
            if (maxD == m_epsilonSquared)
            {
                // 退化情况：点云似乎只包含单个点
                return m_mesh.setup(0,
                                    std::min((size_t)1, vertexCount - 1),
                                    std::min((size_t)2, vertexCount - 1),
                                    std::min((size_t)3, vertexCount - 1));
            }
            assert(selectedPoints.first != selectedPoints.second);

            // 找到距离两个选定极值点连线最远的点
            const Ray<T> r(m_vertexData[selectedPoints.first],
                           (m_vertexData[selectedPoints.second] - m_vertexData[selectedPoints.first]));
            maxD = m_epsilonSquared;
            size_t maxI = std::numeric_limits<size_t>::max();
            const size_t vCount = m_vertexData.size();
            for (size_t i = 0; i < vCount; i++)
            {
                const T distToRay = mathutils::getSquaredDistanceBetweenPointAndRay(m_vertexData[i], r);
                if (distToRay > maxD)
                {
                    maxD = distToRay;
                    maxI = i;
                }
            }
            if (maxD == m_epsilonSquared)
            {
                // 点云似乎属于R^3的一维子空间：
                // 凸包没有体积 => 返回退化的三角形
                // 选择任意不同于selectedPoints.first和selectedPoints.second的点
                // 作为三角形的第三个点

                auto it = m_vertexData.begin();
                for (; it != m_vertexData.end() &&
                       !(*it != m_vertexData[selectedPoints.first] && *it != m_vertexData[selectedPoints.second]);
                     it++)
                {
                }
                const size_t thirdPoint = (it == m_vertexData.end()) ? selectedPoints.first : std::distance(m_vertexData.begin(), it);

                for (it = m_vertexData.begin();
                     it != m_vertexData.end() &&
                     !(*it != m_vertexData[selectedPoints.first] &&
                       *it != m_vertexData[selectedPoints.second] &&
                       *it != m_vertexData[thirdPoint]);
                     it++)
                {
                }
                const size_t fourthPoint = (it == m_vertexData.end()) ? selectedPoints.first : std::distance(m_vertexData.begin(), it);

                return m_mesh.setup(selectedPoints.first, selectedPoints.second, thirdPoint, fourthPoint);
            }

            // 这三个点构成四面体的基础三角形
            assert(selectedPoints.first != maxI && selectedPoints.second != maxI);
            std::array<size_t, 3> baseTriangle{selectedPoints.first, selectedPoints.second, maxI};
            const Vector3<T> baseTriangleVertices[] = {m_vertexData[baseTriangle[0]],
                                                       m_vertexData[baseTriangle[1]],
                                                       m_vertexData[baseTriangle[2]]};

            // 下一步是找到四面体的第四个顶点
            // 自然地，我们选择距离三角形平面最远的点
            maxD = m_epsilon;
            maxI = 0;
            const Vector3<T> N = mathutils::getTriangleNormal(baseTriangleVertices[0],
                                                              baseTriangleVertices[1],
                                                              baseTriangleVertices[2]);
            Plane<T> trianglePlane(N, baseTriangleVertices[0]);
            for (size_t i = 0; i < vCount; i++)
            {
                const T d = abs(mathutils::getSignedDistanceToPlane(m_vertexData[i], trianglePlane));
                if (d > maxD)
                {
                    maxD = d;
                    maxI = i;
                }
            }
            if (maxD == m_epsilon)
            {
                // 所有点似乎都位于R^3的二维子空间上
                // 如何处理？添加一个额外的点到点云，使凸包具有体积
                m_planar = true;
                const vec3 N1 = mathutils::getTriangleNormal(baseTriangleVertices[1],
                                                             baseTriangleVertices[2],
                                                             baseTriangleVertices[0]);
                m_planarPointCloudTemp.clear();
                m_planarPointCloudTemp.insert(m_planarPointCloudTemp.begin(),
                                              m_vertexData.begin(),
                                              m_vertexData.end());
                const vec3 extraPoint = N1 + m_vertexData[0];
                m_planarPointCloudTemp.push_back(extraPoint);
                maxI = m_planarPointCloudTemp.size() - 1;
                m_vertexData = VertexDataSource<T>(m_planarPointCloudTemp);
            }

            // 强制使用逆时针方向
            // （如果用户偏好顺时针方向，在创建最终网格时交换每个三角形的两个顶点）
            const Plane<T> triPlane(N, baseTriangleVertices[0]);
            if (triPlane.isPointOnPositiveSide(m_vertexData[maxI]))
            {
                std::swap(baseTriangle[0], baseTriangle[1]);
            }

            // 创建四面体半边网格，并计算每个三角形定义的平面
            m_mesh.setup(baseTriangle[0], baseTriangle[1], baseTriangle[2], maxI);
            for (auto &f : m_mesh.m_faces)
            {
                auto v = m_mesh.getVertexIndicesOfFace(f);
                const Vector3<T> &va = m_vertexData[v[0]];
                const Vector3<T> &vb = m_vertexData[v[1]];
                const Vector3<T> &vc = m_vertexData[v[2]];
                const Vector3<T> N1 = mathutils::getTriangleNormal(va, vb, vc);
                const Plane<T> plane(N1, va);
                f.m_P = plane;
            }

            // 最后，为四面体外的每个顶点分配一个面
            // （四面体内的顶点不再起作用）
            for (size_t i = 0; i < vCount; i++)
            {
                for (auto &face : m_mesh.m_faces)
                {
                    if (addPointToFace(face, i))
                    {
                        break;
                    }
                }
            }
        }

        /**
         * @brief 重新排列半边列表，使其形成循环
         * @param horizonEdges 半边列表
         * @return 成功返回true，失败返回false
         */
        inline bool reorderHorizonEdges(std::vector<size_t> &horizonEdges)
        {
            const size_t horizonEdgeCount = horizonEdges.size();
            for (size_t i = 0; i < horizonEdgeCount - 1; i++)
            {
                const size_t endVertex = m_mesh.m_halfEdges[horizonEdges[i]].m_endVertex;
                bool foundNext = false;
                for (size_t j = i + 1; j < horizonEdgeCount; j++)
                {
                    const size_t beginVertex = m_mesh.m_halfEdges[m_mesh.m_halfEdges[horizonEdges[j]].m_opp].m_endVertex;
                    if (beginVertex == endVertex)
                    {
                        std::swap(horizonEdges[i + 1], horizonEdges[j]);
                        foundNext = true;
                        break;
                    }
                }
                if (!foundNext)
                {
                    return false;
                }
            }
            assert(m_mesh.m_halfEdges[horizonEdges[horizonEdges.size() - 1]].m_endVertex == m_mesh.m_halfEdges[m_mesh.m_halfEdges[horizonEdges[0]].m_opp].m_endVertex);
            return true;
        }

        /**
         * @brief 找到点云的极值索引
         * @return 包含6个索引的数组[max x, min x, max y, min y, max z, min z]
         */
        inline std::array<size_t, 6> getExtremeValues()
        {
            std::array<size_t, 6> outIndices{0, 0, 0, 0, 0, 0};
            T extremeVals[6] = {m_vertexData[0].x,
                                m_vertexData[0].x,
                                m_vertexData[0].y,
                                m_vertexData[0].y,
                                m_vertexData[0].z,
                                m_vertexData[0].z};
            const size_t vCount = m_vertexData.size();
            for (size_t i = 1; i < vCount; i++)
            {
                const Vector3<T> &pos = m_vertexData[i];
                if (pos.x > extremeVals[0])
                {
                    extremeVals[0] = pos.x;
                    outIndices[0] = i;
                }
                else if (pos.x < extremeVals[1])
                {
                    extremeVals[1] = pos.x;
                    outIndices[1] = i;
                }
                if (pos.y > extremeVals[2])
                {
                    extremeVals[2] = pos.y;
                    outIndices[2] = i;
                }
                else if (pos.y < extremeVals[3])
                {
                    extremeVals[3] = pos.y;
                    outIndices[3] = i;
                }
                if (pos.z > extremeVals[4])
                {
                    extremeVals[4] = pos.z;
                    outIndices[4] = i;
                }
                else if (pos.z < extremeVals[5])
                {
                    extremeVals[5] = pos.z;
                    outIndices[5] = i;
                }
            }
            return outIndices;
        }

        /**
         * @brief 计算顶点数据的缩放因子
         * @param extremeValues 极值索引数组
         * @return 缩放因子（最大绝对坐标值）
         */
        inline T getScale(const std::array<size_t, 6> &extremeValues)
        {
            T s = 0;
            s = std::max(s, std::abs(*((const T *)(&m_vertexData[extremeValues[0]]) + 0)));
            s = std::max(s, std::abs(*((const T *)(&m_vertexData[extremeValues[1]]) + 0)));
            s = std::max(s, std::abs(*((const T *)(&m_vertexData[extremeValues[2]]) + 1)));
            s = std::max(s, std::abs(*((const T *)(&m_vertexData[extremeValues[3]]) + 1)));
            s = std::max(s, std::abs(*((const T *)(&m_vertexData[extremeValues[4]]) + 2)));
            s = std::max(s, std::abs(*((const T *)(&m_vertexData[extremeValues[5]]) + 2)));

            return s;
        }

        /**
         * 每个面包含一个指向索引向量的唯一指针
         * 然而，许多（通常是大多数）面在其正侧没有任何点
         * 特别是在迭代结束时。当从网格中删除面时，
         * 其关联的点向量（如果存在）被移动到索引向量池
         * 当需要向网格添加在正侧有点的新面时，重用这些向量
         * 这减少了需要处理的std::vector数量，对性能影响显著
         */
        Pool<std::vector<size_t>> m_indexVectorPool;
        inline std::unique_ptr<std::vector<size_t>> getIndexVectorFromPool();
        inline void reclaimToIndexVectorPool(std::unique_ptr<std::vector<size_t>> &ptr);

        /**
         * @brief 如果点位于平面正侧，则将点与面关联
         * @param f 面
         * @param pointIndex 点索引
         * @return 如果点在正侧返回true
         */
        inline bool addPointToFace(typename MeshBuilder<T>::Face &f, size_t pointIndex);

        /**
         * @brief 创建凸包半边网格
         * 这是QuickHull算法的核心迭代过程
         * 更新m_mesh，从中创建getConvexHull函数返回的ConvexHull对象
         */
        inline void createConvexHalfEdgeMesh()
        {
            m_visibleFaces.clear();
            m_horizonEdges.clear();
            m_possiblyVisibleFaces.clear();

            // 计算基础四面体
            setupInitialTetrahedron();
            assert(m_mesh.m_faces.size() == 4);

            // 用分配了点的面初始化面栈
            m_faceList.clear();
            auto &f = m_mesh.m_faces;

            if (f[0].m_pointsOnPositiveSide &&
                f[0].m_pointsOnPositiveSide->size() > 0)
            {
                m_faceList.push_back(0);
                f[0].m_inFaceStack = 1;
            }
            if (f[1].m_pointsOnPositiveSide &&
                f[1].m_pointsOnPositiveSide->size() > 0)
            {
                m_faceList.push_back(1);
                f[1].m_inFaceStack = 1;
            }
            if (f[2].m_pointsOnPositiveSide &&
                f[2].m_pointsOnPositiveSide->size() > 0)
            {
                m_faceList.push_back(2);
                f[2].m_inFaceStack = 1;
            }
            if (f[3].m_pointsOnPositiveSide &&
                f[3].m_pointsOnPositiveSide->size() > 0)
            {
                m_faceList.push_back(3);
                f[3].m_inFaceStack = 1;
            }

            // 处理面直到面列表为空
            size_t iter = 0;
            while (!m_faceList.empty())
            {
                iter++;
                if (iter == std::numeric_limits<size_t>::max())
                {
                    // 可见面遍历用迭代计数器标记访问过的面
                    // （标记该面在此迭代中已被访问）
                    // 最大值表示未访问的面。此时必须重置迭代计数器
                    // 在64位机器上这不应该是问题
                    iter = 0;
                }

                const size_t topFaceIndex = m_faceList.front();
                m_faceList.pop_front();

                auto &tf = m_mesh.m_faces[topFaceIndex];
                tf.m_inFaceStack = 0;

                assert(!tf.m_pointsOnPositiveSide || tf.m_pointsOnPositiveSide->size() > 0);
                if (!tf.m_pointsOnPositiveSide || tf.isDisabled())
                {
                    continue;
                }

                // 选择距离此三角形平面最远的点作为拉伸的目标点
                const vec3 &activePoint = m_vertexData[tf.m_mostDistantPoint];
                const size_t activePointIndex = tf.m_mostDistantPoint;

                // 找出在活动点正侧的面（这些是"可见面"）
                // 栈顶的面当然是其中之一
                // 同时，创建地平线边列表
                m_horizonEdges.clear();
                m_possiblyVisibleFaces.clear();
                m_visibleFaces.clear();
                m_possiblyVisibleFaces.emplace_back(topFaceIndex, std::numeric_limits<size_t>::max());
                while (m_possiblyVisibleFaces.size())
                {
                    const auto faceData = m_possiblyVisibleFaces.back();
                    m_possiblyVisibleFaces.pop_back();
                    auto &pvf = m_mesh.m_faces[faceData.m_faceIndex];
                    assert(!pvf.isDisabled());

                    if (pvf.m_visibilityCheckedOnIteration == iter)
                    {
                        if (pvf.m_isVisibleFaceOnCurrentIteration)
                        {
                            continue;
                        }
                    }
                    else
                    {
                        const Plane<T> &P = pvf.m_P;
                        pvf.m_visibilityCheckedOnIteration = iter;
                        const T d = P.m_N.dotProduct(activePoint) + P.m_D;
                        if (d > 0)
                        {
                            // 面可见
                            pvf.m_isVisibleFaceOnCurrentIteration = 1;
                            pvf.m_horizonEdgesOnCurrentIteration = 0;
                            m_visibleFaces.push_back(faceData.m_faceIndex);
                            // 检查相邻面
                            for (auto heIndex : m_mesh.getHalfEdgeIndicesOfFace(pvf))
                            {
                                if (m_mesh.m_halfEdges[heIndex].m_opp != faceData.m_enteredFromHalfEdge)
                                {
                                    m_possiblyVisibleFaces.emplace_back(m_mesh.m_halfEdges[m_mesh.m_halfEdges[heIndex].m_opp].m_face, heIndex);
                                }
                            }
                            continue;
                        }
                        assert(faceData.m_faceIndex != topFaceIndex);
                    }

                    // 面不可见。因此，我们来自的半边是地平线边的一部分
                    pvf.m_isVisibleFaceOnCurrentIteration = 0;
                    m_horizonEdges.push_back(faceData.m_enteredFromHalfEdge);
                    // 存储哪条半边是地平线边
                    // 面的其他半边不会是最终网格的一部分，所以它们的数据槽可以被回收
                    const auto halfEdges = m_mesh.getHalfEdgeIndicesOfFace(m_mesh.m_faces[m_mesh.m_halfEdges[faceData.m_enteredFromHalfEdge].m_face]);
                    const std::int8_t ind = (halfEdges[0] == faceData.m_enteredFromHalfEdge) ? 0 : (halfEdges[1] == faceData.m_enteredFromHalfEdge ? 1 : 2);
                    m_mesh.m_faces[m_mesh.m_halfEdges[faceData.m_enteredFromHalfEdge].m_face].m_horizonEdgesOnCurrentIteration |= (1 << ind);
                }
                const size_t horizonEdgeCount = m_horizonEdges.size();

                // 对地平线边进行排序使其形成循环
                // 由于数值不稳定性，这可能会失败
                // 在这种情况下，我们放弃尝试解决此点的地平线边
                // 并接受凸包中的轻微退化
                if (!reorderHorizonEdges(m_horizonEdges))
                {
                    m_diagnostics.m_failedHorizonEdges++;
                    std::cerr << "Failed to solve horizon edge." << std::endl;
                    auto it = std::find(tf.m_pointsOnPositiveSide->begin(), tf.m_pointsOnPositiveSide->end(), activePointIndex);
                    tf.m_pointsOnPositiveSide->erase(it);
                    if (tf.m_pointsOnPositiveSide->size() == 0)
                    {
                        reclaimToIndexVectorPool(tf.m_pointsOnPositiveSide);
                    }
                    continue;
                }

                // 除了地平线边，可见面的所有半边都可以标记为禁用
                // 它们的数据槽将被重用
                // 面也将被禁用，但我们需要记住在它们正侧的点
                // 因此保存指向它们的指针
                m_newFaceIndices.clear();
                m_newHalfEdgeIndices.clear();
                m_disabledFacePointVectors.clear();
                size_t disableCounter = 0;
                for (auto faceIndex : m_visibleFaces)
                {
                    auto &disabledFace = m_mesh.m_faces[faceIndex];
                    auto halfEdges = m_mesh.getHalfEdgeIndicesOfFace(disabledFace);
                    for (size_t j = 0; j < 3; j++)
                    {
                        if ((disabledFace.m_horizonEdgesOnCurrentIteration & (1 << j)) == 0)
                        {
                            if (disableCounter < horizonEdgeCount * 2)
                            {
                                // Use on this iteration
                                m_newHalfEdgeIndices.push_back(halfEdges[j]);
                                disableCounter++;
                            }
                            else
                            {
                                // Mark for reusal on later iteration step
                                m_mesh.disableHalfEdge(halfEdges[j]);
                            }
                        }
                    }
                    // Disable the face, but retain pointer to the points that were
                    // on the positive side of it. We need to assign those points to
                    // the new faces we create shortly.
                    auto t = std::move(m_mesh.disableFace(faceIndex));
                    if (t)
                    {
                        // Because we should not assign point vectors to faces unless needed...
                        assert(t->size());
                        m_disabledFacePointVectors.push_back(std::move(t));
                    }
                }
                if (disableCounter < horizonEdgeCount * 2)
                {
                    const size_t newHalfEdgesNeeded = horizonEdgeCount * 2 - disableCounter;
                    for (size_t i = 0; i < newHalfEdgesNeeded; i++)
                    {
                        m_newHalfEdgeIndices.push_back(m_mesh.addHalfEdge());
                    }
                }

                // Create new faces using the edgeloop
                for (size_t i = 0; i < horizonEdgeCount; i++)
                {
                    const size_t AB = m_horizonEdges[i];

                    auto horizonEdgeVertexIndices = m_mesh.getVertexIndicesOfHalfEdge(m_mesh.m_halfEdges[AB]);
                    size_t A, B, C;
                    A = horizonEdgeVertexIndices[0];  // 地平线边起点
                    B = horizonEdgeVertexIndices[1];  // 地平线边终点
                    C = activePointIndex;              // 活动点（拉伸目标）

                    const size_t newFaceIndex = m_mesh.addFace();
                    m_newFaceIndices.push_back(newFaceIndex);

                    const size_t CA = m_newHalfEdgeIndices[2 * i + 0];  // C到A的半边
                    const size_t BC = m_newHalfEdgeIndices[2 * i + 1];  // B到C的半边

                    // 设置新三角形的半边链
                    m_mesh.m_halfEdges[AB].m_next = BC;
                    m_mesh.m_halfEdges[BC].m_next = CA;
                    m_mesh.m_halfEdges[CA].m_next = AB;

                    m_mesh.m_halfEdges[BC].m_face = newFaceIndex;
                    m_mesh.m_halfEdges[CA].m_face = newFaceIndex;
                    m_mesh.m_halfEdges[AB].m_face = newFaceIndex;

                    m_mesh.m_halfEdges[CA].m_endVertex = A;
                    m_mesh.m_halfEdges[BC].m_endVertex = C;

                    auto &newFace = m_mesh.m_faces[newFaceIndex];

                    const Vector3<T> planeNormal = mathutils::getTriangleNormal(m_vertexData[A], m_vertexData[B], activePoint);
                    newFace.m_P = Plane<T>(planeNormal, activePoint);
                    newFace.m_he = AB;

                    m_mesh.m_halfEdges[CA].m_opp = m_newHalfEdgeIndices[i > 0 ? i * 2 - 1 : 2 * horizonEdgeCount - 1];
                    m_mesh.m_halfEdges[BC].m_opp = m_newHalfEdgeIndices[((i + 1) * 2) % (horizonEdgeCount * 2)];
                }

                // 将禁用面正侧的点分配给新面
                for (auto &disabledPoints : m_disabledFacePointVectors)
                {
                    assert(disabledPoints);
                    for (const auto &point : *(disabledPoints))
                    {
                        if (point == activePointIndex)
                        {
                            continue;
                        }
                        for (size_t j = 0; j < horizonEdgeCount; j++)
                        {
                            if (addPointToFace(m_mesh.m_faces[m_newFaceIndices[j]], point))
                            {
                                break;
                            }
                        }
                    }
                    // 点不再需要：将它们移回向量池以供重用
                    reclaimToIndexVectorPool(disabledPoints);
                }

                // 如果需要，增加面栈大小
                for (const auto newFaceIndex : m_newFaceIndices)
                {
                    auto &newFace = m_mesh.m_faces[newFaceIndex];
                    if (newFace.m_pointsOnPositiveSide)
                    {
                        assert(newFace.m_pointsOnPositiveSide->size() > 0);
                        if (!newFace.m_inFaceStack)
                        {
                            m_faceList.push_back(newFaceIndex);
                            newFace.m_inFaceStack = 1;
                        }
                    }
                }
            }

            // 清理
            m_indexVectorPool.clear();
        }

        /**
         * @brief 将凸包构造到MeshBuilder对象中
         * 该对象可以转换为ConvexHull或Mesh对象
         * @param pointCloud 输入点云
         * @param CCW 是否使用逆时针方向（当前未使用）
         * @param useOriginalIndices 是否使用原始索引（当前未使用）
         * @param eps epsilon值，数值容差
         */
        inline void buildMesh(const VertexDataSource<T> &pointCloud, bool CCW, bool useOriginalIndices, T eps)
        {
            // CCW当前未使用
            (void)CCW;
            // useOriginalIndices当前未使用
            (void)useOriginalIndices;

            if (pointCloud.size() == 0)
            {
                m_mesh = MeshBuilder<T>();
                return;
            }
            m_vertexData = pointCloud;

            // 首先：找到极值并用它们计算点云的缩放因子
            m_extremeValues = getExtremeValues();
            m_scale = getScale(m_extremeValues);

            // 使用的epsilon取决于缩放因子
            m_epsilon = eps * m_scale;
            m_epsilonSquared = m_epsilon * m_epsilon;

            // 重置诊断数据
            m_diagnostics = DiagnosticsData();

            // 当所有点似乎位于R^3的二维子空间时，会出现平面情况
            m_planar = false;
            createConvexHalfEdgeMesh();
            if (m_planar)
            {
                // 移除额外添加的点
                const size_t extraPointIndex = m_planarPointCloudTemp.size() - 1;
                for (auto &he : m_mesh.m_halfEdges)
                {
                    if (he.m_endVertex == extraPointIndex)
                    {
                        he.m_endVertex = 0;
                    }
                }
                m_vertexData = pointCloud;
                m_planarPointCloudTemp.clear();
            }
        }

        /**
         * @brief 公共getConvexHull函数将设置VertexDataSource对象并调用此函数
         */
        inline ConvexHull<T> getConvexHull(const VertexDataSource<T> &pointCloud, bool CCW, bool useOriginalIndices, T eps)
        {
            buildMesh(pointCloud, CCW, useOriginalIndices, eps);
            return ConvexHull<T>(m_mesh, m_vertexData, CCW, useOriginalIndices);
        }

    public:
        /**
         * @brief 计算给定点云的凸包
         * @param pointCloud 3D点的向量
         * @param CCW 输出网格三角形是否应具有逆时针方向
         * @param useOriginalIndices 输出网格是否应使用与原始点云相同的顶点索引
         *                          如果为false，则生成仅包含凸包顶点的新顶点缓冲
         * @param eps 考虑点在平面正侧的最小距离（对于缩放为1的点云）
         * @return ConvexHull对象
         */
        inline ConvexHull<T> getConvexHull(const std::vector<Vector3<T>> &pointCloud,
                                           bool CCW,
                                           bool useOriginalIndices,
                                           T eps = defaultEps<T>())
        {
            {
                VertexDataSource<T> vertexDataSource(pointCloud);
                return getConvexHull(vertexDataSource, CCW, useOriginalIndices, eps);
            }
        }

        /**
         * @brief 计算给定点云的凸包
         * @param vertexData 指向点云第一个3D点的指针
         * @param vertexCount 点云中的顶点数
         * @param CCW 输出网格三角形是否应具有逆时针方向
         * @param useOriginalIndices 输出网格是否应使用与原始点云相同的顶点索引
         *                          如果为false，则生成仅包含凸包顶点的新顶点缓冲
         * @param eps 考虑点在平面正侧的最小距离（对于缩放为1的点云）
         * @return ConvexHull对象
         */
        inline ConvexHull<T> getConvexHull(const Vector3<T> *vertexData,
                                           size_t vertexCount,
                                           bool CCW,
                                           bool useOriginalIndices,
                                           T eps = defaultEps<T>())
        {
            {
                VertexDataSource<T> vertexDataSource(vertexData, vertexCount);
                return getConvexHull(vertexDataSource, CCW, useOriginalIndices, eps);
            }
        }

        /**
         * @brief 计算给定点云的凸包
         * 此函数假设顶点数据在内存中以以下格式存储：x_0,y_0,z_0,x_1,y_1,z_1,...
         * @param vertexData 指向点云第一个点的X分量的指针
         * @param vertexCount 点云中的顶点数
         * @param CCW 输出网格三角形是否应具有逆时针方向
         * @param useOriginalIndices 输出网格是否应使用与原始点云相同的顶点索引
         *                          如果为false，则生成仅包含凸包顶点的新顶点缓冲
         * @param eps 考虑点在平面正侧的最小距离（对于缩放为1的点云）
         * @return ConvexHull对象
         */
        inline ConvexHull<T> getConvexHull(const T *vertexData,
                                           size_t vertexCount,
                                           bool CCW,
                                           bool useOriginalIndices,
                                           T eps = defaultEps<T>())
        {
            VertexDataSource<T> vertexDataSource((const vec3 *)vertexData, vertexCount);
            return getConvexHull(vertexDataSource, CCW, useOriginalIndices, eps);
        }

        /**
         * @brief 计算给定点云的凸包
         * 此函数假设顶点数据在内存中以以下格式存储：x_0,y_0,z_0,x_1,y_1,z_1,...
         * @param vertexData 指向点云第一个点的X分量的指针
         * @param vertexCount 点云中的顶点数
         * @param CCW 输出网格三角形是否应具有逆时针方向
         * @param eps 考虑点在平面正侧的最小距离（对于缩放为1的点云）
         * @return 带半边结构的网格对象形式的点云凸包
         */
        inline HalfEdgeMesh<T, size_t> getConvexHullAsMesh(const T *vertexData,
                                                           size_t vertexCount,
                                                           bool CCW,
                                                           T eps = defaultEps<T>())
        {
            VertexDataSource<T> vertexDataSource((const vec3 *)vertexData, vertexCount);
            buildMesh(vertexDataSource, CCW, false, eps);
            return HalfEdgeMesh<T, size_t>(m_mesh, m_vertexData);
        }

        /**
         * @brief 获取上次生成的凸包的诊断信息
         * @return 诊断数据
         */
        inline const DiagnosticsData &getDiagnostics()
        {
            return m_diagnostics;
        }
    };

    // ----------------------- 内联函数定义 -----------------------

    /**
     * @brief 从对象池获取索引向量
     */
    template <typename T>
    inline std::unique_ptr<std::vector<size_t>> QuickHull<T>::getIndexVectorFromPool()
    {
        auto r = std::move(m_indexVectorPool.get());
        r->clear();
        return r;
    }

    /**
     * @brief 将索引向量回收到对象池
     */
    template <typename T>
    inline void QuickHull<T>::reclaimToIndexVectorPool(std::unique_ptr<std::vector<size_t>> &ptr)
    {
        const size_t oldSize = ptr->size();
        if ((oldSize + 1) * 128 < ptr->capacity())
        {
            // 减少内存使用！迭代开始时需要巨大的向量
            // 因为面的正侧有很多点。后来，较小的向量就足够了。
            ptr.reset(nullptr);
            return;
        }
        m_indexVectorPool.reclaim(ptr);
    }

    /**
     * @brief 如果点位于面的平面正侧，则将点添加到面
     * @param f 面
     * @param pointIndex 点索引
     * @return 如果点在正侧返回true
     */
    template <typename T>
    inline bool QuickHull<T>::addPointToFace(typename MeshBuilder<T>::Face &f, size_t pointIndex)
    {
        const T D = mathutils::getSignedDistanceToPlane(m_vertexData[pointIndex], f.m_P);
        if (D > 0 && D * D > m_epsilonSquared * f.m_P.m_sqrNLength)
        {
            if (!f.m_pointsOnPositiveSide)
            {
                f.m_pointsOnPositiveSide = std::move(getIndexVectorFromPool());
            }
            f.m_pointsOnPositiveSide->push_back(pointIndex);
            if (D > f.m_mostDistantPointDist)
            {
                f.m_mostDistantPointDist = D;
                f.m_mostDistantPoint = pointIndex;
            }
            return true;
        }
        return false;
    }

} // namespace quickhull

#endif