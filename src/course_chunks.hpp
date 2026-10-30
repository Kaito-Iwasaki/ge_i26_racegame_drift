#pragma once
#include "meshbuild.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <utility>
#include <vector>

struct CourseChunk {
    Vec3 min, max;
    int first = 0, count = 0;
};
struct ChunkedCourse {
    std::vector<std::uint32_t> indices;
    std::vector<CourseChunk> chunks;
};
// Group triangles spatially, but use their complete bounds so no triangle is clipped at a cell edge.
inline ChunkedCourse chunkCourse(const CpuMesh& mesh) {
    struct Bucket { CourseChunk bounds; std::vector<std::uint32_t> indices; };
    std::map<std::pair<int,int>, Bucket> buckets;
    const float inf = std::numeric_limits<float>::infinity();
    for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        Vec3 p[3];
        for (int j=0;j<3;++j) {
            size_t k = size_t(mesh.indices[i+j]) * 9;
            p[j] = {mesh.vertices[k],mesh.vertices[k+1],mesh.vertices[k+2]};
        }
        const float x=(p[0].x+p[1].x+p[2].x)/3.0f, z=(p[0].z+p[1].z+p[2].z)/3.0f;
        auto& b=buckets[{int(std::floor(x/128.0f)),int(std::floor(z/128.0f))}];
        if (b.indices.empty()) { b.bounds.min={inf,inf,inf}; b.bounds.max={-inf,-inf,-inf}; }
        for (int j=0;j<3;++j) {
            b.indices.push_back(mesh.indices[i+j]);
            b.bounds.min.x=std::min(b.bounds.min.x,p[j].x); b.bounds.min.y=std::min(b.bounds.min.y,p[j].y); b.bounds.min.z=std::min(b.bounds.min.z,p[j].z);
            b.bounds.max.x=std::max(b.bounds.max.x,p[j].x); b.bounds.max.y=std::max(b.bounds.max.y,p[j].y); b.bounds.max.z=std::max(b.bounds.max.z,p[j].z);
        }
    }
    ChunkedCourse result; result.indices.reserve(mesh.indices.size()); result.chunks.reserve(buckets.size());
    for (auto& entry : buckets) {
        Bucket& b=entry.second; b.bounds.first=int(result.indices.size()); b.bounds.count=int(b.indices.size());
        result.indices.insert(result.indices.end(),b.indices.begin(),b.indices.end()); result.chunks.push_back(b.bounds);
    }
    return result;
}
// Conservative clip-space AABB test: skip only when all corners lie outside one plane.
inline bool courseChunkVisible(const CourseChunk& c, const Mat4& m) {
    for (int plane=0;plane<6;++plane) {
        int axis=plane/2; float sign=(plane%2==0)?1.0f:-1.0f;
        float a=m.m[3]+sign*m.m[axis], b=m.m[7]+sign*m.m[4+axis];
        float d=m.m[11]+sign*m.m[8+axis], e=m.m[15]+sign*m.m[12+axis];
        float x=a>=0?c.max.x:c.min.x, y=b>=0?c.max.y:c.min.y, z=d>=0?c.max.z:c.min.z;
        if (a*x+b*y+d*z+e < 0.0f) return false;
    }
    return true;
}
