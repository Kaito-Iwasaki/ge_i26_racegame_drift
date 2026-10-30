#pragma once
#include "geom.hpp"
#include "view_frustum.hpp"
#include <algorithm>
#include <cstddef>
#include <deque>
#include <vector>

// Road-space rear tyre ribbons; no graphics or game-input dependency.
struct SkidMarks {
    struct Edge { Vec3 left, right; };
    struct Segment { Edge a,b; double born; float opacity; };
    double elapsed=0.0;
    float minSpacing=kMinSpacing;
    std::deque<Segment> segments;
    Edge previous[2]{};
    bool connected[2]{false,false};
    static constexpr std::size_t kMaxSegments=2000;
    static constexpr float kLife=30.0f;
    static constexpr float kMinSpacing=0.32f;
    static constexpr float kMaxSpacing=6.0f;

    void clear() { segments.clear(); connected[0]=connected[1]=false; elapsed=0.0; }
    void sample(int wheel, bool active, Edge edge, float intensity) {
        if (wheel<0 || wheel>=2) return;
        if (!active) { connected[wheel]=false; return; }
        if (!connected[wheel]) { previous[wheel]=edge; connected[wheel]=true; return; }
        const Vec3 a=(previous[wheel].left+previous[wheel].right)*.5f;
        const Vec3 b=(edge.left+edge.right)*.5f;
        const float distance=length(b-a);
        if (distance<minSpacing) return;
        if (distance>kMaxSpacing) { previous[wheel]=edge; return; }
        segments.push_back({previous[wheel],edge,elapsed,clampf(intensity,0.0f,1.0f)});
        previous[wheel]=edge;
        while(segments.size()>kMaxSegments) segments.pop_front();
    }
    void update(float dt) {
        dt=std::max(0.0f,dt);
        elapsed+=double(dt);
        while(!segments.empty() && elapsed-segments.front().born>=double(kLife)) segments.pop_front();
    }
    void build(Vec3 camera,std::vector<float>& vertices,std::vector<unsigned>& indices) const {
        vertices.clear();indices.clear();
        append(camera,vertices,indices);
    }
    // The two cars can share an upload; a conservative sphere keeps edge marks.
    void append(Vec3 camera,std::vector<float>& vertices,std::vector<unsigned>& indices,
                const ViewFrustum* frustum=nullptr) const {
        const float radius2=250.f*250.f;
        for(const auto& s:segments) {
            const Vec3 mid=(s.a.left+s.a.right+s.b.left+s.b.right)*.25f;
            const float dx=mid.x-camera.x,dz=mid.z-camera.z;
            if(dx*dx+dz*dz>radius2 || std::fabs(mid.y-camera.y)>60.0f) continue;
            if(frustum) {
                float r2=0;
                for(Vec3 p:{s.a.left,s.a.right,s.b.left,s.b.right}) {Vec3 d=p-mid;r2=std::max(r2,dot(d,d));}
                if(!frustum->intersectsSphere(mid,std::sqrt(r2))) continue;
            }
            const float alpha=(.40f+.28f*s.opacity)*clampf(float((double(kLife)-(elapsed-s.born))/8.0),0,1);
            const unsigned base=static_cast<unsigned>(vertices.size()/7);
            for(Vec3 p:{s.a.left,s.a.right,s.b.right,s.b.left})
                vertices.insert(vertices.end(),{p.x,p.y,p.z,.012f,.012f,.012f,alpha});
            indices.insert(indices.end(),{base,base+1,base+2,base,base+2,base+3});
        }
    }
};
