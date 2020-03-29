#pragma once

#include "Object.hpp"

#include <cstring>

bool rayTriangleIntersect(const Vector3f& v0, const Vector3f& v1, const Vector3f& v2, const Vector3f& orig,
                          const Vector3f& dir, float& tnear, float& u, float& v)
{
    // TODO: Implement this function that tests whether the triangle
    // that's specified bt v0, v1 and v2 intersects with the ray (whose
    // origin is *orig* and direction is *dir*)
    // Also don't forget to update tnear, u and v.
    // std::cout<<"v0 "<<v0<<std::endl;
    // std::cout<<"v1 "<<v1<<std::endl;
    // std::cout<<"v2 "<<v2<<std::endl;
    // std::cout<<"orig "<<orig<<std::endl;
    // std::cout<<"dir "<<dir<<std::endl;
    Vector3f E1 = v1 - v0;
    Vector3f E2 = v2 - v0;
    Vector3f S = orig - v0;
    Vector3f S1 = crossProduct(dir,E2);
    Vector3f S2 = crossProduct(S,E1);
    float S1dotE1 = dotProduct(S1,E1);
    if( S1dotE1 == 0) return false;
    float Inv_S1dotE1 = 1.0 / S1dotE1;

    tnear = dotProduct(S2,E2) * Inv_S1dotE1;
    if(tnear < 0) return false;

    float b1 = dotProduct(S1,S) * Inv_S1dotE1;
    float b2 = dotProduct(S2,dir) * Inv_S1dotE1;
    if(b1 < 0 || b2 < 0 || b1 > 1 || b1 + b2 > 1)
        return false;
    // std::cout<<"b1 "<<b1<<std::endl;
    // std::cout<<"b2 "<<b2<<std::endl;
    
    u = b1;
    v = b2;
    return true;

    // float Inv_Det = 1.0/S1dotE1;
    
    // tnear = dotProduct(S2,E2) * Inv_Det;

    // u = dotProduct(S1,S) * Inv_Det;

    // if(u < 0 || u > 1) return false;

    // v = dotProduct(S2,dir) * Inv_Det;
    
    // if(v < 0 || u + v > 1) return false;

    // return true;


    // float Det = (dotProduct(crossProduct(dir,E2),E1));

    // if(Det == 0) return false;

    // u = dotProduct(crossProduct(dir,E2),S) / Det;

    // v = dotProduct(crossProduct(S,E2),dir) / Det;

    // tnear = dotProduct(crossProduct(S,E1),E2 ) / Det;

    // if (u < 0 || u > 1 || v < 0 || v > 1) return false;

    // return true;
}

class MeshTriangle : public Object
{
public:
    MeshTriangle(const Vector3f* verts, const uint32_t* vertsIndex, const uint32_t& numTris, const Vector2f* st)
    {
        uint32_t maxIndex = 0;
        for (uint32_t i = 0; i < numTris * 3; ++i)
            if (vertsIndex[i] > maxIndex)
                maxIndex = vertsIndex[i];
        maxIndex += 1;
        vertices = std::unique_ptr<Vector3f[]>(new Vector3f[maxIndex]);
        memcpy(vertices.get(), verts, sizeof(Vector3f) * maxIndex);
        vertexIndex = std::unique_ptr<uint32_t[]>(new uint32_t[numTris * 3]);
        memcpy(vertexIndex.get(), vertsIndex, sizeof(uint32_t) * numTris * 3);
        numTriangles = numTris;
        stCoordinates = std::unique_ptr<Vector2f[]>(new Vector2f[maxIndex]);
        memcpy(stCoordinates.get(), st, sizeof(Vector2f) * maxIndex);
    }

    bool intersect(const Vector3f& orig, const Vector3f& dir, float& tnear, uint32_t& index,
                   Vector2f& uv) const override
    {
        bool intersect = false;
        for (uint32_t k = 0; k < numTriangles; ++k)
        {
            const Vector3f& v0 = vertices[vertexIndex[k * 3]];
            const Vector3f& v1 = vertices[vertexIndex[k * 3 + 1]];
            const Vector3f& v2 = vertices[vertexIndex[k * 3 + 2]];
            float t, u, v;
            if (rayTriangleIntersect(v0, v1, v2, orig, dir, t, u, v) && t < tnear)
            {
                //std::cout<<"t "<<t<<std::endl;
                tnear = t;
                uv.x = u;
                uv.y = v;
                index = k;
                intersect |= true;
            }
        }

        return intersect;
    }

    void getSurfaceProperties(const Vector3f&, const Vector3f&, const uint32_t& index, const Vector2f& uv, Vector3f& N,
                              Vector2f& st) const override
    {
        const Vector3f& v0 = vertices[vertexIndex[index * 3]];
        const Vector3f& v1 = vertices[vertexIndex[index * 3 + 1]];
        const Vector3f& v2 = vertices[vertexIndex[index * 3 + 2]];
        Vector3f e0 = normalize(v1 - v0);
        Vector3f e1 = normalize(v2 - v1);
        N = normalize(crossProduct(e0, e1));
        const Vector2f& st0 = stCoordinates[vertexIndex[index * 3]];
        const Vector2f& st1 = stCoordinates[vertexIndex[index * 3 + 1]];
        const Vector2f& st2 = stCoordinates[vertexIndex[index * 3 + 2]];
        st = st0 * (1 - uv.x - uv.y) + st1 * uv.x + st2 * uv.y;
    }

    Vector3f evalDiffuseColor(const Vector2f& st) const override
    {
        float scale = 5;
        float pattern = (fmodf(st.x * scale, 1) > 0.5) ^ (fmodf(st.y * scale, 1) > 0.5);
        return lerp(Vector3f(0.815, 0.235, 0.031), Vector3f(0.937, 0.937, 0.231), pattern);
    }

    std::unique_ptr<Vector3f[]> vertices;
    uint32_t numTriangles;
    std::unique_ptr<uint32_t[]> vertexIndex;
    std::unique_ptr<Vector2f[]> stCoordinates;
};
