// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    Eigen::Vector3f ap(x - _v[0][0],y - _v[0][1],1);
    Eigen::Vector3f ab(_v[1][0] - _v[0][0],_v[1][1] - _v[0][1],1);

    Eigen::Vector3f bp(x - _v[1][0],y - _v[1][1],1);
    Eigen::Vector3f bc(_v[2][0] - _v[1][0],_v[2][1] - _v[1][1],1);

    Eigen::Vector3f cp(x - _v[2][0],y - _v[2][1],1);
    Eigen::Vector3f ca(_v[0][0] - _v[2][0],_v[0][1] - _v[2][1],1);

    bool singlea = ab.cross(ap)(2) > 0;
    bool singleb = bc.cross(bp)(2) > 0;
    bool singlec = ca.cross(cp)(2) > 0;
    
    return singlea == singleb && singleb == singlec;
    
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    // try each triangle
    auto v = t.toVector4();
    
    // creat a bounding box to reduce calculation
    float minx = v[0](0);
    float miny = v[0](1);
    float maxx = v[0](0);
    float maxy = v[0](1);

    for(auto item:v)
    {
        if(item(0) < minx) minx = item(0);
        else if(item(0) > maxx) maxx = item(0);
        
        if(item(1) < miny) miny = item(1);
        else if(item(1) > maxy) maxy = item(1);
    }
    
    minx = minx > 1 ? minx : 1;
    miny = miny > 1 ? miny : 1;

    minx = minx < width ? minx: width - 2;
    miny = miny < height ? miny : height - 2;

    maxx = maxx > 1 ? maxx : 1;
    maxy = maxy > 1 ? maxy : 1;

    maxx = maxx < width ? maxx: width - 2;
    maxy = maxy < height ? maxy : height - 2;

    for(int x = minx;x < maxx;x ++)
    {
        for(int y = miny;y < maxy;y ++)
        {
            if (x < 0 || x >= width) std::cout<<"out bounce x "<< x << " minx "<<minx << " maxx " <<maxx<<std::endl;
            if (y < 0 || y >= height) std::cout<<"out bounce y "<< y << " miny "<<miny << " maxy " <<maxy<<std::endl;

            // if(x > width || y > height) continue;
            // try msaa 4 x 4
            Vector3f averageColor = {0,0,0};
            for(int index = 0;index < 4;index ++)
            {
                if ((index == 0 && insideTriangle(x + 0.25,y + 0.75,t.v)) || 
                    (index == 1 && insideTriangle(x + 0.25,y + 0.25,t.v)) ||
                    (index == 2 && insideTriangle(x + 0.75,y + 0.25,t.v)) ||
                    (index == 3 && insideTriangle(x + 0.75,y + 0.75,t.v)))
                {
                    auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;

                    if(depth_buf[get_index_MSAA2x2(x,y,index)] > z_interpolated)
                    {
                        depth_buf[get_index_MSAA2x2(x,y,index)] = z_interpolated;
                        frame_buf_msaa[get_index_MSAA2x2(x,y,index)] = t.getColor();
                    }
                }
                averageColor += frame_buf_msaa[get_index_MSAA2x2(x,y,index)];
            }
            set_pixel(Eigen::Vector3f(x,y,1), averageColor / 4.0);
        }
    }
    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        std::fill(frame_buf_msaa.begin(), frame_buf_msaa.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w *  h);
    frame_buf_msaa.resize(w * h * 4);
    depth_buf.resize(w *  h * 4);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)* width + x;
}

// x,y real coor
// index msaa index
int rst::rasterizer::get_index_MSAA2x2(int x, int y,int index)
{
    if (index < 2)
        return ((height-1-y) * 2 * width  + x ) * 2 + index;
    else
        return (((height-1-y) * 2 + 1 ) * width  + x ) * 2 + index - 2;
}


void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on