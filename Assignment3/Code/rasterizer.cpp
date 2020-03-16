//
// Created by goksu on 4/6/19.
//

#include <algorithm>
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

rst::col_buf_id rst::rasterizer::load_normals(const std::vector<Eigen::Vector3f>& normals)
{
    auto id = get_next_id();
    nor_buf.emplace(id, normals);

    normal_id = id;

    return {id};
}


// Bresenham's line drawing algorithm
void rst::rasterizer::draw_line(Eigen::Vector3f begin, Eigen::Vector3f end)
{
    auto x1 = begin.x();
    auto y1 = begin.y();
    auto x2 = end.x();
    auto y2 = end.y();

    Eigen::Vector3f line_color = {255, 255, 255};

    int x,y,dx,dy,dx1,dy1,px,py,xe,ye,i;

    dx=x2-x1;
    dy=y2-y1;
    dx1=fabs(dx);
    dy1=fabs(dy);
    px=2*dy1-dx1;
    py=2*dx1-dy1;

    if(dy1<=dx1)
    {
        if(dx>=0)
        {
            x=x1;
            y=y1;
            xe=x2;
        }
        else
        {
            x=x2;
            y=y2;
            xe=x1;
        }
        Eigen::Vector2i point = Eigen::Vector2i(x, y);
        set_pixel(point,line_color);
        for(i=0;x<xe;i++)
        {
            x=x+1;
            if(px<0)
            {
                px=px+2*dy1;
            }
            else
            {
                if((dx<0 && dy<0) || (dx>0 && dy>0))
                {
                    y=y+1;
                }
                else
                {
                    y=y-1;
                }
                px=px+2*(dy1-dx1);
            }
//            delay(0);
            Eigen::Vector2i point = Eigen::Vector2i(x, y);
            set_pixel(point,line_color);
        }
    }
    else
    {
        if(dy>=0)
        {
            x=x1;
            y=y1;
            ye=y2;
        }
        else
        {
            x=x2;
            y=y2;
            ye=y1;
        }
        Eigen::Vector2i point = Eigen::Vector2i(x, y);
        set_pixel(point,line_color);
        for(i=0;y<ye;i++)
        {
            y=y+1;
            if(py<=0)
            {
                py=py+2*dx1;
            }
            else
            {
                if((dx<0 && dy<0) || (dx>0 && dy>0))
                {
                    x=x+1;
                }
                else
                {
                    x=x-1;
                }
                py=py+2*(dx1-dy1);
            }
//            delay(0);
            Eigen::Vector2i point = Eigen::Vector2i(x, y);
            set_pixel(point,line_color);
        }
    }
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

static bool insideTriangle(float x, float y, const Vector4f* _v){
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
    // Vector3f v[3];
    // for(int i=0;i<3;i++)
    //     v[i] = {_v[i].x(),_v[i].y(), 1.0};
    // Vector3f f0,f1,f2;
    // f0 = v[1].cross(v[0]);
    // f1 = v[2].cross(v[1]);
    // f2 = v[0].cross(v[2]);
    // Vector3f p(x,y,1.);
    // if((p.dot(f0)*f0.dot(v[2])>0) && (p.dot(f1)*f1.dot(v[0])>0) && (p.dot(f2)*f2.dot(v[1])>0))
    //     return true;
    // return false;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector4f* v){
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}
static std::tuple<float, float, float> computeBarycentric3D(float x, float y, const Vector4f* v)
{

}

void rst::rasterizer::draw(std::vector<Triangle *> &TriangleList) {

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (const auto& t:TriangleList)
    {

        Triangle newtri = *t;

        std::array<Eigen::Vector4f, 3> mm {
                (view * model * t->v[0]),
                (view * model * t->v[1]),
                (view * model * t->v[2])
        };

        std::array<Eigen::Vector3f, 3> viewspace_pos;

        std::transform(mm.begin(), mm.end(), viewspace_pos.begin(), [](auto& v) {
            return v.template head<3>();
        });

        Eigen::Vector4f v[] = {
                mvp * t->v[0],
                mvp * t->v[1],
                mvp * t->v[2]
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec.x()/=vec.w();
            vec.y()/=vec.w();
            vec.z()/=vec.w();
        }

        Eigen::Matrix4f inv_trans = (view * model).inverse().transpose();
        Eigen::Vector4f n[] = {
                inv_trans * to_vec4(t->normal[0], 0.0f),
                inv_trans * to_vec4(t->normal[1], 0.0f),
                inv_trans * to_vec4(t->normal[2], 0.0f)
        };

        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            //screen space coordinates
            newtri.setVertex(i, v[i]);
        }

        for (int i = 0; i < 3; ++i)
        {
            //view space normal
            newtri.setNormal(i, n[i].head<3>());
        }

        newtri.setColor(0, 148,121.0,92.0);
        newtri.setColor(1, 148,121.0,92.0);
        newtri.setColor(2, 148,121.0,92.0);

        // Also pass view space vertice position
        rasterize_triangle(newtri, viewspace_pos);
    }
}

static Eigen::Vector3f interpolate(float alpha, float beta, float gamma, const Eigen::Vector3f& vert1, const Eigen::Vector3f& vert2, const Eigen::Vector3f& vert3, float weight)
{
    return (alpha * vert1 + beta * vert2 + gamma * vert3) / weight;
}

static Eigen::Vector2f interpolate(float alpha, float beta, float gamma, const Eigen::Vector2f& vert1, const Eigen::Vector2f& vert2, const Eigen::Vector2f& vert3, float weight)
{
    auto u = (alpha * vert1[0] + beta * vert2[0] + gamma * vert3[0]);
    auto v = (alpha * vert1[1] + beta * vert2[1] + gamma * vert3[1]);

    u /= weight;
    v /= weight;

    return Eigen::Vector2f(u, v);
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t, const std::array<Eigen::Vector3f, 3>& view_pos) 
{
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

    // end get bounding box
    for(int x = minx;x < maxx;x ++)
    {
        for(int y = miny;y < maxy;y ++)
        {
            if (x < 0 || x >= width) std::cout<<"out bounce x "<< x << " minx "<<minx << " maxx " <<maxx<<std::endl;
            if (y < 0 || y >= height) std::cout<<"out bounce y "<< y << " miny "<<miny << " maxy " <<maxy<<std::endl;

            Vector3f averageColor = {0,0,0};
            for(int index = 0;index < 4;index ++)
            {
                float tx,ty;
                float bufIndex = get_index_MSAA2x2(x,y,index);
                
                if(index == 0) {tx = x + 0.25; ty = y + 0.25;}
                if(index == 1) {tx = x + 0.75; ty = y + 0.25;}
                if(index == 2) {tx = x + 0.25; ty = y + 0.75;}
                if(index == 3) {tx = x + 0.75; ty = y + 0.75;}

                if (insideTriangle(tx,ty,t.v))
                {
                    auto[alpha, beta, gamma] = computeBarycentric2D(tx, ty, t.v);
                    if(alpha < 0 || beta < 0 || gamma < 0) continue;
                    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;

                    auto interpolated_color = interpolate(      alpha,  beta,   gamma,  t.color[0] , t.color[1] , t.color[2],1.0) ;
                    auto interpolated_normal = interpolate(     alpha , beta,   gamma,  t.normal[0], t.normal[1] , t.normal[2], 1.0);
                    auto interpolated_texcoords = interpolate(  alpha , beta,   gamma,  t.tex_coords[0] , t.tex_coords[1] , t.tex_coords[2],1.0) ;
                    auto interpolated_shadingcoords = interpolate(alpha , beta, gamma,  view_pos[0] , view_pos[1] , view_pos[2],1.0) ;
                    auto interpolated_worldspace_pos = (alpha * t.v[0] + beta * t.v[1] + gamma * t.v[2]) * w_reciprocal;

                    fragment_shader_payload payload( interpolated_color, interpolated_normal.normalized(), interpolated_texcoords, texture ? &*texture : nullptr);
                    payload.view_pos = interpolated_shadingcoords;
                    payload.world_pos = interpolated_worldspace_pos.head(3);

                    if(depth_buf[bufIndex] > z_interpolated)
                    {
                        depth_buf[bufIndex] = z_interpolated;
                        Vector3f color = fragment_shader(payload);
                        frame_buf_msaa[bufIndex] = color;
                    }
                }
                averageColor += frame_buf_msaa[bufIndex];
            }
            set_pixel(Eigen::Vector2i(x,y), averageColor / 4.0);
        }
    }

    // TODO: From your HW3, get the triangle rasterization code.
    // TODO: Inside your rasterization loop:
    //    * v[i].w() is the vertex view space depth value z.
    //    * Z is interpolated view space depth for the current pixel
    //    * zp is depth between zNear and zFar, used for z-buffer

    // float Z = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    // float zp = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    // zp *= Z;

    // TODO: Interpolate the attributes:
    // auto interpolated_color
    // auto interpolated_normal
    // auto interpolated_texcoords
    // auto interpolated_shadingcoords

    // Use: fragment_shader_payload payload( interpolated_color, interpolated_normal.normalized(), interpolated_texcoords, texture ? &*texture : nullptr);
    // Use: payload.view_pos = interpolated_shadingcoords;
    // Use: Instead of passing the triangle's color directly to the frame buffer, pass the color to the shaders first to get the final color;
    // Use: auto pixel_color = fragment_shader(payload);
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
    return (height-y)*width + x;
}

// x,y real coor
// index msaa index
int rst::rasterizer::get_index_MSAA2x2(int x, int y,int index)
{
    if (index < 2)
        return ((y) * 2 * width  + x ) * 2 + index;
    else
        return (((y) * 2 + 1 ) * width  + x ) * 2 + index - 2;
}

void rst::rasterizer::set_pixel(const Vector2i &point, const Eigen::Vector3f &color)
{
    //old index: auto ind = point.y() + point.x() * width;
    int ind = (height-point.y())*width + point.x();
    frame_buf[ind] = color;
}

void rst::rasterizer::set_vertex_shader(std::function<Eigen::Vector3f(vertex_shader_payload)> vert_shader)
{
    vertex_shader = vert_shader;
}

void rst::rasterizer::set_fragment_shader(std::function<Eigen::Vector3f(fragment_shader_payload)> frag_shader)
{
    fragment_shader = frag_shader;
}

rst::rasterizer::~rasterizer()
{
    frame_buf.clear();
    depth_buf.clear();
    texture = std::nullopt;
}

