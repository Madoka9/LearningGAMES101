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
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector3f q(x, y, 0);
    Vector3f ab = _v[1] - _v[0], bc = _v[2] - _v[1], ca = _v[0] - _v[2];
    Vector3f aq = q - _v[0], bq = q - _v[1], cq = q - _v[2];
    return ab.cross(aq).dot(bc.cross(bq)) > 0 && ab.cross(aq).dot(ca.cross(cq)) > 0 && bc.cross(bq).dot(ca.cross(cq)) > 0;
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
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
        // Bounding Box
    Vector2f MinBBox = Vector2f(
        std::min(v[0].x(), std::min(v[1].x(), v[2].x())),
        std::min(v[0].y(), std::min(v[1].y(), v[2].y()))
    );
    Vector2f MaxBBox = Vector2f(
        std::max(v[0].x(), std::max(v[1].x(), v[2].x())),
        std::max(v[0].y(), std::max(v[1].y(), v[2].y()))
    );
    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
    if (AA == AAMethod::off)
    {
        for (int x = floor(MinBBox.x()); x <= floor(MaxBBox.x()); x++)
        {
            for (int y = floor(MinBBox.y()); y <= floor(MaxBBox.y()); y++)
            {

                    if (insideTriangle(x + 0.5, y + 0.5, t.v))
                    {
                        /*
                        * 获取当前像素深度
                        */
                        float CurDepth = GetPixelDepth(x + 0.5, y + 0.5, t);
                        int CurIndex = get_index(x, y);
                        if (CurDepth < depth_buf[CurIndex])
                        {
                            //Update DepthBuffer
                            depth_buf[CurIndex] = CurDepth;
                            //Set Pixel
                            Vector3f Point = { (float)x, (float)y, CurDepth };
                            Vector3f Color = t.getColor();
                            set_pixel(Point, Color);
                        }

                    }
            }
        }
    }
    else if (AA == AAMethod::MSAAx4)
    {
        int Multiple = 4;
        MSAA(Multiple, MinBBox, MaxBBox, t);
    }
    else if (AA == AAMethod::MSAAx8)
    {
        int Multiple = 8;
        MSAA(Multiple, MinBBox, MaxBBox, t);
    }

}

void rst::rasterizer::MSAA(int& Multiple, Vector2f MinBBox, Vector2f MaxBBox, const Triangle& t)
{
    for (int x = floor(MinBBox.x()); x <= floor(MaxBBox.x()); x++)
    {
        for (int y = floor(MinBBox.y()); y <= floor(MaxBBox.y()); y++)
        {
            float TempDepth = std::numeric_limits<float>::infinity();
            Vector3f TempColor = { 0,0,0 };
            int SampleSize = Multiple / 2;
            float delta = 1. / Multiple;
            for (int i = 0; i < Multiple; i++)
            {
                float dx = i % SampleSize * delta * 2 + delta;
                float dy = floor(i / SampleSize) * delta * 2 + delta;
                if (insideTriangle(x + dx, y + dy, t.v))
                {
                    //Update AA SamplerBuffer
                    float CurDepth = GetPixelDepth(x + dx, y + dy, t);
                    int CurIndex = get_index(x, y) * Multiple + i;
                    if (CurDepth < DepthSampleBuffer[CurIndex])
                    {
                        DepthSampleBuffer[CurIndex] = CurDepth;
                        FrameSampleBuffer[CurIndex] = t.getColor();
                    }
                    TempDepth = std::min(TempDepth, CurDepth);
                }
            }
            //Update Raster Buffer
            Vector3f Point = { (float)x, (float)y, TempDepth };
            for (int i = 0; i < Multiple; i++)
            {
                TempColor += FrameSampleBuffer[get_index(x, y) * Multiple + i] * (1. / Multiple);
            }
            set_pixel(Point, TempColor);
        }
    }
}

float rst::rasterizer::GetPixelDepth(float x, float y, const Triangle& t)
{
    auto v = t.toVector4();
    float alpha, beta, gamma;
    std::tuple<float, float, float>{alpha, beta, gamma} = computeBarycentric2D(x, y, t.v);
    float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    z_interpolated *= w_reciprocal;
    return z_interpolated;
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
        std::fill(frame_buf.begin(), frame_buf.end(), BackgroundColor);
        std::fill(FrameSampleBuffer.begin(), FrameSampleBuffer.end(), BackgroundColor);
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        std::fill(DepthSampleBuffer.begin(), DepthSampleBuffer.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
     
    FrameSampleBuffer.resize((int)AA * 4 * w * h);              //用于不同Triangle之间的混合，解决黑边问题
    DepthSampleBuffer.resize((int)AA * 4 * w * h);              //用于MSAA
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on