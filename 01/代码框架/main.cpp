#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;
const Vector3f RotateAxis =  Vector3f(1,1,1);
Eigen::Matrix4f RotationAboutAxis(float Angle, Vector3f Axis);
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}
/*
* 本次作业的任务是填写一个旋转矩阵和一个透视投影矩阵。
*/
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    Eigen::Matrix4f M_rot1, M_rot2;
    float a = rotation_angle / 180.f * MY_PI;
    M_rot1 <<
        cos(a), -sin(a), 0, 0,
        sin(a), cos(a), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    M_rot2 << RotationAboutAxis(rotation_angle, RotateAxis);

    model = M_rot2;
    return model;
}
/*
* 罗德里格斯任意轴旋转
*/
Eigen::Matrix4f RotationAboutAxis(float Angle, Vector3f Axis)
{
    Eigen::Matrix4f  I, N;
    Eigen::Vector4f n;                          
    Eigen::RowVector4f nT;
    float a = Angle / 180.f * MY_PI;
    n << Axis.x(), Axis.y(), Axis.z(), 1;
    nT << Axis.x(), Axis.y(), Axis.z(), 1;

    N <<
        0, -n.z(), n.y(), 0,
        n.z(), 0, -n.x(), 0,
        -n.y(), n.x(), 0, 0,
        0, 0, 0, 1;
    I = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f R = cos(a) * I + (1 - cos(a)) * n * nT + sin(a) * N;
    return R; 

}
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    float n = zNear, f = zFar;

    // 透视投影->正交投影  挤压
    Eigen::Matrix4f M_persp2orhtho;
    M_persp2orhtho << 
        n, 0, 0, 0,
        0, n, 0, 0,
        0, 0, n + f, -n * f,
        0, 0, 1, 0;

    // 正交投影->正则立方体
    // 将视锥信息为r,l,t,b
    //aspect_ratio: 长宽比
    float fovY = eye_fov / 180.f * MY_PI;
    float t = tan(fovY / 2) * (-n), b = -t;
    float r = aspect_ratio * t, l = -r;
    // 转换到正则立方体
    Eigen::Matrix4f M_ortho, M_trans, M_scale;
    M_trans << 
        1, 0, 0, -(r + l) / 2,
        0, 1, 0, -(t + b) / 2,
        0, 0, 1, -(n + f) / 2,
        0, 0, 0, 1;
    M_scale << 
        2 / (r - l), 0, 0, 0,
        0, 2 / (t - b), 0, 0,
        0, 0, 2 / (n - f), 0,
        0, 0, 0, 1;
    M_ortho = M_scale * M_trans;

    // 计算得到投影矩阵
    projection = M_ortho * M_persp2orhtho;
    
    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
