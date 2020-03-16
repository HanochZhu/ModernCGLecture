# 简介

## 得分点

1、代码可正常编译

- 在linux中build目录下执行make，既可看到效果

2、参数插值

- 在rasterizer.cpp文件中实现对颜色、法向量、纹理坐标、位置的插值，并传递给fragment_shader_payload.如下代码所示

```cpp
                    auto interpolated_color = interpolate(      alpha,  beta,   gamma,  t.color[0] , t.color[1] , t.color[2],1.0) ;
                    auto interpolated_normal = interpolate(     alpha , beta,   gamma,  t.normal[0], t.normal[1] , t.normal[2], 1.0);
                    auto interpolated_texcoords = interpolate(  alpha , beta,   gamma,  t.tex_coords[0] , t.tex_coords[1] , t.tex_coords[2],1.0) ;
                    auto interpolated_shadingcoords = interpolate(alpha , beta, gamma,  view_pos[0] , view_pos[1] , view_pos[2],1.0) ;
```

3、Blinn-phong的实现

- 参考课程讲诉的算法实现，核心代码如下：

```cpp
        result_color += diffuseEngergy * std::max(0.0f,(float)dirI.dot(normal));

        // specular depend on dir of view
        Vector3f halfVector = (dirI + eye_dir).normalized();// / (float)(dirI + eye_dir).norm();

        result_color += specularEngergy *  std::pow(std::max(0.0f,(float)halfVector.dot(normal)),100); 
```

实现结果路径/images/out_phong.png

4、Texture

在phong模型的基础上实现对贴图的采样。但目前采样相对简单，如果后面有时间再更新

```
    if (payload.texture)
    {
        // TODO: Get the texture value at the texture coordinates of the current fragment
        return_color = payload.texture->getColor(payload.tex_coords(0),payload.tex_coords(1));
    }
```

实现结果路径/images/out_texture.png

5、Bump

Bump实现算法参考互联网和注释内容

```
    float x = normal(0);
    float y = normal(1);
    float z = normal(2);
    float u = tex_coord(0);
    float v = tex_coord(1);
    float w = texture->width;
    float h = texture->height;
    Vector3f t = {x * y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z)};
    Vector3f b = normal.cross(t);
    Eigen::Matrix3f TBN;
    TBN<< t,b,normal;
    
    float dU = kh * kn * (texture->getColor(u + 1 / w,v).norm() - texture->getColor(u,v).norm());
    float dV = kh * kn * (texture->getColor(u,v + 1 / h).norm() - texture->getColor(u,v).norm());
    Vector3f ln = {-dU, -dV, 1};
    normal = (TBN * ln).normalized();
```

对应的图片路径为/images/out_bump.png和/images/out_displace.png

6、更多模型

尝试了作业带的rock，但是感觉不是很有意义。



## 总结

趟过了不少坑啊。