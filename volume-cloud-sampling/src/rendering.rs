use std::{f32::consts::PI, fs, path::Path, process::Command};

use glam::{Mat4, Quat, Vec2, Vec3A, Vec4Swizzles};
use image::io::Reader;
use miette::{miette, IntoDiagnostic, Result};

use crate::{point_cloud::Point, Arguments};

pub fn look_at_quaternion(look_from: Vec3A, look_to: Vec3A, up: Vec3A) -> Quat {
    let a = up;
    let b = (look_to - look_from).normalize();

    let phi = -(a.dot(b)).acos();
    let cos_half_phi = (phi / 2.0).cos();
    let sin_half_phi = (phi / 2.0).sin();

    let mut axis = a.cross(b);
    if a.x == b.x && a.y == b.y && a.z == b.z {
        return Quat::IDENTITY;
    }
    axis = axis.normalize();

    let mut x = axis.x * sin_half_phi;
    let mut y = axis.y * sin_half_phi;
    let mut z = axis.z * sin_half_phi;
    let mut w = cos_half_phi;

    let l = (x * x + y * y + z * z + w * w).sqrt();
    x /= l;
    y /= l;
    z /= l;
    w /= l;

    Quat::from_xyzw(x, y, z, w)
}

pub fn det(m: Mat4) -> f32 {
    let m = m.transpose().to_cols_array_2d();

    m[0][0]
        * (m[1][1] * m[2][2] * m[3][3] + m[1][2] * m[2][3] * m[3][1] + m[1][3] * m[2][1] * m[3][2]
            - m[1][3] * m[2][2] * m[3][1]
            - m[1][2] * m[1][2] * m[3][3]
            - m[1][1] * m[2][3] * m[3][2])
        - m[1][0]
            * (m[0][1] * m[2][2] * m[3][3]
                + m[0][2] * m[2][3] * m[3][1]
                + m[0][3] * m[2][1] * m[3][2]
                - m[0][3] * m[2][2] * m[3][1]
                - m[0][2] * m[2][1] * m[3][3]
                - m[0][1] * m[2][3] * m[3][2])
        + m[2][0]
            * (m[0][1] * m[1][2] * m[3][3]
                + m[0][2] * m[1][3] * m[3][1]
                + m[0][3] * m[1][1] * m[3][2]
                - m[0][3] * m[1][2] * m[3][1]
                - m[0][2] * m[1][1] * m[3][3]
                - m[0][1] * m[1][3] * m[3][2])
        - m[3][0]
            * (m[0][1] * m[1][2] * m[2][3]
                + m[0][2] * m[1][3] * m[2][1]
                + m[0][3] * m[1][1] * m[2][2]
                - m[0][3] * m[1][2] * m[2][1]
                - m[0][2] * m[1][1] * m[2][3]
                - m[0][1] * m[1][3] * m[2][2])
}

pub fn inverse(m: Mat4) -> Mat4 {
    let mut res = [[0.0; 4]; 4];
    let det_inv = 1.0 / det(m);
    println!("{}", det_inv);
    let m = m.transpose().to_cols_array_2d();

    let m11 = m[0][0];
    let m12 = m[0][1];
    let m13 = m[0][2];
    let m14 = m[0][3];
    let m21 = m[1][0];
    let m22 = m[1][1];
    let m23 = m[1][2];
    let m24 = m[1][3];
    let m31 = m[2][0];
    let m32 = m[2][1];
    let m33 = m[2][2];
    let m34 = m[2][3];
    let m41 = m[3][0];
    let m42 = m[3][1];
    let m43 = m[3][2];
    let m44 = m[3][3];

    res[0][0] = (m22 * m33 * m44 + m23 * m34 * m42 + m24 * m32 * m43
        - m22 * m34 * m43
        - m23 * m32 * m44
        - m24 * m33 * m42)
        * det_inv;
    res[0][1] = (m12 * m34 * m43 + m13 * m32 * m44 + m14 * m33 * m42
        - m12 * m33 * m44
        - m13 * m34 * m42
        - m14 * m32 * m43)
        * det_inv;
    res[0][2] = (m12 * m23 * m44 + m13 * m24 * m42 + m14 * m22 * m43
        - m12 * m24 * m43
        - m13 * m22 * m44
        - m14 * m23 * m42)
        * det_inv;
    res[0][3] = (m12 * m24 * m33 + m13 * m22 * m34 + m14 * m23 * m32
        - m12 * m23 * m34
        - m13 * m24 * m32
        - m14 * m22 * m33)
        * det_inv;

    res[1][0] = (m21 * m34 * m43 + m23 * m31 * m44 + m24 * m33 * m41
        - m21 * m33 * m44
        - m23 * m34 * m41
        - m24 * m31 * m43)
        * det_inv;
    res[1][1] = (m11 * m33 * m44 + m13 * m34 * m41 + m14 * m31 * m43
        - m11 * m34 * m43
        - m13 * m31 * m44
        - m14 * m33 * m41)
        * det_inv;
    res[1][2] = (m11 * m24 * m43 + m13 * m21 * m44 + m14 * m23 * m41
        - m11 * m23 * m44
        - m13 * m24 * m41
        - m14 * m21 * m43)
        * det_inv;
    res[1][3] = (m11 * m23 * m34 + m13 * m24 * m31 + m14 * m21 * m33
        - m11 * m24 * m33
        - m13 * m21 * m34
        - m14 * m23 * m31)
        * det_inv;

    res[2][0] = (m21 * m32 * m44 + m22 * m34 * m41 + m24 * m31 * m42
        - m21 * m34 * m42
        - m22 * m31 * m44
        - m24 * m32 * m41)
        * det_inv;
    res[2][1] = (m11 * m34 * m42 + m12 * m31 * m44 + m14 * m32 * m41
        - m11 * m32 * m44
        - m12 * m34 * m41
        - m14 * m31 * m42)
        * det_inv;
    res[2][2] = (m11 * m22 * m44 + m12 * m24 * m41 + m14 * m21 * m42
        - m11 * m24 * m42
        - m12 * m21 * m44
        - m14 * m22 * m41)
        * det_inv;
    res[2][3] = (m11 * m24 * m32 + m12 * m21 * m34 + m14 * m22 * m31
        - m11 * m22 * m34
        - m12 * m24 * m31
        - m14 * m21 * m32)
        * det_inv;

    res[3][0] = (m21 * m33 * m42 + m22 * m31 * m43 + m23 * m32 * m41
        - m21 * m32 * m43
        - m22 * m33 * m41
        - m23 * m31 * m42)
        * det_inv;
    res[3][1] = (m11 * m32 * m43 + m12 * m33 * m41 + m13 * m31 * m42
        - m11 * m33 * m42
        - m12 * m31 * m43
        - m13 * m32 * m41)
        * det_inv;
    res[3][2] = (m11 * m23 * m42 + m12 * m21 * m43 + m13 * m22 * m41
        - m11 * m22 * m43
        - m12 * m23 * m41
        - m13 * m21 * m42)
        * det_inv;
    res[3][3] = (m11 * m22 * m33 + m12 * m23 * m31 + m13 * m21 * m32
        - m11 * m23 * m32
        - m12 * m21 * m33
        - m13 * m22 * m31)
        * det_inv;

    Mat4::from_cols_array_2d(&res).transpose()
}

pub struct Feature {
    point_id: usize,
    position: Vec2,
}

impl Feature {
    pub fn new(point_id: usize, position: Vec2) -> Self {
        Self { point_id, position }
    }
}

pub struct Image {
    image_name: String,
    features: Vec<Feature>,
    camera_position: Vec3A,
    transform: Mat4,
}

impl Image {
    pub fn new_no_features(image_name: String, camera_position: Vec3A, transform: Mat4) -> Self {
        Self {
            image_name,
            camera_position,
            features: Vec::new(),
            transform,
        }
    }

    pub fn feature_from_point(&mut self, point: &Point, point_id: usize) {
        let projected_point = self.transform * point.location().extend(1.0);
        let position_2d = projected_point.xy() / projected_point.w;
        if position_2d.x < -1.0
            || position_2d.x > 1.0
            || position_2d.y < -1.0
            || position_2d.y > 1.0
        {
            return;
        }

        let feature = Feature::new(point_id, position_2d);
        self.features.push(feature);
    }

    pub fn image_lines(&self, id: usize) -> String {
        let mut content = String::new();

        let look_from = Vec3A::new(
            -self.camera_position.y,
            self.camera_position.x,
            self.camera_position.z,
        );
        let up = -Vec3A::Z;
        let rotation = look_at_quaternion(look_from, Vec3A::ZERO, up);

        content.push_str(&format!(
            "{} {} {} {} {} {} {} {} {} {}\n",
            id, rotation.w, rotation.x, rotation.y, rotation.z, 0, 0, -1, 1, self.image_name
        ));
        for (i, feature) in self.features.iter().enumerate() {
            if i != 0 {
                content.push(' ');
            }
            content.push_str(&format!(
                "{} {} {}",
                feature.position.x, feature.position.y, feature.point_id,
            ))
        }
        content.push('\n');
        content
    }
}

pub fn to_rotation_matrix(q: Quat) -> Mat4 {
    let x = q.x;
    let y = q.y;
    let z = q.z;
    let w = q.w;

    let xx = x * x;
    let yy = y * y;
    let zz = z * z;
    let xy = x * y;
    let yz = y * z;
    let xz = x * z;
    let xw = x * w;
    let yw = y * w;
    let zw = z * w;

    let res = Mat4::from_cols_array(&[
        1.0 - 2.0 * (yy + zz),
        2.0 * (xy + zw),
        2.0 * (xz - yw),
        0.0,
        2.0 * (xy - zw),
        1.0 - 2.0 * (xx + zz),
        2.0 * (yz + xw),
        0.0,
        2.0 * (xz + yw),
        2.0 * (yz - xw),
        1.0 - 2.0 * (xx + yy),
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
    ]);
    res.transpose()
}

pub fn view_matrix(origin: Vec3A, target: Vec3A, up: Vec3A) -> Mat4 {
    let q = look_at_quaternion(origin, target, up);

    let mut rotation_matrix = to_rotation_matrix(q);
    rotation_matrix.w_axis.x = origin.x;
    rotation_matrix.w_axis.y = origin.y;
    rotation_matrix.w_axis.z = origin.z;
    let object_translation_matrix = Mat4::from_translation((-0.5, -0.5, -0.5).into());

    (rotation_matrix).inverse() * object_translation_matrix
}

pub fn projection_matrix(fov: f32, near: f32, far: f32) -> Mat4 {
    let w = fov * near;
    let h = w;

    let left = -w;
    let right = w;
    let bottom = -h;
    let top = h;

    let a00 = 2.0 * near / (right - left);
    let a02 = (right + left) / (right - left);
    let a11 = 2.0 * near / (top - bottom);
    let a12 = (top + bottom) / (top - bottom);
    let a22 = -(far + near) / (far - near);
    let a23 = -2.0 * far * near / (far - near);
    let a32 = -1.0;
    let a33 = 0.0;

    let mut res = Mat4::IDENTITY;
    res.x_axis.x = a00;
    res.x_axis.z = a02;
    res.y_axis.y = a11;
    res.y_axis.z = a12;
    res.z_axis.z = a22;
    res.z_axis.w = a23;
    res.w_axis.z = a32;
    res.w_axis.w = a33;

    res.transpose()
}

pub fn generate_images(arguments: &Arguments, point_cloud: &[Point]) -> Result<Vec<Image>> {
    let mut images = Vec::new();

    let volume_dimensions = arguments.volume_dimensions;
    let output_resolution = arguments.render_resolution;
    let output_directory = Path::new(&arguments.output_directory);
    let focal_length = arguments.focal_length;

    let images_amount_f32 = arguments.images_amount as f32;
    let phi = PI * (5.0f32.sqrt() - 1.0);

    let fov = (1.0 / (2.0 * focal_length)).tan() * 2.0;
    let target = Vec3A::new(0.0, 0.0, 0.0);
    let up = Vec3A::new(0.0, 0.0, -1.0);

    let mut images_txt_contents = String::new();

    for i in 0..arguments.images_amount {
        let i_f32 = i as f32;
        let image_file = format!("{}.ppm", i);
        let output_image_file = output_directory.join("images").join(image_file);
        fs::create_dir_all(output_image_file.parent().unwrap()).into_diagnostic()?;

        let y = 1.0 - (i_f32 / (images_amount_f32 - 1.0)) * 2.0;
        let radius = (1.0 - y * y).sqrt();
        let theta = phi * i_f32;

        let x = theta.cos() * radius;
        let z = theta.sin() * radius;

        let origin = Vec3A::new(x, y, z);

        let view_matrix = -view_matrix(origin, target, up);
        let projection_matrix = projection_matrix(fov, 0.1, 50.0);
        let pvm_matrix = projection_matrix * view_matrix;

        let mvp_matrix = inverse(pvm_matrix).transpose();

        let tf_argument = if let Some(tf_path) = &arguments.transfer_function_filepath {
            ["--tf".to_string(), tf_path.to_string()].to_vec()
        } else {
            Vec::new()
        };

        let mut matrix_argument = mvp_matrix.to_string_vec();
        matrix_argument.insert(0, "--mvp-matrix".to_string());

        let output = Command::new(arguments.vpt_binary.clone())
            .args(["--output", output_image_file.to_str().unwrap()])
            .args([
                "--out-resolution",
                &output_resolution.to_string(),
                &output_resolution.to_string(),
            ])
            .args(["--volume", &arguments.volume_filepath.clone()])
            .args([
                "--volume-dimensions",
                &volume_dimensions[0].to_string(),
                &volume_dimensions[1].to_string(),
                &volume_dimensions[2].to_string(),
            ])
            .args(["--steps", &arguments.render_steps.to_string()])
            .args(["--extinction", &arguments.render_extinction.to_string()])
            .args(["--focal-length", &arguments.focal_length.to_string()])
            .args(["--iterations", &arguments.render_iterations.to_string()])
            .args(tf_argument)
            .arg("--linear")
            .args(matrix_argument)
            .output()
            .into_diagnostic()?;

        //println!("{:?} ({})", output, arguments.volume_filepath);

        if !output.status.success() {
            return Err(miette!("Failed to render image {}: {:?}", i, output.stderr));
        }

        let mut jpg_image_file = output_image_file.clone();
        jpg_image_file.set_extension("jpg");

        Reader::open(&output_image_file)
            .into_diagnostic()?
            .decode()
            .into_diagnostic()?
            .save(jpg_image_file.clone())
            .into_diagnostic()?;

        let mut image = Image::new_no_features(
            jpg_image_file.file_name().unwrap().to_string_lossy().into(),
            origin,
            pvm_matrix,
        );

        for (i, point) in point_cloud.iter().enumerate() {
            image.feature_from_point(point, i);
        }

        images_txt_contents.push_str(&image.image_lines(i));

        images.push(image);
    }

    fs::create_dir_all(output_directory.join("sparse/0")).into_diagnostic()?;
    fs::write(
        output_directory.join("sparse/0/images.txt"),
        images_txt_contents,
    )
    .into_diagnostic()?;

    let f = arguments.focal_length * arguments.render_resolution as f32;
    let w2 = arguments.render_resolution / 2;
    let camera_content = format!(
        "1 PINHOLE {} {} {} {} {}",
        arguments.render_resolution, arguments.render_resolution, f, w2, w2
    );
    fs::write(
        output_directory.join("sparse/0/cameras.txt"),
        camera_content,
    )
    .into_diagnostic()?;

    Ok(images)
}

pub trait MatrixDisplay {
    fn to_string_vec(&self) -> Vec<String>;
}

impl MatrixDisplay for Mat4 {
    fn to_string_vec(&self) -> Vec<String> {
        let v = self.transpose().to_cols_array();
        let mut result = Vec::new();
        for e in v.iter() {
            result.push(e.to_string());
        }
        result
    }
}
