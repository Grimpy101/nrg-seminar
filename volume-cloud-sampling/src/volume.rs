use glam::{Vec3A, Vec4};
use miette::{IntoDiagnostic, Result};
use std::fs;

use crate::Arguments;

pub struct Volume {
    width: usize,
    height: usize,
    depth: usize,
    data: Vec<u8>,
}

impl Volume {
    pub fn new_from_args(arguments: &Arguments) -> Result<Volume> {
        let data = fs::read(&arguments.volume_filepath).into_diagnostic()?;

        let width = arguments.volume_dimensions[0];
        let height = arguments.volume_dimensions[1];
        let depth = arguments.volume_dimensions[2];

        Ok(Volume {
            width,
            height,
            depth,
            data,
        })
    }

    fn index_3d_to_1d(&self, x: usize, y: usize, z: usize) -> usize {
        z * self.height * self.width + y * self.width + x
    }

    pub fn sample(&self, world_position: Vec3A, is_linear: bool) -> u8 {
        let x = world_position.x * 0.5 + 0.5;
        let y = world_position.y * 0.5 + 0.5;
        let z = world_position.z * 0.5 + 0.5;

        let absolute_x = x * (self.width - 1) as f32;
        let absolute_y = y * (self.height - 1) as f32;
        let absolute_z = z * (self.depth - 1) as f32;

        if is_linear {
            let min_x = absolute_x.floor();
            let max_x = absolute_x.ceil();

            let min_y = absolute_y.floor();
            let max_y = absolute_y.ceil();

            let min_z = absolute_z.floor();
            let max_z = absolute_z.ceil();

            let min_x_usize = (min_x as usize).clamp(0, self.width);
            let max_x_usize = (max_x as usize).clamp(0, self.width);
            let min_y_usize = (min_y as usize).clamp(0, self.height);
            let max_y_usize = (max_y as usize).clamp(0, self.height);
            let min_z_usize = (min_z as usize).clamp(0, self.depth);
            let max_z_usize = (max_z as usize).clamp(0, self.depth);

            let i000 = self.index_3d_to_1d(min_x_usize, min_y_usize, min_z_usize);
            let c000 = self.data[i000] as f32;

            let i100 = self.index_3d_to_1d(max_x_usize, min_y_usize, min_z_usize);
            let c100 = self.data[i100] as f32;

            let i010 = self.index_3d_to_1d(min_x_usize, max_y_usize, min_z_usize);
            let c010 = self.data[i010] as f32;

            let i001 = self.index_3d_to_1d(min_x_usize, min_y_usize, max_z_usize);
            if i001 >= self.data.len() {
                println!("{} {} {}", min_x_usize, min_y_usize, max_z_usize);
            }
            let c001 = self.data[i001] as f32;

            let i101 = self.index_3d_to_1d(max_x_usize, min_y_usize, max_z_usize);
            let c101 = self.data[i101] as f32;

            let i110 = self.index_3d_to_1d(max_x_usize, max_y_usize, min_z_usize);
            let c110 = self.data[i110] as f32;

            let i011 = self.index_3d_to_1d(min_x_usize, max_y_usize, max_z_usize);
            let c011 = self.data[i011] as f32;

            let i111 = self.index_3d_to_1d(max_x_usize, max_y_usize, max_z_usize);
            let c111 = self.data[i111] as f32;

            let dx = absolute_x - min_x;
            let dy = absolute_y - min_y;
            let dz = absolute_z - min_z;

            let c00 = c000 * (1.0 - dx) + c100 * dx;
            let c01 = c001 * (1.0 - dx) + c101 * dx;
            let c10 = c010 * (1.0 - dx) + c110 * dx;
            let c11 = c011 * (1.0 - dx) + c111 * dx;

            let c0 = c00 * (1.0 - dy) + c10 * dy;
            let c1 = c01 * (1.0 - dy) + c11 * dy;

            let c = c0 * (1.0 - dz) + c1 * dz;

            return c as u8;
        }

        let x = absolute_x as usize;
        let y = absolute_y as usize;
        let z = absolute_z as usize;

        let i = self.index_3d_to_1d(x, y, z);
        self.data[i]
    }
}

pub struct TransferFunction {
    width: usize,
    height: usize,
    data: Vec<u8>,
}

impl TransferFunction {
    pub fn new_from_args(arguments: &Arguments) -> Result<TransferFunction> {
        if let Some(transfer_function_filepath) = &arguments.transfer_function_filepath {
            let tf_dimensions = arguments.transfer_function_dimensions.unwrap();

            let data = fs::read(transfer_function_filepath).into_diagnostic()?;
            let width = tf_dimensions[0];
            let height = tf_dimensions[1];

            return Ok(TransferFunction {
                width,
                height,
                data,
            });
        }

        Ok(TransferFunction {
            width: 2,
            height: 1,
            data: vec![0, 0, 0, 0, 255, 0, 0, 255],
        })
    }

    fn sample(&self, position: f32) -> Vec4 {
        let relative_y = 0.5;
        let relative_x = position;

        let absolute_y = relative_y * self.height as f32;
        let absolute_x = relative_x * self.width as f32;

        let x = absolute_x as usize;
        let y = absolute_y as usize;
        let i = (x + y * self.width) * 4;

        let r = self.data[i] as f32 / 255.0;
        let g = self.data[i + 1] as f32 / 255.0;
        let b = self.data[i + 2] as f32 / 255.0;
        let a = self.data[i + 3] as f32 / 255.0;

        Vec4::new(r, g, b, a)
    }

    pub fn get_color(&self, value: u8) -> Vec4 {
        let relative = value as f32 / 255.0;
        self.sample(relative)
    }
}
