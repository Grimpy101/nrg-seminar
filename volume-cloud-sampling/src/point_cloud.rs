use glam::{Vec3A, Vec4Swizzles};
use miette::Result;
use rand::{Rng, SeedableRng};
use rand_xoshiro::Xoshiro256Plus;

use crate::{
    volume::{TransferFunction, Volume},
    Arguments,
};

pub struct Point {
    location: Vec3A,
    color: Vec3A,
}

impl Point {
    pub fn new(location: Vec3A, color: Vec3A) -> Self {
        Self { location, color }
    }

    pub fn location(&self) -> Vec3A {
        self.location
    }

    pub fn color(&self) -> Vec3A {
        self.color
    }
}

fn get_random_vector(rng: &mut Xoshiro256Plus) -> Vec3A {
    let x = rng.gen::<f32>() * 2.0 - 1.0;
    let y = rng.gen::<f32>() * 2.0 - 1.0;
    let z = rng.gen::<f32>() * 2.0 - 1.0;

    Vec3A::new(x, y, z)
}

pub fn generate_point_cloud(arguments: &Arguments) -> Result<Vec<Point>> {
    let mut points = Vec::new();
    let mut random_generator = Xoshiro256Plus::from_rng(rand::thread_rng())
        .expect("Could not get random number generator!");

    let volume = Volume::new_from_args(arguments)?;
    let transfer_function = TransferFunction::new_from_args(arguments)?;

    for _ in 0..arguments.point_count {
        let location = get_random_vector(&mut random_generator);

        let volume_sample = volume.sample(location, true);
        let color = transfer_function.get_color(volume_sample).xyz().into();

        let point = Point::new(location, color);
        points.push(point);
    }

    Ok(points)
}
