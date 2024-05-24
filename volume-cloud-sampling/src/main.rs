use std::{fs, path::Path};

use argh::FromArgs;
use export::export_to_ply;
use miette::{miette, IntoDiagnostic, Result};
use point_cloud::generate_point_cloud;
use rendering::generate_images;

mod export;
mod point_cloud;
mod rendering;
mod volume;

pub fn parse_volume_dimensions(value: &str) -> Result<[usize; 3], String> {
    let split: Vec<&str> = value.split('x').collect();
    if split.len() != 3 {
        return Err(
            "Volume dimensions received incorrect number of arguments, expected 3".to_string(),
        );
    }

    let x = split[0].parse::<usize>().map_err(|e| e.to_string())?;
    let y = split[1].parse::<usize>().map_err(|e| e.to_string())?;
    let z = split[2].parse::<usize>().map_err(|e| e.to_string())?;

    Ok([x, y, z])
}

pub fn parse_transfer_function_dimensions(value: &str) -> Result<[usize; 2], String> {
    let split: Vec<&str> = value.split('x').collect();
    if split.len() != 2 {
        return Err(
            "Transfer function dimensions received incorrect number of arguments, expected 2"
                .to_string(),
        );
    }

    let x = split[0].parse::<usize>().map_err(|e| e.to_string())?;
    let y = split[1].parse::<usize>().map_err(|e| e.to_string())?;

    Ok([x, y])
}

/// Samples the volume to get point cloud.
#[derive(FromArgs)]
pub struct Arguments {
    /// the amount of points to create
    #[argh(option)]
    point_count: u32,
    /// path to the raw volume file
    #[argh(option)]
    volume_filepath: String,
    /// volume dimensions
    #[argh(option, from_str_fn(parse_volume_dimensions))]
    volume_dimensions: [usize; 3],
    /// path to the transfer function
    #[argh(option)]
    transfer_function_filepath: Option<String>,
    /// transfer function dimensions
    #[argh(option, from_str_fn(parse_transfer_function_dimensions))]
    transfer_function_dimensions: Option<[usize; 2]>,
    /// amount of renders to do
    #[argh(option)]
    images_amount: usize,
    /// location of the vpt binary
    #[argh(option)]
    vpt_binary: String,
    /// resolution of rendered images
    #[argh(option)]
    render_resolution: usize,
    /// render steps
    #[argh(option)]
    render_steps: usize,
    /// render iterations
    #[argh(option)]
    render_iterations: usize,
    /// render extinction parameter
    #[argh(option)]
    render_extinction: usize,
    /// focal length of the camera
    #[argh(option)]
    focal_length: f32,
    /// output directory
    #[argh(option)]
    output_directory: String,
    /// PLY output filepath
    #[argh(option)]
    ply_output: Option<String>,
}

fn main() -> Result<()> {
    let arguments: Arguments = argh::from_env();
    if arguments.transfer_function_filepath.is_some()
        && arguments.transfer_function_dimensions.is_none()
    {
        return Err(miette!(
            "Transfer function file provided, but not its dimensions!"
        ));
    }

    let point_cloud = generate_point_cloud(&arguments)?;
    println!("Created point cloud");

    let images = generate_images(&arguments, &point_cloud)?;
    println!("Created images and files");

    let mut points_txt_contents = String::new();
    for (i, point) in point_cloud.iter().enumerate() {
        let a = format!(
            "{} {} {} {} {} {} {} {} {} {}\n",
            i,
            point.location().x,
            point.location().y,
            point.location().z,
            point.color().x,
            point.color().y,
            point.color().z,
            0,
            0,
            0
        );
        points_txt_contents.push_str(&a);
    }
    println!("Created points file");

    let output_directory = Path::new(&arguments.output_directory).join("sparse/0/points3D.txt");
    fs::write(output_directory, points_txt_contents).into_diagnostic()?;

    if arguments.ply_output.is_some() {
        export_to_ply(&arguments, point_cloud)?;
    }

    Ok(())
}
